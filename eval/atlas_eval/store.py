"""Durable job store for evaluation sweeps: SQLite for job state, files named
by their SHA-256 for results.

A job is one (configuration, sequence) run. Its life:

    queued --lease--> leased --complete--> done
       ^                |  \\--fail / lease expired--> queued again after a
       |                |                              backoff delay, or failed
       +----------------+                              after max_attempts

Guarantees, and how:

- **Durable.** Every change is one SQLite transaction (WAL, synchronous=FULL).
  A result file is written to a temporary name, fsynced, renamed into place,
  and its directory fsynced BEFORE the transaction that marks the job done,
  so a done job always has its file. A crash between the two leaves an
  unreferenced file and a job that runs again: wasted work, not lost work.
- **Leases with fencing tokens.** A worker owns a job only while its lease is
  unexpired, and every call names the lease's random token. When a lease
  expires (worker crashed, hung, or lost the network), the job is queued
  again; a later complete/heartbeat with the old token is rejected
  (StaleLease). So a job can be EXECUTED twice, but only one result is ever
  ACCEPTED. This is at-least-once execution with one accepted result per
  job, not exactly-once execution.
- **Idempotent.** A job's id is the hash of its spec, so submitting the same
  sweep twice adds nothing. Repeating complete() with the same token and
  result (a retry after a lost reply) returns the same answer. Result files
  are content-addressed: the same bytes always land at the same name.
- **Bounded.** At most `capacity` jobs are queued or leased; submit() raises
  QueueFull beyond that, and the submitter backs off (backpressure).
- **Restart grace.** A starting coordinator gives live leases one full
  period (grant_restart_grace), since workers could not heartbeat while it
  was down.
- **Retries with backoff.** A failed or expired attempt waits
  min(cap, base * 2^(attempt-1)) seconds, times a random factor in
  [0.5, 1] (jitter, so retries do not all line up), before it can be leased.

The clock and random source are injectable for tests. One Store object is
safe to share between threads (one lock); one coordinator process owns the
database.
"""
from __future__ import annotations

import hashlib
import json
import os
import random
import secrets
import sqlite3
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Iterator, Optional


class QueueFull(Exception):
    """The bounded queue is at capacity; submit again later."""


class StaleLease(Exception):
    """The caller's lease is no longer the job's current lease."""


class UnknownJob(Exception):
    pass


def canonical(obj: Any) -> bytes:
    """JSON with sorted keys and no spaces: equal content, equal bytes, equal hash."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def job_id_for(spec: dict) -> str:
    return hashlib.sha256(canonical(spec)).hexdigest()[:16]


@dataclass
class Lease:
    job_id: str
    token: str
    spec: dict
    attempt: int
    expires: float


SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
    id            TEXT PRIMARY KEY,
    spec          TEXT NOT NULL,
    state         TEXT NOT NULL CHECK (state IN ('queued', 'leased', 'done', 'failed')),
    attempts      INTEGER NOT NULL DEFAULT 0,
    token         TEXT,
    worker        TEXT,
    lease_expires REAL,
    not_before    REAL NOT NULL DEFAULT 0,
    result_hash   TEXT,
    last_error    TEXT,
    created       REAL NOT NULL,
    seq           INTEGER NOT NULL,
    finished      REAL
);
CREATE INDEX IF NOT EXISTS jobs_ready ON jobs (state, not_before, seq);
CREATE TABLE IF NOT EXISTS events (
    seq    INTEGER PRIMARY KEY AUTOINCREMENT,
    at     REAL NOT NULL,
    job_id TEXT,
    kind   TEXT NOT NULL,
    worker TEXT,
    detail TEXT
);
"""


class Store:
    def __init__(self, root: str, *, capacity: int = 64, lease_seconds: float = 30.0, max_attempts: int = 3,
                 backoff_base: float = 2.0, backoff_cap: float = 60.0,
                 clock: Callable[[], float] = time.time, rng: Optional[random.Random] = None):
        if capacity < 1 or lease_seconds <= 0 or max_attempts < 1:
            raise ValueError("capacity, lease_seconds and max_attempts must be positive")
        self.root = root
        self.objects = os.path.join(root, "objects")
        os.makedirs(self.objects, exist_ok=True)
        self.capacity = capacity
        self.lease_seconds = lease_seconds
        self.max_attempts = max_attempts
        self.backoff_base = backoff_base
        self.backoff_cap = backoff_cap
        self.clock = clock
        self.rng = rng or random.Random()
        self._lock = threading.Lock()
        self._db = sqlite3.connect(os.path.join(root, "jobs.sqlite"), isolation_level=None, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.executescript(SCHEMA)

    def close(self) -> None:
        self._db.close()

    @contextmanager
    def _tx(self) -> Iterator[sqlite3.Connection]:
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                yield self._db
            except BaseException:
                self._db.execute("ROLLBACK")
                raise
            self._db.execute("COMMIT")

    def _event(self, db: sqlite3.Connection, now: float, job_id: Optional[str], kind: str,
               worker: Optional[str] = None, detail: Optional[str] = None) -> None:
        db.execute("INSERT INTO events (at, job_id, kind, worker, detail) VALUES (?, ?, ?, ?, ?)",
                   (now, job_id, kind, worker, detail))

    def backoff(self, attempts: int) -> float:
        """Delay before attempt `attempts + 1`, in seconds."""
        return min(self.backoff_cap, self.backoff_base * 2 ** (attempts - 1)) * self.rng.uniform(0.5, 1.0)

    # ---- submitting -------------------------------------------------------
    def submit(self, spec: dict) -> tuple[str, bool]:
        """Adds a job; returns (job id, True if new). Same spec, same id, added once."""
        job_id = job_id_for(spec)
        now = self.clock()
        with self._tx() as db:
            if db.execute("SELECT 1 FROM jobs WHERE id = ?", (job_id,)).fetchone():
                return job_id, False
            active = db.execute("SELECT COUNT(*) FROM jobs WHERE state IN ('queued', 'leased')").fetchone()[0]
            if active >= self.capacity:
                raise QueueFull(f"{active} jobs queued or running (capacity {self.capacity})")
            seq = db.execute("SELECT COALESCE(MAX(seq), 0) + 1 FROM jobs").fetchone()[0]
            db.execute("INSERT INTO jobs (id, spec, state, created, seq) VALUES (?, ?, 'queued', ?, ?)",
                       (job_id, canonical(spec).decode(), now, seq))
            self._event(db, now, job_id, "submitted")
        return job_id, True

    # ---- leases -----------------------------------------------------------
    def _retry_or_fail(self, db: sqlite3.Connection, job_id: str, attempts: int, error: str, now: float) -> str:
        if attempts >= self.max_attempts:
            db.execute("UPDATE jobs SET state = 'failed', token = NULL, worker = NULL, lease_expires = NULL, "
                       "last_error = ?, finished = ? WHERE id = ?", (error, now, job_id))
            self._event(db, now, job_id, "gave_up", detail=f"after {attempts} attempts: {error}")
            return "failed"
        delay = self.backoff(attempts)
        db.execute("UPDATE jobs SET state = 'queued', token = NULL, worker = NULL, lease_expires = NULL, "
                   "not_before = ?, last_error = ? WHERE id = ?", (now + delay, error, job_id))
        self._event(db, now, job_id, "requeued", detail=f"retry after {delay:.1f} s: {error}")
        return "queued"

    def _reap(self, db: sqlite3.Connection, now: float) -> int:
        expired = db.execute("SELECT id, attempts, worker FROM jobs WHERE state = 'leased' AND lease_expires <= ?",
                             (now,)).fetchall()
        for row in expired:
            self._event(db, now, row["id"], "lease_expired", row["worker"])
            self._retry_or_fail(db, row["id"], row["attempts"], f"lease expired (worker {row['worker']})", now)
        return len(expired)

    def reap(self) -> int:
        """Requeues jobs whose lease has expired; returns how many."""
        with self._tx() as db:
            return self._reap(db, self.clock())

    def lease(self, worker: str) -> Optional[Lease]:
        """Gives the oldest ready job to `worker`, or None if nothing is ready."""
        now = self.clock()
        with self._tx() as db:
            self._reap(db, now)
            row = db.execute("SELECT id, spec, attempts FROM jobs WHERE state = 'queued' AND not_before <= ? "
                             "ORDER BY seq LIMIT 1", (now,)).fetchone()
            if row is None:
                return None
            token = secrets.token_hex(8)
            attempt = row["attempts"] + 1
            expires = now + self.lease_seconds
            db.execute("UPDATE jobs SET state = 'leased', attempts = ?, token = ?, worker = ?, lease_expires = ? "
                       "WHERE id = ?", (attempt, token, worker, expires, row["id"]))
            self._event(db, now, row["id"], "leased", worker, f"attempt {attempt}")
        return Lease(row["id"], token, json.loads(row["spec"]), attempt, expires)

    def grant_restart_grace(self) -> int:
        """Gives every live lease one full lease period from now; returns how many.

        Called when a coordinator starts. While it was down, workers could not
        heartbeat, so without this every lease older than the outage would
        expire at once and running jobs would be repeated. A worker that really
        died during the outage costs at most one extra lease period.
        """
        now = self.clock()
        with self._tx() as db:
            n = db.execute("UPDATE jobs SET lease_expires = MAX(lease_expires, ?) WHERE state = 'leased'",
                           (now + self.lease_seconds,)).rowcount
            if n:
                self._event(db, now, None, "restart_grace", detail=f"{n} live lease(s) extended")
        return n

    def heartbeat(self, job_id: str, token: str) -> float:
        """Extends a live lease; returns its new expiry. Raises StaleLease if it is not live."""
        now = self.clock()
        with self._tx() as db:
            self._reap(db, now)
            row = db.execute("SELECT state, token FROM jobs WHERE id = ?", (job_id,)).fetchone()
            if row is None:
                raise UnknownJob(job_id)
            if row["state"] != "leased" or row["token"] != token:
                stale = True
                self._event(db, now, job_id, "stale_heartbeat_rejected", detail=f"job is {row['state']}")
            else:
                stale = False
                expires = now + self.lease_seconds
                db.execute("UPDATE jobs SET lease_expires = ? WHERE id = ?", (expires, job_id))
        if stale:
            raise StaleLease(f"job {job_id}: lease is no longer held")
        return expires

    # ---- results ----------------------------------------------------------
    def put_object(self, data: bytes) -> str:
        """Stores bytes under their SHA-256 and returns it. Same bytes, same file; safe to repeat."""
        digest = hashlib.sha256(data).hexdigest()
        folder = os.path.join(self.objects, digest[:2])
        path = os.path.join(folder, digest)
        if os.path.exists(path):
            return digest
        os.makedirs(folder, exist_ok=True)
        tmp = f"{path}.tmp-{os.getpid()}-{threading.get_ident()}-{secrets.token_hex(4)}"
        with open(tmp, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)  # atomic: readers see the whole file or none
        fd = os.open(folder, os.O_RDONLY)
        try:
            os.fsync(fd)  # make the new name itself durable
        finally:
            os.close(fd)
        return digest

    def get_object(self, digest: str) -> bytes:
        with open(os.path.join(self.objects, digest[:2], digest), "rb") as f:
            data = f.read()
        if hashlib.sha256(data).hexdigest() != digest:
            raise IOError(f"object {digest} is corrupt")
        return data

    def complete(self, job_id: str, token: str, result: dict, artifacts: Optional[dict[str, bytes]] = None,
                 worker: Optional[str] = None) -> str:
        """Publishes a result for a live lease; returns the result's hash.

        Files first (durable, content-addressed), then one transaction marks
        the job done. A repeat with the same token and result returns the same
        hash; any other late or foreign completion raises StaleLease.
        """
        refs = {name: self.put_object(data) for name, data in sorted((artifacts or {}).items())}
        with self._lock:
            row = self._db.execute("SELECT spec FROM jobs WHERE id = ?", (job_id,)).fetchone()
        if row is None:
            raise UnknownJob(job_id)
        document = {"job_id": job_id, "spec": json.loads(row["spec"]), "result": result, "artifacts": refs}
        digest = self.put_object(canonical(document))
        now = self.clock()
        outcome = "stale"
        with self._tx() as db:
            self._reap(db, now)
            row = db.execute("SELECT state, token, result_hash FROM jobs WHERE id = ?", (job_id,)).fetchone()
            if row["state"] == "leased" and row["token"] == token:
                db.execute("UPDATE jobs SET state = 'done', result_hash = ?, finished = ?, last_error = NULL "
                           "WHERE id = ?", (digest, now, job_id))
                self._event(db, now, job_id, "completed", worker, digest[:12])
                outcome = "accepted"
            elif row["state"] == "done" and row["token"] == token and row["result_hash"] == digest:
                outcome = "repeat"
            else:
                self._event(db, now, job_id, "stale_result_rejected", worker,
                            f"job is {row['state']}; result {digest[:12]} not published")
        if outcome == "stale":
            raise StaleLease(f"job {job_id}: lease is no longer held; result not published")
        return digest

    def fail(self, job_id: str, token: str, error: str, worker: Optional[str] = None) -> str:
        """Reports a failed attempt; returns the job's new state ('queued' or 'failed')."""
        now = self.clock()
        with self._tx() as db:
            self._reap(db, now)
            row = db.execute("SELECT state, token, attempts FROM jobs WHERE id = ?", (job_id,)).fetchone()
            if row is None:
                raise UnknownJob(job_id)
            if row["state"] == "leased" and row["token"] == token:
                self._event(db, now, job_id, "attempt_failed", worker, error[:500])
                return self._retry_or_fail(db, job_id, row["attempts"], error[:500], now)
        raise StaleLease(f"job {job_id}: lease is no longer held")

    # ---- reading ----------------------------------------------------------
    def counts(self) -> dict[str, int]:
        with self._lock:
            rows = self._db.execute("SELECT state, COUNT(*) AS n FROM jobs GROUP BY state").fetchall()
        out = {"queued": 0, "leased": 0, "done": 0, "failed": 0}
        out.update({r["state"]: r["n"] for r in rows})
        return out

    def jobs(self) -> list[dict]:
        with self._lock:
            rows = self._db.execute("SELECT id, spec, state, attempts, worker, lease_expires, result_hash, last_error "
                                    "FROM jobs ORDER BY seq").fetchall()
        return [{**dict(r), "spec": json.loads(r["spec"])} for r in rows]

    def events(self) -> list[dict]:
        with self._lock:
            return [dict(r) for r in self._db.execute("SELECT * FROM events ORDER BY seq").fetchall()]

    def results(self) -> list[dict]:
        """Every accepted result document, read back from its file (hash checked)."""
        return [json.loads(self.get_object(j["result_hash"])) for j in self.jobs() if j["state"] == "done"]
