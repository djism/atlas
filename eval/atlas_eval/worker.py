"""Worker: lease a job, run it while heartbeating, publish the result, repeat.

Failure handling:

- Coordinator unreachable or 5xx: every call is retried with exponential
  backoff and jitter (0.2 s doubling to 5 s) for up to `patience` seconds, so
  a coordinator restart does not kill the workers.
- Heartbeat answered 409 (lease lost: it expired, e.g. after this process was
  paused): the run is cancelled and nothing is published.
- The run raises: the attempt is reported with /fail; the coordinator decides
  whether to retry (with backoff) or give up.
- /complete answered 409: another worker's result was accepted (or will be);
  this result is discarded. The coordinator, not the worker, enforces this,
  so even a worker that missed every heartbeat cannot publish twice.
"""
from __future__ import annotations

import base64
import json
import random
import sys
import threading
import time
import urllib.error
import urllib.request
from typing import Callable

from .runners import Cancelled


class Unreachable(Exception):
    pass


def log(name: str, message: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {name:<12} {message}", file=sys.stderr, flush=True)


class Client:
    def __init__(self, url: str, patience: float = 120.0, timeout: float = 30.0):
        self.url = url.rstrip("/")
        self.patience = patience
        self.timeout = timeout

    def call(self, method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
        """Returns (HTTP status, JSON body). Retries connection errors and 5xx with backoff."""
        data = None if body is None else json.dumps(body).encode()
        deadline = time.monotonic() + self.patience
        delay = 0.2
        while True:
            request = urllib.request.Request(self.url + path, data=data, method=method,
                                             headers={"Content-Type": "application/json"})
            try:
                with urllib.request.urlopen(request, timeout=self.timeout) as response:
                    raw = response.read()
                    return response.status, (json.loads(raw) if raw else {})
            except urllib.error.HTTPError as e:
                with e:
                    raw = e.read()
                if e.code < 500:
                    return e.code, (json.loads(raw) if raw else {})
                problem = f"HTTP {e.code}"
            except (urllib.error.URLError, ConnectionError, TimeoutError, OSError) as e:
                problem = str(getattr(e, "reason", e))
            if time.monotonic() + delay > deadline:
                raise Unreachable(f"{method} {path}: {problem}")
            time.sleep(delay * random.uniform(0.5, 1.0))
            delay = min(delay * 2, 5.0)


def run_worker(url: str, name: str, runners: dict[str, Callable], *, exit_when_idle: bool = False,
               poll: float = 1.0, patience: float = 120.0) -> int:
    """Runs until stopped (or, with exit_when_idle, until no job is queued or running). Returns jobs published."""
    client = Client(url, patience=patience)
    published = 0
    while True:
        status, lease = client.call("POST", "/lease", {"worker": name})
        if status == 204:
            if exit_when_idle:
                counts = client.call("GET", "/status")[1]["counts"]
                if counts["queued"] == 0 and counts["leased"] == 0:
                    log(name, f"no work left; published {published}")
                    return published
            time.sleep(poll * random.uniform(0.5, 1.0))
            continue
        if status != 200:
            raise RuntimeError(f"lease: HTTP {status} {lease}")
        job, token, spec = lease["job_id"], lease["token"], lease["spec"]
        log(name, f"leased {job} attempt {lease['attempt']}: {describe(spec)}")
        lost = threading.Event()
        done = threading.Event()
        # The thread gets THIS job's id, token and events as arguments. (It
        # once read them from the loop's variables, which by then could name
        # the next job: a late 409 for a finished job cancelled the next one.)
        beat = threading.Thread(target=_heartbeat, daemon=True,
                                args=(client, name, job, token, lease["lease_seconds"] / 3, done, lost))
        beat.start()
        started = time.monotonic()
        try:
            runner = runners.get(spec.get("kind"))
            if runner is None:
                raise RuntimeError(f"no runner for kind {spec.get('kind')!r}")
            result, artifacts = runner(spec, lease["attempt"], lost)
        except Cancelled:
            _stop(done, beat)
            continue
        except Exception as e:  # report and move on: the coordinator decides about retries
            _stop(done, beat)
            log(name, f"run failed: {e}")
            client.call("POST", "/fail", {"job_id": job, "token": token, "worker": name, "error": str(e)})
            continue
        _stop(done, beat)
        result = {**result, "worker": name, "attempt": lease["attempt"], "wall_seconds": round(time.monotonic() - started, 2)}
        st, reply = client.call("POST", "/complete", {
            "job_id": job, "token": token, "worker": name, "result": result,
            "artifacts": {k: base64.b64encode(v).decode() for k, v in artifacts.items()}})
        if st == 200:
            published += 1
            log(name, f"published {job} -> {reply['result_hash'][:12]}")
        elif st == 409:
            log(name, f"result for {job} REJECTED (lease no longer held); discarded")
        else:
            log(name, f"complete: HTTP {st} {reply}")


def _heartbeat(client: Client, name: str, job: str, token: str, interval: float,
               done: threading.Event, lost: threading.Event) -> None:
    while not done.wait(interval):
        try:
            status, _ = client.call("POST", "/heartbeat", {"job_id": job, "token": token})
        except Unreachable:
            continue
        if status == 409:
            if not done.is_set():
                log(name, f"lease on {job} lost; stopping the run")
            lost.set()
            return


def _stop(done: threading.Event, beat: threading.Thread) -> None:
    """Ends the job's heartbeat and waits for it, so no heartbeat outlives its job."""
    done.set()
    beat.join()


def describe(spec: dict) -> str:
    if spec.get("kind") == "kitti-map-odometry":
        return f"seq {spec['sequence']} " + " ".join(f"{k}={v}" for k, v in spec["params"].items())
    return json.dumps(spec, sort_keys=True)
