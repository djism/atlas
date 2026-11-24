"""End-to-end demonstration with real processes:

1. start a coordinator, a submitter (blocked by the bounded queue as needed),
   and N workers;
2. kill -9 a worker that is running a job (a crash): its lease expires and the
   job is run again by someone else; a fresh worker joins;
3. kill -9 the coordinator and restart it on the same store: workers and the
   submitter retry with backoff and carry on;
4. SIGSTOP a worker that is running a job for longer than its lease, then
   SIGCONT it (a long pause, like a VM stall): meanwhile its job is
   re-leased; when it wakes, its heartbeat or its result is rejected;
5. when everything is finished, build the report from the stored results.

Prints a timeline; per-process logs go to <store>/logs/.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time

from .report import build_report
from .store import Store
from .worker import Client, Unreachable

CLI = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cli.py")


def add_arguments(d) -> None:
    d.add_argument("--store", required=True, help="a NEW directory for this run")
    d.add_argument("--port", type=int, default=8766)
    d.add_argument("--workers", type=int, default=3)
    d.add_argument("--lease", type=float, default=6.0, help="lease seconds (short, so the demo is quick)")
    d.add_argument("--capacity", type=int, default=8, help="bounded queue size (small, to show backpressure)")
    d.add_argument("--kitti", action="store_true", help="run real KITTI jobs (needs --engine, --odometry, --velodyne)")
    d.add_argument("--engine")
    d.add_argument("--odometry")
    d.add_argument("--velodyne", nargs="*")
    d.add_argument("--sequences", nargs="+", default=["04", "07"])
    d.add_argument("--grid", nargs="*", default=["voxel=0.75,1.0,1.5", "deskew=0,1", "correct=0,1"])
    d.add_argument("--test-jobs", type=int, default=24)
    d.add_argument("--test-seconds", type=float, default=2.0)
    d.add_argument("--no-chaos", action="store_true", help="no crashes: a plain distributed run")
    d.add_argument("--timeout", type=float, default=3600)


def run(a) -> int:
    if os.path.isdir(a.store) and os.listdir(a.store):
        print(f"{a.store} is not empty; use a new directory so the report covers only this run", file=sys.stderr)
        return 2
    logs = os.path.join(a.store, "logs")
    os.makedirs(logs, exist_ok=True)
    url = f"http://127.0.0.1:{a.port}"
    client = Client(url, patience=30)
    t0 = time.monotonic()
    procs: dict[str, subprocess.Popen] = {}

    def say(message: str) -> None:
        print(f"[{time.monotonic() - t0:6.1f} s] {message}", flush=True)

    def start(name: str, args: list[str]) -> None:
        log = open(os.path.join(logs, f"{name}.log"), "a")
        procs[name] = subprocess.Popen([sys.executable, CLI, *args], stdout=log, stderr=subprocess.STDOUT)

    def start_coordinator() -> None:
        start("coordinator", ["coordinator", "--store", a.store, "--port", str(a.port), "--lease", str(a.lease),
                              "--capacity", str(a.capacity)])
        client.call("GET", "/status")  # waits (with backoff) until it answers

    worker_args = []
    if a.kitti:
        worker_args = ["--engine", a.engine, "--odometry", a.odometry, "--velodyne", *a.velodyne]

    def start_worker(name: str) -> None:
        start(name, ["worker", "--url", url, "--name", name, "--exit-when-idle", "--poll", "0.5", *worker_args])

    def holder(name: str, min_age: float = 0.5, timeout: float = 600) -> str:
        """Waits until worker `name` holds a lease (for min_age seconds); returns the job id."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = client.call("GET", "/status")[1]
            if status["counts"]["queued"] + status["counts"]["leased"] == 0:
                raise RuntimeError(f"the work finished before {name} could be interrupted; use more or longer jobs")
            jobs = status["jobs"]
            held = [j["id"] for j in jobs if j["state"] == "leased" and j["worker"] == name]
            if held:
                time.sleep(min_age)
                jobs = client.call("GET", "/status")[1]["jobs"]
                if any(j["id"] == held[0] and j["state"] == "leased" and j["worker"] == name for j in jobs):
                    return held[0]
            time.sleep(0.3)
        raise TimeoutError(f"{name} never held a lease")

    try:
        start_coordinator()
        say(f"coordinator up at {url} (lease {a.lease:g} s, queue capacity {a.capacity})")
        if a.kitti:
            start("submitter", ["sweep", "--url", url, "--engine", a.engine, "--sequences", *a.sequences, "--grid", *a.grid])
        else:
            start("submitter", ["submit-test", "--url", url, "--jobs", str(a.test_jobs), "--seconds",
                                str(a.test_seconds), "--fail-every", "7"])
        while sum(client.call("GET", "/status")[1]["counts"].values()) == 0:
            time.sleep(0.2)
        names = [f"w{i + 1}" for i in range(a.workers)]
        for name in names:
            start_worker(name)
        say(f"submitter and {a.workers} workers started: {', '.join(names)}")

        if not a.no_chaos:
            job = holder("w1")
            procs["w1"].kill()
            say(f"CRASH: kill -9 w1 while it runs job {job}")
            time.sleep(2)
            start_worker(f"w{a.workers + 1}")
            say(f"w{a.workers + 1} joins")

            before = client.call("GET", "/status")[1]["counts"]
            procs["coordinator"].kill()
            procs["coordinator"].wait()
            say(f"CRASH: kill -9 the coordinator (state before: {before})")
            time.sleep(4)
            start_coordinator()
            say(f"coordinator restarted on the same store: {client.call('GET', '/status')[1]['counts']}")

            job = holder("w2")
            procs["w2"].send_signal(signal.SIGSTOP)
            pause = 2 * a.lease + 2
            say(f"PAUSE: SIGSTOP w2 for {pause:g} s (longer than its {a.lease:g} s lease) while it runs job {job}")
            time.sleep(pause)
            procs["w2"].send_signal(signal.SIGCONT)
            say("w2 resumes; its old lease is gone")

        deadline = time.monotonic() + a.timeout
        while time.monotonic() < deadline:
            try:
                counts = client.call("GET", "/status")[1]["counts"]
            except Unreachable:
                break
            if procs["submitter"].poll() is not None and counts["queued"] == 0 and counts["leased"] == 0:
                break
            time.sleep(1)
        say(f"finished: {counts}")
        for name, p in procs.items():
            if name.startswith("w"):
                try:
                    p.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    p.kill()
    finally:
        for p in procs.values():
            if p.poll() is None:
                p.terminate()
        for p in procs.values():
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()

    store = Store(a.store)
    report = build_report(store, "Atlas evaluation sweep" if a.kitti else "Atlas evaluation demo (test jobs)")
    with open(os.path.join(a.store, "report.md"), "w") as f:
        f.write(report)
    counts = store.counts()
    accepted = [e["job_id"] for e in store.events() if e["kind"] == "completed"]
    ok = counts["done"] == len(store.jobs()) and len(accepted) == len(set(accepted)) == counts["done"]
    say(f"report: {os.path.join(a.store, 'report.md')}; every job done with exactly one accepted result: {ok}")
    return 0 if ok else 1
