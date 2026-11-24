"""Atlas distributed evaluation: command line. Run from the repository root:

    python3 eval/cli.py coordinator --store results/eval/sweep1
    python3 eval/cli.py worker --name w1 --engine build/atlas_kitti --odometry <dir> --velodyne 07=<dir> ...
    python3 eval/cli.py sweep --engine build/atlas_kitti --sequences 04 07 --grid voxel=0.75,1.0,1.5 deskew=0,1
    python3 eval/cli.py status
    python3 eval/cli.py report --store results/eval/sweep1
    python3 eval/cli.py demo --store results/eval/demo [--kitti ...]   (crash and recovery, end to end)

Standard library only (Python 3.9+). See docs/evaluation.md.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from atlas_eval import demo  # noqa: E402
from atlas_eval.coordinator import log, serve  # noqa: E402
from atlas_eval.report import build_report  # noqa: E402
from atlas_eval.runners import KittiRunner, file_hash, test_runner  # noqa: E402
from atlas_eval.store import Store  # noqa: E402
from atlas_eval.sweep import kitti_specs, parse_grid, submit_all  # noqa: E402
from atlas_eval.worker import Client, run_worker  # noqa: E402


def velodyne_map(items: list[str]) -> dict[str, str]:
    out = {}
    for item in items or []:
        seq, _, path = item.partition("=")
        if not seq or not path:
            raise SystemExit(f"--velodyne expects SEQ=DIR, got {item!r}")
        out[seq] = os.path.expanduser(path)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="eval/cli.py", description="Atlas distributed evaluation")
    sub = ap.add_subparsers(dest="cmd", required=True)
    url_default = "http://127.0.0.1:8765"

    c = sub.add_parser("coordinator", help="serve the job store over HTTP")
    c.add_argument("--store", required=True)
    c.add_argument("--host", default="127.0.0.1")
    c.add_argument("--port", type=int, default=8765)
    c.add_argument("--capacity", type=int, default=64, help="max jobs queued or running")
    c.add_argument("--lease", type=float, default=30.0, help="lease length, seconds")
    c.add_argument("--max-attempts", type=int, default=3)

    w = sub.add_parser("worker", help="lease and run jobs")
    w.add_argument("--url", default=url_default)
    w.add_argument("--name", required=True)
    w.add_argument("--engine", help="path to atlas_kitti (for KITTI jobs)")
    w.add_argument("--odometry", help=".../odometry/dataset")
    w.add_argument("--velodyne", nargs="*", help="SEQ=DIR of raw scans, e.g. 07=~/.../velodyne_points/data")
    w.add_argument("--exit-when-idle", action="store_true")
    w.add_argument("--poll", type=float, default=1.0)

    s = sub.add_parser("sweep", help="submit a KITTI parameter grid")
    s.add_argument("--url", default=url_default)
    s.add_argument("--engine", required=True, help="atlas_kitti binary the jobs must use (its hash goes in each job)")
    s.add_argument("--sequences", nargs="+", required=True)
    s.add_argument("--grid", nargs="*", default=[], help="key=v1,v2 ... (keys: voxel, max_range, deskew, correct, frames)")

    t = sub.add_parser("submit-test", help="submit N test jobs (no KITTI data needed)")
    t.add_argument("--url", default=url_default)
    t.add_argument("--jobs", type=int, default=12)
    t.add_argument("--seconds", type=float, default=2.0)
    t.add_argument("--fail-every", type=int, default=0, help="every k-th job fails its first attempt")

    st = sub.add_parser("status", help="print job counts and running jobs")
    st.add_argument("--url", default=url_default)

    r = sub.add_parser("report", help="write a Markdown report from a store directory")
    r.add_argument("--store", required=True)
    r.add_argument("--out", help="write here as well as printing")

    d = sub.add_parser("demo", help="run a sweep while crashing and pausing workers and restarting the coordinator")
    demo.add_arguments(d)

    a = ap.parse_args(argv)
    if a.cmd == "coordinator":
        store = Store(a.store, capacity=a.capacity, lease_seconds=a.lease, max_attempts=a.max_attempts)
        server = serve(store, a.host, a.port)
        print(f"listening on http://{a.host}:{server.server_address[1]}", flush=True)
        log(f"store {a.store}: {store.counts()}")
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            server.shutdown()
        return 0
    if a.cmd == "worker":
        runners = {"test": test_runner}
        if a.engine:
            runners["kitti-map-odometry"] = KittiRunner(a.engine, os.path.expanduser(a.odometry or ""), velodyne_map(a.velodyne))
        run_worker(a.url, a.name, runners, exit_when_idle=a.exit_when_idle, poll=a.poll)
        return 0
    if a.cmd == "sweep":
        submit_all(a.url, kitti_specs(file_hash(a.engine), a.sequences, parse_grid(a.grid)))
        return 0
    if a.cmd == "submit-test":
        specs = [{"kind": "test", "n": i, "seconds": a.seconds,
                  "fail_attempts": 1 if a.fail_every and i % a.fail_every == 0 else 0} for i in range(a.jobs)]
        submit_all(a.url, specs)
        return 0
    if a.cmd == "status":
        body = Client(a.url, patience=5).call("GET", "/status")[1]
        print(body["counts"])
        for j in body["jobs"]:
            if j["state"] == "leased":
                print(f"  {j['id']} on {j['worker']} (attempt {j['attempts']})")
        return 0
    if a.cmd == "report":
        text = build_report(Store(a.store))
        if a.out:
            with open(a.out, "w") as f:
                f.write(text)
        print(text)
        return 0
    if a.cmd == "demo":
        return demo.run(a)
    return 2


if __name__ == "__main__":
    sys.exit(main())
