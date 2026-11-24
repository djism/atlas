"""Build a Markdown report from the durable results alone (no coordinator needed)."""
from __future__ import annotations

from collections import Counter

from .store import Store

DEFAULT = {"voxel": 1.0, "deskew": 1, "correct": 1}


def _params(p: dict) -> str:
    return " ".join(f"{k}={v}" for k, v in sorted(p.items()))


def build_report(store: Store, title: str = "Atlas evaluation sweep") -> str:
    jobs = store.jobs()
    events = store.events()
    results = store.results()
    counts = store.counts()
    kinds = Counter(e["kind"] for e in events)
    lines = [f"# {title}", ""]
    lines += ["Generated from the store's result files (each checked against its SHA-256), not from logs.", ""]

    kitti = [r for r in results if r["spec"].get("kind") == "kitti-map-odometry"]
    for seq in sorted({r["spec"]["sequence"] for r in kitti}):
        rows = sorted((r for r in kitti if r["spec"]["sequence"] == seq),
                      key=lambda r: (r["result"]["t_err_percent"], _params(r["spec"]["params"])))
        res0 = rows[0]["result"]
        lines += [f"## Sequence {seq} ({res0['frames']} scans, {res0['path_length_m']:.0f} m)", "",
                  "| Parameters | KITTI t_err | r_err (deg/100 m) | End error | ms per scan (median) | Attempt | Result |",
                  "| --- | --- | --- | --- | --- | --- | --- |"]
        for r in rows:
            p, x = r["spec"]["params"], r["result"]
            name = _params(p) + (" **(default)**" if all(p.get(k) == v for k, v in DEFAULT.items()) else "")
            digest = next(j["result_hash"] for j in jobs if j["id"] == r["job_id"])
            lines.append(f"| {name} | {x['t_err_percent']:.2f} % | {x['r_err_deg_per_100m']:.2f} | "
                         f"{x['end_error_m']:.2f} m | {x['ms_median']:.0f} | {x['attempt']} | `{digest[:12]}` |")
        lines.append("")
    if kitti:
        lines += ["Rows sorted by t_err. The default was chosen on sequence 04 before this sweep; the sweep shows",
                  "sensitivity and is not used to pick new parameters (that would tune on the held-out sequence).",
                  "Times come from workers running side by side and are not comparable to single-run timings.", ""]

    other = [r for r in results if r["spec"].get("kind") != "kitti-map-odometry"]
    if other:
        lines += [f"Other results: {len(other)} ({', '.join(sorted({r['spec'].get('kind', '?') for r in other}))}).", ""]

    lines += ["## What happened while running it", "",
              "| | Count |", "| --- | --- |",
              f"| Jobs | {len(jobs)} ({counts['done']} done, {counts['failed']} failed, "
              f"{counts['queued'] + counts['leased']} unfinished) |",
              f"| Attempts started (leases granted) | {kinds['leased']} |",
              f"| Leases expired (worker crashed, paused or cut off) | {kinds['lease_expired']} |",
              f"| Attempts that reported an error | {kinds['attempt_failed']} |",
              f"| Jobs requeued with backoff | {kinds['requeued']} |",
              f"| Late heartbeats rejected (that worker stopped its run) | {kinds['stale_heartbeat_rejected']} |",
              f"| Late results rejected (duplicate execution caught) | {kinds['stale_result_rejected']} |",
              f"| Jobs given up after the attempt limit | {kinds['gave_up']} |",
              f"| Results accepted | {kinds['completed']} |", ""]
    notable = [e for e in events if e["kind"] in ("lease_expired", "stale_heartbeat_rejected", "stale_result_rejected",
                                                    "gave_up", "attempt_failed")]
    if notable:
        t0 = events[0]["at"]
        lines += ["Notable events (seconds from the first submission):", ""]
        for e in notable:
            lines.append(f"- {e['at'] - t0:7.1f} s  {e['kind']}  job {e['job_id']}"
                         + (f"  worker {e['worker']}" if e["worker"] else "") + (f"  ({e['detail']})" if e["detail"] else ""))
        lines.append("")
    accepted = Counter(e["job_id"] for e in events if e["kind"] == "completed")
    doubles = [j for j, n in accepted.items() if n > 1]
    lines.append(f"Check: every done job has exactly one accepted result: {'yes' if not doubles else 'NO: ' + str(doubles)}.")
    return "\n".join(lines) + "\n"
