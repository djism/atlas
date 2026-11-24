"""Per-frame data for the web replay explorer: web/public/replay/<seq>/frames.json.

The point data (map.bin, scans.bin) comes from `atlas_kitti export-replay`;
this script adds everything per frame, from recorded files only:

- Atlas's estimated pose (3x4, Velodyne frame of scan 0) and diagnostics from
  `atlas_kitti run ... diag=<csv>`: what the engine itself saw;
- the answer key (KITTI poses converted to the Velodyne frame), KISS-ICP's
  estimate, and the step error / drift of each, used only for error rows.

Standard library only. Full pipeline for one sequence (docs/replay.md):

    atlas_kitti run <odometry> 07 <scans> results/replay/atlas_07.txt diag=results/replay/atlas_07_diag.csv
    atlas_kitti export-replay <scans> results/replay/atlas_07.txt web/public/replay/07 scan_points=640 map_voxel=0.5 \\
        > results/replay/meta_07.json
    python3 -I tools/export_replay.py <odometry> 07 results/replay/atlas_07.txt results/kiss_07.txt \\
        results/replay/atlas_07_diag.csv results/replay/meta_07.json web/public/replay/07/frames.json
"""
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # -I leaves the script's folder off the path
from export_web_results import inv, mul, read_poses, read_tr  # noqa: E402


def position(m):
    return [m[0][3], m[1][3], m[2][3]]


def step_errors(truth, estimate):
    """Distance between estimated and true motion from frame i-1 to i, meters (0 for frame 0)."""
    out = [0.0]
    for i in range(1, len(estimate)):
        e = mul(inv(mul(inv(truth[i - 1]), truth[i])), mul(inv(estimate[i - 1]), estimate[i]))
        out.append(math.sqrt(e[0][3] ** 2 + e[1][3] ** 2 + e[2][3] ** 2))
    return out


def main():
    odometry, seq, atlas_path, kiss_path, diag_path, meta_path, out = sys.argv[1:8]
    tr = read_tr(f"{odometry}/sequences/{seq}/calib.txt")
    truth = [mul(mul(inv(tr), c), tr) for c in read_poses(f"{odometry}/poses/{seq}.txt")]
    atlas, kiss = read_poses(atlas_path), read_poses(kiss_path)
    with open(f"{odometry}/sequences/{seq}/times.txt") as f:
        times = [float(line) for line in f if line.strip()]
    with open(diag_path) as f:
        diag = list(csv.DictReader(f))
    with open(meta_path) as f:
        meta = json.load(f)
    n = len(truth)
    if not (len(atlas) == len(kiss) == len(diag) == len(times) == meta["frames"] == n):
        raise ValueError(f"length mismatch: truth {n}, atlas {len(atlas)}, kiss {len(kiss)}, diag {len(diag)}, "
                         f"times {len(times)}, point data {meta['frames']}")

    speed, turn = [0.0], [0.0]
    for i in range(1, n):
        m = mul(inv(truth[i - 1]), truth[i])
        speed.append(math.sqrt(sum(v * v for v in position(m))) / (times[i] - times[i - 1]))
        turn.append(math.degrees(math.atan2(m[1][0], m[0][0])))  # + = left (counterclockwise seen from above)
    r = lambda v, k: round(v, k)  # noqa: E731
    data = {
        "sequence": seq,
        "frames": n,
        "scanPoints": meta["scan_points"],
        "mapPoints": meta["map_points"],
        "mapVoxel": meta["map_voxel"],
        "mapRange": meta["map_range"],
        "time": [r(t, 2) for t in times],
        # Atlas's estimate: row-major 3x4 [R | t] per frame (scans are drawn with it).
        "atlasPose": [[r(v, 5) if j % 4 != 3 else r(v, 3) for j, v in enumerate(sum((row for row in p[:3]), []))]
                      for p in atlas],
        "truthPos": [[r(v, 3) for v in position(p)] for p in truth],
        "kissPos": [[r(v, 3) for v in position(p)] for p in kiss],
        "atlasStepCm": [r(100 * v, 2) for v in step_errors(truth, atlas)],
        "kissStepCm": [r(100 * v, 2) for v in step_errors(truth, kiss)],
        "atlasDriftM": [r(math.dist(position(a), position(t)), 3) for a, t in zip(atlas, truth)],
        "speedMps": [r(v, 2) for v in speed],
        "turnDeg": [r(v, 3) for v in turn],
        "iterations": [int(d["iterations"]) for d in diag],
        "pairs": [int(d["pairs"]) for d in diag],
        "sourcePoints": [int(d["source_points"]) for d in diag],
        "sigmaM": [r(float(d["sigma_m"]), 4) for d in diag],
        "engineMs": [r(float(d["ms"]), 1) for d in diag],
        "engineMapPoints": [int(d["map_points"]) for d in diag],
    }
    with open(out, "w") as f:
        json.dump(data, f, separators=(",", ":"))
    print(f"{out}: {n} frames, {os.path.getsize(out) / 1e3:.0f} kB; worst Atlas step "
          f"{max(data['atlasStepCm']):.1f} cm, worst drift {max(data['atlasDriftM']):.2f} m")


if __name__ == "__main__":
    main()
