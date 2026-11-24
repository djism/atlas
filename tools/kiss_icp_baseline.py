"""Baseline for Atlas: run KISS-ICP on raw KITTI scans and save its poses.

KISS-ICP (Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP -- Simple,
Accurate, and Robust Registration If Done the Right Way", RA-L 2023; MIT
license) is used as an external package, not vendored. Setup, outside the repo:

    python3.11 -m venv /tmp/kiss-venv            # KISS-ICP 1.3.0 wheels: Python 3.8-3.13
    /tmp/kiss-venv/bin/pip install "kiss-icp==1.3.0"

Run (frames are raw-drive indices; sequence 04 is frames 0-270 of
2011_09_30_drive_0016):

    /tmp/kiss-venv/bin/python tools/kiss_icp_baseline.py \
        <raw_drive>/velodyne_points/data 0 270 results/kiss_04.txt [threads]

Then score it exactly like Atlas:

    atlas_kitti score <odometry_dir> 04 results/kiss_04.txt "KISS-ICP 1.3.0"

Inputs match Atlas's: the same raw scans, KISS-ICP's own KITTI correction
(_correct_kitti_scan) and its own per-point timestamps (clockwise sweep,
0.5 * (1 - azimuth / pi)), so it deskews; default configuration (voxel
1.0 m = max range 100 m / 100). Its odometry-format loader would pass no
timestamps and skip deskewing, which suits the benchmark's compensated scans
but not raw ones.
Output: one line per frame, 12 numbers (row-major [R | t]), Velodyne frame,
first scan at the identity.
"""
import sys
import time

import numpy as np
from kiss_icp.config.parser import load_config
from kiss_icp.kiss_icp import KissICP
from kiss_icp.pybind import kiss_icp_pybind as kp


def main():
    scan_dir, first, last, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
    threads = int(sys.argv[5]) if len(sys.argv) > 5 else 0  # 0 = automatic
    config = load_config(None)
    config.registration.max_num_threads = threads
    odometry = KissICP(config)
    poses, ms = [], []
    for i in range(first, last + 1):
        points = np.fromfile(f"{scan_dir}/{i:010d}.bin", dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
        points = np.asarray(kp._correct_kitti_scan(kp._Vector3dVector(points)))
        timestamps = 0.5 * (-np.arctan2(points[:, 1], points[:, 0]) / np.pi + 1.0)
        start = time.perf_counter()
        odometry.register_frame(points, timestamps)
        ms.append(1000 * (time.perf_counter() - start))
        poses.append(odometry.last_pose.copy())
    np.savetxt(out, np.array([p[:3, :4].reshape(-1) for p in poses]), fmt="%.9e")
    print(f"KISS-ICP: {len(poses)} frames, voxel {config.mapping.voxel_size} m, threads {threads or 'auto'}: "
          f"median {np.median(ms):.1f} ms per scan (includes Python call overhead)")


if __name__ == "__main__":
    main()
