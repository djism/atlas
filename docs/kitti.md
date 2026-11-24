# KITTI data (Phase 3)

Phase 3 measures Atlas on real driving data: the KITTI odometry benchmark
(Geiger, Lenz, Urtasun, "Are we ready for Autonomous Driving? The KITTI Vision
Benchmark Suite", CVPR 2012; raw data: Geiger et al., "Vision meets Robotics:
The KITTI Dataset", IJRR 2013). A car drives around Karlsruhe with a Velodyne
HDL-64E on the roof: one 360-degree sweep every 0.1 s, about 120,000 points
per sweep. Sequences 00-10 come with ground-truth poses (the answer key, from
GPS/IMU); 11-21 are held out for the official leaderboard.

## License

KITTI is published under **Creative Commons Attribution-NonCommercial-ShareAlike
3.0**, and the site states it is "made available for academic use only".
For Atlas this means: non-commercial use, attribution (the papers above), and
anything published that is derived from KITTI data (e.g. point clouds in a
future replay explorer) under the same license. The data itself never goes
into this repository.

## What is downloaded, and where

Downloads need a free account at
https://www.cvlibs.net/datasets/kitti/user_register.php. The full laser set is
one 80 GB zip; this machine had 34 GB free, so Atlas starts from the **raw
recordings**, one drive at a time, keeping only what it needs:

```
~/Desktop/Datasets/kitti/                      (outside the repo; /Datasets/ is also git-ignored)
├── raw/2011_09_30/2011_09_30_drive_0016_sync/
│   ├── velodyne_points/data/                  279 scans, 0000000000.bin ... 0000000278.bin
│   ├── velodyne_points/timestamps{,_start,_end}.txt
│   └── oxts/                                  GPS/IMU log
├── raw/2011_09_30/2011_09_30_drive_0027_sync/ sequence 07 (held out): 1106 scans, oxts
├── odometry/dataset/poses/00.txt ... 10.txt   ground truth (04.txt: 271 poses)
├── odometry/dataset/sequences/NN/             calib.txt, times.txt
└── devkit/                                    readme and official evaluation code
```

The devkit readme maps benchmark sequences to raw drives. **Sequence 04 is
`2011_09_30_drive_0016`, frames 000000-000270** (271 scans; the drive has 279,
so frames 271-278 are outside the sequence). **Sequence 07 is
`2011_09_30_drive_0027`, frames 000000-001100** (1101 scans of 1106), downloaded
2026-11-24 as the held-out sequence (2.0 GB of scans and GPS kept; the 4.4 GB
zip is not needed once extracted). The first download attempt was
`2011_09_26_drive_0027`, a different drive with the same number: it is not a
benchmark sequence and has no ground truth. Check the date in the name.

Kept: 541 MB (scans 561 MB uncompressed in the zip listing = 535 MiB on disk,
plus GPS, poses, calibration). Deleted after verifying byte counts: the zips
and the camera images (never extracted).

### Raw scans are not the benchmark's scans

The devkit says the benchmark's scans are **motion-compensated** ("compensating
for the egomotion"), "in contrast to the raw data". The laser spins for 0.1 s
while the car moves, so a raw scan is slightly smeared: at highway speed the
start and end of one sweep are taken over a meter apart. Consequences:

- Atlas's numbers on raw scans are not directly comparable to published
  KITTI leaderboard numbers. The baseline (KISS-ICP) will be run on the
  **same** raw scans, so that comparison is fair.
- The raw data has each sweep's start and end time, so removing the smear
  ("deskewing") can be implemented and measured later.
- The ground-truth poses still apply: they are per frame, at the camera
  trigger time.

## Frames

- **Velodyne frame** (scans): origin at the sensor, **x forward, y left,
  z up**, meters. Checked on all 279 scans: the median point height is -1.50
  to -1.79 m (KITTI's sensor is about 1.73 m above the road, and most points
  hit the road), and about half the points have x > 0.
- **Ground-truth poses** are of the **left camera** (x right, y down, z
  forward), each the camera frame at frame i expressed in the camera frame at
  frame 0. `calib.txt`'s `Tr` is the Velodyne frame in the camera frame
  (camera_point = Tr * velodyne_point). In Velodyne terms:

  ```
  velodyne_i_in_velodyne_0 = Tr^-1 * camera_i_in_camera_0 * Tr
  ```

  Read right to left: a point in Velodyne frame i goes to camera i (Tr), to
  camera 0 (the ground truth), and back to Velodyne 0 (Tr^-1).
  `velodynePoses()` does this; 3D poses are `Pose3D` (`geometry3d.hpp`, with
  `transformPoint3D`, `composePoses3D`, `inversePose3D`, `relativePose3D`;
  the `3D` suffix avoids ambiguous calls with the 2D functions).

## File format and reader

Each `.bin` is consecutive little-endian float32 records `(x, y, z,
reflectance)`, 16 bytes per point; the count is file size / 16. The first
record of scan 0, as bytes and as values:

| Bytes | float32 | Meaning |
| --- | --- | --- |
| `6de7 6d42` | 59.476 | x: 59.5 m forward |
| `df4f 6b41` | 14.707 | y: 14.7 m left |
| `e7fb 1140` | 2.281 | z: 2.3 m up |
| `0000 0000` | 0.0 | reflectance (0 to 1) |

`readKittiScan()` (`include/atlas/kitti.hpp`) rejects files that cannot be
opened, sizes that are not a multiple of 16, non-finite values, and
big-endian machines; an empty file is an empty scan. `summarizeScan()` gives
point count, ranges, median height, forward share, and max reflectance.
`birdsEyeView()` and `writeBmp()` (`include/atlas/image.hpp`) draw a top-down
picture: forward up, left left, sensor at the center.

```bash
c++ -std=c++17 -O2 -Iinclude src/*.cpp apps/kitti.cpp -o atlas_kitti
./atlas_kitti scan ~/Desktop/Datasets/kitti/raw/2011_09_30/2011_09_30_drive_0016_sync/velodyne_points/data/0000000000.bin results/scan0.bmp
```

Scan 0: 124,231 points; range 1.50 / 12.27 / 80.00 m (min / median / max);
median z -1.70 m; forward share 0.51. These match an independent Python
decode of the same file exactly. All 279 scans: 121,965 to 127,321 points,
max range 80.01 m, no read errors. The picture shows the road running
forward, the rings of the 64 beams on the ground, the blind circle around
the car, vegetation to the left and a side road to the right.

## Ground truth of sequence 04

`atlas_kitti truth ~/Desktop/Datasets/kitti/odometry/dataset 04`:

| Fact | Value |
| --- | --- |
| Frames | 271 poses, 271 times |
| Time between frames | 0.1011 to 0.1065 s |
| Path length | 393.6 m |
| Speed | 12.59 to 15.68 m/s, median 13.66 (49 km/h) |
| Motion per frame (median, Velodyne axes) | 1.423 m forward, 0.001 m left, 0.020 m up |
| End position (Velodyne frame 0) | 393.6 m forward, 0.4 m right, 5.2 m up |
| Rotation matrices | orthonormal within 1.5e-7 |

The car moves about 1.4 m during one 0.1 s sweep, so the raw-scan smear can
reach that size.

From the camera-axes end position (0.3 m left, 7.7 m up) I predicted
(393.6, +0.3, +7.7) in Velodyne axes and was wrong: the Velodyne is tilted
0.37 deg and turned 0.11 deg relative to the camera (`Tr`'s rotation), and
over 393.6 m that is 2.55 m of height and 0.73 m sideways. An independent
Python conversion gives the same (393.6, -0.45, 5.18). A calibration angle
too small to notice becomes meters over a drive.

## Do consecutive scans overlap under the ground truth?

`atlas_kitti overlap <odometry> 04 <velodyne_dir> <frame>` places every 60th
point of scan k+1 into scan k's frame with three guesses and measures the
distance to the nearest point of scan k (brute force):

| Scans | Points used | No motion | Camera axes (wrong frame) | Velodyne ground truth |
| --- | --- | --- | --- | --- |
| 0 -> 1 | all | 0.086 m (90 %: 0.785) | 1.295 m | 0.086 m (90 %: 0.348) |
| 0 -> 1 | >= 0.5 m above road | 0.566 m | 1.071 m | **0.085 m** |
| 100 -> 101 | >= 0.5 m above road | 0.135 m | 0.750 m | **0.054 m** |
| 200 -> 201 | >= 0.5 m above road | 0.185 m | 0.877 m | **0.064 m** |

(medians unless marked.) The wrong frame lifts the scan by the forward motion
(camera z is Velodyne "up"). The surprise is "no motion": over all points its
median equals the ground truth's. Most points are road, and a road point moved
1.3 m forward still lands next to road; the hedges and tree lines along this
straight road are nearly the same. It is the 2D hallway problem in 3D: most of
the scan says little about forward motion. Only raised points (trees, poles,
cars) separate the guesses clearly, and even then "no motion" is only 13 to
19 cm off at frames 100 and 200. Consequence for 3D ICP: forward motion on
straight roads is weakly observable; check that, as in 2D.

By direction from the car, the leftover ground-truth distance is 4 to 6 cm
right, 5 to 8 cm left, 8 to 10 cm ahead, 9 to 11 cm behind. Consecutive raw
scans are smeared almost the same way, so the smear mostly cancels between
them; it should matter more against a map built from many scans.

## A map from the ground truth

`atlas_kitti map <odometry> 04 <velodyne_dir> results/kitti_04_truth_map.bmp 5`
places every 5th scan (every 10th point) in Velodyne frame 0 with the ground
truth: 55 scans, 691,263 points, 0.65 s. The road is one straight band about
390 m long with sharp edges (curbs, a wall, hedges) and side roads that line
up; wrong poses or frames would blur or double those lines.

## First scan-to-scan 3D odometry

Pieces (all new, no external libraries):

- `voxelDownsample()` keeps the first point per cube. `VoxelGrid3D` files
  points by cube and searches the 27 cubes around a query, so with cubes at
  least as wide as the pairing distance it returns exactly the brute-force
  nearest point within that distance (tested against brute force).
- `alignIcp3D()`: point-to-point ICP solved by Gauss-Newton. A small rotation
  dtheta moves a point x by about dtheta x x, so each pair's residual
  `x + dt + dtheta x x - q` is linear in the six unknowns (dt, dtheta); the
  normal equations are a 6x6 system (Gaussian elimination), and the update is
  applied on the left: T <- (rotation(dtheta), dt) * T (`rotationFromVector`,
  Rodrigues' formula).
- `ScanOdometry3D`: each scan registered against the previous one from a
  constant-velocity guess, motions chained. `compareTrajectories()` scores it
  against the ground truth.

### Synthetic checks, and a mistake in the first test

A synthetic street (road, side walls, poles, a building face) moved by a known
motion. The first test moved the SAME random points into the new frame, and
ICP recovered even a pure hallway (road and two walls) exactly: every point's
twin was in the other scan, so the random pattern itself fixed the alignment.
A real laser hits different spots each sweep, so the tests now sample the
surfaces afresh. Then: with landmarks, within 0 to 8 mm and 0.5 mrad (three
seeds); hallway only, the forward error stays at -0.25 to -0.29 m of the
guess's -0.30 m while sideways and height are recovered, and ICP reports
"converged" or creeps to the iteration limit depending on the seed: the
status is no evidence of correctness.

Odometry tests: (a) the same points every frame and the reference unthinned,
so registrations are exact and any error is chaining (end within 1 mm, with a
different motion each frame: identical motions compose the same in any order,
which hid a reversed-order bug in the first version); (b) fresh samples, with
bounds above the spread measured over 20 seeds (worst step 0.8 to 3.0 cm, end
0.4 to 3.9 cm). The first bounds sat at the median and would have failed about
half the time. Deliberate breaks caught: flipped sign in the rotation part of
the Jacobian, reversed chaining order.

### Sequence 04, raw scans

`atlas_kitti odometry <odometry> 04 <velodyne_dir> [source_voxel]`, all 271
frames, previous scan thinned to 0.25 m, pairs within 1 m; one thread on an
Apple M4 (10 cores, 16 GB), Apple clang 17, -O2; time excludes reading files:

| Source thinning | Points per scan | Per-frame error median / 90 % / max | Forward bias per frame | End error (drift) | Heading drift | ms per scan |
| --- | --- | --- | --- | --- | --- | --- |
| 0.5 m | 12,491 | 3.7 / 6.7 / 10.8 cm | +2.55 cm | 18.5 m (4.7 %) | +4.4 deg | 123 |
| 1.0 m | 4,865 | 4.2 / 7.4 / 12.7 cm | +2.63 cm | 27.5 m (7.0 %) | +6.9 deg | 48 |
| 1.5 m | 2,699 | 5.2 / 7.9 / 10.7 cm | +2.63 cm | 36.0 m (9.2 %) | +9.2 deg | 25 |

Every frame reported "converged". At 1.0 m the end error is mostly sideways
(+26.4 m) and down (-6.6 m), from orientation drift: heading +6.9 deg, roll
+3.7 deg. It is systematic: random per-frame errors of this size (0.05 deg)
would sum to about 0.8 deg over 270 frames, not 6.9.

Two clues point at the raw-scan smear (the car moves about 1.4 m during each
sweep): the forward bias is the same with every thinning (so not a sampling
effect) and amounts to about 1.8 % of each motion; and the heading drift keeps
its sign and only shrinks with denser scans. The next experiment changes one
thing: deskew the scans with their timestamps, everything else equal.

Caveats: these settings were looked at on sequence 04 itself, so they are
tuned on the test data; another sequence must confirm them. Scan-to-scan
odometry is the simplest form; published methods (e.g. KISS-ICP) register
against a local map, use adaptive gates and robust weights, and deskew. They
report well under 1 % on KITTI, on the benchmark's compensated scans, so these
numbers are not comparable yet.

## Deskewing: a smaller effect than expected

KITTI raw timestamps: each sweep takes 0.104 s, and `timestamps.txt` is the
exact midpoint of `timestamps_start.txt` and `timestamps_end.txt`, when the
laser faces forward and the cameras fire (the ground-truth time). So a sweep
starts and ends facing backwards. `sweepFraction()` gives each point's time in
its sweep from its azimuth; `deskewScan()` re-expresses every point from the
sensor's position at mid-sweep, moving it by `scaleMotion(sweep_motion, s - 0.5)`
(constant velocity within the sweep; `rotationVector()` is the axis-angle log
used to take a fraction of a rotation).

The spin direction was measured, not assumed: the files are stored ring by
ring, so their order does not give time. Deskewing pairs with the TRUE motion
both ways (`atlas_kitti deskew-check`), raised points, median distance:

| Scan pairs (14 each) | No deskew | Clockwise | Counterclockwise |
| --- | --- | --- | --- |
| 1 frame apart | 0.055 m | 0.055 m | 0.057 m |
| 5 frames apart (~7 m) | 0.092 m | **0.084 m, better in 14/14** | 0.105 m, worse in 14/14 |

Clockwise (seen from above) it is. Neighboring scans barely change because
they are smeared almost identically.

Odometry with deskewing (`ScanOdometryOptions::deskew`: the new scan deskewed
with the predicted motion, the stored one re-deskewed with the motion just
estimated), everything else unchanged, and an ORACLE row deskewed with the
true motion (uses the answer key: an upper bound, not a result):

| Thinning | Raw | Deskewed (own prediction) | Oracle |
| --- | --- | --- | --- |
| 1.0 m | 6.99 % drift, heading +6.92 deg, +2.63 cm/frame | **6.49 %**, +6.22 deg, +2.27 cm | 6.36 %, +6.35 deg, +2.30 cm |
| 0.5 m | 4.69 %, +4.44 deg, +2.55 cm | **4.04 %**, +3.52 deg, +2.28 cm | 3.97 %, +3.66 deg, +2.19 cm |

Deskewing with the estimator's own prediction gets nearly all the oracle's
gain, and even perfect deskewing leaves about 90 % of the drift. **The smear
was a minor cause; my leading suspect was mostly wrong.**

## Where the bias comes from

`atlas_kitti icp-from-truth`: ICP started exactly at the true motion (scans
deskewed with the truth), 14 pairs. If it walks away, the matching itself
prefers a wrong answer. Split by which source points are used:

| Points | Ends forward of the truth by (median) | Pairs ending forward | Heading |
| --- | --- | --- | --- |
| all | +1.6 cm | 10 / 14 | +0.017 deg |
| road only (z <= -1.2 m) | -3.3 cm | 6 / 14 | -0.003 deg |
| raised only (z > -1.2 m) | **+2.3 cm** | **12 / 14** | **+0.019 deg** |

The odometry's forward bias (+2.3 cm per frame) and heading drift (6.3 deg /
270 frames = 0.023 deg per frame) match what the raised points (trees,
hedges, poles) do. The road pulls the other way, toward "no motion": its
rings are centered on the sensor, so they line up best when the two sensors
coincide. Both are known weaknesses of point-to-POINT matching, where a
surface's sampling pattern pulls the answer. Point-to-plane ICP only counts
distance across a surface, so sliding along a hedge or along the road's rings
costs nothing. That is the next experiment.

(14 values: the median here is the 7th; a scratch version that took the 8th
gave +2.5 / -1.5 / +1.7 cm. Same pair counts, same conclusion.)

## Scan-to-map odometry and the KISS-ICP baseline (Phase 3 result)

### The baseline, run on the same scans

KISS-ICP 1.3.0 (Vizzo et al., RA-L 2023, MIT license) was installed in a
throwaway Python 3.11 environment outside the repo and run by
`tools/kiss_icp_baseline.py` on the same raw scans, with its own KITTI
correction and its own per-point timestamps (so it deskews), default settings.
(Its KITTI odometry loader passes no timestamps, which suits the benchmark's
compensated scans but would skip deskewing raw ones.) Its source showed its
design: deskew with the last motion; thin twice (0.5 m for the map, 1.5 m to
register); point-to-POINT ICP against a local voxel map (20 points per 1 m
voxel, 100 m around the car) with Geman-McClure robust weights and an
adaptive pairing distance (3 sigma, sigma learned from how far estimates
move from their predictions). Its timestamp formula, -atan2(y, x), confirms
the clockwise sweep measured above.

### The metric, ported and checked

`kittiMetric()` ports the devkit's `calcSequenceErrors`: segments of 100 to
800 m (by ground-truth path length, "strictly longer than"), starting every
10th frame; translation error as % of length, rotation error per length;
scored in the camera frame (`cameraPoses()` = Tr * V * Tr^-1). The devkit's
own functions, copied verbatim into a harness and fed KISS-ICP's poses
converted to the camera frame in Python, give 43 segments, 0.4138 % and
0.1902 deg/100 m; the port gives 43, 0.41 %, 0.19. Sequence 04 is 394 m, so
only the 100, 200 and 300 m segments exist.

### KITTI's scan correction

KITTI's Velodyne has a known elevation-angle error of about 0.205 deg;
KISS-ICP corrects every scan. `correctKittiScan()` rotates each point 0.205
deg upward about p x z (range unchanged; road points rise about 3 cm, points
at 80 m move 29 cm) and matches KISS-ICP's own function to 1e-14 m on a real
scan. Alone it did not remove the scan-to-scan bias.

### `MapOdometry3D`: the same design, implemented in Atlas

`LocalMap3D` (capped voxels, far voxels removed), `alignToMap()` (robust
point-to-point Gauss-Newton), `MapOdometry3D` (deskew, two thinnings,
adaptive threshold, map update). Written from the published design, not
copied code.

**A bug worth remembering.** The first version diverged at frame 44 of
sequence 04 (NaN poses). Per-frame rotation grew 1.5x per frame on a straight
road. Cause: `relativePose3D` inverts a rotation by transposing it, exact only
for perfect rotations, and this odometry computes each motion back from two
poses and feeds it into the next guess. A rounding-sized flaw grew about 2.5x
per frame (orthonormality error 0.008 at frame 36, 3.8 at frame 42: 2.5^36 *
1e-16 ~ 0.01). Scan-to-scan odometry never had the loop (its motions came
straight from ICP). Deskewing only amplified it; a synthetic turning-sensor
test showed the deskew rotation itself is right (1 cm residual vs 1.2 m
smear; the opposite sign gives 1.6 m). Fix: `orthonormalized()` (Gram-Schmidt)
on the pose and motion every frame. Libraries such as Sophus store rotations
as unit quaternions and renormalize, which is why KISS-ICP does not see this.

### Results, sequence 04 (271 frames, 394 m, raw scans)

`atlas_kitti odometry <odometry> 04 <velodyne_dir>` and `atlas_kitti score`;
Apple M4, Apple clang 17 -O2:

| Method | KITTI t_err | KITTI r_err | End error | Per-frame error (median) | Time per scan |
| --- | --- | --- | --- | --- | --- |
| Atlas scan-to-scan, raw | 3.89 % | 2.02 deg/100 m | 27.5 m | 4.2 cm | 50 ms, 1 thread |
| Atlas scan-to-scan, deskewed | 3.70 % | 1.92 deg/100 m | 25.5 m | 4.1 cm | 59 ms, 1 thread |
| **Atlas scan-to-map** | **0.48 %** | **0.12 deg/100 m** | 1.49 m | **1.7 cm** | **43 ms, 1 thread** |
| KISS-ICP 1.3.0, same scans | **0.41 %** | 0.19 deg/100 m | **0.48 m** | 2.5 cm | 71 ms, 1 thread; 19 ms, all cores |

(KISS-ICP times include Python call overhead. Its multi-threaded runs differ
by up to 1e-12 m between runs; scores identical.)

What made the difference, by removing one piece at a time from Atlas's
scan-to-map pipeline: without deskewing 1.07 % end drift, without the KITTI
correction 2.05 %, with both 0.38 %. The forward bias went from +2.6 cm per
frame (scan-to-scan) to -0.2 cm: matching against a dense map of many scans
removes the sampling-pattern pull diagnosed above.

Voxel size is not a fragile choice: 0.75 m 0.48 % (81 ms), 1.0 m 0.48 %
(47 ms), 1.5 m 0.52 % (22 ms).

### Profile and memory

Streaming (read a scan, process it): 42.6 ms median, 71 ms 90th percentile,
155 ms worst per scan; 16.5 ICP iterations per scan; local map up to 300,574
points; peak resident memory 79 MB (KISS-ICP via Python: 148 MB). The
`sample` profiler: 62.5 % of the time is nearest-neighbor lookups in the
local map (the expected hot spot; threads would be the next step), and 11 %
was reading files: `readKittiScan` read through `istreambuf_iterator`, one
char at a time. One bulk read: 9.5 -> 1.1 ms per scan.

### Point-to-plane: not built in this phase

It was planned to fix the sampling bias; the local map fixed it, and Atlas
now matches a published point-to-point baseline. It stays an optional
experiment.

### Results, sequence 07: held out (1101 frames, 694 m, raw scans)

Sequence 07 is urban: four sharp turns, about 747 deg of turning in total,
a 77-frame stop, and a route that ends 9.5 m from its start. It was
downloaded only after every parameter above was fixed, and nothing was
changed for it: same commands, same defaults, for Atlas and for KISS-ICP.

| Method | KITTI t_err | KITTI r_err | End error | Per-frame error (median) | Time per scan | Peak memory |
| --- | --- | --- | --- | --- | --- | --- |
| Atlas scan-to-scan, raw | 6.32 % | 3.71 deg/100 m | 34.4 m | 3.8 cm | 22 ms, 1 thread | |
| Atlas scan-to-scan, deskewed | 5.46 % | 3.18 deg/100 m | 28.5 m | 3.8 cm | 32 ms, 1 thread | |
| **Atlas scan-to-map** | **0.43 %** | **0.25 deg/100 m** | 0.80 m | **1.9 cm** | 28 ms median, 52 ms 90th, 216 ms worst; 1 thread | 106 MB |
| KISS-ICP 1.3.0, same scans | 0.75 % | 0.37 deg/100 m | **0.72 m** | 2.5 cm | 47 ms, 1 thread; 15 ms, all cores | 195 MB |

317 KITTI segments. Times: Apple M4; Atlas from a streaming run (read a
scan, process it, discard it), which also gives the memory; the `odometry`
command keeps all scans for three methods and peaks at 1.2 GB, which is
not the method's cost. KISS-ICP times include Python call overhead; its
1-thread and all-core poses were identical. The same streaming run on
sequence 04 reproduced 43 ms and 78 MB.

Reading it:

- **The held-out result holds up.** Atlas did not get worse on a sequence it
  was never looked at on (0.48 % on 04, 0.43 % on 07). Scan-to-scan got
  worse (3.9 -> 6.3 %): more turning exposes its rotation drift.
- **Atlas vs KISS-ICP: one win each.** KISS-ICP was ahead on 04 (0.41 vs
  0.48 %), Atlas on 07 (0.43 vs 0.75 %). Per frame, Atlas's step error is
  lower in every 100-frame block of 07, with KISS-ICP's worst near frames
  200-300 (up to 20 cm steps at 7.5 m/s). Atlas follows KISS-ICP's
  design but not its code, so they differ in small details; I have not
  found which one causes the gap, so this is a measurement, not an
  explanation. Two sequences cannot rank the methods.
- **End error misleads on a loop.** The route returns near its start, so
  errors made on the way out partly cancel on the way back: scan-to-map's
  0.80 m end error is 0.12 % of the path, while the segment metric, which
  measures every 100-800 m stretch, says 0.43 %. The segment metric is the
  number to quote.
- Paths: `results/path_07.svg` and `results/path_04.svg`, from
  `atlas_kitti plot` (truth in gray; Atlas scan-to-map and KISS-ICP lie on
  it at this scale; scan-to-scan visibly bends away at every turn).

### Reproduce

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel
K=~/Desktop/Datasets/kitti; O=$K/odometry/dataset
V07=$K/raw/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/data
./build/atlas_kitti odometry $O 07 $V07          # three Atlas methods, saves results/atlas_07_*.txt
<venv>/bin/python -I tools/kiss_icp_baseline.py $V07 0 1100 results/kiss_07.txt 1
./build/atlas_kitti score $O 07 results/kiss_07.txt "KISS-ICP 1.3.0"
./build/atlas_kitti plot $O 07 results/path_07.svg \
    results/atlas_07_map.txt "Atlas scan-to-map" results/kiss_07.txt "KISS-ICP 1.3.0"
```

Sequence 04: the same with `04`, drive `2011_09_30_drive_0016`, frames 0-270.

### What this does and does not show

- Two sequences (394 m country road, 694 m urban loop), 1.1 km in all. The
  design and defaults come from KISS-ICP, whose authors developed it on
  KITTI; the parameters were looked at on 04 only, and 07 was held out.
- Odometry only: no loop closure, so error still grows with distance.
- Not comparable to the official leaderboard (raw scans, training sequence).
- Synthetic test: with no velocity yet, frame 1 starts from a zero guess and
  in a sparse synthetic street sometimes stays near zero (4 of 8 seeds lose
  the first 1.2 m). On sequence 04 frame 1 was fine.

## Verification and limits

Unit tests use hand-made files (CI has no KITTI data): two records copied
byte for byte from real scan 0, exact values and order, empty, truncated,
NaN, and missing files, path formatting, summary values, image orientation
(forward-left lands top-left), and BMP row order and padding. Deliberate
breaks each fail a test: swapping image axes, dropping the size check, and
writing BMP rows top-down (the first version of that test used a point in the
center row, where both orders agree; it now uses an off-center point).

Pose, calibration and time readers are tested with hand-made files: a `Tr`
with the real axis swap and a camera motion of 1.5 m forward, 0.2 m down must
become 1.5 m along Velodyne +x and 0.2 m along -z; malformed lines (11 or 13
numbers, a stretched matrix, no `Tr`, a non-number time) are rejected.
Deliberate breaks each fail tests: using camera poses directly, reversing the
conversion, and dropping the minus sign in the inverse.

Deskew tests: sweep fractions (forward 0.5, left 0.25 and right 0.75 when
clockwise), a 1.4 m forward sweep moving a left point back 0.35 m and a right
point forward 0.35 m with the point ahead unchanged, and rotation-vector round
trips; breaks caught: flipped direction, deskewing from the sweep start
instead of the middle, and a rotation log missing its angle/sin factor.

New tests: the rounding feedback loop itself (grows without
`orthonormalized()`, stays at 1e-12 with it), the KITTI correction (0.205 deg,
range kept), the metric (perfect estimate 0 %, a 1 % overshoot 1 %, segment
count as the devkit's "strictly longer"), camera/Velodyne round trip, the
local map (cap, removal), and scan-to-map odometry on a synthetic street
(from frame 1: within 35 cm over 46 m; bounds above the 8-seed spread).
Deliberate breaks caught: no re-orthonormalization, `>=` instead of the
devkit's `>`, no robust weights.

Plot tests: forward is straight up, left is left, equal scale on both axes,
names escaped, empty or NaN input rejected; mirroring the plot fails two of
them.

Not yet: more sequences (highway, longer drives), loop closure, multi-threading, point-to-plane,
a weak-direction check in 3D. Reading a scan loads the whole file into memory (about 2 MB),
which is fine at this size.
