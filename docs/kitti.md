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
├── odometry/dataset/poses/00.txt ... 10.txt   ground truth (04.txt: 271 poses)
├── odometry/dataset/sequences/NN/             calib.txt, times.txt
└── devkit/                                    readme and official evaluation code
```

The devkit readme maps benchmark sequences to raw drives. **Sequence 04 is
`2011_09_30_drive_0016`, frames 000000-000270** (271 scans; the drive has 279,
so frames 271-278 are outside the sequence).

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

Not yet: point-to-plane, a local map, robust weights, a weak-direction check,
the official KITTI drift metric, a second sequence. Reading a scan loads the whole file into memory (about 2 MB),
which is fine at this size.
