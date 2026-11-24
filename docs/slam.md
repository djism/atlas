# Toward SLAM: loop closure (Phase 6)

Odometry compares each scan with the recent past, so small errors add up and
are never corrected: on sequence 07, Atlas ends 0.8 m from where it truly is,
and the drift peaks at 2.1 m mid-drive. Sequence 07 is a loop. When the car
returns to a place it has seen, the scans there should match the old ones: a
second, independent measurement of where the car is relative to that earlier
moment. Loop closure finds such a match, checks it, and then adjusts the
whole trajectory so it agrees with the odometry and the loop at once.

Plan, one step at a time:

1. **Pose-graph optimizer** (done, below).
2. **Keyframes and loop candidates** (done, below).
3. **Geometric verification** (done, below).
4. **Optimize and measure** (done, below).

**Result on held-out sequence 07** (nothing tuned on it): position error
1.13 -> 0.81 m RMS, worst 2.10 -> 1.40 m, end error 0.80 -> 0.11 m; the KITTI
segment metric is unchanged (0.43 -> 0.42 %), as expected for a correction of
the overall shape. Sequence 04 has no loop and is returned unchanged.

## Step 1: the pose graph

Nodes are poses; edges are measured relative poses (one odometry step, or one
loop closure). Each edge says "pose j, seen from pose i, is here", with a
weight for how much to trust it (1 / sigma^2, separately for translation and
rotation). Optimizing moves all poses at once to disagree least with every
edge, in the weighted least-squares sense, like springs pulling the
trajectory into the shape that stretches them least.

**Residual of an edge.** E = measured^-1 * (pose_i^-1 * pose_j): the motion
left over after "undoing" the measurement. The residual is E's translation
and rotation vector. If pose_j = pose_i * measured * D, the residual is
exactly D (tested).

**Solver.** Levenberg-Marquardt: build the normal equations from every
edge's 6x6 Jacobians (central differences of the residual for a small motion
of each end, applied in the world frame), damp them, solve by Cholesky, keep
the step only if the total cost falls. Pose 0 is held fixed: it anchors the
map (otherwise the whole graph could slide freely and the equations would be
singular). After each step every rotation is re-orthonormalized.

Dense normal equations: memory (6n)^2 and time (6n)^3 for n poses, so the
graph will hold keyframes (a few hundred), not every scan. No new dependency:
`solveCholesky()` is 30 lines.

`include/atlas/posegraph3d.hpp`, `src/posegraph3d.cpp`.

### Tests

- The residual equals the extra motion D (to 1e-12); Cholesky solves a known
  system (exact answer -1/3, 5/3, 2/3; my first expected values were wrong
  and the test caught it) and refuses an indefinite matrix.
- Consistent measurements: the poses do not move (cost below 1e-12).
- A synthetic loop: 40 poses on a 20 m circle, odometry with a 0.4 deg
  heading bias per step plus noise. Drift 3.47 m RMS; with one trusted loop
  edge back to the start, 0.034 m after 5 iterations. The optimized loop
  agrees with its measurement to under 5 cm, pose 0 stays put, all poses stay
  proper rotations.
- A wrong loop (claims the end is 8 m away from where it is) leaves the map
  more than 3x worse than the right one: why step 3 (verification) matters.
- Weights mean trust: two disagreeing measurements of one motion (1.0 m and
  0.10 rad with weight 3; 2.0 m and 0.02 rad with weight 1) give the weighted
  averages, 1.25 m and 0.08 rad.
- Invalid input (no poses, index out of range, self edge, negative weight)
  throws.

Deliberate breaks, each caught: the residual without inverting the
measurement, pose 0 not held fixed, the update with the wrong sign, and the
rotation weights ignored (the last one was missed until the weighted-average
test was added).

### What step 1 does not show

Synthetic data only. The loop edge here is handed over exactly; on real data
it must be found and verified (steps 2 and 3), and that is where real systems
fail.

## Step 2: keyframes and loop candidates

**Keyframes** (`selectKeyframes`): frame 0, then a new one after 5 m of
travel or 15 deg of turning. Standing still adds none. Sequence 07: 143
keyframes from 1,101 scans; sequence 04: 68.

**Candidates** (`findLoopCandidates`): for each keyframe, the nearest
earlier keyframe within 10 m in Atlas's ESTIMATED map, at least 50 m back
along the path (so a return, not the stretch just driven). Searching by
estimated position only works while drift stays well below the radius; here
drift peaks at 2.1 m. A real system adds place recognition (comparing what
places look like) for larger drift; Atlas does not have it.

`atlas_kitti loops <odometry> <seq> <estimate.txt> [keyframe_m] [radius_m]
[min_gap_m]` prints each candidate with its true distance from the answer
key (grading only; the search never sees it). Takes 30 ms.

| Sequence, radius | Candidates | Same place (truly within 10 m) |
| --- | --- | --- |
| 07, 10 m | 4: frames 1029, 1050, 1065, 1079 return to frames 0, 0, 15, 27 (680 m later) | 4 |
| 07, 5 m | 3 | 3 |
| 07, 20 m | 6 | 4 |
| 07, 40 m | 16 (including streets 39 m apart, e.g. 263 -> 366) | 4 |
| 04, 10 m | 0 (straight road, no return) | 0 |

Best pair: frame 15 and frame 1065, 0.75 m apart in the estimate, 0.30 m in
truth. Estimated and true distances agree within about 0.4 m on every
candidate, so proximity search is reliable on this drive.

A first version graded "real return" as "truly within the search radius",
which called all 16 candidates at 40 m real: circular. The label is now a
fixed 10 m. Whether a candidate is a good loop is a different question:
whether its scans can be aligned to the right relative pose. That is step 3,
graded against the answer key's relative pose.

### Tests

Keyframes every 5 m on a straight road, none while standing still, every 15
deg when turning in place; path length; no candidates on a straight road;
on a circle, candidates only where it closes; an out-and-back street pairs
the way back with the way out (3 m apart); bad input throws. Dropping the
path-gap rule fails all three candidate tests (every neighbour becomes a
"loop").

## Step 3: geometric verification

One wrong loop bends the whole map (step 1's test: over 3x worse), so every
candidate must earn its place. `verifyLoop()`:

1. A local map around the earlier keyframe (`buildSubmap`): the 21 scans
   around it (KITTI-corrected, deskewed, placed with Atlas's own poses),
   thinned to one point per 0.25 m.
2. The later scan (thinned to 0.5 m) aligned to it by point-to-point ICP,
   starting from the odometry's guess: a coarse pass (2 m gate, for
   metre-level drift), then a fine pass (0.5 m gate).
3. The fit judged on **structure only** (points more than about 0.4 m above
   the road: walls, poles, buildings): at least 60 % within 0.3 m of the map,
   RMS at most 0.15 m, converged, and no jump of more than 5 m from the guess.

**Why structure only.** The first version judged all points and got both
decisions wrong on the synthetic test: a true revisit rejected ("fit too
loose") and a different street 63 % "fitting", because flat road matches flat
road anywhere. On structure: 99.8 % at the same place, 0 % (later 4 %) on a
different street. Counting the road again fails two tests.

**Accuracy.** Thinning the scan to 1 m left 8 cm of error on the synthetic
test; 0.5 m gives 3.6 cm.

### Real data: sequence 07

`atlas_kitti verify-loops <odometry> 07 <scans> results/atlas_07_map.txt`
(6 s; saves accepted loops to `results/loops_07.txt`). Each candidate also
gets a **decoy**: its later scan paired with a place 300 frames away, with the
same guess, as if the search had been fooled.

| Loop (frames) | Verdict | Structure fit | Odometry guess vs answer key | Verified vs answer key |
| --- | --- | --- | --- | --- |
| 0 -> 1029 | accept | 77 % | 0.80 m, 0.40 deg | 0.36 m, 0.21 deg |
| 0 -> 1050 | accept | 86 % | 0.72 m, 0.42 deg | 0.32 m, 0.25 deg |
| 15 -> 1065 | accept | 95 % | 0.79 m, 0.22 deg | 0.21 m, 0.21 deg |
| 27 -> 1079 | accept | 88 % | 0.80 m, 0.24 deg | 0.04 m, 0.14 deg |
| 4 decoys | all reject | 9-14 % | 153-158 m | (ICP did not converge) |

**Where the remaining 0.2-0.36 m comes from.** The errors are almost all
sideways and shrink from frame 1029 to 1079. Two tests:

- The four loops agree with **each other** within 2-9 cm, through short
  odometry links (loop 1, odometry between the earlier frames, loop 2,
  compared with odometry between the later frames). Verification is not
  making random per-loop errors.
- Over frames 1029-1079 (15.5 m), Atlas and **KISS-ICP both** differ from
  the answer key by 0.43-0.44 m, in the same sideways direction, while
  differing from each other by about 9 cm. Two independent LiDAR odometries
  agreeing against the GPS/IMU answer key points to the answer key: it has a
  sideways error of a few decimetres over this short stretch (2.8 % over
  15 m, far above either method's drift).

So the accepted loops are better than the table suggests, and the answer key
limits how precisely they can be graded here. (A structure-only fine ICP pass,
tried to remove a suspected road-ring bias, changed nothing and was reverted.)

### Tests

Synthetic city block: a true revisit with a drifted guess (0.9 m, 0.6 m,
3 deg off) is accepted and right to 3.6 cm, 0.03 deg; a different street is
rejected with 4 % structure fit; too few points is rejected; the local map
puts every scan's points in one frame.

Thresholds were set by judgement and checked on sequence 07, the only data
with loops here, so they are not validated on held-out data.

## Step 4: optimize and measure

`closeLoops()`: the keyframes are the nodes; each pair of consecutive
keyframes gets an odometry edge (their relative pose in the estimate), each
verified loop an edge; keyframe 0 is fixed. Every scan then moves rigidly
with the keyframe at or before it.

**Weights from measured noise, not tuned on sequence 07.** Odometry: sigma
grows with the distance between keyframes at the drift rate measured on
sequence 04 (0.48 %, 0.12 deg / 100 m), with floors of 2 cm and 0.002 rad.
Loops: 10 cm and 0.005 rad, from how well the four loops agree with each other
(2-9 cm).

`atlas_kitti close-loops <odometry> 07 results/atlas_07_map.txt
results/loops_07.txt` (pose graph: 143 keyframes, 10 iterations, 1.1 s):

| Sequence 07 | Odometry | + loop closure |
| --- | --- | --- |
| Position error, RMS over every frame | 1.13 m | **0.81 m** (-28 %) |
| Worst position error (frame 749) | 2.10 m | **1.40 m** (-33 %) |
| End error | 0.80 m | **0.11 m** (-86 %) |
| KITTI t_err / r_err | 0.43 % / 0.25 | 0.42 % / 0.24 |

Position error is against the answer key in scan 0's frame, where both
start; no alignment. `results/path_07_slam.svg` plots both.

**Reading it.** The loop ties the end of the drive to its start, and the
graph spreads that correction along the whole route. The end error drops
most; the mid-drive bulge (frame 749) shrinks by a third but stays, because a
single loop at the end constrains the start-to-end relation, not every
stretch in between. The KITTI metric measures local drift over 100-800 m
stretches, which a loop closure does not change. Remember the answer key's
own local error of about 0.4 m at the end of this drive (step 3).

**A wrong loop is worse than none.** Replacing one of the four loops with a
copy shifted 5 m: end error 1.24 m (odometry: 0.80 m), KITTI t_err 0.49 %
(0.43 %). Three right loops do not outvote one confident wrong one. That is
why verification exists, and why robust back ends (that can switch off a loop
which disagrees with the rest) are the natural next improvement.

**Sequence 04** (no loop): no candidates, no loops, trajectory returned
unchanged byte for byte.

### Tests

Synthetic 400-frame loop with a heading bias: 13.5 m RMS drift to 1.9 m after
one loop, over every frame; frames between keyframes keep their place
relative to their keyframe; no loops leaves the trajectory unchanged; loop
frames must be keyframes. Correcting frames with the unoptimized keyframes
fails the drift test.

## What Phase 6 so far is, and is not

- It is: pose-graph loop closure on real LiDAR data, with verification, a
  measured improvement on a held-out drive, and a demonstration that one bad
  loop is harmful.
- It is not: full SLAM in the product sense. Loops are found by proximity in
  the estimate (no place recognition), there is one loop region in one drive,
  the back end is not robust to wrong loops, and it runs offline after the
  drive, not online.
