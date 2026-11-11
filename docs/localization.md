# Scan-to-map localization

Scan-to-scan tracking estimates each motion from the previous scan and adds it
to the previous pose. Every small error is kept forever, so the pose drifts.
`MapLocalizer` registers each scan against a fixed map instead. Its answer is
the sensor pose in the map directly, so an error in one scan does not carry
into the next.

## Inputs, outputs, frames

- Map points: `OccupancyMap::occupiedPoints()`, in **map** coordinates. Here
  the map frame is the world frame, because the map was built at answer-key poses.
- Each scan: points in the **current sensor** frame.
- Initial pose: the sensor pose in the map at the first scan. This is pose
  tracking; finding the pose with no starting guess (global localization) is
  a different problem.
- Output per scan: the sensor pose in the map, status, kept pairs, residual,
  and the inlier fraction (share of scan points with a map partner within 0.3 m).

The same ICP code does the work, with the map as "first" and the scan as
"second". ICP returns the second frame's pose in the first frame, which here
means the sensor in the map. The guess is the previous estimate moved by the
previous motion. The map's line fits (normals) are computed once, because the
map does not change.

## Results: three laps of a 1.5 m circle

10 cm per scan, 360 beams, 10 m range, room with alcove and box, map at 5 cm
cells from 7 noise-free survey scans. Errors are against the answer key.

| Scan noise | Tracker, worst | Localizer, worst | Localizer, mean |
| --- | --- | --- | --- |
| 1 cm | 1.5 cm | 2.8 mm | 1.1 mm |
| 3 cm | 10.7 cm, growing | 9.1 mm | 3.3 mm |

No lost scans; at least 92% of scan points matched the map. About 6 ms per scan
(-O2, Apple M-series). A wrong starting pose (10 cm, 2 degrees) is corrected
by the first scan; a tracker would keep that offset forever.

## Findings

1. **Cell centers biased the map.** Using occupied cell centers as map
   points gave a steady 2.4 cm error: a center can be up to half a cell from
   the wall. Each cell now also stores the mean of the beam end points that
   landed in it. That removed the bias and raised the inlier fraction from 76%
   to 92%. Corner cells average two walls, so 4 of 796 points sit up to 1 cm
   off.
2. **A changed world shows in the inlier fraction.** A box the map never saw,
   1 m ahead, dropped the inlier fraction from 0.95 to 0.82. The pose stayed
   within 2 mm, because the walls still matched.
3. **Parallel walls let ICP slide.** At 3 m range, one scan saw only the long
   bottom wall and the box face parallel to it. Nothing fixed the position
   along them: ICP slid 25 cm with no rise in residual and reported Converged.
   The next guess then used that slide as the robot's velocity, and the error
   grew 0.25, 0.50, 0.80, 1.11 m.
4. **The motion check limits, but does not solve, this.** A result more than
   15 cm (later 0.3 m and 0.5 rad) from the guess is rejected (the robot cannot jump). It
   caught the 25 cm slide, but smaller slides (9 cm) still passed and added up.

## Degeneracy: trust the scan only where it has evidence

A matched point on a wall can only correct position along that wall's normal
n. Averaged over all kept pairs, the evidence for moving in a unit direction u
is the mean of (n . u)^2 = u^T A u, where A = mean(n n^T) is a 2x2 matrix.
Its eigenvalues `weak` and `strong` sum to 1 (unit normals), and the
eigenvector of `weak` is the direction with the least evidence
(`translationObservability()` in geometry):

| View | Normals | weak, strong |
| --- | --- | --- |
| Corner, both walls equally | half up, half right | 0.5, 0.5 |
| Mostly one wall | 90% up, 10% right | 0.1, 0.9 |
| Parallel walls | all up | 0, 1 (weak direction: along the walls) |

Measured: every scan at 10 m range had weak >= 0.34. The scan that slid at
3 m range had weak = 0.003, with weak direction (-1.000, 0.004): exactly the
direction it slid.

When weak < 0.05, `MapLocalizer` keeps ICP's answer across the weak direction
w and in heading, but along w it keeps the prediction:
`pose += ((guess - pose) . w) w`. The step is flagged `weakly_constrained`.

| 3 m range, 227 scans, one seed each | Worst error | Mean |
| --- | --- | --- |
| Path A, 1 cm noise, no handling | 5.06 m | 2.01 m |
| Path A, 1 cm noise, weak < 0.05 | 3.4 cm | 4 mm |
| Path B, 3 cm noise, no handling | 2.66 m | 1.11 m |
| Path B, 3 cm noise, weak < 0.05 | 1.70 m | 0.49 m |

Threshold 0.05 beat 0.1 and 0.15 on both paths: scans with weak between 0.05 and
0.1 still carry useful sideways evidence. It was chosen on these two runs, not
on held-out data.

**Correction from more seeds.** The 3.4 cm above was one noise seed, and it
was the luckiest. Over 20 seeds of the same run (path A, 3 m, 1 cm), the
localizer's worst error was under 10 cm in 11 seeds, 11 to 23 cm in 6, and over
1 m in 3 (1.1, 3.2, 5.4 m). The tracker's worst error was 1.2 to 8.7 m in every
seed. Degeneracy handling stops the sudden slide; it does not make short-range
localization reliable.

Path B shows what is left. Along an unobservable direction the only
information is the prediction, which is constant velocity from past estimates.
In the first weak stretch it drifted about 3 mm per 10 cm (to 6 cm); once
walls in both directions came back, the map pulled the error back to 1 mm. In
a later stretch the prediction drifted faster, reached 0.8 m, and could not
recover. A better prediction (wheel odometry or an IMU, as real robots have)
is what bounds this; the scan simply has no information there.

## Repeatable experiment

`atlas_experiment` (apps/experiment.cpp, loop in `runExperiment()`) runs every
combination of path (A: 1.8 m circle around (4.6, 2.6); B: 1.5 m around
(4.5, 2.8)), sensor range (10 m, 3 m), and noise (1 cm, 3 cm), two laps each,
for several seeds. Tracker and localizer get identical scans. The map is
surveyed with a 10 m sensor in every case. `--csv path` writes one row per
run and method (put it under `results/`, which Git ignores).

```bash
cmake --build build && ./build/atlas_experiment --seeds 5 --csv results/runs.csv
```

Result with 5 seeds (worst error over seeds; lost, weak, and jumps summed),
after the fit check and heading recovery below:

| Path | Range | Noise | Tracker worst / drift | Localizer worst / mean | Lost | Weak | Jumps | Failed |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | 10 m | 1 cm | 3.7 cm / 0.13% | 3 mm / 1.1 mm | 0 | 0 | 0 | 0/5 |
| A | 10 m | 3 cm | 19.4 cm / 0.65% | 10 mm / 3.2 mm | 0 | 0 | 0 | 0/5 |
| A | 3 m | 1 cm | 8.0 m / 35% | 16.1 cm / 1.4 cm | 145 | 109 | 0 | 5/5 |
| A | 3 m | 3 cm | 20.1 m / 89% | 17.6 m / 2.54 m | 557 | 104 | 33 | 5/5 |
| B | 10 m | 1 cm | 2.3 cm / 0.10% | 4 mm / 1.0 mm | 0 | 0 | 0 | 0/5 |
| B | 10 m | 3 cm | 9.9 cm / 0.33% | 11 mm / 3.0 mm | 0 | 0 | 0 | 0/5 |
| B | 3 m | 1 cm | 6.6 m / 35% | 9.6 m / 1.12 m | 358 | 208 | 59 | 5/5 |
| B | 3 m | 3 cm | 52.3 m / 251% | 11.3 m / 2.08 m | 544 | 129 | 106 | 5/5 |

Failed means localizer worst error above 10 cm. Drift is the tracker's final
error divided by distance driven (worst seed). About 50 s at -O2 (the heading
search runs on every failed scan, which is frequent at 3 m). Before the fit
check and recovery the 3 m localizer rows were 22.2 m, 11.2 m, 3.7 m, and
19.1 m worst: one row improved a lot, the others moved both ways. The 10 m
rows did not change.

Reading it: with a 10 m sensor the map localizer never fails and is 10 to 20
times more accurate than the tracker, whose drift keeps growing. With a 3 m
sensor both fail in every seed; the localizer's mean error is lower, its worst
is not reliably better.

Bug found while building this: the harness first surveyed the map with the
run's sensor, so 3 m runs got a map from a 3 m survey. The survey now has its
own sensor setting (`survey_lidar`, default 10 m).

## Fit check and heading recovery (found in the sandbox)

In the browser house (12 x 8 m, two rooms), the localizer ran away at the
doorway, where the robot stops and spins in place by about 1.7 rad. Reproduced
in C++, it failed 10 of 10 runs even with scans every 0.1 s:

- The robot starts spinning; the constant-velocity guess has no rotation. With
  a 0.32 rad heading error, wall points 5 m away land about 1.6 m from their
  map partners, beyond the 0.3 m pair gate. Only nearby points pair, and ICP
  converged to a wrong pose that only 33% of the scan supported. In the small
  test room every point is near, which is why the room tests never showed it.
- Nothing checked the fit, so that pose was accepted. The next guess built on it.

Two changes:

1. **Fit check.** A result is rejected if fewer than half of the scan points
   found a map partner within the gate (`min_inlier_fraction` 0.5). Good fits
   measured about 0.9, with six unmapped boxes about 0.8; wrong convergences
   0.1 to 0.35.
2. **Heading recovery.** When the guess fails, headings within 1 rad of the
   guess are tried every 0.1 rad. Each is scored by how many scan points land
   within 10 cm of the map (`countInliers`), no ICP needed; then ICP starts
   from the best, and the result must pass the fit check. Searching candidate
   poses by scoring the raw scan against the map is the idea behind correlative
   scan matching.

Doorway spin with scans every 0.10, 0.15, 0.18, and 0.20 s, 10 seeds each:
40 of 40 failed before, 0 of 40 after. A regression test runs the doorway in
the house and also checks that turning off either change brings the runaway
back.

A second sandbox failure: right after a spin, the robot starts driving and the
constant-velocity guess says "stand still". With scans 0.2 s or more apart the
first step forward exceeded the 15 cm jump limit and was rejected over and
over (6 to 10 of 10 runs failed). The translation limit is now 0.3 m: 0 of 10
failed for scan gaps up to 0.3 s, and every earlier test still passes.

The deeper cause in the browser was the simulation, not the localizer: the
page took a new scan only after the previous one finished, so a busy browser
quietly lowered the sensor rate, at times below 2 Hz. The sandbox now captures
a scan every 0.1 s of simulated time and pauses the simulated world if the
engine falls more than two scans behind (see [sandbox](sandbox.md)).

## Motion prediction: constant velocity vs wheel odometry

Each scan, ICP starts from a guess. Until now the guess was **constant
velocity**: "the robot will move like it did last scan". That guess is made
from the localizer's own previous answers. Real robots also count wheel turns
(odometry): an independent, noisy measurement of how far they moved and turned.

`simulateOdometry()` reports one step of true motion with assumed errors (not
measured on a real robot): wheels over-report distance by 2 %, random 5 % of
the distance and of each turn, and 0.02 rad of heading error per meter driven.
`MapLocalizer::update(scan, motion)` uses it: guess = previous pose moved by
the reported motion (the sensor frame at this scan, in the frame at the
previous scan). The jump check and recovery are unchanged; they now judge the
scan against the odometry guess.

Two failure patterns of constant velocity, both found by measuring:

1. **Sudden start of a turn.** A robot stops, then turns in place at 0.6 rad
   per scan. Constant velocity predicts no turn. ICP from that guess settled on
   a pose 13 cm and 0.24 rad wrong with 59 % of the scan matching: above the
   50 % fit threshold and inside both jump limits, so it was accepted. On a
   35-scan path through one such corner, constant velocity was worse than 5 cm
   in 18 of 20 seeds (two above 1 m); odometry was within 3 mm in 20 of 20.
2. **Feedback.** A wrong answer becomes the next velocity, so the next guess is
   wrong too. With a 3 m sensor, where many scans are lost or weak, this ran
   away on every circle run. Odometry does not depend on past answers, so one
   bad scan does not poison the next guess.

`atlas_experiment --seeds 5`, second table (1 cm scan noise, 5 seeds; "failed"
= worst error above 10 cm; S stops at each corner of a two-lap loop and turns
in place at 0.6 rad per scan):

| Path, range | Constant velocity worst (failed) | Odometry worst (failed) | Odometry alone |
| --- | --- | --- | --- |
| A circle, 10 m | 3 mm (0/5) | 3 mm (0/5) | 20 cm |
| A circle, 3 m | 16 cm (5/5) | 7.3 cm (0/5) | 20 cm |
| B circle, 3 m | 9.6 m (5/5) | 7.0 cm (0/5) | 19 cm |
| S stop-and-go, 10 m | 10.1 m (5/5) | 3 mm (0/5) | 90 cm |
| S stop-and-go, 3 m | 15 cm (3/5) | 9.2 cm (0/5) | 90 cm |

"Odometry alone" chains the readings with no scans (dead reckoning): it drifts
without bound, so wheels do not replace the map. Each one covers the other's
weakness: odometry gives a guess that cannot run away; the map removes drift.

How much this depends on the assumed wheel quality (`--odometry-scale`):

- 3x worse odometry: 10 m rows unchanged (3 mm). 3 m rows fail again (A 3/5,
  B 4/5, S 5/5) at 20 to 28 cm, not meters.
- 10x worse: 10 m circles still 3 to 4 mm; S at 10 m reaches 9.9 cm, just
  under the failure line. 3 m rows fail by meters.

So with the 3 m sensor, the localizer is mostly coasting on odometry between
the few scans that see enough walls; the result is only as good as the wheels.
Short range is not solved; it stops running away when odometry is decent.

## Limits

- Needs a good initial pose. Recovery searches heading only, around the
  guess's position; a position error beyond the 0.3 m gate is not recovered
  (no global relocalization).
- With short range many scans see one wall or nothing; at 1.5 m range in this
  room most scans had no points at all.
- Only translation observability is analyzed. A view that leaves rotation
  unconstrained (e.g. a round room) is not detected; a full 3x3 analysis would.
- Along a weak direction, and on lost scans, the pose is the prediction
  (constant velocity or odometry), i.e. dead reckoning.
- The jump check assumes motion between scans is predictable within 0.3 m and
  0.5 rad, per scan, so it depends on the scan rate. The limits started at
  15 cm and 0.2 rad; both rejected correct answers in the sandbox (a sudden
  in-place turn, a sudden start). With odometry the limits could be much
  tighter (they bound the odometry error, not the motion); not yet tuned.
- The constant-velocity guess is per scan: uneven scan timing makes it wrong.
  Odometry is timing-independent (it reports what happened between scans).
- The odometry model is simplified and its noise values are assumptions. It
  has no wheel slip events (e.g. a bump), and its errors are independent per
  step. Odometry and scans are assumed to be captured at the same instants.
- The localizer does not weight odometry against the scan: it is only the
  starting guess (and the answer when a scan is rejected). A filter (e.g. an
  extended Kalman filter) would combine both by their uncertainties.
- Brute-force nearest neighbors over all map points; the map is noise-free.
- Test suite: about 5 s optimized, mostly these laps and the sudden-turn runs.
