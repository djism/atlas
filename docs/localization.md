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
   15 cm or 0.2 rad from the guess is rejected (the robot cannot jump). It
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

Result with 5 seeds (worst error over seeds; lost, weak, and jumps summed):

| Path | Range | Noise | Tracker worst / drift | Localizer worst / mean | Lost | Weak | Jumps | Failed |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | 10 m | 1 cm | 3.7 cm / 0.13% | 3 mm / 1.1 mm | 0 | 0 | 0 | 0/5 |
| A | 10 m | 3 cm | 19.4 cm / 0.65% | 10 mm / 3.2 mm | 0 | 0 | 0 | 0/5 |
| A | 3 m | 1 cm | 8.0 m / 35% | 22.2 m / 1.42 m | 332 | 100 | 77 | 5/5 |
| A | 3 m | 3 cm | 20.1 m / 89% | 11.2 m / 0.93 m | 651 | 97 | 226 | 5/5 |
| B | 10 m | 1 cm | 2.3 cm / 0.10% | 4 mm / 1.0 mm | 0 | 0 | 0 | 0/5 |
| B | 10 m | 3 cm | 9.9 cm / 0.33% | 11 mm / 3.0 mm | 0 | 0 | 0 | 0/5 |
| B | 3 m | 1 cm | 6.6 m / 35% | 3.7 m / 0.35 m | 389 | 240 | 236 | 5/5 |
| B | 3 m | 3 cm | 52.3 m / 251% | 19.1 m / 2.44 m | 604 | 137 | 185 | 5/5 |

Failed means localizer worst error above 10 cm. Drift is the tracker's final
error divided by distance driven (worst seed). About 30 s at -O2.

Reading it: with a 10 m sensor the map localizer never fails and is 10 to 20
times more accurate than the tracker, whose drift keeps growing. With a 3 m
sensor both fail in every seed; the localizer's mean error is lower, its worst
is not reliably better.

Bug found while building this: the harness first surveyed the map with the
run's sensor, so 3 m runs got a map from a 3 m survey. The survey now has its
own sensor setting (`survey_lidar`, default 10 m).

## Limits

- Needs a good initial pose; once the error exceeds the 0.3 m gate it cannot
  recover by itself (no relocalization).
- With short range many scans see one wall or nothing; at 1.5 m range in this
  room most scans had no points at all.
- Only translation observability is analyzed. A view that leaves rotation
  unconstrained (e.g. a round room) is not detected; a full 3x3 analysis would.
- Along a weak direction the pose is dead reckoning from constant velocity.
- The jump check assumes motion between scans is predictable within 15 cm.
  Fast starts (no velocity known yet) would be rejected.
- Tests do not show that the constant-velocity guess is needed: at 10 cm per
  scan, starting ICP from the previous pose works equally well.
- Brute-force nearest neighbors over all map points; the map is noise-free.
- Test suite: about 2.6 s optimized, 14 s unoptimized, mostly these laps.
