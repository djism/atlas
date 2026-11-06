# 2D ICP: guessing pairs, then solving

`estimateFromMatchedPoints` needs known pairs. Real scans have none. Iterative
Closest Point (ICP) guesses pairs from the current pose, solves, and repeats.

```
current = initial_guess
repeat:
    matches  = findNearestCorrespondences(first, second, current)
    paired   = first[matches[i]] for each i
    estimate = estimateFromMatchedPoints(paired, second)
    stop if estimate differs from current by <= tolerances
    current  = estimate
```

Frames: each `second[i]` (second-sensor axes) is mapped into first-sensor axes
with the guess before measuring distance. Each pass solves the **full** pose of
the second frame in the first frame from the original points; the guess only
selects pairs. Stopping when the pose stops changing means the pairing stopped
changing. `converged` therefore means **stable, not correct**.

`IcpResult` records `pose_per_pass` and `nearest_pair_rmse`. That RMSE is over
guessed nearest-neighbor pairs, which may be wrong; it is not the known-pair RMSE.

## Convergence example (x-axis)

First scan sees points at x = 0, 1, 5. The sensor moves 0.6 m forward, so the
second scan sees -0.6, 0.4, 4.4. Initial guess: no motion.

| Pass | Pairs (second -> first) | Pose x (m) | Nearest-pair RMSE (m) |
| --- | --- | --- | --- |
| 1 | -0.6->0, **0.4->0 (wrong)**, 4.4->5 | 0.267 | 0.471 |
| 2 | -0.6->0, 0.4->1, 4.4->5 | 0.600 | 0 |
| 3 | unchanged | 0.600 | 0 (stop) |

All points lie on a line with y = 0, so every `cross` term is zero and the
rotation stays exactly 0. Non-collinear scenes can tilt during early passes.

## Failure 1: large initial error (local minimum)

The five-point test cloud turned 162 degrees, with a zero initial guess,
stabilizes at about 14 degrees. It reports `converged`, but the final
nearest-pair RMSE stays about 2.03 m. Here the residual warns of trouble.
The same cloud turned 28.6 degrees from a zero guess is recovered exactly.
How large an initial error is tolerated depends on the scene; two examples do
not measure that range.

## Failure 2: identical-looking corridor

Poles stand at world x = 0..10 (rows y = +2 and -2). The first scan sees x = 0..9.
After moving 1 m, the second scan sees world x = 1..10, which in its own frame is
again 0..9. The scans are identical, so ICP reports zero motion with **zero**
residual. No residual can reveal this: the data does not contain the motion.
Additional sensors, motion priors, or distinctive structure are needed.

## Noise

Seeded Gaussian noise (`addGaussianNoise`) on both scans of the five-point cloud
(one seed per level, an illustration rather than statistics):

| Noise sigma | Position error | Rotation error |
| --- | --- | --- |
| 0 | 0 | 0 |
| 2 cm | 2.4 cm | 0.11 deg |
| 5 cm | 6.0 cm | 0.29 deg |
| 20 cm | 24 cm | 1.19 deg |

Error grows with noise but pairing stays correct, because landmarks are meters
apart. Noise alone did not break ICP here.

## Partial overlap: points without partners

ICP pairs **every** second-scan point. A pole that comes into view only after
moving has no true partner, but is forced onto the nearest first-scan point.
Same cloud, true pose (1, 0.5, 0.5 rad), noise-free, zero initial guess:

| Case | Position error | Rotation error |
| --- | --- | --- |
| Second scan has extra pole at (8, -5) | 1.55 m | 15.9 deg |
| Second scan has extra pole at (3, 1) | 0.34 m | 0.9 deg |
| Second scan has extra pole at (-6, -6) | 1.90 m | 18.8 deg |
| **First** scan has the extra pole | 0 | 0 |

All cases report `converged`. The lone point always left the largest residual
(e.g. 5.05 m versus at most 2.52 m for true pairs), which a distance gate can use.
An extra point in the first scan is harmless here because nothing must pair with
it; it would matter only if it were nearer to some second point than its true
partner. With five points, one lone point is 1/6 of the data; real scans have far
more points, but partial overlap at scan edges affects many of them.

## Distance gate

`IcpOptions::max_pair_distance` drops pairs whose gap, measured in first-sensor
axes under the **current** guess, exceeds the gate. Default: infinity (off).
If fewer than `min_pairs` survive, ICP stops with `IcpStatus::TooFewPairs`
("localization lost") and keeps the last pose. Same lone-pole scene:

| Gate | Zero initial guess | Good guess (0.9, 0.4, 0.45 rad) |
| --- | --- | --- |
| off | 1.55 m, 15.9 deg off | 0.96 m off |
| 4 m | exact (kept 5, 5, 5) | exact |
| 2-3 m | 1.85 m, 34.8 deg off (kept 3, 3): **worse than no gate** | exact |
| 1 m | exact (kept **2 -> 5** as the guess improved) | exact |
| 0.5 m | lost (too few pairs) | exact |
| 0.1 m | lost | lost |

With 2 cm noise and the good guess, gates of 0.5-4 m all give about 2.4 cm
error: the noise level.

Lessons:

- Judge a gate against the answer key. It lowers residuals by construction.
- Early passes need a gate larger than the guess error; otherwise true pairs are
  dropped. A good initial guess (e.g. predicted from previous motion while
  tracking) allows a tight gate. This is why tracking systems predict motion.
- The effect of gate size is not monotonic here: which pairs survive pass 1 is
  scene-specific. The 1 m success rests on two lucky correct pairs.
- "Lost" is reported as a status, not hidden or thrown, so a UI can show it.

## Current limits

- Brute-force nearest neighbor, O(first x second) per pass.
- A fixed distance gate only; no adaptive or statistics-based rejection yet.
- Equal-weight point-to-point error; one wrong pair can pull the estimate
  (see [alignment](alignment.md)).
- If all points pair to one first-scan point, the estimator's exception propagates.
- Demonstrated on small synthetic scenes with simple isotropic noise.
