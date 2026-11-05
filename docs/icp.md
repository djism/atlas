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

## Current limits

- Brute-force nearest neighbor, O(first x second) per pass.
- Every second-scan point is paired; no rejection of pairs without a true partner.
- Equal-weight point-to-point error; one wrong pair can pull the estimate
  (see [alignment](alignment.md)).
- If all points pair to one first-scan point, the estimator's exception propagates.
- Demonstrated on noise-free scenes only.
