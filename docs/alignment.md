# Rigid alignment from many known pairs

`estimateFromMatchedPoints(first, second)` finds the rotation `R(theta)` and
translation `t` that minimize

```
sum_i | first[i] - (R * second[i] + t) |^2
```

where `first[i]` (first-sensor axes) and `second[i]` (second-sensor axes) are
**known** observations of the same stationary landmark. The result is the pose
of the second sensor frame expressed in the first sensor frame.

## Step 1: centering removes translation

If every pair satisfies `first_i = R * second_i + t`, averaging gives
`first_c = R * second_c + t`. Subtracting:

```
first_i - first_c = R * (second_i - second_c)
```

`t` cancels. Centered points keep the sensor's axis directions; only the origin
moves to the centroid. After solving for `R`, `t = first_c - R * second_c`.

Demo numbers: first centroid (3, 0), second centroid (1.969616, -0.347296).
Centered first A = (-1, 1); centered second A = (-0.811160, 1.158456).
Rotating the latter by +10 degrees gives (-1, 1).

## Step 2: dot and cross give the best angle

Let `p` be a centered first point and `q` the matching centered second point.

```
dot   = sum(q.x*p.x + q.y*p.y)   = sum |q||p| cos(angle q->p)
cross = sum(q.x*p.y - q.y*p.x)   = sum |q||p| sin(angle q->p)
```

Expanding the objective, only `cos(theta)*dot + sin(theta)*cross` depends on
theta, and it is maximized at `theta = atan2(cross, dot)`. Pairs farther from the
centroid get more weight, as their residuals are more sensitive to angle error.

Demo numbers: dot = 3.939231, cross = 0.694593, atan2 -> 10.000 degrees.

## Degeneracy (rejected with std::invalid_argument)

- All points in a scan coincide: centered vectors vanish, `atan2(0, 0)` is meaningless.
  Tested as rms spread <= 1e-9 * largest coordinate magnitude.
- `hypot(dot, cross)` near zero while points are spread (e.g. a mirrored
  symmetric shape): no angle is preferred. By Cauchy-Schwarz,
  `hypot(dot, cross) <= sqrt(spread_first) * sqrt(spread_second)`, with equality
  for noise-free rigid data; we reject below 1e-9 of that bound.

Both checks are relative to the data, so they do not depend on units.
Collinear points are **accepted**: a line has a direction, so 2D rotation is defined.

## What least squares does and does not promise

With noise, no pose aligns every pair. The estimate is the pose that best fits
the given pairs, and its matched-point RMSE is at most that of the true pose on
the same data. In the fixed test example (offsets up to 2 cm), the estimate is
about 4 mm and 0.15 degrees from truth, with RMSE 0.0181 m versus 0.0214 m for
the true pose.

It does not discover correspondences, and it is not robust: one wrong pair can
pull the whole estimate, because squared error weights large residuals heavily.
`computeMatchedRmse` measures error over known pairs; a low nearest-neighbor
residual would not prove correct pairing.
