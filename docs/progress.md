# Progress journal

## 2026-11-04 — v0.1.0: establish the tested 2D foundation

### Implemented

- Extracted frame transformations, range-limited scan simulation, and nearest-point search.
- Packaged the two-known-pair rotation/translation exercise as a reusable estimator.
- Added a deterministic demonstration of one meter forward and a ten-degree turn.
- Added CMake, numerical/input checks, and a Linux/macOS CI workflow.
- Documented frame semantics, assumptions, versioning, and the daily commit habit.

### Lesson

Measurements depend on the sensor's frame. We estimate relative motion by finding
a rigid transformation that makes corresponding stationary observations agree.
Simulation knows the answer; the estimator receives only the observations.

### Validation

GCC C++17 compilation passed with `-Wall -Wextra -Wpedantic -Werror`.
The test executable passed geometry, simulation, matching, motion recovery, and
invalid-input checks. The demo recovered (1, approximately 0) meters and 10 degrees,
with matched-point RMSE approximately zero. Remote CI has not run yet.

### Next learning task

Implement and understand `computeCentroid()` for a nonempty point set. Explain why
subtracting each scan's centroid removes translation before generalizing rotation
estimation to many known pairs. Do not introduce unknown matching simultaneously.

## 2026-11-04 (session 2) — least-squares alignment from many known pairs

### Implemented

- Tested `computeCentroid()`, including the property that translating every point
  leaves centered points unchanged.
- Added `estimateFromMatchedPoints()`: centroids, `dot`/`cross` accumulation,
  `theta = atan2(cross, dot)`, `t = first_c - R * second_c`. Returns the second
  sensor frame's pose in the first sensor frame.
- Scale-aware rejection of coincident points and ambiguous rotation (e.g. a
  mirrored symmetric shape); collinear points are accepted.
- Added `computeMatchedRmse()` over known pairs; demo now uses the general
  estimator and the temporary centroid prints were removed.
- Wrote `docs/alignment.md` with the derivation and worked demo numbers.

### Lesson

Centering cancels translation, so rotation can be solved alone. Summing dot and
cross over pairs is the exact least-squares rotation, not an average of angles.
With noise, least squares fits the data at least as well as the truth does, and
can therefore differ from the truth. Testing the optimality property caught a
0.29-degree injected bias that a loose "close to truth" tolerance missed.

### Validation

Apple Clang, C++17, `-Wall -Wextra -Wpedantic -Werror`: demo and tests build and pass.
Demo recovers (1, 0) m and 10 degrees with zero matched-point RMSE. Mutation
checks on scratch copies (flipped cross sign, removed ambiguity guard, rotation
bias) each made tests fail. CMake was not available locally; remote CI
result for this commit is reported separately.

### Limitations

Known correspondences only. Not robust to wrong pairs or outliers. Noise
behavior is shown for one fixed example, not statistically characterized.
The 1e-9 degeneracy tolerance guards numerical nonsense, not sensor quality.

### Next smallest step

Show what one wrong pairing does to the estimate (bias, RMSE), motivating
correspondence search. Then iterate nearest-neighbor matching and alignment as
a first 2D ICP loop, with a convergence example and a failure example.

## 2026-11-05 — from known pairs to basic 2D ICP

### Implemented

- One-wrong-pair experiment (test + `docs/alignment.md`): one of five pairs wrong
  turned a +10 degree turn into -10.3 degrees and shifted position ~1.8 m.
- `findNearestCorrespondences()`: maps second-scan points into the first frame
  with a guessed pose, then pairs each with its nearest first-scan point.
- `alignIcp()` in new `icp.hpp`/`icp.cpp`: pair -> solve -> stop when the pose
  stops changing; returns per-pass poses and nearest-pair RMSE.
- Demo prints ICP passes for the driving scene (scan order reversed), the
  x-axis example, and the identical-corridor failure.
- `docs/icp.md` explains the loop, convergence table, and failure cases.

### Lesson

Least squares trusts its pairs: one wrong pair contaminates every residual.
Pairing must happen in one frame, so the guess maps second-scan points first.
ICP converges when the pairing stops changing; that is stability, not
correctness. A 162-degree start stabilizes on a wrong pose (high residual), and
an identical-looking corridor gives a wrong pose with zero residual.

### Validation

Apple Clang, C++17, `-Wall -Wextra -Wpedantic -Werror`: demo and tests pass.
Mutation checks on scratch copies: skipping the pose transform in pairing and
stopping ICP after one pass each made tests fail. CMake not available locally;
remote CI result reported separately.

### Limitations

Noise-free scenes only. Every second-scan point is paired (no rejection of
points without a partner). Brute-force nearest neighbor. Two failure examples,
not a measured basin of convergence.

### Next smallest step

Add fixed noise and partial overlap to an ICP scene (a landmark visible in only
one scan), observe the bias, then add a simple distance gate that rejects pairs
farther than a threshold, and measure its effect against the answer key.

## 2026-11-06 — roadmap, seeded noise, partial overlap, distance gate

### Implemented

- `docs/roadmap.md`: Atlas as engine + evidence + experience; Phases 1-5 as the
  ~8-week flagship, with done criteria, target architecture, and quality bar.
- `addGaussianNoise(points, stddev, seed)`: own Box-Muller over `std::mt19937_64`,
  because `std::normal_distribution` differs between libstdc++ and libc++.
- Partial-overlap tests: a pole seen only in the second scan biased ICP by up to
  1.9 m and 19 degrees; an extra first-scan pole caused no error.
- Distance gate and `min_pairs` in `IcpOptions`; `IcpStatus` with explicit
  `TooFewPairs`; per-pass kept-pair counts. Demo prints LOST when relevant.

### Lesson

Noise shifts the answer by about the noise size; a point without a partner can
wreck it, because ICP must pair every second-scan point. A gate fixes this only
if it is larger than the current guess error: from a zero guess, a 2-3 m gate
was worse than none, while a good guess made 0.5-4 m gates all work. Gate size
effects are not monotonic. "Lost" should be reported, not hidden.

### Validation

Apple Clang, C++17, `-Wall -Wextra -Wpedantic -Werror`: demo and tests pass.
Noise statistics over 40000 values: mean, stddev within 3%, and 68.3% +/- 1%
within one sigma. Mutation checks: uniform noise and a missing Box-Muller
factor failed the shape check; ignoring the gate failed 13 checks; gating with
the initial instead of the current guess failed the 2 -> 5 growth check.
Two mistakes caught: a default `min_pairs` of 3 silently "lost" the two-landmark
demo (fixed: default 2, undersized scans are invalid input), and the first gate
tests could not detect stale-guess gating (fixed with the growth test).

### Limitations

Small synthetic scenes; isotropic noise; one seed per illustration; fixed gate.
No observability/degeneracy diagnostic yet: the identical corridor still reports
success.

### Next smallest step

Degeneracy diagnostic: report when the scene does not constrain motion in some
direction (corridor case), so ICP can say "unobservable" instead of converged.

## 2026-11-06 (session 2) — ambiguity check for aliasing and wrong minima

### Implemented

- `countInliers()`: points with a partner within a distance under a pose.
- `checkAmbiguity()`: restarts ICP from the solution plus offsets, and flags a
  clearly different rival that explains at least 80% as many points.
- Demo prints the corridor warning: rival at x = 0.9 m explains 18/20 poles.

### Lesson

"The scene cannot tell" has two forms. Aliasing (repeating poles) is not caught
by the fixed-pair information-matrix test, because paired poles constrain motion
fully; degeneracy (featureless walls) is, and waits for line-based ICP. Inlier
counts beat RMSE for comparing hypotheses: in the corridor the wrong answer had
RMSE 0 and the truth 0.3 m. With a half-turn restart, the check found the true
pose for the 162-degree local minimum, which explained more points.

### Validation

Apple Clang, C++17, `-Werror`: demo and tests pass. Mutation checks: removing
the separation test and ignoring the rival fraction each failed tests. A
turn-only test documents a missed rival: "not ambiguous" is not proof of uniqueness.

### Limitations

Finds only rivals reachable from chosen offsets; one ICP run per offset;
offsets, inlier distance, and fraction are scene-dependent.

### Next smallest step

Ray-cast 2D LiDAR: describe a room as wall segments and compute where each beam
hits, producing scans like a real planar LiDAR for the home-robot scenario.

### Daily checkpoint template

- Concept understood:
- Behavior implemented:
- Verification and result:
- Current limitation:
- Next smallest step:
