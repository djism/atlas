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

### Daily checkpoint template

- Concept understood:
- Behavior implemented:
- Verification and result:
- Current limitation:
- Next smallest step:
