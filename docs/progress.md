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

### Daily checkpoint template

- Concept understood:
- Behavior implemented:
- Verification and result:
- Current limitation:
- Next smallest step:
