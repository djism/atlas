# Atlas

**Learning vehicle motion estimation and SLAM through a C++ implementation.**

Latest release: **v0.1.0 — 2D geometry and motion recovery with known correspondences.**
In progress (unreleased): least-squares alignment from many known matched pairs.

Given two observations of stationary landmarks, Atlas estimates the translation
and rotation that express the second observation in the first sensor's frame.
The first milestone reproduces a controlled experiment: one meter forward and
a ten-degree left turn.

This is an educational foundation. It is not a complete SLAM system, a Tesla
implementation, or vehicle-ready software. Real-data odometry, robust matching,
3D mapping, loop closure, distributed replay, and an interactive explorer are
planned work, not implemented features.

## Build and run

Requirements: a C++17 compiler and CMake 3.16 or newer. No third-party C++ libraries.
On macOS, Apple Clang from the Xcode Command Line Tools works; CMake is a separate prerequisite.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/atlas_demo
```

If CMake is not installed, the same code can be compiled directly on macOS/Linux:

```bash
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/geometry.cpp src/simulation.cpp src/matching.cpp apps/demo.cpp -o atlas_demo
./atlas_demo
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/geometry.cpp src/simulation.cpp src/matching.cpp tests/test_atlas.cpp -o atlas_tests
./atlas_tests
```

Expected demo values (rounding and negative zero may vary):

```text
Estimated translation: 1.000000, 0.000000
Estimated rotation (degrees): 10.000000
Aligned: 2.000000, 1.000000 | matched-point error (m): 0.000000
Aligned: 4.000000, -1.000000 | matched-point error (m): 0.000000
Matched-point RMSE (m): 0.000000
```

## What the code does

1. The simulator knows fixed world landmarks and two true sensor poses.
2. It generates two scans in the respective sensor frames.
3. `estimateFromMatchedPoints()` receives only those scans, with known matching order.
4. It subtracts each scan's centroid, removing translation from the problem.
5. It finds the least-squares rotation as `atan2(cross, dot)` over all centered pairs,
   then translation as `first_centroid - R * second_centroid`. See [alignment](docs/alignment.md).
6. The demo applies the estimate and reports error against the matched first-scan points.

`estimateFromTwoPairs()` remains available as a simpler two-pair teaching baseline.

The true poses are the answer key for this experiment, not estimator inputs.
Nearest-neighbor search exists as a separate building block; it is **not** used
to establish the correspondences in the motion-recovery demo. ICP is not yet implemented.

## Project layout

| Path | Purpose |
| --- | --- |
| `include/atlas/` | Public types and function declarations |
| `src/` | Geometry, sensor simulation, and matching implementations |
| `apps/demo.cpp` | Reproducible two-scan experiment |
| `tests/test_atlas.cpp` | Numerical and invalid-input checks |
| `docs/frames.md` | Coordinate conventions and an explanation of the experiment |
| `docs/alignment.md` | Multi-point alignment derivation, degeneracy, and limits |
| `docs/progress.md` | Daily milestones, validation, and next learning task |
| `.github/workflows/ci.yml` | Linux/macOS build and test workflow |
| `CHANGELOG.md` | Versioned milestone history |

## Conventions and limits

Distances are meters; angles are counterclockwise radians. Sensor +x is forward
and +y is left. The sensor and car frames coincide in this simulation.
`Pose2D` describes the source frame in a destination frame. See [frames](docs/frames.md).

The simulator is an ideal 360-degree range filter over point landmarks, with no
noise or occlusion. Visibility can change, so scan indices are not stable landmark
identities. The general estimator requires at least two correctly paired points,
not all coincident, and rejects geometrically ambiguous rotations. With noise it
returns the best fit to the given pairs, not the exact truth; it is not robust to
incorrect pairs. Small alignment error alone does not establish correct localization. Geometry/simulation inputs are assumed
finite; general numerical hardening remains future work.

Tests cover transform inversion, frame conventions, sensor-coordinate output,
range boundaries, nearest neighbors, centroids, two-pair and multi-point relative
motion, angle wrapping, a fixed noisy least-squares example, and invalid inputs.
CI is configured for Linux/macOS; a successful local run does not imply remote CI has run.

## Versioning and daily workflow

Use semantic versions for completed milestones: `v0.1.0`, `v0.2.0`, and so on while
interfaces are evolving. Use commits for daily progress; a release tag is not
required every day. Record a release in the changelog before tagging it. Never move
an existing published release tag.

Each working session should end with one understandable, tested increment:

1. State what concept or behavior changed.
2. Build, run relevant tests, and inspect the demo.
3. Update `docs/progress.md`; update the changelog for release changes.
4. Review `git diff`, commit, and push.

Use messages such as `feat: add centroid computation` or `test: cover changing scan visibility`.
Daily pushes are a working-session habit; this repository does not auto-generate
commits or schedule unattended pushes.

## Roadmap

- [x] 2D frame transformations and ideal scan simulation
- [x] Nearest-point search and motion recovery from two known pairs
- [x] Centroids and rigid alignment using many known pairs
- [ ] Iterative correspondence search and 2D ICP
- [ ] Noisy observations, rejection gates, and failure diagnostics
- [ ] 3D registration and evaluation on driving data
- [ ] Local mapping and verified loop closure
- [ ] Distributed replay jobs with retries and idempotent outputs
- [ ] Interactive trajectory and map explorer

These are goals for an eight-week development effort, with scope reviewed as
measured progress accumulates.
