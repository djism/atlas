# Atlas

**Learning vehicle motion estimation and SLAM through a C++ implementation.**

Latest release: **v0.1.0 — 2D geometry and motion recovery with known correspondences.**
In progress (unreleased): least-squares alignment from many known pairs, basic 2D ICP,
seeded noise, and a distance gate (roadmap Phase 1).

Given two observations of stationary landmarks, Atlas estimates the translation
and rotation that express the second observation in the first sensor's frame.
The first milestone reproduces a controlled experiment: one meter forward and
a ten-degree left turn.

This is an educational foundation. It is not a complete SLAM system, any company's
stack, or vehicle-ready software. Real-data odometry, robust matching, 3D mapping,
loop closure, distributed evaluation, and the interactive sandbox are planned work,
not implemented features.

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
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/geometry.cpp src/simulation.cpp src/matching.cpp src/icp.cpp apps/demo.cpp -o atlas_demo
./atlas_demo
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/geometry.cpp src/simulation.cpp src/matching.cpp src/icp.cpp tests/test_atlas.cpp -o atlas_tests
./atlas_tests
```

Expected demo values (rounding and negative zero may vary):

```text
Estimated translation: 1.000000, 0.000000
Estimated rotation (degrees): 10.000000
Aligned: 2.000000, 1.000000 | matched-point error (m): 0.000000
Aligned: 4.000000, -1.000000 | matched-point error (m): 0.000000
Matched-point RMSE (m): 0.000000

ICP: x-axis example, sensor moved 0.6 m
  pass 1: pose (0.266667, 0.000000, 0.000000 deg) | nearest-pair RMSE (m): 0.471405
  pass 2: pose (0.600000, 0.000000, 0.000000 deg) | nearest-pair RMSE (m): 0.000000
  ...
ICP: FAILURE - identical-looking corridor, moved 1 m
  ... | position error (m): 1.000000
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

Without known pairs, `alignIcp()` runs point-to-point 2D ICP: pair each second-scan
point with its nearest first-scan point under the current guess, solve, repeat until
the pose stops changing. The demo shows a converging case and a failure where ICP
reports a perfect fit to a wrong pose. "Converged" means stable, not correct.
An optional distance gate drops pairs that are too far apart under the current
guess; if too few survive, ICP reports `TooFewPairs` (localization lost) instead
of a pose. Gate size must suit the initial-guess error. See [ICP](docs/icp.md).

`addGaussianNoise()` adds seeded, cross-platform reproducible Gaussian noise to scans.

## Project layout

| Path | Purpose |
| --- | --- |
| `include/atlas/` | Public types and function declarations |
| `src/` | Geometry, sensor simulation and noise, matching, and ICP implementations |
| `apps/demo.cpp` | Reproducible known-pair and ICP experiments |
| `tests/test_atlas.cpp` | Numerical and invalid-input checks |
| `docs/frames.md` | Coordinate conventions and an explanation of the experiment |
| `docs/alignment.md` | Multi-point alignment derivation, degeneracy, and limits |
| `docs/icp.md` | ICP loop, convergence example, and failure cases |
| `docs/progress.md` | Daily milestones, validation, and next learning task |
| `docs/roadmap.md` | Phases, done criteria, target architecture, product quality bar |
| `.github/workflows/ci.yml` | Linux/macOS build and test workflow |
| `CHANGELOG.md` | Versioned milestone history |

## Conventions and limits

Distances are meters; angles are counterclockwise radians. Sensor +x is forward
and +y is left. The sensor and car frames coincide in this simulation.
`Pose2D` describes the source frame in a destination frame. See [frames](docs/frames.md).

The simulator is a 360-degree range filter over point landmarks, with no occlusion;
optional noise is independent per coordinate, a simplification of real LiDAR error,
which lies mostly along the beam. Visibility can change, so scan indices are not stable landmark
identities. The general estimator requires at least two correctly paired points,
not all coincident, and rejects geometrically ambiguous rotations. With noise it
returns the best fit to the given pairs, not the exact truth; it is not robust to
incorrect pairs. Small alignment error alone does not establish correct localization.
Geometry/simulation inputs are assumed finite; general numerical hardening remains
future work.

Tests cover transform inversion, frame conventions, sensor-coordinate output,
range boundaries, nearest neighbors, centroids, two-pair and multi-point relative
motion, angle wrapping, a fixed noisy least-squares example, noise statistics,
ICP convergence and failure cases, partial overlap, distance gating, and invalid inputs.
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
- [x] Iterative correspondence search and basic 2D ICP (noise-free examples)
- [ ] **Phase 1:** robust 2D localization — noise, partial overlap, gates,
      diagnostics, ray-cast LiDAR, scan tracking, occupancy-grid localization
- [ ] **Phase 2:** interactive sandbox v1 — engine in the browser (WebAssembly),
      home-robot scenario where visitors rearrange the world and watch localization
- [ ] **Phase 3:** 3D LiDAR odometry on KITTI, measured against a baseline (headline result)
- [ ] **Phase 4:** distributed evaluation sweeps with leases, retries, idempotent results
- [ ] **Phase 5:** replay explorer for real-data results
- [ ] **Later:** loop closure and pose graph, a separate planner module,
      street-car and warehouse-robot scenarios, challenge mode

See [docs/roadmap.md](docs/roadmap.md) for goals, done criteria, architecture,
and the product quality bar.

These are goals for an eight-week development effort, with scope reviewed as
measured progress accumulates.
