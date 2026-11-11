# Atlas

**Learning vehicle motion estimation and SLAM through a C++ implementation.**

Latest release: **v0.1.0 — 2D geometry and motion recovery with known correspondences.**
In progress (unreleased, roadmap Phase 1 complete): least-squares alignment, point-to-point
and point-to-line 2D ICP, ray-cast LiDAR, scan tracking, an occupancy-grid map, scan-to-map
localization with degeneracy handling, a scripted experiment, a simplified planner, and a live
browser sandbox running the C++ engine via WebAssembly.

Given two observations of stationary landmarks, Atlas estimates the translation
and rotation that express the second observation in the first sensor's frame.
The first milestone reproduces a controlled experiment: one meter forward and
a ten-degree left turn.

This is an educational foundation. It is not a complete SLAM system, any company's
stack, or vehicle-ready software. Real-data odometry, robust matching, 3D mapping,
loop closure, and distributed evaluation are planned work,
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
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/geometry.cpp src/simulation.cpp src/matching.cpp src/icp.cpp src/tracker.cpp src/planner.cpp src/mapping.cpp src/localizer.cpp src/experiment.cpp apps/demo.cpp -o atlas_demo
./atlas_demo
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/geometry.cpp src/simulation.cpp src/matching.cpp src/icp.cpp src/tracker.cpp src/planner.cpp src/mapping.cpp src/localizer.cpp src/experiment.cpp tests/test_atlas.cpp -o atlas_tests
./atlas_tests
c++ -O2 -std=c++17 -Iinclude src/*.cpp apps/experiment.cpp -o atlas_experiment
./atlas_experiment --seeds 5   # about 30 s; add --csv results/runs.csv for per-run rows
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
of a pose. Gate size must suit the initial-guess error. `checkAmbiguity()` restarts
ICP from nearby offsets and warns when a clearly different pose explains almost as
many points, as in the identical corridor. See [ICP](docs/icp.md).

`addGaussianNoise()` adds seeded, cross-platform reproducible Gaussian noise to scans.

`simulateLidarRanges()` ray-casts a planar LiDAR against wall segments, so walls
occlude what is behind them ([lidar](docs/lidar.md)). On such scans, point-to-line
ICP (`IcpMethod::PointToLine`) matches points to local wall lines and is several
times more accurate than point-to-point; it reports `Degenerate` in a featureless
hallway. `ScanTracker` chains scan-to-scan estimates; its drift is never corrected
without a map. A simplified A* planner (`planPath()`) plans on an inflated grid from the
true pose, separately from localization.

`OccupancyMap` builds a probabilistic grid map from scans at known poses (log-odds;
free, occupied, unknown) ([mapping](docs/mapping.md)). `MapLocalizer` registers each scan
against that map, so errors no longer chain from scan to scan. When the visible walls
leave one direction unconstrained (parallel walls), `translationObservability()` detects
it and the localizer keeps its prediction along that direction. `atlas_experiment` runs
scripted routes for several seeds and reports drift and failure flags
([localization](docs/localization.md)). Measured: with a 10 m sensor the localizer never
failed and was 10 to 20 times more accurate than the tracker; with a 3 m sensor both
failed in every seed.

## Browser sandbox

**Live: https://djism.github.io/atlas/** (published from `main` by CI).

`web/` is an interactive site where the same C++ engine, compiled to WebAssembly,
simulates the LiDAR, builds a map, localizes the robot against it, matches scans,
and plans routes live in a Web Worker. In "Get it lost", the map localizer and the
no-map tracker run side by side on the same scans, and visitors can move furniture
so the house no longer matches the map. The world is simulated. Every lab setup
has a shareable link. See [sandbox](docs/sandbox.md) to build and run it.

## Project layout

| Path | Purpose |
| --- | --- |
| `include/atlas/` | Public types and function declarations |
| `src/` | Geometry, LiDAR simulation and noise, matching, ICP, tracking, and planning |
| `apps/demo.cpp` | Reproducible known-pair, ICP, and mapping examples |
| `apps/experiment.cpp` | Scripted tracker-vs-localizer experiment with failure flags |
| `tests/test_atlas.cpp` | Numerical and invalid-input checks |
| `docs/frames.md` | Coordinate conventions and an explanation of the experiment |
| `docs/alignment.md` | Multi-point alignment derivation, degeneracy, and limits |
| `docs/icp.md` | ICP loop, point-to-line ICP, convergence example, and failure cases |
| `docs/lidar.md` | Ray-cast LiDAR model and first point-to-point findings |
| `docs/mapping.md` | Occupancy grid mapping: log-odds, beam traversal, findings |
| `docs/localization.md` | Scan-to-map localization, degeneracy handling, experiment results |
| `docs/sandbox.md` | Browser sandbox: build, architecture, measurements, limits |
| `wasm/` | Emscripten bindings and build script for the browser engine |
| `web/` | React + TypeScript + Vite site (Three.js rendering, engine in a Web Worker) |
| `docs/progress.md` | Daily milestones, validation, and next learning task |
| `docs/roadmap.md` | Phases, done criteria, target architecture, product quality bar |
| `.github/workflows/ci.yml` | Linux/macOS C++ build and test workflow (not the web build) |
| `CHANGELOG.md` | Versioned milestone history |

## Conventions and limits

Distances are meters; angles are counterclockwise radians. Sensor +x is forward
and +y is left. The sensor and car frames coincide in this simulation.
`Pose2D` describes the source frame in a destination frame. See [frames](docs/frames.md).

The point-landmark simulator is a 360-degree range filter with no occlusion; the
ray-cast LiDAR adds occlusion by walls. Optional noise is independent per coordinate, a simplification of real LiDAR error,
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
ICP convergence and failure cases, partial overlap, distance gating, ambiguity
detection, ray-cast hits and occlusion, normals, point-to-line ICP, tracking,
hallway degeneracy, planning, occupancy mapping, scan-to-map localization,
translation observability, the experiment harness, and invalid inputs.
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
- [x] **Phase 1:** robust 2D localization — noise, partial overlap, gates, ambiguity
      check, ray-cast LiDAR, point-to-line ICP, scan tracking, occupancy-grid map,
      scan-to-map localization, degeneracy handling, scripted experiment
      (known failure: short sensor range; holds with decent wheel odometry)
- [ ] **Phase 2:** interactive sandbox v1 — started: engine in the browser
      (WebAssembly), home-robot scenario with a live 3D lab, map localizer vs
      tracker in the lab, robot plans and steers from its estimate, simulated
      wheel odometry as the localizer's guess, shareable links, deployed to
      GitHub Pages; remaining: browser smoke test before publishing
- [ ] **Phase 3:** 3D LiDAR odometry on KITTI, measured against a baseline (headline result)
- [ ] **Phase 4:** distributed evaluation sweeps with leases, retries, idempotent results
- [ ] **Phase 5:** replay explorer for real-data results
- [ ] **Later:** loop closure and pose graph, a separate planner module,
      street-car and warehouse-robot scenarios, challenge mode

See [docs/roadmap.md](docs/roadmap.md) for goals, done criteria, architecture,
and the product quality bar.

These are goals for an eight-week development effort, with scope reviewed as
measured progress accumulates.
