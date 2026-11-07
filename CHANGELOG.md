# Changelog

## Unreleased

- Added `computeCentroid()` for nonempty 2D point sets.
- Added `estimateFromMatchedPoints()`: least-squares rigid alignment from two or
  more known pairs, with scale-aware rejection of coincident or ambiguous input.
- Added `computeMatchedRmse()` for known-pair alignment error.
- Demo now uses the general estimator; `estimateFromTwoPairs()` remains as a baseline.
- Tests cover centroids, multi-point recovery, angle wrapping, reordering,
  collinear input, invalid input, and a fixed noisy least-squares example.
- Documented the derivation and limitations in `docs/alignment.md`.
- Added a one-wrong-pair experiment test showing a confident, badly wrong pose.
- Added `findNearestCorrespondences()`: nearest-neighbor pairing under a guessed pose.
- Added `alignIcp()` (point-to-point 2D ICP) with per-pass poses, nearest-pair
  RMSE, and convergence reporting, in `icp.hpp`/`icp.cpp`.
- Tests and demo cover ICP convergence, a large-rotation local minimum, and an
  identical-corridor failure with zero residual. Documented in `docs/icp.md`.
- Added `addGaussianNoise()`: seeded Gaussian noise via Box-Muller over
  `std::mt19937_64`, reproducible across standard libraries; statistical tests.
- Tests and docs show partial overlap: a point seen only in the second scan
  biases ICP, while an extra first-scan point is harmless here.
- **Changed ICP API:** `IcpResult::converged` is now `converged()`, backed by
  `IcpStatus` (`Converged`, `HitIterationLimit`, `TooFewPairs`).
- Added a distance gate (`IcpOptions::max_pair_distance`, off by default) and
  `min_pairs`; per-pass kept-pair counts; gate-size experiments in `docs/icp.md`.
- Added `docs/roadmap.md`: phases, done criteria, architecture, product quality bar.
- Added `countInliers()` and `checkAmbiguity()`: ICP restarts from offsets flag a
  different pose that explains almost as many points (corridor aliasing, wrong
  local minima). Demo prints the corridor warning; documented in `docs/icp.md`.
- Added ray-cast 2D LiDAR: `Segment2D`, `simulateLidarRanges()` (first hit per
  beam, with occlusion), `rangesToPoints()`. Documented in `docs/lidar.md`.
- Added `estimateNormals()` and point-to-line ICP (`IcpMethod::PointToLine`);
  new `IcpStatus::Degenerate` when walls leave motion unconstrained.
- Added `composePoses()` and `ScanTracker` (scan-to-scan tracking with a
  constant-velocity guess and lost-scan flags).
- Added a simplified grid planner: inflated `OccupancyGrid`, A*, path smoothing.
- Added `wasm/`: Emscripten bindings (`simulateScan`, `alignScans`, `planRoute`,
  `Tracker`) and a build script.
- Added `web/`: React + TypeScript + Vite site with a live Three.js home, a
  draggable ICP figure, and a three-experiment lab, all calling the C++ engine
  in a Web Worker. Documented in `docs/sandbox.md`.

## 0.1.0 — 2026-11-04

- Added C++17 2D point and pose types with explicit frame conventions.
- Added forward/inverse rigid transforms and ideal range-limited scan simulation.
- Added nearest-point search and a two-known-pair relative pose estimator.
- Added deterministic demo, numerical checks, CMake build, and GitHub Actions CI.
- Documented scope, daily milestones, and planned progression toward SLAM.
