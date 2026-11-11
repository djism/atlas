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
- Added `OccupancyMap`: log-odds occupancy grid built from LiDAR scans at known
  poses, with exact beam traversal, clamping, and mean hit points per cell
  (`occupiedPoints()`). Demo prints a text map. Documented in `docs/mapping.md`.
- Added `MapLocalizer`: scan-to-map localization with a constant-velocity guess,
  inlier fraction, and a motion sanity check that rejects jumps.
- Added `translationObservability()` and `IcpResult::observability`; the
  localizer keeps its prediction along a direction the scan cannot constrain.
- Added an `alignIcp` overload that reuses precomputed first-scan normals.
- Added `runExperiment()`, `circlePath()`, and the `atlas_experiment` CLI:
  tracker vs localizer over paths, ranges, noise levels, and seeds, with
  failure flags and optional CSV. Documented in `docs/localization.md`.
- Added `surveyMap()`: builds a map from survey scans; used by the experiment
  and the browser.
- **Changed localizer behavior:** results that fit less than half of the scan
  are rejected (`min_inlier_fraction`); failed scans trigger a heading search
  scored by `countInliers()` (`recovered`, `poor_fit` in `LocalizerStep`); jump
  limits are now 0.3 m and 0.5 rad (were 0.15 m and 0.2 rad).
- WebAssembly: `buildMap` and a `Localizer` class.
- Sandbox "Get it lost" tab: map localizer (blue) beside the no-map tracker
  (gray), the map drawn as points, furniture editable, map-match metric.
- Sandbox sensor now runs on simulated time (a scan every 0.1 s of simulated
  motion; the world waits if the engine is behind).
- Sandbox robot plans and steers from its estimated pose (map estimate plus
  commanded motion since the scan), with a simulated bumper and replanning;
  one estimator pair per view in the worker.
- Added `relativePose()` (geometry) and `simulateOdometry()` with
  `OdometryNoise` (simulation): seeded wheel odometry with bias and noise
  proportional to distance and turn.
- Added `MapLocalizer::update(scan, motion)`: the guess comes from a measured
  motion (wheel odometry) instead of constant velocity.
- Experiment: `waypointPath()` (stop, turn in place, drive), odometry per run,
  dead-reckoning score, and a second CLI table comparing motion predictions;
  `--odometry-scale`. Documented in `docs/localization.md`.
- WebAssembly: `Localizer.updateWithMotion` and `simulateOdometry`. Sandbox
  feeds simulated wheel odometry to the map localizer; "Get it lost" can
  switch to constant velocity for comparison. Lab panel no longer overflows
  the stage on short windows.
- Sandbox: shareable links. The address bar holds the lab setup (tab, sensor,
  motion guess, furniture now and when the map was surveyed); "Copy link".
  Codec in `web/src/lib/share.ts` with unit tests (`npm test`, Node's runner).
- Added `.github/workflows/pages.yml`: builds the engine with pinned
  Emscripten 6.0.13 and publishes the sandbox to GitHub Pages.
- Playwright smoke tests (`web/e2e`, installed Chrome, no browser download)
  gate the Pages deploy: hero, ICP figure, map localizer, shared link, planner,
  phone layout; any console error fails.
- Fixed: the ICP figure crashed every frame on slow (software WebGL) machines
  when an engine answer arrived after the frame's timestamp. Added a tab icon
  (every load logged a 404 for `/favicon.ico`).

## 0.1.0 — 2026-11-04

- Added C++17 2D point and pose types with explicit frame conventions.
- Added forward/inverse rigid transforms and ideal range-limited scan simulation.
- Added nearest-point search and a two-known-pair relative pose estimator.
- Added deterministic demo, numerical checks, CMake build, and GitHub Actions CI.
- Documented scope, daily milestones, and planned progression toward SLAM.
