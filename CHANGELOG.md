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

- Phase 3 started (KITTI, raw drive 2011_09_30_drive_0016 = sequence 04):
  `readKittiScan()`, `readKittiPoses()`, `readKittiVelodyneInCamera()`,
  `readKittiTimes()`, `velodynePoses()` (camera to Velodyne frame),
  `summarizeScan()`; `Point3D`, `Pose3D` and 3D transforms (`*3D` names);
  `birdsEyeView()` and `writeBmp()`; `nearestDistances()`.
- Added `voxelDownsample()` and `VoxelGrid3D` (exact nearest neighbor within
  the voxel size), `alignIcp3D()` (point-to-point, Gauss-Newton),
  `ScanOdometry3D` and `compareTrajectories()`.
- Added `rotationVector()`, `scaleMotion()`, `sweepFraction()`, `deskewScan()`
  and `ScanOdometryOptions::deskew` (clockwise sweep, measured).
- `atlas_kitti` CLI: `scan`, `truth`, `overlap`, `map`, `odometry`,
  `deskew-check`, `icp-from-truth`. Documented in `docs/kitti.md`.
- `.gitignore`: `/Datasets/`.
- Added `correctKittiScan()` (KITTI's 0.205 deg elevation correction),
  `orthonormalized()`, `LocalMap3D`, `alignToMap()` (robust point-to-point),
  `MapOdometry3D` (scan-to-map odometry: deskew, two thinnings, adaptive
  threshold, local map), `kittiMetric()` (devkit metric), `cameraPoses()`,
  `writeKittiPoses()`; `VoxelGrid3D` index and radius queries.
- `atlas_kitti`: `score`; `odometry` now runs scan-to-scan and scan-to-map,
  reports the KITTI metric, and saves poses. `tools/kiss_icp_baseline.py`.
- Sequence 04: Atlas scan-to-map 0.48 % / 0.12 deg per 100 m vs KISS-ICP
  0.41 % / 0.19 on the same raw scans.
- Fixed: odometry poses lost orthonormality through a compose/inverse
  feedback loop (rounding grew 2.5x per frame, diverging at frame 44).
  `readKittiScan` now reads in one block (9.5 -> 1.1 ms per scan).
- Added `trajectorySvg()` and `atlas_kitti plot` (estimated vs true paths,
  top-down, equal axes).
- Held-out sequence 07 (694 m, nothing changed): Atlas scan-to-map 0.43 % /
  0.25 deg per 100 m, 28 ms per scan, 106 MB; KISS-ICP 0.75 % / 0.37. Phase 3 done.
- Website: "Real data" section with the recorded KITTI results (drive switch,
  path toggles, results table, per-stretch step errors), labeled as recorded.
  `tools/export_web_results.py` (standard library only) writes its path data;
  `web/src/lib/kittiPlot.ts` lays it out (unit-tested); a smoke test covers it.
- Phase 4: distributed evaluation in `eval/` (Python standard library):
  coordinator (HTTP + SQLite), workers, bounded queue with backpressure,
  leases with heartbeats and fencing tokens, retries with exponential backoff
  and jitter, content-addressed durable results, restart grace, report from
  stored results, and a crash/recovery demo. `atlas_kitti run` (one
  configured run, JSON output). 19 tests (CI job "eval", Linux).
- KITTI sweep (24 jobs) under a worker crash, a coordinator crash and
  restart, and a paused worker: all done, one accepted result each. Default
  settings best or tied-best on both sequences.
- Fixed (found by that sweep): a finished job's late heartbeat could cancel
  the worker's next job; a coordinator outage longer than a lease re-ran
  healthy jobs (now restart grace).

## 0.1.0 — 2026-11-04

- Added C++17 2D point and pose types with explicit frame conventions.
- Added forward/inverse rigid transforms and ideal range-limited scan simulation.
- Added nearest-point search and a two-known-pair relative pose estimator.
- Added deterministic demo, numerical checks, CMake build, and GitHub Actions CI.
- Documented scope, daily milestones, and planned progression toward SLAM.
