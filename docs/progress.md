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

## 2026-11-06 to 2026-11-07 — ray-cast LiDAR, point-to-line ICP, tracking, planning, live sandbox

### Implemented

- Ray-cast 2D LiDAR: `Segment2D` walls, `simulateLidarRanges` (first hit per
  beam, so walls occlude), `rangesToPoints` into sensor axes. See [lidar](lidar.md).
- `estimateNormals` (local line fit per point) and point-to-line ICP
  (`IcpMethod::PointToLine`, one Gauss-Newton step per pass). New status
  `Degenerate` when the walls do not pin down the motion (hallway).
- `composePoses` and `ScanTracker`: chains scan-to-scan estimates with a
  constant-velocity guess; on failure it predicts and flags the scan lost.
- Grid planner (simplified): `OccupancyGrid` inflated by the robot radius, A*,
  and path smoothing; statuses Found / StartBlocked / GoalBlocked / NoPath.
- `wasm/bindings.cpp` + `build.sh`: the core compiled with Emscripten 6.0.13,
  exposing `simulateScan`, `alignScans`, `planRoute`, and a `Tracker` class.
- `web/`: React + TypeScript + Vite site in the approved "Monochrome" design.
  Hero with a live Three.js home, six-module summary, a draggable ICP figure,
  and a three-tab lab (match scans, get it lost, rearrange the house). The
  engine runs in a Web Worker. See [sandbox](sandbox.md).

### Lesson

Even noise-free wall scans have no exact point pairs: after moving, beams hit
different spots on the same walls. Point-to-line fixes this (360 beams: 3.3 mm
in 17 passes becomes 0.7 mm in 5). Chaining scans adds error without bound, and
the constant-velocity fallback can make it much worse: with 1.5 m range the
estimate ran 28 m away. Registering every scan is not the same as being right.

### Validation

Apple Clang, C++17, `-Wall -Wextra -Wpedantic`: all tests pass, demo runs.
Tests cover ray hits and misses, occlusion, normals on lines and corners,
point-to-line accuracy versus point-to-point, a tracker loop with no lost scans,
a hallway flagged `Degenerate` on every step, and planner cases (doorway open
and closed, U trap, blocked start/goal, diagonal squeeze). Web: typecheck and
production build pass. A headless Chrome tour (software WebGL) checked the hero,
the ICP figure (5 passes, 3.7 mm; re-converged after a drag; point-to-point 20
passes), the tracker (1.3 cm after 2.6 m), short-range failure, and replanning
around dropped boxes, with no console errors and no overflow at 390 px.

### Limitations

No map, so drift is never corrected. Planner uses the true pose and known floor
plan. CI builds only the C++; the web build and browser checks are local. Not
deployed. Mouse dragging in the hero was not tested by the automated tour.

### Next smallest step

Occupancy-grid map and scan-to-map localization: match each scan against a map
of the house, not only the previous scan, so drift is corrected.

## 2026-11-09 — occupancy map, scan-to-map localization, degeneracy, experiment (Phase 1 done)

### Implemented

- `OccupancyMap` (mapping.hpp/cpp): log-odds grid, exact beam traversal
  (Amanatides and Woo), clamping, three states, mean hit point per cell.
  See [mapping](mapping.md).
- `MapLocalizer` (localizer.hpp/cpp): registers each scan against map points,
  so the result is the pose in the map and errors do not chain. Jump check
  rejects answers more than 15 cm or 0.2 rad from the guess.
- `translationObservability()`: eigenvalues of mean(n n^T) over matched
  normals. When the weak one is below 0.05, the localizer keeps its prediction
  along the weak direction. ICP reports it; map normals are fitted once.
- `runExperiment()`, `circlePath()`, and `atlas_experiment`: scripted runs
  over paths, ranges, noise, and seeds with failure flags and CSV.
  See [localization](localization.md).

### Lesson

The map fixes drift only where the scan has information. With a 10 m sensor
the localizer never failed (worst 3 to 11 mm vs the tracker's 2 to 19 cm). In
a view of parallel walls ICP slid 25 cm and still reported Converged, and the
constant-velocity guess turned that slide into a runaway. Detecting the weak
direction stops the slide, but along it only the prediction is left, and that
drifts. One seed proves little: a 3.4 cm result was the best of 20 seeds;
3 of 20 failed by more than 1 m.

### Validation

Apple Clang, C++17, `-Werror`: all tests, demo, and experiment build and pass;
WASM engine and web build pass. Tests: single beams on 1 m cells, a room map
(occupied cells within 5 cm of walls, at least 95% wall coverage), a removed
box clearing, three laps with 3 cm noise (localizer within 2 cm, no growth),
a wrong start corrected, an unmapped box lowering inliers, a rejected jump,
observability on hand-checkable normals, the short-range path with and
without degeneracy handling, and the experiment harness. Mutation checks
caught: no-return end marked as hit, missing clamp, traversal that never steps
in y, disabled jump check, cell centers as map points, flipped projection
sign, strong eigenvector as weak, and ignoring the scan in weak steps. Not
caught: dropping the constant-velocity guess (not needed at 10 cm per scan).

Mistakes caught: a test probed the wrong side of a cell border; cell centers
biased the map by about 2 cm (fixed with mean hit points); my test path drove
through the box; the experiment first surveyed the map with the run's 3 m sensor.

### Limitations

Pose tracking only (needs the initial pose; no relocalization beyond the
0.3 m gate). Short range fails. Rotation observability is not analyzed. The
map is built at answer-key poses with no noise. Threshold 0.05 chosen on two
runs. Unoptimized test suite takes about 15 s.

### Next smallest step

Show the measured result in the sandbox: add the map localizer to the "Get it
lost" tab beside the tracker, then plan from the estimated pose (Phase 2).

## 2026-11-09 (session 2) — map localizer in the sandbox; doorway runaway fixed

### Implemented

- `surveyMap()`; WASM `buildMap` and `Localizer`; "Get it lost" shows the map
  localizer (blue) and the no-map tracker (gray) on the same scans, the map as
  pale points, and a map-match share. Furniture can be moved in that tab.
- Localizer: fit check (reject if under half the scan matches), heading
  recovery (score headings within 1 rad by inlier count, then ICP), jump
  limits 0.3 m and 0.5 rad.
- Sandbox sensor on simulated time with a bounded queue.

### Lesson

The browser found what scripted circles could not. At the doorway the robot
spins in place; a heading error of 0.3 rad pushes far wall points outside the
0.3 m pair gate, ICP converges wrongly, and with no fit check that answer was
accepted. The simulation also hid a timing bug: scans only when the previous
one finished, so a busy page lowered the sensor rate below 2 Hz. My first
checks were too short and too similar (same seed, same timing); varying GPU,
CPU speed, and window size reproduced the user's report.

### Validation

C++ doorway reproduction: 40 of 40 runs failed before, 0 of 40 after. Stop,
spin, then drive with slow scans: 6 to 10 of 10 failed, then 0. Regression
tests in the house layout fail if either the fit check or the recovery is
turned off. Browser: four 5-minute runs (normal, CPU 2x and 3x slower, other
window sizes) with 0 lost scans. Experiment 10 m rows unchanged; 3 m rows
still fail (one improved from 22 m to 16 cm worst). Engine time per scan with
both estimators: about 21 ms median in headless Chrome.

### Limitations

Short range still fails. The jump limits and constant-velocity guess are per
scan, not per second. Recovery searches heading only. The sandbox map is
surveyed at answer-key poses and never updated. Brute-force nearest neighbors.

### Next smallest step

Plan from the estimated pose, or first simulate wheel odometry as the motion
prediction so spins and sudden starts no longer depend on guessing.

## 2026-11-10 — the sandbox robot plans and steers from its estimate

### Implemented

- The robot's belief is the latest map-localizer estimate composed with the
  motion commanded since that scan was captured (perfect odometry for now).
- The planner starts from the belief and the robot steers from it; commands
  move the true robot. A simulated bumper stops it before a wall or box, counts
  a bump, and triggers a replan from the belief.
- One tracker/localizer pair per view in the worker (hero and lab no longer
  share estimators). The robot waits ("Localizing") until its first estimate.

### Lesson

Before this, the planner used the answer-key pose, so localization had no
consequences. Now a wrong estimate makes the robot drive wrongly, which is
what "Get it lost" is meant to show.

### Validation

Browser: four 5-minute runs (normal, CPU 2x and 3x slower, two window sizes)
with 0 bumps and 0 lost scans; planner tab with 3 dropped boxes replanned 9
times with 0 bumps; at 1.5 m range the estimate was 5.8 m off and the robot
bumped ("it was not where it thought"). Typecheck and production build pass.
No browser checks run in CI yet.

### Limitations

The planner is given the true floor plan, not the robot's own map. Odometry
is perfect. A badly lost robot keeps pushing after a bump (no back-up
behavior). Short range still fails.

### Next smallest step

Simulated wheel odometry with slip and noise, used as the localizer's motion
prediction instead of constant velocity.

### Daily checkpoint template

- Concept understood:
- Behavior implemented:
- Verification and result:
- Current limitation:
- Next smallest step:
