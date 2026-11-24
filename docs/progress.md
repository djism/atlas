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

## 2026-11-10 (session 2) — wheel odometry as the localizer's motion prediction

### Implemented

- C++: `relativePose()`, `simulateOdometry()` (assumed 2 % bias, 5 % random
  distance and turn error, 0.02 rad per meter), `MapLocalizer::update(scan,
  motion)`, `waypointPath()`, odometry and dead reckoning in the experiment,
  a second `atlas_experiment` table and `--odometry-scale`.
- Sandbox: each scan request carries the true motion since the previous
  tracked scan; the worker turns it into a noisy wheel reading in C++ and only
  that reaches the localizer. "Get it lost" has a Wheel odometry / Repeat last
  motion switch.

### Lesson

The constant-velocity guess is built from the localizer's own past answers,
so one wrong answer becomes a wrong next guess (feedback). Wheels are an
independent measurement. Two failures found by measuring: a turn starting
from standstill (ICP from a no-turn guess accepted a pose 13 cm and 0.24 rad
off at 59 % fit), and runaway with a 3 m sensor. Dead reckoning alone drifts
20 cm to 0.9 m, so wheels do not replace the map.

### Validation

`atlas_experiment --seeds 5`: 3 m circles failed 10 of 10 with constant
velocity, 0 of 10 with odometry (worst 7 cm); stop-and-go at 10 m failed 5 of
5 (worst 10.1 m) vs 0 of 5 (3 mm). With 3x worse odometry the 3 m rows fail
again at 20 to 28 cm. Regression test: sudden turn from standstill (constant
velocity above 5 cm in 18 of 20 seeds; odometry within 3 mm in 20 of 20).
Mutation checks: ignoring the motion, dropping heading drift, and the
float-accumulation step bug in `waypointPath` are each caught. Browser: 3 m
and 1.5 m runs, normal and CPU 2x slower: wheels worst 15 to 28 cm with 0
bumps; last motion 4.2 to 20.5 m off with up to 5 bumps. Default range, 3 x
5 min: 0 lost scans, 0 bumps.

### Limitations

Odometry noise values are assumptions. The localizer uses odometry only as
the guess; it does not weight odometry against the scan (no filter). Between
scans the sandbox belief adds exact commanded motion. The tracker still uses
constant velocity. Short range is only as good as the wheels.

### Next smallest step

URL state for the sandbox, then deployment and a browser smoke test in CI;
then Phase 3 (KITTI).

## 2026-11-10 (session 3) — shareable links and GitHub Pages deployment

### Implemented

- Shareable lab links: `?lab=lost&range=1.5&guess=last&room=sofa:1.20,3.50;box:3.20,4.10&mapped=`.
  `room` is the furniture now, `mapped` the furniture when the map was
  surveyed; the Field snapshots the house at each restart and `loadLayout()`
  surveys from `mapped`, then arranges `room`. "Copy link" button; the address
  bar follows changes via `history.replaceState`.
- Pages workflow: pinned emsdk commit and Emscripten 6.0.13 (cached), engine
  built from committed C++, `npm test`, `vite build --base /atlas/`, deploy.

### Lesson

A link must carry what makes the experiment, not just what is on screen: the
same moved furniture gave 51 % map match with `mapped` and 89 % without, so a
single layout would have silently changed the result. Without WebGL the page
first erased a shared layout from the URL (no Field to read it from); fixed.

### Validation

9 codec unit tests (round trips, defaults omitted, hand-edited and garbage
links, percent-encoded links); a mutation of the `mapped` default is caught.
Browser: six links set the right tab, sliders, guess, and furniture; bad
links are cleaned; Copy link writes the URL. Production build served under
`/atlas/` locally passes the same checks; a clean copy of the tree runs every
workflow step. 0 console errors.

### Limitations

A link reproduces the setup, not the exact run (asynchronous engine results
and machine timing). No browser test runs before publishing.

### Next smallest step

Browser smoke test in the Pages workflow, gating deployment; then Phase 3.

## 2026-11-11 — browser smoke tests gate the deploy

### Implemented

- Playwright 1.64.1 (pinned, project-local, drives the installed Chrome; no
  browser download). Six tests against the production build under `/atlas/`;
  any console error fails. Runs in the Pages workflow before publishing;
  failed runs keep the report for 14 days. `npm run build:pages`,
  `npm run test:e2e`; `SMOKE_NO_GPU`, `SMOKE_CPU_SLOWDOWN` rehearse CI.
- Separate TypeScript config for the tests, so app code never sees Node globals.

### Lesson

The tests found two bugs my hand-written tours missed: a 404 for the tab icon
on every load (those tours listened for script errors, not failed requests),
and a per-frame crash in the ICP figure with software WebGL (negative elapsed
time). A fast GPU hid it. Also: a single reading of map match let a deliberate
break pass on a momentary dip; the tests now judge 10 s of readings.

### Validation

Deliberate breaks: engine disabled fails 5 of 6; shared links mapping the
moved house fails on map match (92 %). Software WebGL, and software WebGL with
a 3x slower CPU: 6 of 6. Clean copy with the exact workflow steps and CI=1:
6 of 6 in 2.4 min.

### Limitations

Chrome only; runs on pushes to `main`, not pull requests. Loose bounds: they
catch a broken site, not small accuracy changes. Phase 2 still lacks
residual/inlier charts and exact replay; no real first-visitor test.

### Next smallest step

Phase 3: get KITTI odometry data loading (outside Git), read one Velodyne
scan into C++, and visualize it, before any 3D ICP.

## 2026-11-11 to 2026-11-23 — Phase 3: KITTI, first 3D odometry, deskewing, bias diagnosis

### Implemented

- KITTI data (raw drive 2011_09_30_drive_0016 = odometry sequence 04, 541 MB
  kept outside the repo; the 80 GB benchmark set did not fit in 34 GB free).
- Readers for scans, poses, calibration, times; camera-to-Velodyne conversion
  `Tr^-1 * T * Tr`; top-down images; overlap and map checks.
- Voxel-hash nearest neighbors, point-to-point 3D ICP (Gauss-Newton),
  scan-to-scan odometry, trajectory comparison; deskewing.

### Lessons

- A 0.37 deg camera-Velodyne tilt moves the end of a 394 m drive by 2.5 m;
  my prediction ignoring it was wrong.
- Over all points, "no motion" overlapped as well as the truth (road and
  hedges along a straight road): the hallway problem in 3D.
- My first synthetic ICP test moved the same points, so ICP "solved" a pure
  hallway; real lasers hit different spots. Identical motions per frame hid a
  chaining-order bug. Test bounds at the median of 20 seeds would have flaked.
- Deskewing was my leading suspect for the drift and explains about a tenth.
  ICP started at the truth walks away: raised points pull +2.3 cm forward and
  +0.019 deg heading per pair (matching the odometry's bias), road rings pull
  backward. Point-to-point matching is biased by sampling patterns.

### Validation

Reader matches an independent Python decode; all 279 scans read; median
height -1.50 to -1.79 m in every scan (frame check). Ground-truth map sharp.
Spin direction: clockwise deskew better in 14/14 pairs 5 frames apart,
counterclockwise worse in 14/14. Sequence 04 odometry: 6.99 % drift raw,
6.49 % deskewed, 6.36 % oracle (1 m thinning); 4.69 / 4.04 / 3.97 % at 0.5 m;
48 to 137 ms per scan on one Apple M4 core. Unit tests with hand-made data;
deliberate breaks caught for frame conversion, Jacobian sign, chaining order,
spin direction, deskew reference time, rotation log, BMP row order.

### Limitations

One sequence, and parameters looked at on it. Scan-to-scan only (no local
map), point-to-point, no robust weights, no KITTI segment metric, no baseline
run yet. Not comparable to published numbers (those use compensated scans).

### Next smallest step

Point-to-plane 3D ICP; success test set in advance: `icp-from-truth` stops
walking away, then rerun the odometry.

## 2026-11-23 (session 2) — scan-to-map odometry matches KISS-ICP on sequence 04

### Implemented

- KITTI scan correction; local voxel map; robust map ICP; scan-to-map
  odometry with adaptive threshold (KISS-ICP's design, Atlas's code);
  devkit metric; KISS-ICP 1.3.0 baseline script (isolated venv outside the
  repo); `score` command; profile and memory; bulk scan reading.

### Lessons

- Reading KISS-ICP's source beat guessing: it is point-to-point too; its gains
  come from the local map, robust weights and adaptive threshold. My plan to
  build point-to-plane next would have been the wrong step.
- A divergence that looked like a deskew bug was floating-point rounding
  amplified 2.5x per frame by a compose/inverse feedback loop (transpose as
  inverse). A synthetic turning-sensor test cleared deskewing; printing the
  orthonormality error per frame found the cause.
- Verify ports against the original: the metric was checked against the
  devkit's own functions; the scan correction against KISS-ICP's to 1e-14.
- Profile before optimizing: 62 % nearest neighbors (expected), 11 % reading
  files one char at a time (a mistake, fixed 8.6x).

### Validation

Sequence 04: Atlas 0.48 % t_err, 0.12 deg/100 m, end 1.49 m, 1.7 cm per-frame
median, 43 ms per scan one thread, 79 MB; KISS-ICP 0.41 %, 0.19 deg/100 m,
0.48 m, 2.5 cm, 71 ms one thread / 19 ms all cores, 148 MB. Removing deskew
or the correction: 1.07 % / 2.05 % end drift. Voxel 0.75/1.0/1.5 m: 0.48 /
0.48 / 0.52 %. New unit tests and deliberate breaks for each piece.

### Limitations

One sequence (394 m, country road), design and defaults from KISS-ICP, no
held-out check yet. Odometry, not SLAM. Single-threaded.

### Next smallest step

Download a second sequence (07: raw drive 2011_09_30_drive_0027, urban,
frames 0-1100) and run Atlas and KISS-ICP on it with no changes.

## 2026-11-24 — held-out sequence 07: Phase 3 done

### Implemented

- Downloaded and extracted sequence 07 (raw drive `2011_09_30_drive_0027`,
  1101 frames, 694 m urban loop); scans and GPS only.
- `trajectorySvg()` (image module) and `atlas_kitti plot`: top-down paths of
  estimates over the ground truth, equal axes, oriented like the bird's-eye
  images. Plots for sequences 04 and 07 in `results/` (not in Git).

### Lessons

- A held-out check is only honest if nothing changes: same commands, same
  defaults, both methods, decided before the data arrived.
- End error flatters a loop: sequence 07 returns near its start, so
  scan-to-map's 0.80 m end error (0.12 %) hides a 0.43 % segment drift.
- Measure memory with the program shape that matters: the evaluation CLI
  peaks at 1.2 GB because it keeps every scan; streaming odometry needs 106 MB.
- Check the date in a KITTI drive name: `2011_09_26_drive_0027` is a
  different drive from `2011_09_30_drive_0027` and has no ground truth.

### Validation

Sequence 07, no changes: Atlas scan-to-map 0.43 % t_err, 0.25 deg/100 m,
end 0.80 m, 1.9 cm per-frame median, 28 ms per scan median on one M4 core,
106 MB; KISS-ICP 1.3.0 0.75 %, 0.37 deg/100 m, 0.72 m, 2.5 cm, 47 ms one
thread / 15 ms all cores, 195 MB. Atlas scan-to-scan 6.32 % raw, 5.46 %
deskewed. Atlas's mean step error is lower than KISS-ICP's in all eleven
100-frame blocks. Sequence 04 streaming rerun reproduced 43 ms and 78 MB.
Plot tests pass; mirroring the plot fails two of them.

### Limitations

Two sequences, 1.1 km; one win each against KISS-ICP, so no ranking. The
cause of KISS-ICP's larger error on 07 is not identified. Odometry, not SLAM.
Raw scans, so not comparable to the official leaderboard.

### Next smallest step

Phase 4: a coordinator and workers that run (sequence, configuration) jobs
like the two above, with leases, retries and durable, idempotent results.
More sequences (e.g. a highway one) would make the Phase 4 sweep meaningful.

## 2026-11-24 (session 2) — real-data results on the website

### Implemented

- "Real data" section after "What I built": drive switch (07 held out, 04
  tuned on), top-down paths with on/off chips, results table, per-stretch
  step-error bars, three notes, credits. Labeled "recorded offline, not
  running live in this page"; footer now says which parts are live.
- `tools/export_web_results.py` (standard library only) writes
  `web/src/data/kittiPaths.ts`; `web/src/lib/kittiPlot.ts` fits paths on
  equal axes. Approved mock-up first (claude.ai design canvas).

### Validation

5 new unit tests (orientation, equal scale, fit and centering, start marker
and scale bar, bad input); flipping the vertical axis fails 2, removing the
centering fails 1. New smoke test (switch drives, verdict, bar counts, a
hidden path); all 7 smoke tests pass locally. The standard-library export
reproduces the numpy per-stretch errors exactly. Checked by screenshot at
1440 px and 390 px: no sideways scroll, no console errors.

### Limitations

Table numbers are typed from docs/kitti.md, not generated; paths are every
third pose. Static recorded results, not the Phase 5 replay explorer.

## 2026-11-24 (session 3) — Phase 4: distributed evaluation

### Implemented

- `eval/`: Store (SQLite job state + SHA-256-named result files), HTTP
  coordinator, workers with heartbeats, sweep submitter with backpressure,
  report from stored results, crash/recovery demo; `atlas_kitti run`.
- CI job "eval" (Linux) runs the 19 Python tests.

### Lessons

- The first real run "passed" (24/24, one result each) and still had a bug:
  the logs showed a healthy job cancelled. Read the logs, not only the
  verdict. Cause: a Python closure read loop variables late, so a finished
  job's heartbeat cancelled the next job. Reproduced with slow heartbeats in
  a test (6 of 10 jobs abandoned) before fixing.
- Correct is not the same as efficient: an outage longer than a lease is
  handled safely by the rules (expire, re-run) but wastes work; restart grace
  fixes that.
- Standard HTTPServer does a reverse-DNS lookup on bind that hung for tens
  of seconds here; the restarted coordinator looked dead.
- Leases + fencing give at-least-once execution with one accepted result per
  job; a paused worker really did finish its run, and was refused.

### Validation

KITTI sweep, 24 jobs, 227 s on one M4: kill -9 w1, kill -9 coordinator and
restart, SIGSTOP w2 for 14 s (lease 6 s). All 24 done; 26 attempts; exactly
2 expired leases; w2's late heartbeat and result refused; accuracy identical
to the first run. Method findings: correction matters most on 04 (0.48 ->
1.07 %), deskew on 07 (0.43 -> 0.76 %); default best or tied on both.
Tests: 19 pass; 8 deliberate breaks each caught.

### Limitations

One coordinator (restartable, not replicated), no authentication, workers
were processes on one machine, wall-clock leases, orphaned engine runs after
kill -9, no garbage collection of rejected results.

### Next smallest step

Phase 5: a replay explorer for these recorded results (clearly labeled
replay), or more KITTI sequences for the sweep.

## 2026-11-24 (session 4) — Phase 5: replay explorer

### Implemented

- Mock-up from real data (frame 476 of sequence 07), approved; built as a
  second page of the site with Three.js, React and pure helpers.
- C++: `run ... diag=` per-frame log (poses unchanged, byte for byte) and
  `export-replay` (Atlas's own map, thinned scans). `tools/export_replay.py`.
- Home-page error bars and an "Open the replay explorer" button link in.

### Lessons

- Looking frame by frame finds what averages hide: Atlas's worst steps are
  sharp turns (7-8 cm at frames 476 and 916) while 99 %+ of points still
  match, so inlier counts would not flag them; KISS-ICP's worst is a fast
  straight (frames 247-267, up to 20 cm).
- Draw the map from the estimator's own poses, not the truth: then the map
  shows the drift the robot really had.
- Colour carries meaning: a strong-blue map hid the live scan; a quiet
  blue-gray map lets the scan (the only bright blue points) read at once.

### Validation

8 new unit tests (decoding, consistency, links, charts, notable frames,
headlines, committed data); 2 browser tests extended or added (page,
home-page link, phone width), passing with a GPU and with software WebGL on
a 3x slower CPU. All 8 smoke tests and 22 unit tests pass.

### Limitations

Replay only; scans thinned to 640 points; not deskewed in the view; two
sequences; about 9.8 MB of derived data in Git.

### Next smallest step

Phase 6 candidates: loop closure on sequence 07 (it ends 9.5 m from its
start), more KITTI sequences in the sweep, or Phase 2 polish (charts in the
lab, a first-visitor test).

## 2026-11-24 (session 5) — Phase 2 polish: charts and exact replay

### Implemented

- Lab charts (mock-up from real lab readings, approved): pass-by-pass
  residual vs true error; 35 s error history (log scale) and map match.
- Lockstep simulation: fixed 20 ms ticks of simulated time, scan answers and
  routes take effect at a fixed simulated delay, the world waits for late
  answers, bump/replan timing in ticks, restart rewinds the robot and seeds.
- `?trace=1` run recording; docs/visitor-test.md for the human test.

### Lessons

- Determinism under async work: do not apply results when they arrive; apply
  them at a time fixed in simulated time, and make time wait if they are late.
- A test for "the same" must be able to fail: applying answers on arrival
  made the exact-replay test fail; a first attempt at that check did not even
  compile, so it silently tested the old build. Check the build step too.
- Residual is not error: in the ICP demo the pair RMSE stops at 1.1 cm while
  the true error is 3.7 mm. The chart now shows both.
- Errors span mm to m in the lab: only a log scale shows both estimators.

### Validation

5 new unit tests (27 total); smoke tests extended (pass bars, history lines)
and a new exact-replay test (150 scans identical at normal speed and with a
3x slower CPU); all 9 smoke tests pass with a GPU and with software WebGL on
a 3x slower CPU.

### Limitations

The first-visitor test still needs real people. Exact replay holds only
without interaction (dragging is not recorded).

### Next smallest step

Run docs/visitor-test.md with three people; or Phase 6 (loop closure on
sequence 07).

## 2026-11-24 (session 6) — Phase 6 step 1: pose-graph optimizer

### Implemented

- `posegraph3d.hpp/cpp`: edge residual (measured^-1 * relative pose),
  Levenberg-Marquardt over all poses but the first, numerical Jacobians,
  dense Cholesky. Added to CMake.

### Lessons

- A pose graph is springs: each measurement pulls; the weights say how stiff.
  Two disagreeing measurements end at their weighted average.
- Why loop closures must be verified: one wrong loop bends the whole map.
- The first pose must be fixed, or the problem has no unique answer.
- A test's expected value can be the bug: my hand-computed Cholesky answer was
  wrong; an exact fraction solve settled it.

### Validation

Synthetic loop: 3.47 m RMS drift to 0.034 m in 5 iterations; consistent
graph unchanged; weighted averages exact; wrong loop over 3x worse than the
right one. Four deliberate breaks each caught (one only after adding a test).

### Next smallest step

Keyframes every few metres on sequence 07 and loop candidates (old keyframes
near the current one), printing what is found before any optimization.

## 2026-11-24 (session 6, step 2) — keyframes and loop candidates

- `loopclosure3d.hpp/cpp`: `selectKeyframes` (5 m / 15 deg), `pathLengths`,
  `findLoopCandidates` (nearest earlier keyframe within 10 m in the estimate,
  at least 50 m back along the path). `atlas_kitti loops`.
- Sequence 07: 143 keyframes, 4 candidates, all at the end of the drive
  returning to its start, all truly within 8.5 m. Sequence 04: none.
- Lesson: my first grading was circular (real = within the search radius);
  the real test of a loop is whether its scans align to the right pose.
- Tests for keyframes and candidates; dropping the path-gap rule fails three.

### Next smallest step

Verify each candidate: ICP between the later keyframe's scan and a local map
around the earlier keyframe, starting from the odometry guess; accept on
fitness; grade the resulting relative pose against the answer key; show a
deliberately wrong candidate rejected.

## 2026-11-24 (session 6, step 3) — loop verification

- `buildSubmap`, `verifyLoop` (two-pass ICP, fit on structure above the road);
  `atlas_kitti verify-loops` with a decoy per candidate; saves accepted loops.
- Sequence 07: 4 of 4 loops accepted, 4 of 4 decoys rejected (fit 77-95 % vs
  9-14 %). Verified relative poses 0.04-0.36 m from the answer key (guess
  0.72-0.80 m).
- Lessons: road points match anywhere, so judge fit on structure; a suspected
  cause (road rings) was tested, did not help, and was reverted; the four
  loops agree with each other to 2-9 cm, and Atlas and KISS-ICP agree with
  each other against the answer key (0.43 m sideways over 15 m at the end of
  the drive): the answer key has local decimetre errors.

### Next smallest step

Build the pose graph (keyframes, odometry edges, the four loops), optimize,
and measure before/after: absolute trajectory error, KITTI metric, end error.
Choose edge weights from measured noise, not by hand.

## 2026-11-24 (session 6, step 4) — loop closure measured

- `closeLoops()`, `atlas_kitti close-loops`. Weights from sequence 04's drift.
- Sequence 07 (held out): position RMS 1.13 -> 0.81 m, worst 2.10 -> 1.40 m,
  end 0.80 -> 0.11 m, KITTI t_err 0.43 -> 0.42 %. Sequence 04 unchanged.
- One wrong loop (shifted 5 m) among three right ones: end 1.24 m, t_err
  0.49 %: worse than no loop closure.
- Lesson: a loop at the end fixes the end best; a mid-drive bulge needs more
  loops. The KITTI metric is the wrong lens for loop closure; position error
  is the right one.

### Next smallest step

Show it: loop closure in the replay explorer (estimate before and after,
the four loops drawn), mocked up first. Or a robust back end (switchable
loop edges) tested with the wrong-loop case.

### Daily checkpoint template

- Concept understood:
- Behavior implemented:
- Verification and result:
- Current limitation:
- Next smallest step:
