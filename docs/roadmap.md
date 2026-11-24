# Atlas roadmap

Atlas aims to be one coherent product with three faces:

1. **Engine** — a tested C++ localization and mapping core (2D, then 3D LiDAR).
2. **Evidence** — measured accuracy and runtime on public driving data, produced by
   a distributed evaluation system.
3. **Experience** — an interactive web sandbox where visitors change a robot's world
   and watch localization hold or fail, running the real engine in the browser.

Every phase ends with something working, tested, documented, and demonstrable.
Scope is reviewed weekly; depth is protected over breadth.

## Product principles

- **Honest labels.** Every view says whether it is *live* (engine running in the
  browser), *replay* (recorded outputs), or *simulated* data. Localization is
  never presented as planning. "Converged" is never presented as "correct".
- **Measured claims only.** Accuracy, runtime, and memory are reported with
  hardware, dataset, and configuration. Failures are documented, not hidden.
- **One engine.** The sandbox, the evaluation CLI, and the distributed workers all
  call the same C++ core. No reimplementation in JavaScript.
- **Modular by design.** A scenario is data plus small plugins, so new worlds are
  mostly content, not new engineering.
- **Understandable.** Each algorithm has a doc with a small worked example, and the
  sandbox can show its internals (pairs, residuals, gates, per-pass poses).

## Architecture (target)

```
                    +--------------------------------------+
                    |  atlas core (C++17, no UI, no I/O)   |
                    |  geometry, scans, ICP, gates, maps,  |
                    |  odometry, (later) pose graph        |
                    +---+----------------+-------------+---+
                        |                |             |
             Emscripten/WASM        CLI tools     worker binary
                        |          (KITTI eval)        |
                 Web Worker              |       coordinator, leases,
                        |                |       retries, durable results
         React + TypeScript UI           |             |
         scenario plugins, editor,   result files <----+
         diagnostics panels              |
                        +------ replay explorer (Three.js)
```

Scenario = **map** (walls, objects as polygons) + **robot model** (footprint, speed)
+ **sensor model** (range, beams, field of view, noise) + **editable palette**
(what the visitor can add or move) + **path source** (scripted route or teleop).

## Phases

### Phase 0 — 2D foundations (done)

Frames and transforms, ideal scans, known-pair least-squares alignment, nearest-
neighbor pairing, basic point-to-point 2D ICP with convergence and failure examples.

### Phase 1 — Robust 2D localization (done 2026-11-09)

Goal: a 2D engine good enough to power the sandbox honestly.

- Seeded, reproducible sensor noise and partial overlap (points without partners).
- Distance gate for pairs; inlier ratio and residual diagnostics.
- Degeneracy diagnostic (e.g. corridor-like geometry that leaves motion unobservable).
- Ray-cast 2D LiDAR against polygon walls/objects (replacing point landmarks).
- Scan sequence tracking: chain scan-to-scan estimates, measure drift vs answer key.
- Occupancy grid map and scan-to-map localization, so "the world changed versus
  the map" can be demonstrated and measured.

Done when: tests cover each piece, including failure cases; a CLI run over a
scripted trajectory reports drift and failure flags; docs explain each concept.

Outcome: `atlas_experiment` shows the map localizer never failing with a 10 m
sensor (3 to 11 mm worst error vs 2 to 19 cm for the tracker), and both methods
failing with a 3 m sensor. Planning from the estimated pose moves to Phase 2.

### Phase 2 — Interactive sandbox v1 (target: weeks 2–3)

Goal: the first public, playable demo.

- C++ core compiled to WebAssembly (Emscripten), running in a Web Worker.
- React + TypeScript + Vite app; Three.js 3D field plus Canvas2D figures (started 2026-11-07).
- Scenario plugin interface; first scenario: **home robot** (rooms, furniture).
- Visitor can move furniture, add obstacles, change sensor noise/range, and watch:
  scan rays, pairs, gated outliers, estimated vs true pose, residual/inlier charts,
  and an explicit "localization lost" state.
- The robot plans and steers from its estimated pose, so bad localization
  shows as bad driving (done 2026-11-10; planner still uses the true floor plan).
- Simulated wheel odometry as the localizer's motion prediction, with a
  constant-velocity comparison switch (done 2026-11-10).
- Shareable setup links and GitHub Pages deployment from CI (done 2026-11-10).
- Playwright smoke tests gate every deploy (done 2026-11-11).
- Deterministic seeds; shareable scenario state in the URL. (Partly: links
  reproduce the setup and per-scan seeds are fixed, but runs are not exact
  replays; engine results arrive asynchronously.)
- Static hosting with CI deploy; Playwright smoke test. (Done.)

Status 2026-11-11: deliverables done except residual/inlier **charts** (the lab
shows these as numbers) and exact replay. The 30-second criterion below has
not been tested with a real first-time visitor. Decision: move to Phase 3 and
treat these as polish, per "protect Phase 3".

Status 2026-11-24: charts done (pass-by-pass residual vs true error; 35 s
error and map-match history) and exact replay done (lockstep simulated time;
a browser test requires two runs of one link, one on a 3x slower CPU, to match
scan for scan). Remaining: the first-visitor test, which needs real people;
the script is docs/visitor-test.md.

Done when: a first-time visitor understands what to do within about 30 seconds,
the UI stays smooth while the engine runs, and every view is labeled live/simulated.

### Phase 3 — 3D LiDAR odometry on real data (done 2026-11-24) — headline result

Goal: credible, measured odometry on a public driving dataset (KITTI odometry first).

- Dataset loader (data stays out of Git), voxel downsampling, spatial index (k-d tree).
- 3D point-to-point then point-to-plane ICP, constant-velocity motion prior,
  local map of recent scans.
- Evaluation: standard KITTI drift metrics plus absolute/relative trajectory error,
  per sequence; held-out sequences for any tuned parameter.
- Baseline comparison against an established open-source method (e.g. KISS-ICP),
  run on the same hardware and sequences.
- Profiling before optimization; report latency per frame and memory.
- New dependencies (e.g. Eigen) discussed and added only here, where needed.

Status 2026-11-23: one raw drive (sequence 04, 394 m) downloaded (the 80 GB
set did not fit). Done: readers, frames, voxel-hash nearest neighbors
(instead of a k-d tree), 3D ICP, deskewing (clockwise, measured), the KITTI
scan correction, scan-to-map odometry (local map, robust kernel, adaptive
threshold, following KISS-ICP's design), the devkit metric (checked against
the devkit's own code), KISS-ICP 1.3.0 run on the same scans, profile and
memory. Sequence 04: Atlas 0.48 % t_err, 0.12 deg/100 m, 43 ms per scan on one
core, 79 MB; KISS-ICP 0.41 %, 0.19 deg/100 m. Point-to-plane was not needed
(the local map removed the bias it was meant to fix). No external C++
dependency was needed.

Status 2026-11-24: done. Held-out sequence 07 (694 m urban loop), run with
no changes: Atlas 0.43 %, 0.25 deg/100 m, 28 ms per scan, 106 MB; KISS-ICP
0.75 %, 0.37 deg/100 m. One win each across the two sequences, so no
ranking. Trajectory plots via `atlas_kitti plot`; results table, failure
analysis and reproduce commands in `docs/kitti.md`.

Done when: a results table with hardware/config, failure analysis, and plots is
reproducible from documented commands.

### Phase 4 — Distributed evaluation (done 2026-11-24)

Goal: show distributed-systems skill on a real need — sweeping configurations
across many sequences.

- Coordinator plus independent workers; bounded job queue; leases and heartbeats.
- Retries with backoff; idempotent, content-addressed result publication.
- Demonstrated worker crash and recovery; duplicate-execution safety.
- Aggregated report generated from durable results.
- Claims limited to what is shown: no exactly-once or fleet-scale claims.

Status 2026-11-24: done (`eval/`, docs/evaluation.md). Python standard
library (HTTP + SQLite) around the unchanged C++ engine (`atlas_kitti run`).
A 24-job KITTI sweep survived a worker kill -9, a coordinator kill -9 and
restart, and a worker paused past its lease: all 24 done, one accepted
result each, exactly the 2 interrupted jobs re-run, the paused worker's
late result refused. The first run exposed a worker bug (a late heartbeat
cancelled the next job) and a design gap (outage longer than a lease); both
fixed and regression-tested. Workers ran as processes on one machine.

### Phase 5 — Replay explorer (done 2026-11-24)

Goal: let visitors inspect the real-data results.

- Three.js view of point clouds, estimated vs reference trajectory, per-frame
  residuals and inliers, timeline scrubbing; clearly labeled **replay**.
- Links from the results table into the exact frames where failures occur.

Status 2026-11-24: done (docs/replay.md; live at /atlas/replay/). Sequences
07 and 04: Atlas's own map, every scan (thinned), estimated vs true vs
KISS-ICP paths, per-frame step error, drift, inliers, ICP iterations,
adaptive threshold and time from the engine's own log; scrubbing, playback,
cameras, data-chosen "jump to" frames, shareable frame links; the home page's
error bars open the matching stretch. About 9.8 MB of KITTI-derived data is
committed (CC BY-NC-SA 3.0 notice alongside), decided with the user.

**End of the ~8-week flagship:** Phases 1–5. If time runs short, protect Phase 3.

### Phase 6+ — Extensions (after the flagship)

- Keyframes, geometrically verified loop closure, pose-graph optimization; show
  drift reduction on real sequences (moves Atlas from odometry toward full SLAM).
- **Planner module**, separate from localization and labeled as such: occupancy-grid
  A* plus a simple path follower, so the robot can reroute around new obstacles.
- More scenarios: **street car** (parked cars, pedestrians as moving obstacles) and
  **warehouse robot** (shelves, repetitive aisles that stress localization).
- Challenge mode: visitors try to break localization; leaderboard of failure cases.

## Product quality bar (applies to every shipped view)

- Smooth interaction on a typical laptop; engine work never blocks the UI thread.
- Usable at phone width with reduced detail; keyboard-accessible controls;
  readable contrast in light and dark themes.
- Clear empty, loading, and engine-error states; no silent failures.
- Unit tests for UI logic, browser smoke tests, and C++ tests in the same CI.
- A README with a short screen recording, a results table, and honest limitations.

## How progress is tracked

Daily: one understandable, tested increment, recorded in `docs/progress.md`.
Per phase: a changelog entry and, when the milestone is coherent, a semantic tag.
