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
- Deterministic seeds; shareable scenario state in the URL.
- Static hosting with CI deploy; Playwright smoke test.

Done when: a first-time visitor understands what to do within about 30 seconds,
the UI stays smooth while the engine runs, and every view is labeled live/simulated.

### Phase 3 — 3D LiDAR odometry on real data (target: weeks 3–6) — headline result

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

Done when: a results table with hardware/config, failure analysis, and plots is
reproducible from documented commands.

### Phase 4 — Distributed evaluation (target: weeks 6–7)

Goal: show distributed-systems skill on a real need — sweeping configurations
across many sequences.

- Coordinator plus independent workers; bounded job queue; leases and heartbeats.
- Retries with backoff; idempotent, content-addressed result publication.
- Demonstrated worker crash and recovery; duplicate-execution safety.
- Aggregated report generated from durable results.
- Claims limited to what is shown: no exactly-once or fleet-scale claims.

### Phase 5 — Replay explorer (target: weeks 7–8)

Goal: let visitors inspect the real-data results.

- Three.js view of point clouds, estimated vs reference trajectory, per-frame
  residuals and inliers, timeline scrubbing; clearly labeled **replay**.
- Links from the results table into the exact frames where failures occur.

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
