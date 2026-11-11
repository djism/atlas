# Atlas Sandbox

A single-page site that explains Atlas and lets visitors try it. A simulated
robot patrols a two-room home in 3D while the **real Atlas C++ engine**,
compiled to WebAssembly, simulates its LiDAR, tracks its pose, matches scans,
and plans its route live in the browser. No algorithm is reimplemented in
TypeScript.

## Run it

Prerequisites: Node 18+ and the Emscripten SDK (default location `~/emsdk`;
set `EMSDK=/path` otherwise). The SDK is activated by the build script only.

```bash
cd web
npm install          # project-local dependencies (web/node_modules, git-ignored)
npm run build:wasm   # C++ core + wasm/bindings.cpp -> web/src/engine/generated/
npm run dev          # http://localhost:5173
npm run build        # typecheck + production bundle in web/dist/
```

Built with Emscripten 6.0.13; the engine is a single ~132 KB module.
Rendering uses Three.js 0.169.0 (project-local npm dependency, exact version).

## The page

| Section | What the visitor does | Engine calls |
| --- | --- | --- |
| Hero | Watches the robot patrol a 3D home; drags furniture, clicks the floor to drop a box, toggles 3D/Top, hovers for world coordinates | `simulateScan`, `planRoute` |
| What I built | Reads six modules, each with its source file and one measured number | none |
| How it works | Drags a blue scan in a 2D figure, releases, and watches ICP pull it back pass by pass; switches point-to-line / point-to-point | `alignScans` |
| Lab: Match two scans | The same ICP experiment on the 3D field, with pair lines | `alignScans` |
| Lab: Get it lost | Compares two estimators on the same scans: the map localizer (blue ring) and the no-map tracker (gray ring). Shortens range, adds noise, or moves furniture and drops boxes so the house no longer matches the map (pale dots) | `buildMap`, `simulateScan`, `Localizer.update`, `Tracker.update` |
| Lab: Rearrange the house | Drags furniture, drops boxes, blocks the doorway; watches replanning | `planRoute` |

Visual direction ("Monochrome"): white page, ink type, one signal blue
reserved for what the sensor sees and what the engine estimates. Tokens are in
`web/src/styles/tokens.css`; scene colors mirror them in `web/src/field/palette.ts`.

## How it fits together

```
Main thread (React + Three.js)               Web Worker (C++ via WASM)
  Field: true robot motion, 3D scene  --->    simulateScan   ray-cast LiDAR + noise
  answer key (true pose)                      Tracker.update point-to-line ICP, chaining
  IcpDemo: drag, replay, verdicts     <---    alignScans     ICP from a guess; poses, RMSE, pairs per pass
  panels and readouts                         planRoute      inflated 5 cm grid + A* + smoothing
                                              buildMap       occupancy map from survey scans
                                              Localizer.update scan-to-map, degeneracy handling
```

- **Honest labels.** The page says it is live and the world is simulated. The
  true pose is used only to simulate the sensor and to compute the error readout
  against the answer key. The tracker receives only scans and its starting pose.
- **One engine.** `wasm/bindings.cpp` is a thin layer: it converts flat arrays to
  `Point2D`/`Segment2D` and calls the library. The core has no browser code.
- **Request/reply by id.** `web/src/engine/client.ts` sends typed messages
  (`web/src/engine/protocol.ts`) to one shared worker and matches replies by id.
  Each loop waits for its own reply before sending the next request.
- **`alignScans`** runs `alignIcp` from the visitor's guess and returns the guess
  plus the pose after each pass, nearest-pair RMSE per pass, and the nearest
  pairs at each step so the page can draw them. Tolerances: 1e-4 m and 1e-4 rad,
  at most 40 passes (tighter tolerances let 8 mm noise keep ICP busy to the limit).
- **Map (surveyed).** On entering "Get it lost" or pressing "Re-map and
  restart", the worker builds an occupancy map (5 cm) from one noise-free scan
  at each patrol point, using the house as it is at that moment and a 20 m
  survey sensor. These are answer-key poses: a mapping drive before the
  experiment. Moving furniture afterwards makes the house differ from the map;
  the old outline stays in the drawn map points.
- **Sensor on the simulated clock.** The robot's motion is integrated in
  0.02 s slices, and the true pose is captured every 0.1 s of simulated time
  for one scan. If the engine is more than two scans behind, simulated time
  waits. A slow machine makes the robot slower, never the sensor. (First
  version: a new scan only when the previous one finished, so a busy browser
  lowered the sensor rate below 2 Hz, and the localizer ran away at the
  doorway spin. Four 5-minute runs, including one with the CPU slowed 3x,
  now finish with 0 lost scans.)
- **The robot drives on its belief.** In every view except "Match two scans",
  the robot plans and steers from its **estimate**, never its true position.
  The belief is the latest map-localizer estimate composed with the motion
  commanded since that scan was captured (exact commands between scans; see
  wheel odometry below for what the localizer itself gets).
  Each command (turn, then forward step) is applied to the true robot. If the
  step would put the robot's disc into a wall or furniture, a simulated bumper
  stops it, odometry sees no forward motion, and it replans from its belief.
  The robot does not move until its first estimate arrives ("Localizing").
- **Wheel odometry.** Each scan request carries the TRUE motion since this
  view's previous tracked scan; the worker passes it through C++
  `simulateOdometry()` (assumed 2 % bias, 5 % random distance and turn error,
  0.02 rad per meter; same values as `atlas_experiment`). Only that noisy
  report reaches the map localizer, as its starting guess
  (`updateWithMotion`). "Get it lost" can switch the blue ring to "Repeat last
  motion" (constant velocity) for comparison. The gray tracker always uses
  constant velocity.
- **Planner (simplified).** Plans from the belief, but on the **true** floor
  plan (furniture where it really is). A real robot would plan on its own map.
- **One estimator pair per view.** The hero and the lab each simulate a robot;
  the worker keeps a tracker and localizer per view (`owner` in the protocol).
- **Scenario plugins.** A scenario (`web/src/scenarios/`) is data: walls,
  furniture (with heights for 3D), patrol points, robot, default sensor.

## ICP verdicts shown to visitors

| Chip | Meaning |
| --- | --- |
| Converged | The pose stopped changing and is within 2 cm and 0.02 rad of the answer key |
| Converged, but X off | The pose stopped changing at the wrong place: a local minimum |
| Hit the pass limit | Still moving after 40 passes |
| Degenerate / Too few pairs | The engine refused to give a pose |

## Measured in headless Chrome (MacBook, arm64, software WebGL)

| Experiment | Result |
| --- | --- |
| Tracker, default settings | 0.2 to 1.3 cm position error after about 2.6 m; ~4.8 ms per scan; 0 lost |
| ICP figure, point-to-line | 5 passes, 3.7 mm and 0.04 deg from the answer key |
| ICP figure after a ~2 m drag | Re-converged in 6 passes |
| ICP figure, point-to-point | 20 passes for the same scans |
| Range 1.5 m | 20 to 68 lost scans; error 47 cm, and in one run 28 m |
| Planner | ~0.4 ms per plan on the 5 cm grid |
| Get it lost, default sensor, 20 s | With map 1.1 mm, no map 2.2 mm, map match 93%; 1,085 map points |
| Get it lost, 6 unmapped boxes dropped | With map 0.3 mm (chip: "House differs from the map", 79% match); no map 3.9 cm |
| Get it lost, 3 m range, re-mapped, 25 s | With map 1.4 mm, no map 8.3 cm |
| Robot steering from its estimate, 4 x 5 min (normal, CPU 2x/3x slower, other window sizes) | 0 bumps, 0 lost scans |
| Planner tab, 3 boxes dropped, 25 s | 9 replans from the estimate, 0 bumps |
| Get it lost, 1.5 m range (before wheel odometry) | Estimate 5.8 m off; the robot steered into the desk area and bumped ("Bumped: it was not where it thought") |
| Wheel odometry vs repeat last motion, 3 m range, 2 min + 3 min (CPU 2x slower) | Wheels: worst 15 / 17 cm, 0 bumps. Last motion: worst 8.5 / 20.5 m, over 0.5 m off 90 / 94 % of the time, 0 / 1 bumps |
| Same, 1.5 m range | Wheels: worst 28 / 26 cm, 0 bumps (most scans lost: coasting on the wheels). Last motion: worst 4.2 / 4.3 m, 3 / 5 bumps |
| Default 8 m range, 5 min x 3 (wheels; wheels with CPU 3x slower at 1280 px; last motion) | 0 lost scans, 0 bumps in all three; worst live gap 14 to 18 cm, mostly scan lag |
| Engine time per scan, both estimators | median 21 ms, 90th percentile 26 ms (was 4.8 ms with the tracker only) |

These are single live runs, not seed sweeps (the doorway runaway and its fix
are described in [localization](localization.md)); `atlas_experiment` is the
measured comparison. In the first version, dropping boxes made the robot
replan and start turning at 0.32 rad per scan; the localizer's 0.2 rad jump
limit rejected the correct answers and it ended 1.1 m off. The limit is now
0.5 rad, with a regression test.

The 28 m case is a real failure mode. When matches fail, the tracker predicts
with constant velocity; after bad matches that velocity can be wrong, and the
estimate runs away. A motion sanity check and, more fundamentally, localizing
against a map are the fixes.

## Limits

- The map is surveyed at answer-key poses and never updated while localizing.
- Localization uses brute-force nearest neighbors over all map points; that
  is most of the 21 ms per scan. A spatial index is the known fix.
- The jump limits assume how fast motion can change between scans.
- Planner is given the true floor plan (it plans from the estimate); no dynamic
  obstacles. Between scans (at most about 0.3 s) the belief adds the exact
  commanded motion; wheel noise enters once per scan interval, at the localizer.
- Wheel noise values are assumptions. With 3x worse wheels, short range fails
  again in `atlas_experiment` (20 to 28 cm); see [localization](localization.md).
- After a bump against a wall it cannot see past, a badly lost robot keeps
  pushing and replanning; there is no recovery behavior.
- One scenario (home). Street-car and warehouse scenarios are planned.
- Not yet deployed; no browser tests in CI (CI builds and tests the C++ only).
- Bundle is ~690 KB (~190 KB gzip), mostly Three.js; not yet code-split.
- Isotropic point noise, not along-beam range noise.
