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

Built with Emscripten 6.0.13; the engine is a single ~116 KB module.
Rendering uses Three.js 0.169.0 (project-local npm dependency, exact version).

## The page

| Section | What the visitor does | Engine calls |
| --- | --- | --- |
| Hero | Watches the robot patrol a 3D home; drags furniture, clicks the floor to drop a box, toggles 3D/Top, hovers for world coordinates | `simulateScan`, `planRoute` |
| What I built | Reads six modules, each with its source file and one measured number | none |
| How it works | Drags a blue scan in a 2D figure, releases, and watches ICP pull it back pass by pass; switches point-to-line / point-to-point | `alignScans` |
| Lab: Match two scans | The same ICP experiment on the 3D field, with pair lines | `alignScans` |
| Lab: Get it lost | Shortens sensor range or adds noise; watches the tracker's estimate (blue ring) drift from the true robot | `simulateScan`, `Tracker.update` |
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
- **Planner (simplified).** Plans from the robot's **true** position on the
  **known** floor plan, so it does not depend on localization. Real robots plan
  from their estimate; closing that loop is roadmap work.
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

The 28 m case is a real failure mode. When matches fail, the tracker predicts
with constant velocity; after bad matches that velocity can be wrong, and the
estimate runs away. A motion sanity check and, more fundamentally, localizing
against a map are the fixes.

## Limits

- Scan-to-scan odometry only: no map, so drift is never corrected.
- Planner is simplified (true position, known floor plan); no dynamic obstacles.
- One scenario (home). Street-car and warehouse scenarios are planned.
- Not yet deployed; no browser tests in CI (CI builds and tests the C++ only).
- Bundle is ~690 KB (~190 KB gzip), mostly Three.js; not yet code-split.
- Isotropic point noise, not along-beam range noise.
