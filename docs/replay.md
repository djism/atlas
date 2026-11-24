# Replay explorer (Phase 5)

**Live:** https://djism.github.io/atlas/replay/ (for example
`?seq=07&frame=476`). A replay of recorded runs, labeled as such on every
screen: nothing is computed in the page.

## What it shows

Pick a drive (KITTI sequence 07, held out, or 04), then scrub, play (1x or
4x), or jump to the frames that matter:

- **3D view:** Atlas's map (blue-gray, built from Atlas's *own* estimated
  poses, so its drift is baked in exactly as the estimator built it), the
  current scan (bright blue) drawn where *Atlas believed* the car was, Atlas's
  estimated path growing as the replay plays, the true path (black), and
  optionally KISS-ICP's path. Cameras: chase, top (heading up), free orbit.
- **Frame panel:** against the answer key (Atlas and KISS-ICP step error,
  Atlas's drift so far), and what the engine saw with no answer key (matched
  points, ICP iterations, adaptive match distance, map size, time for the
  scan). A headline from the true motion ("Left turn, 4.3 m/s").
- **Timeline:** step error (Atlas and KISS-ICP), drift, inliers and ICP
  iterations on one time axis; click any chart to jump. A range slider works
  from the keyboard.
- **Jump to:** chosen from the data (`notableFrames`): Atlas's three worst
  steps at least 50 frames apart, labeled turn / straight / slow from the true
  motion; KISS-ICP's worst step; the largest drift.
- **Links:** the address always holds `?seq=..&frame=..`, so any moment can be
  shared. On the home page, every bar pair in "Error per step along the
  drive" opens that stretch here.

What it showed on sequence 07: Atlas's worst steps are frame 5 (barely moving,
no motion seen yet) and two left turns (frames 476 and 916, 7-8 cm), while
inliers stay above 99 % there, so the inlier count alone would not flag them.
KISS-ICP's worst stretch is a fast straight (frames 247-267, up to 20 cm).
Atlas's drift peaks at 2.1 m (frame 749) and ends at 0.8 m on this loop.

## How the data is made

```bash
K=~/Desktop/Datasets/kitti; O=$K/odometry/dataset; V=$K/raw/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/data
./build/atlas_kitti run $O 07 $V results/replay/atlas_07.txt diag=results/replay/atlas_07_diag.csv
./build/atlas_kitti export-replay $V results/replay/atlas_07.txt web/public/replay/07 scan_points=640 map_voxel=0.5 \
    > results/replay/meta_07.json
python3 -I tools/export_replay.py $O 07 results/replay/atlas_07.txt results/kiss_07.txt \
    results/replay/atlas_07_diag.csv results/replay/meta_07.json web/public/replay/07/frames.json
```

(`results/kiss_07.txt` from `tools/kiss_icp_baseline.py`, see docs/kitti.md.)
The `diag=` output does not change the run: the poses are byte-identical to
the earlier runs.

| File | Content | Size (07 / 04) |
| --- | --- | --- |
| `map.bin` | int16 x, y, z in decimeters, scan 0's Velodyne frame; one point per 0.5 m cube from every KITTI-corrected scan within 50 m, placed with Atlas's estimated pose | 2.2 / 1.6 MB |
| `scans.bin` | int16 x, y, z in centimeters, each scan in its own sensor frame, thinned to exactly 640 points (every k-th within 50 m); frame i starts at byte i x 640 x 6 | 4.2 / 1.0 MB |
| `frames.json` | per frame: time, Atlas pose (3x4), true and KISS-ICP positions, step errors, drift, true speed and turn, and the engine's own log (iterations, pairs, source points, adaptive threshold, ms, map size) | 230 / 55 kB |

All little-endian. The page checks lengths and file sizes before drawing
(`checkReplay`) and says so if they disagree. About 9.8 MB is committed in
`web/public/replay/` (decided with the user: the Pages build runs in CI,
which has no KITTI data). These files are derived from KITTI and carry its
license, CC BY-NC-SA 3.0 (`web/public/replay/README.md`).

## Code

| File | Role |
| --- | --- |
| `apps/kitti.cpp` | `run ... diag=` (per-frame log), `export-replay` (map.bin, scans.bin) |
| `tools/export_replay.py` | frames.json (standard library) |
| `web/replay/index.html`, `web/src/replay/main.tsx` | second page of the Vite build |
| `web/src/replay/ReplayView.ts` | Three.js view: map, scan, paths, car, cameras |
| `web/src/replay/ReplayApp.tsx` | page: loading with progress, panel, timeline, playback, link state |
| `web/src/lib/replay.ts` | pure helpers: decoding, consistency check, links, charts, notable frames, headlines |
| `web/src/styles/replay.css` | styles from the site's tokens |

## Tests

- Unit (`npm test`, 8 new): int16 decoding is little-endian with the right
  unit; scan k is the k-th block; inconsistent lengths and sizes are caught;
  links round-trip and bad values fall back; charts put larger values higher,
  keep one-frame spikes when bucketing, clip, and map clicks back to frames;
  notable frames are spaced and labeled; headlines follow the motion; the
  committed data files are consistent.
- Browser (Playwright): the page loads frame 476 with the recorded numbers
  (6.8 cm, "Left turn, 4.3 m/s"), steps, jumps (and updates the address),
  switches all three cameras, plays, switches drive; a home-page bar opens the
  replay at its stretch; no sideways scroll at 390 px; no console errors.
  Also passes with software WebGL and a 3x slower CPU (16 s).

## Limits

- Replay only: the engine does not run here (the live engine is on the home
  page, in 2D).
- Scans are thinned to 640 of about 120,000 points, and the map to one point
  per 0.5 m; good enough to see the street, not to judge fine alignment.
- Scans are drawn as recorded (not deskewed), so fast turns smear slightly.
- Two sequences. Adding one means re-running the commands above (about 10 MB
  more in Git each time).
