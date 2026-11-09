# Occupancy grid mapping

Scan-to-scan tracking compares each scan only with the previous one, so its
errors add up and nothing pulls them back. To correct drift, the robot needs a
fixed reference to compare against: a map. This page builds one; see
[localization](localization.md) for using it. Each cell also keeps the mean of
the beam end points that landed in it (`occupiedPoints()`), which is more
accurate than the cell center for localization.

## What the map stores

The map is a grid of square cells over a rectangle in the map frame. Here the
map frame is the world frame, because scans are added at poses in world
coordinates. Each cell holds one number, its **log-odds** of being occupied:

```
l = ln(p / (1 - p))        p = 1 - 1 / (1 + e^l)
```

`l = 0` means p = 0.5 (no evidence). Log-odds are used because evidence
**adds**: each observation adds a fixed weight, and agreeing observations add up
while disagreeing ones cancel. With probabilities you would have to multiply
and renormalize instead.

| Event | Weight | Alone |
| --- | --- | --- |
| A beam ended in the cell (hit) | +0.85 | p = 0.70 |
| A beam passed through the cell (miss) | -0.40 | p = 0.40 |

A cell reads as **Occupied** at p >= 0.65, **Free** at p <= 0.45, and
**Unknown** in between, including cells no beam ever reached. Values are clamped
to [-4, 4] (p between 0.018 and 0.982), so a cell never becomes so certain that
it cannot change its mind when the world changes.

## One beam, 1 m cells

Sensor at (0.5, 0.5) facing +x; a wall 3 m ahead ends the beam at (3.5, 0.5).

```
row 0:  [free] [free] [free] [OCC] [unknown] [unknown]
         col 0  col 1  col 2  col 3    col 4     col 5
```

Cells behind the wall stay unknown: the beam never got there. That is
occlusion, and it is why the inside of a box is never mapped.

## Walking a beam through the grid

`integrateScan` visits every cell a beam crosses, in order, using a grid
traversal (Amanatides and Woo). Let t be the distance along the beam. Keep the
t at which the beam crosses the next vertical cell border (`next_x`) and the next
horizontal border (`next_y`). Whichever is smaller is the next border crossed:
step one cell in that direction and advance that border by one cell width along
the beam (`resolution / |dx|`). Stop when the next border is beyond the beam's
end. Every cell visited before the end gets a miss; the end cell gets a hit.

Example: 30 degree beam from (0.5, 0.5), length 2.8 m, 1 m cells. Borders are
crossed at t = 0.58 (x = 1), 1.0 (y = 1), and 1.73 (x = 2). The next one, at
t = 2.89, is past the end. Cells visited: (0,0), (1,0), (1,1), and the end
cell (2,1). Sampling the beam at fixed steps could skip a cell it only clips
at a corner; the traversal cannot.

A beam with no return clears cells up to the sensor's maximum range. This
assumes nothing was within range in that direction, which is true in the
simulator but not for real dark or shiny surfaces.

## Results (360 beams, 5 cm cells, noise-free)

Room 8 x 6 m with an alcove and a 1 m box, scanned from 7 answer-key poses:

- Every occupied cell is within one cell (5 cm) of a real wall.
- At least 95% of wall length has an occupied cell within one cell.
- Open floor is free; the inside of the box and the outside of the room stay unknown.

The world changes: a box face seen 3 times flipped to free after 2 scans
without the box. One scan is not enough to erase repeated evidence; the clamp
keeps this from growing without limit.

## Findings and limits

- **Grazing beams wear walls down.** A beam that nearly runs along a wall
  crosses wall cells before ending further on, so those cells get misses. The
  box face gathered only +1.35 after 3 scans, not 3 x 0.85, and the cell near
  its corner even read as free. The demo's 25 cm map shows this as dots inside
  walls. Common fixes: do not clear the last cell or two before a hit, or weight
  misses by beam angle.
- **Border cells.** A wall exactly on a cell border lands in the cell on the
  sensor's side; which side depends on rounding.
- **Mapping uses known poses.** The poses come from the answer key, like a
  surveyed mapping run. Localization will receive only the map and scans.
- No noise yet, a fixed rectangular extent, and brute-force rendering of
  occupied cells.
