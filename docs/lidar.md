# Ray-cast 2D LiDAR

A planar spinning LiDAR fires beams at fixed angular steps and reports, per beam,
the distance to the **first** surface hit, or no return. Walls therefore hide
what is behind them, and a flat wall appears as a row of points.

`simulateLidarRanges(walls, sensor_in_world, config)` models this with walls as
line segments in the world frame. Beam `i` points at `i * 2*pi / beam_count` in
sensor axes (0 = forward, counterclockwise). The result is one range per beam;
infinity means no return. `rangesToPoints` converts finite ranges to points in
**sensor axes**: `(r cos(angle), r sin(angle))`. Skipped beams mean point indices
are not beam indices.

## One beam, one wall

Sensor at the origin facing +x; wall from a = (3, -1) to b = (3, 2).

- Beam: `(0,0) + t * (1,0) = (t, 0)`, where t is meters traveled.
- Wall: `a + u * (b - a) = (3, -1 + 3u)`, where u runs from 0 (a) to 1 (b).
- Equal when t = 3 and u = 1/3: hit at (3, 0), 3 m away, a third along the wall.

In general, with beam origin o, unit direction d, `e = b - a`, and
`cross(v, w) = v.x * w.y - v.y * w.x`:

```
t = cross(a - o, e) / cross(d, e)        u = cross(a - o, d) / cross(d, e)
```

A hit needs t >= 0 (in front), 0 <= u <= 1 (between the ends), and t <= max range.
The smallest t over all walls wins, which produces occlusion. If `cross(d, e)` is
near zero the beam is parallel to the wall and does not hit it, even when sliding
exactly along it. The beam direction in world axes is sensor heading plus beam angle.

## First finding: point-to-point ICP on wall scans

Room 8 x 6 m with an alcove and a box; robot moves (0.3, 0.05) m and turns 5
degrees; **no noise**; zero initial guess.

| Beams | Position error | Rotation error | Passes |
| --- | --- | --- | --- |
| 90 | 2.9 cm | 0.56 deg | 8 |
| 360 | 3.3 mm | 0.21 deg | 17 |
| 1440 | 1.0 mm | 0.02 deg | 30 (hit pass limit) |

Even without noise, the error is not zero: after moving, beams land on different
spots along the same walls, so exact point correspondences no longer exist.
Denser scans reduce the error but ICP converges slowly. Matching points to lines
fixes most of this; see point-to-line ICP in [icp](icp.md).

## Limits

- Independent per-beam ranges; no range noise yet (along-beam noise is planned).
- No beam divergence, reflectivity, minimum range, or motion during a sweep.
- Brute force: every beam tests every wall, O(beams x walls).
