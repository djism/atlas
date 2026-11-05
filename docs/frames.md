# Frames and the two-scan experiment

A scan is a collection of sensor measurements at one moment. There is one car
and one sensor. First scan means before movement; second scan means after movement.

A frame consists of an origin and axes. A physical landmark stays still while
its coordinates change when described from a moving sensor.

| Frame | Origin | Axes |
| --- | --- | --- |
| World | Fixed chosen reference | Fixed +x and +y |
| First sensor | Sensor before motion | +x forward, +y left at first observation |
| Second sensor | Sensor after motion | +x forward, +y left at second observation |

All distances are meters; headings are counterclockwise radians.
Sensor and car frames coincide here, with no mounting offset.

## Simulator versus estimator

The simulator receives world landmark A=(4,5), B=(6,7), and a far point=(20,20).
At pose (5,3,90 degrees), A and B appear at (2,1) and (4,-1).
At pose (5,4,100 degrees), their coordinates change despite fixed world positions.
A range of 5 meters retains A and B in both scans and excludes the far point.
This controlled example preserves known correspondence ordering.

The estimator receives those two scans only. It does not read the true poses.
The true pose difference is used separately to check the answer.

The world translation is (0,1). Expressed along the first sensor's axes, it is
(1,0): one meter forward. Heading changes by +10 degrees.

## Transform meaning

`transformPoint(p, pose)` computes `R * p + t`. The pose expresses the source
frame's origin and heading in the destination frame. The function itself does
not know which named frames you intend; its caller must keep them consistent.

- Simulation inverse: world -> current sensor, subtract position then rotate by negative heading.
- Demo alignment: second sensor -> first sensor, rotate by relative heading then add relative translation.
- Future mapping: sensor -> world, using an estimated world pose.

For corresponding points, `first = R * second + t`.
The two-pair baseline recovers `R` from the difference between the two segment
directions, then `t` from the average of `first - R * second`. The general
estimator uses centroids and many pairs; see [alignment](alignment.md).

This is coordinate conversion, not physical movement of landmarks. Tiny errors
around 1e-16 are ordinary floating-point rounding. Tests compare with tolerances.

## Correspondence assumptions

Indices match only because this experiment deliberately retains the same two
landmarks in the same order. If visibility changes, equal indices can represent
different landmarks. Nearest-neighbor search proposes a spatial match; it cannot
prove identity and can match several queries to one point. General matching and
rigid alignment will be developed and tested in later milestones.
