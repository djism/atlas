#pragma once

namespace atlas {
struct Point2D {
    double x;
    double y;
};

// Position of a source frame's origin in a destination frame, and its heading.
// All distances are meters; theta is counterclockwise radians.
struct Pose2D {
    double x;
    double y;
    double theta;
};

// Straight line segment from start to end (e.g. a wall), meters.
struct Segment2D {
    Point2D start;
    Point2D end;
};
}  // namespace atlas
