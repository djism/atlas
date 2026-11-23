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

// 3D point, meters. Its frame is stated wherever it is used (e.g. KITTI
// Velodyne frame: x forward, y left, z up, origin at the sensor).
struct Point3D {
    double x;
    double y;
    double z;
};

// Straight line segment from start to end (e.g. a wall), meters.
struct Segment2D {
    Point2D start;
    Point2D end;
};
}  // namespace atlas
