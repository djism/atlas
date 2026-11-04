#include "atlas/geometry.hpp"
#include <cmath>

namespace atlas {
Point2D transformPoint(const Point2D& point, const Pose2D& pose) {
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    return {c * point.x - s * point.y + pose.x,
            s * point.x + c * point.y + pose.y};
}

Point2D inverseTransformPoint(const Point2D& point, const Pose2D& pose) {
    const double dx = point.x - pose.x;
    const double dy = point.y - pose.y;
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    return {c * dx + s * dy, -s * dx + c * dy};
}
}  // namespace atlas
