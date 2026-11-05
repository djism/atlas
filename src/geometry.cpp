#include "atlas/geometry.hpp"
#include <cmath>
#include <stdexcept>

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

Point2D computeCentroid(const std::vector<Point2D>& points) {
    if (points.empty()) {
        throw std::invalid_argument(
            "Cannot compute the centroid of an empty point set"
        );
    }

    double total_x = 0.0;
    double total_y = 0.0;

    for (const auto& point : points) {
        total_x += point.x;
        total_y += point.y;
    }
    return {
        total_x / points.size(),
        total_y / points.size()
    };
}
}  // namespace atlas
