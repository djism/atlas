#include "atlas/simulation.hpp"
#include "atlas/geometry.hpp"
#include <cmath>
#include <stdexcept>

namespace atlas {
std::vector<Point2D> simulateScan(const std::vector<Point2D>& landmarks,
                                const Pose2D& pose, double max_range) {
    if (!std::isfinite(max_range) || max_range < 0.0) {
        throw std::invalid_argument("max_range must be finite and nonnegative");
    }
    std::vector<Point2D> scan;
    for (const auto& landmark : landmarks) {
        const auto point = inverseTransformPoint(landmark, pose);
        if (std::hypot(point.x, point.y) <= max_range) {
            scan.push_back(point);
        }
    }
    return scan;
}
}  // namespace atlas
