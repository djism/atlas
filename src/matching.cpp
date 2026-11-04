#include "atlas/matching.hpp"
#include "atlas/geometry.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace atlas {
namespace {
void requireFinite(const Point2D& p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        throw std::invalid_argument("matching requires finite point coordinates");
    }
}
}

std::size_t findNearestPoint(const Point2D& query, const std::vector<Point2D>& points) {
    if (points.empty()) {
        throw std::invalid_argument("nearest-point search needs at least one point");
    }
    requireFinite(query);
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        requireFinite(points[i]);
        const double distance = std::hypot(query.x - points[i].x, query.y - points[i].y);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }
    return best_index;
}

Pose2D estimateFromTwoPairs(const std::vector<Point2D>& first,
                           const std::vector<Point2D>& second) {
    if (first.size() != 2 || second.size() != 2) {
        throw std::invalid_argument("provide exactly two known corresponding pairs");
    }
    for (const auto& p : first) requireFinite(p);
    for (const auto& p : second) requireFinite(p);
    const double first_dx = first[1].x - first[0].x;
    const double first_dy = first[1].y - first[0].y;
    const double second_dx = second[1].x - second[0].x;
    const double second_dy = second[1].y - second[0].y;
    if (std::hypot(first_dx, first_dy) < 1e-9 ||
        std::hypot(second_dx, second_dy) < 1e-9) {
        throw std::invalid_argument("points in each scan must be distinct");
    }
    const double angle_difference = std::atan2(first_dy, first_dx) -
                                    std::atan2(second_dy, second_dx);
    const double rotation = std::atan2(std::sin(angle_difference), std::cos(angle_difference));
    const Pose2D rotation_only{0.0, 0.0, rotation};
    Point2D total{0.0, 0.0};
    for (std::size_t i = 0; i < first.size(); ++i) {
        const auto rotated = transformPoint(second[i], rotation_only);
        total.x += first[i].x - rotated.x;
        total.y += first[i].y - rotated.y;
    }
    return {total.x / 2.0, total.y / 2.0, rotation};
}
}  // namespace atlas
