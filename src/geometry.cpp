#include "atlas/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
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

Pose2D composePoses(const Pose2D& b_in_a, const Pose2D& c_in_b) {
    const Point2D origin = transformPoint({c_in_b.x, c_in_b.y}, b_in_a);
    const double theta = b_in_a.theta + c_in_b.theta;
    return {origin.x, origin.y, std::atan2(std::sin(theta), std::cos(theta))};
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

std::vector<NormalEstimate> estimateNormals(const std::vector<Point2D>& points,
                                            std::size_t neighbors,
                                            double max_spread_ratio) {
    if (neighbors < 3) {
        throw std::invalid_argument("normal estimation needs at least 3 neighbors");
    }
    if (points.size() < neighbors) {
        throw std::invalid_argument("fewer points than requested neighbors");
    }
    if (!(max_spread_ratio >= 0.0)) {
        throw std::invalid_argument("max_spread_ratio must be nonnegative");
    }
    for (const auto& p : points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            throw std::invalid_argument("normal estimation requires finite points");
        }
    }

    std::vector<NormalEstimate> normals(points.size());
    std::vector<std::size_t> order(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        // Nearest `neighbors` points to points[i] (it is its own nearest).
        std::iota(order.begin(), order.end(), std::size_t{0});
        const auto squared = [&](std::size_t j) {
            const double dx = points[j].x - points[i].x;
            const double dy = points[j].y - points[i].y;
            return dx * dx + dy * dy;
        };
        std::partial_sort(order.begin(), order.begin() + neighbors, order.end(),
                          [&](std::size_t a, std::size_t b) { return squared(a) < squared(b); });

        // Spread (covariance) of the neighborhood about its own centroid.
        double cx = 0.0, cy = 0.0;
        for (std::size_t k = 0; k < neighbors; ++k) { cx += points[order[k]].x; cy += points[order[k]].y; }
        cx /= static_cast<double>(neighbors);
        cy /= static_cast<double>(neighbors);
        double sxx = 0.0, syy = 0.0, sxy = 0.0;
        for (std::size_t k = 0; k < neighbors; ++k) {
            const double dx = points[order[k]].x - cx;
            const double dy = points[order[k]].y - cy;
            sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
        }
        // Closed-form eigenvalues of the 2x2 spread: along-line (larger) and
        // across-line (smaller) spread.
        const double mean = 0.5 * (sxx + syy);
        const double radius = std::hypot(0.5 * (sxx - syy), sxy);
        const double along = mean + radius;
        const double across = mean - radius;
        if (!(along > 0.0) || across > max_spread_ratio * along) continue;  // not line-like
        // Line direction angle, then rotate 90 degrees for the normal.
        const double line_angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
        normals[i].normal = {-std::sin(line_angle), std::cos(line_angle)};
        normals[i].valid = true;
    }
    return normals;
}
}  // namespace atlas
