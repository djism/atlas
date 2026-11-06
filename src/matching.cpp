#include "atlas/matching.hpp"
#include "atlas/geometry.hpp"
#include <algorithm>
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
void requireFinite(const Pose2D& pose) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        throw std::invalid_argument("matching requires a finite pose");
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

Pose2D estimateFromMatchedPoints(const std::vector<Point2D>& first,
                                 const std::vector<Point2D>& second) {
    if (first.size() != second.size()) {
        throw std::invalid_argument("matched point sets must have equal sizes");
    }
    if (first.size() < 2) {
        throw std::invalid_argument("at least two matched pairs are required");
    }
    double largest_coordinate = 0.0;
    for (std::size_t i = 0; i < first.size(); ++i) {
        requireFinite(first[i]);
        requireFinite(second[i]);
        largest_coordinate = std::max({largest_coordinate,
                                       std::abs(first[i].x), std::abs(first[i].y),
                                       std::abs(second[i].x), std::abs(second[i].y)});
    }

    // Centering removes translation: p = first - first_center, q = second - second_center.
    const Point2D first_center = computeCentroid(first);
    const Point2D second_center = computeCentroid(second);
    double dot = 0.0;
    double cross = 0.0;
    double first_spread = 0.0;   // sum |p|^2
    double second_spread = 0.0;  // sum |q|^2
    for (std::size_t i = 0; i < first.size(); ++i) {
        const Point2D p{first[i].x - first_center.x, first[i].y - first_center.y};
        const Point2D q{second[i].x - second_center.x, second[i].y - second_center.y};
        dot += q.x * p.x + q.y * p.y;
        cross += q.x * p.y - q.y * p.x;
        first_spread += p.x * p.x + p.y * p.y;
        second_spread += q.x * q.x + q.y * q.y;
    }

    // Scale-aware degeneracy checks, relative to the data rather than to meters.
    const double relative_tolerance = 1e-9;
    const double count = static_cast<double>(first.size());
    const double spread_floor = relative_tolerance * largest_coordinate;
    if (std::sqrt(first_spread / count) <= spread_floor ||
        std::sqrt(second_spread / count) <= spread_floor) {
        throw std::invalid_argument("points in each scan must not all coincide");
    }
    // Cauchy-Schwarz: hypot(dot, cross) <= sqrt(first_spread * second_spread),
    // with equality for noise-free rigid data. Near zero means no preferred angle.
    if (std::hypot(dot, cross) <=
        relative_tolerance * std::sqrt(first_spread) * std::sqrt(second_spread)) {
        throw std::invalid_argument("rotation is ambiguous for these correspondences");
    }

    const double rotation = std::atan2(cross, dot);  // already in (-pi, pi]
    const Point2D rotated_center = transformPoint(second_center, {0.0, 0.0, rotation});
    return {first_center.x - rotated_center.x, first_center.y - rotated_center.y, rotation};
}

double computeMatchedRmse(const std::vector<Point2D>& first,
                          const std::vector<Point2D>& second,
                          const Pose2D& second_in_first) {
    if (first.empty() || first.size() != second.size()) {
        throw std::invalid_argument("RMSE needs nonempty matched sets of equal size");
    }
    requireFinite(second_in_first);
    double squared_error = 0.0;
    for (std::size_t i = 0; i < first.size(); ++i) {
        requireFinite(first[i]);
        requireFinite(second[i]);
        const Point2D aligned = transformPoint(second[i], second_in_first);
        const double dx = aligned.x - first[i].x;
        const double dy = aligned.y - first[i].y;
        squared_error += dx * dx + dy * dy;
    }
    return std::sqrt(squared_error / static_cast<double>(first.size()));
}

std::vector<std::size_t> findNearestCorrespondences(const std::vector<Point2D>& first,
                                                    const std::vector<Point2D>& second,
                                                    const Pose2D& second_in_first) {
    if (first.empty() || second.empty()) {
        throw std::invalid_argument("correspondence search needs nonempty scans");
    }
    requireFinite(second_in_first);
    std::vector<std::size_t> matches;
    matches.reserve(second.size());
    for (const auto& point : second) {
        requireFinite(point);
        // Compare in ONE frame: move the second-scan point into first-sensor axes.
        const Point2D in_first = transformPoint(point, second_in_first);
        matches.push_back(findNearestPoint(in_first, first));
    }
    return matches;
}

std::size_t countInliers(const std::vector<Point2D>& first,
                         const std::vector<Point2D>& second,
                         const Pose2D& second_in_first,
                         double max_distance) {
    if (!(max_distance > 0.0)) {
        throw std::invalid_argument("inlier max_distance must be positive");
    }
    const auto matches = findNearestCorrespondences(first, second, second_in_first);
    std::size_t inliers = 0;
    for (std::size_t i = 0; i < second.size(); ++i) {
        const Point2D moved = transformPoint(second[i], second_in_first);
        const Point2D& partner = first[matches[i]];
        if (std::hypot(moved.x - partner.x, moved.y - partner.y) <= max_distance) ++inliers;
    }
    return inliers;
}
}  // namespace atlas
