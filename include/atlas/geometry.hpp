#pragma once
#include "atlas/types.hpp"
#include <vector>

namespace atlas {
// source -> destination: R(theta) * point + translation.
Point2D transformPoint(const Point2D& point, const Pose2D& source_in_destination);
// destination -> source: R(theta)^T * (point - translation).
Point2D inverseTransformPoint(const Point2D& point, const Pose2D& source_in_destination);

// Mean of a nonempty point set, in the same frame as the inputs.
// Throws std::invalid_argument for an empty set.
Point2D computeCentroid(const std::vector<Point2D>& points);
}  // namespace atlas
