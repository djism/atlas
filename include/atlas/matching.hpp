#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
// Nonempty finite input. Returns the first nearest index in a tie.
std::size_t findNearestPoint(const Point2D& query, const std::vector<Point2D>& points);

// Educational two-correspondence estimator, not ICP.
// Inputs contain exactly two distinct points each, in KNOWN matching order.
// Returns the pose of the second sensor frame in the first sensor frame.
// This aligns second-scan points into the first frame.
Pose2D estimateFromTwoPairs(const std::vector<Point2D>& first,
                           const std::vector<Point2D>& second);
}  // namespace atlas
