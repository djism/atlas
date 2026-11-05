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

// Least-squares rigid alignment for KNOWN, index-matched pairs:
// first[i] (first-sensor axes) observes the same landmark as second[i]
// (second-sensor axes). Returns the pose of the second sensor frame in the
// first sensor frame, minimizing sum |first[i] - (R * second[i] + t)|^2.
// Does not discover correspondences; wrong pairings give a wrong pose.
// Throws std::invalid_argument for unequal sizes, fewer than two pairs,
// nonfinite coordinates, all-coincident points, or an ambiguous rotation.
Pose2D estimateFromMatchedPoints(const std::vector<Point2D>& first,
                                 const std::vector<Point2D>& second);

// Root-mean-square distance (meters) between first[i] and second[i] after
// mapping second[i] into the first frame with second_in_first. Evaluates
// KNOWN pairs only; it is not a nearest-neighbor residual.
// Throws std::invalid_argument for empty or unequal sets or nonfinite input.
double computeMatchedRmse(const std::vector<Point2D>& first,
                          const std::vector<Point2D>& second,
                          const Pose2D& second_in_first);

// Correspondence GUESS for one ICP pass. Maps each second[i] (second-sensor
// axes) into the first frame using the guessed pose second_in_first, then
// returns matches where matches[i] is the index of the nearest point in first.
// Several second points may share one first point; nearest does not mean same
// landmark. Results depend on the guess. Brute force: O(first * second).
// Throws std::invalid_argument for empty scans or nonfinite input.
std::vector<std::size_t> findNearestCorrespondences(const std::vector<Point2D>& first,
                                                    const std::vector<Point2D>& second,
                                                    const Pose2D& second_in_first);
}  // namespace atlas
