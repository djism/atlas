#pragma once
#include "atlas/geometry3d.hpp"
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
// Every `step`-th point, starting with the first (step 1 = all).
// Throws std::invalid_argument for step 0.
std::vector<Point3D> takeEvery(const std::vector<Point3D>& points, std::size_t step);

// Each point moved from its source frame into the destination frame.
std::vector<Point3D> transformPoints(const std::vector<Point3D>& points, const Pose3D& source_in_destination);

// For each query point (moved into the reference frame by query_in_reference),
// the distance to its nearest reference point, meters. Brute force, O(n * m):
// meant for checks on a few thousand queries, not for ICP (a spatial index is
// the known fix). Throws std::invalid_argument for an empty reference.
std::vector<double> nearestDistances(const std::vector<Point3D>& reference, const std::vector<Point3D>& queries,
                                     const Pose3D& query_in_reference);
}  // namespace atlas
