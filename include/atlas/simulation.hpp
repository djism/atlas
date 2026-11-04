#pragma once
#include "atlas/types.hpp"
#include <vector>

namespace atlas {
// Ideal 360-degree point-landmark sensor; no noise or occlusion.
// Input landmarks: world frame. Output scan: sensor frame.
// Output ordering follows the visible input landmarks; indices are NOT identities.
std::vector<Point2D> simulateScan(const std::vector<Point2D>& landmarks,
                                const Pose2D& sensor_in_world, double max_range);
}  // namespace atlas
