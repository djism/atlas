#pragma once
#include "atlas/types.hpp"
#include <cstdint>
#include <vector>

namespace atlas {
// Ideal 360-degree point-landmark sensor; no noise or occlusion.
// Input landmarks: world frame. Output scan: sensor frame.
// Output ordering follows the visible input landmarks; indices are NOT identities.
std::vector<Point2D> simulateScan(const std::vector<Point2D>& landmarks,
                                const Pose2D& sensor_in_world, double max_range);

// Returns a copy with independent zero-mean Gaussian noise (standard deviation
// stddev, meters) added to each coordinate, in the points' own frame. Simplified
// model: real LiDAR error is mostly along the beam. The same seed gives the same
// noise on every platform (Box-Muller over std::mt19937_64; std::normal_distribution
// output differs between standard libraries). stddev == 0 returns points unchanged.
// Throws std::invalid_argument for negative or nonfinite stddev.
std::vector<Point2D> addGaussianNoise(const std::vector<Point2D>& points,
                                      double stddev, std::uint64_t seed);
}  // namespace atlas
