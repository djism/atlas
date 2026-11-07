#pragma once
#include "atlas/types.hpp"
#include <cstddef>
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

// Planar spinning LiDAR. Beam i points at angle i * 2*pi / beam_count in sensor
// axes (beam 0 = +x forward, counterclockwise).
struct LidarConfig {
    std::size_t beam_count = 360;
    double max_range = 10.0;  // meters
};

// Angle of beam i in sensor axes (radians).
double beamAngle(std::size_t beam, const LidarConfig& config);

// Range per beam (meters) to the NEAREST wall segment hit, given in world frame.
// Infinity means no return (nothing within max_range). Walls occlude whatever is
// behind them. A beam parallel to a wall does not hit it, even when sliding
// exactly along it. Endpoints count as hits.
// Throws std::invalid_argument for zero beams, nonpositive or nonfinite range,
// zero-length or nonfinite walls, or a nonfinite pose.
std::vector<double> simulateLidarRanges(const std::vector<Segment2D>& walls_in_world,
                                        const Pose2D& sensor_in_world,
                                        const LidarConfig& config);

// Converts finite ranges to points in SENSOR axes; beams without a return are
// skipped, so output indices are not beam indices.
std::vector<Point2D> rangesToPoints(const std::vector<double>& ranges,
                                    const LidarConfig& config);
}  // namespace atlas
