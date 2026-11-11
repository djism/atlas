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

// Simulated wheel odometry: what wheel encoders report for one step of true
// motion. Both motions are the sensor frame after the step expressed in the
// frame before it (meters, radians). Simplified from the standard odometry
// motion model (Thrun, Burgard, Fox, "Probabilistic Robotics", ch. 5): each
// standard deviation is proportional to the step, so errors are per step and
// grow with distance driven and angle turned. All values are assumptions, not
// measurements of a real robot.
struct OdometryNoise {
    // Systematic distance error: reported = true * (1 + bias). -0.02 means the
    // wheels report 2 % less than the robot moved (e.g. wrong wheel radius).
    double translation_bias = 0.0;
    double translation_fraction = 0.0;  // random distance error: stddev / distance
    double rotation_fraction = 0.0;     // random turn error: stddev / angle turned
    double rotation_per_meter = 0.0;    // heading error from driving: radians stddev per meter
};

// Reported motion = true translation scaled by (1 + bias + translation_fraction * n1),
// heading = true + rotation_fraction * |turn| * n2 + rotation_per_meter * distance * n3,
// with n1..n3 standard Gaussians from `seed` (same generator as addGaussianNoise).
// Zero noise returns the true motion. Throws std::invalid_argument for a
// nonfinite motion or noise, or negative standard deviations.
Pose2D simulateOdometry(const Pose2D& true_motion, const OdometryNoise& noise, std::uint64_t seed);

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
