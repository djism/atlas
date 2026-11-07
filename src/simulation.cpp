#include "atlas/simulation.hpp"
#include "atlas/geometry.hpp"
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace atlas {
std::vector<Point2D> simulateScan(const std::vector<Point2D>& landmarks,
                                const Pose2D& pose, double max_range) {
    if (!std::isfinite(max_range) || max_range < 0.0) {
        throw std::invalid_argument("max_range must be finite and nonnegative");
    }
    std::vector<Point2D> scan;
    for (const auto& landmark : landmarks) {
        const auto point = inverseTransformPoint(landmark, pose);
        if (std::hypot(point.x, point.y) <= max_range) {
            scan.push_back(point);
        }
    }
    return scan;
}

namespace {
// Uniform double in (0, 1]: top 53 bits of a 64-bit draw, shifted off zero
// so log(u) below is always finite.
double uniformOpenZero(std::mt19937_64& generator) {
    return (static_cast<double>(generator() >> 11) + 1.0) * 0x1.0p-53;
}
// Box-Muller: two uniforms -> one standard Gaussian (mean 0, standard deviation 1).
double standardGaussian(std::mt19937_64& generator) {
    const double u1 = uniformOpenZero(generator);
    const double u2 = uniformOpenZero(generator);
    const double two_pi = 2.0 * std::acos(-1.0);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(two_pi * u2);
}
}  // namespace

std::vector<Point2D> addGaussianNoise(const std::vector<Point2D>& points,
                                      double stddev, std::uint64_t seed) {
    if (!std::isfinite(stddev) || stddev < 0.0) {
        throw std::invalid_argument("noise stddev must be finite and nonnegative");
    }
    std::mt19937_64 generator(seed);
    std::vector<Point2D> noisy = points;
    for (auto& point : noisy) {
        point.x += stddev * standardGaussian(generator);
        point.y += stddev * standardGaussian(generator);
    }
    return noisy;
}

namespace {
double cross(const Point2D& v, const Point2D& w) { return v.x * w.y - v.y * w.x; }

void validateLidar(const LidarConfig& config) {
    if (config.beam_count == 0) {
        throw std::invalid_argument("LiDAR needs at least one beam");
    }
    if (!(config.max_range > 0.0) || !std::isfinite(config.max_range)) {
        throw std::invalid_argument("LiDAR max_range must be finite and positive");
    }
}
}  // namespace

double beamAngle(std::size_t beam, const LidarConfig& config) {
    validateLidar(config);
    return 2.0 * std::acos(-1.0) * static_cast<double>(beam) /
           static_cast<double>(config.beam_count);
}

std::vector<double> simulateLidarRanges(const std::vector<Segment2D>& walls,
                                        const Pose2D& pose,
                                        const LidarConfig& config) {
    validateLidar(config);
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        throw std::invalid_argument("LiDAR pose must be finite");
    }
    for (const auto& wall : walls) {
        if (!std::isfinite(wall.start.x) || !std::isfinite(wall.start.y) ||
            !std::isfinite(wall.end.x) || !std::isfinite(wall.end.y)) {
            throw std::invalid_argument("wall coordinates must be finite");
        }
        if (wall.start.x == wall.end.x && wall.start.y == wall.end.y) {
            throw std::invalid_argument("walls must have nonzero length");
        }
    }

    const Point2D origin{pose.x, pose.y};  // world frame
    std::vector<double> ranges(config.beam_count, std::numeric_limits<double>::infinity());
    for (std::size_t beam = 0; beam < config.beam_count; ++beam) {
        // Beam direction in WORLD axes: sensor heading plus the beam's own angle.
        const double angle = pose.theta + beamAngle(beam, config);
        const Point2D d{std::cos(angle), std::sin(angle)};  // unit length
        for (const auto& wall : walls) {
            const Point2D e{wall.end.x - wall.start.x, wall.end.y - wall.start.y};
            const double denominator = cross(d, e);
            // Parallel (scale-aware: |d| = 1, compare with the wall's length).
            if (std::abs(denominator) <= 1e-12 * std::hypot(e.x, e.y)) continue;
            const Point2D to_start{wall.start.x - origin.x, wall.start.y - origin.y};
            const double t = cross(to_start, e) / denominator;  // meters along the beam
            const double u = cross(to_start, d) / denominator;  // 0..1 along the wall
            if (t >= 0.0 && u >= 0.0 && u <= 1.0 && t <= config.max_range && t < ranges[beam]) {
                ranges[beam] = t;
            }
        }
    }
    return ranges;
}

std::vector<Point2D> rangesToPoints(const std::vector<double>& ranges,
                                    const LidarConfig& config) {
    validateLidar(config);
    if (ranges.size() != config.beam_count) {
        throw std::invalid_argument("ranges must have one entry per beam");
    }
    std::vector<Point2D> points;
    for (std::size_t beam = 0; beam < ranges.size(); ++beam) {
        if (!std::isfinite(ranges[beam])) continue;  // no return
        const double angle = beamAngle(beam, config);
        points.push_back({ranges[beam] * std::cos(angle), ranges[beam] * std::sin(angle)});
    }
    return points;
}
}  // namespace atlas
