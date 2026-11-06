#include "atlas/simulation.hpp"
#include "atlas/geometry.hpp"
#include <cmath>
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
}  // namespace atlas
