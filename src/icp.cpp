#include "atlas/icp.hpp"
#include "atlas/geometry.hpp"
#include "atlas/matching.hpp"
#include <cmath>
#include <stdexcept>

namespace atlas {
IcpResult alignIcp(const std::vector<Point2D>& first,
                   const std::vector<Point2D>& second,
                   const Pose2D& initial_guess,
                   const IcpOptions& options) {
    if (options.max_iterations == 0) {
        throw std::invalid_argument("ICP needs at least one iteration");
    }
    if (!(options.translation_tolerance >= 0.0) || !std::isfinite(options.translation_tolerance) ||
        !(options.rotation_tolerance >= 0.0) || !std::isfinite(options.rotation_tolerance)) {
        throw std::invalid_argument("ICP tolerances must be finite and nonnegative");
    }
    // Infinity is allowed (gate off); NaN and nonpositive values are not.
    if (!(options.max_pair_distance > 0.0)) {
        throw std::invalid_argument("ICP max_pair_distance must be positive");
    }
    if (options.min_pairs < 2) {
        throw std::invalid_argument("ICP min_pairs must be at least 2");
    }
    if (second.size() < options.min_pairs) {
        throw std::invalid_argument("second scan has fewer points than ICP min_pairs");
    }

    IcpResult result;
    result.second_in_first = initial_guess;
    std::vector<Point2D> kept_first;
    std::vector<Point2D> kept_second;
    kept_first.reserve(second.size());
    kept_second.reserve(second.size());
    for (std::size_t pass = 0; pass < options.max_iterations; ++pass) {
        result.iterations = pass + 1;

        // 1. Guess pairs using the current pose (validates scans and pose).
        const auto matches = findNearestCorrespondences(first, second, result.second_in_first);

        // 2. Gate: keep pairs whose gap, in first-sensor axes, is within the limit.
        kept_first.clear();
        kept_second.clear();
        for (std::size_t i = 0; i < second.size(); ++i) {
            const Point2D moved = transformPoint(second[i], result.second_in_first);
            const Point2D& partner = first[matches[i]];
            if (std::hypot(moved.x - partner.x, moved.y - partner.y) <= options.max_pair_distance) {
                kept_first.push_back(partner);
                kept_second.push_back(second[i]);
            }
        }
        if (kept_second.size() < options.min_pairs) {
            result.status = IcpStatus::TooFewPairs;  // keep the last good pose
            return result;
        }

        // 3. Best full pose for the kept pairs.
        const Pose2D estimate = estimateFromMatchedPoints(kept_first, kept_second);
        result.nearest_pair_rmse.push_back(computeMatchedRmse(kept_first, kept_second, estimate));
        result.pose_per_pass.push_back(estimate);
        result.pairs_kept_per_pass.push_back(kept_second.size());

        // 4. Stop once a pass no longer changes the pose.
        const double moved = std::hypot(estimate.x - result.second_in_first.x,
                                        estimate.y - result.second_in_first.y);
        const double turn_change = estimate.theta - result.second_in_first.theta;
        const double turned = std::abs(std::atan2(std::sin(turn_change), std::cos(turn_change)));
        result.second_in_first = estimate;
        if (moved <= options.translation_tolerance && turned <= options.rotation_tolerance) {
            result.status = IcpStatus::Converged;
            return result;
        }
    }
    result.status = IcpStatus::HitIterationLimit;
    return result;
}
}  // namespace atlas
