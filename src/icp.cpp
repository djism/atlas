#include "atlas/icp.hpp"
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

    IcpResult result;
    result.second_in_first = initial_guess;
    std::vector<Point2D> paired_first(second.size());
    for (std::size_t pass = 0; pass < options.max_iterations; ++pass) {
        // 1. Guess pairs using the current pose (validates scans and pose).
        const auto matches = findNearestCorrespondences(first, second, result.second_in_first);
        for (std::size_t i = 0; i < second.size(); ++i) paired_first[i] = first[matches[i]];

        // 2. Best full pose for those pairs.
        const Pose2D estimate = estimateFromMatchedPoints(paired_first, second);
        result.nearest_pair_rmse.push_back(computeMatchedRmse(paired_first, second, estimate));
        result.pose_per_pass.push_back(estimate);
        result.iterations = pass + 1;

        // 3. Stop once a pass no longer changes the pose.
        const double moved = std::hypot(estimate.x - result.second_in_first.x,
                                        estimate.y - result.second_in_first.y);
        const double turn_change = estimate.theta - result.second_in_first.theta;
        const double turned = std::abs(std::atan2(std::sin(turn_change), std::cos(turn_change)));
        result.second_in_first = estimate;
        if (moved <= options.translation_tolerance && turned <= options.rotation_tolerance) {
            result.converged = true;
            break;
        }
    }
    return result;
}
}  // namespace atlas
