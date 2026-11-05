#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
struct IcpOptions {
    std::size_t max_iterations = 30;
    // Stop when one pass changes the pose by less than both tolerances.
    double translation_tolerance = 1e-9;  // meters
    double rotation_tolerance = 1e-9;     // radians
};

struct IcpResult {
    Pose2D second_in_first;  // pose of the second sensor frame in the first frame
    bool converged = false;  // pose (and so pairing) stopped changing before the limit
    std::size_t iterations = 0;
    // RMSE after each pass over NEAREST-NEIGHBOR pairs, which may be wrong.
    // Low values mean the scans overlap well, not that the pose is correct.
    std::vector<double> nearest_pair_rmse;
    // Pose estimated by each pass (same order as nearest_pair_rmse).
    std::vector<Pose2D> pose_per_pass;
};

// Point-to-point 2D ICP. Each pass pairs every second-scan point with its
// nearest first-scan point under the current guess, then solves for the full
// pose with estimateFromMatchedPoints. Local method: a poor initial guess or a
// repetitive scene can converge to a wrong pose. "converged" means stable,
// not correct. Throws std::invalid_argument for invalid input or options, or
// when a pass yields pairs the estimator cannot use (e.g. all to one point).
IcpResult alignIcp(const std::vector<Point2D>& first,
                   const std::vector<Point2D>& second,
                   const Pose2D& initial_guess,
                   const IcpOptions& options = {});
}  // namespace atlas
