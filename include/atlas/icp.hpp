#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <limits>
#include <vector>

namespace atlas {
struct IcpOptions {
    std::size_t max_iterations = 30;
    // Stop when one pass changes the pose by less than both tolerances.
    double translation_tolerance = 1e-9;  // meters
    double rotation_tolerance = 1e-9;     // radians
    // Distance gate (meters, first-sensor axes): before solving, drop pairs whose
    // points are farther apart than this under the CURRENT guess. Infinity = off.
    // Too tight early on rejects true pairs while the guess is still poor.
    double max_pair_distance = std::numeric_limits<double>::infinity();
    // Fewer pairs surviving the gate than this ends ICP with status TooFewPairs.
    // A second scan smaller than this is invalid input (exception), not "lost".
    std::size_t min_pairs = 2;
};

enum class IcpStatus {
    Converged,          // pose (and so pairing) stopped changing: stable, not correct
    HitIterationLimit,  // still changing after max_iterations
    TooFewPairs,        // gate left fewer than min_pairs: treat as localization lost
};

struct IcpResult {
    // Pose of the second sensor frame in the first frame. For TooFewPairs, the
    // last estimate before the failing pass (possibly the initial guess).
    Pose2D second_in_first;
    IcpStatus status = IcpStatus::HitIterationLimit;
    std::size_t iterations = 0;  // passes started, including a failing one
    // Per completed pass: RMSE over KEPT nearest-neighbor pairs (may be wrong
    // pairs; a gate lowers this by construction), pose, and kept-pair count.
    std::vector<double> nearest_pair_rmse;
    std::vector<Pose2D> pose_per_pass;
    std::vector<std::size_t> pairs_kept_per_pass;

    bool converged() const { return status == IcpStatus::Converged; }
};

// Point-to-point 2D ICP. Each pass pairs every second-scan point with its
// nearest first-scan point under the current guess, drops pairs beyond the
// distance gate, then solves for the full pose with estimateFromMatchedPoints.
// Local method: a poor initial guess or a repetitive scene can converge to a
// wrong pose. Throws std::invalid_argument for invalid input or options, or
// when kept pairs are unusable by the estimator (e.g. all to one point).
IcpResult alignIcp(const std::vector<Point2D>& first,
                   const std::vector<Point2D>& second,
                   const Pose2D& initial_guess,
                   const IcpOptions& options = {});
}  // namespace atlas
