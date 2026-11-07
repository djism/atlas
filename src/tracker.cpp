#include "atlas/tracker.hpp"
#include "atlas/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace atlas {
ScanTracker::ScanTracker(const Pose2D& initial_pose_in_world, TrackerOptions options)
    : options_(options), pose_(initial_pose_in_world) {
    if (!std::isfinite(pose_.x) || !std::isfinite(pose_.y) || !std::isfinite(pose_.theta)) {
        throw std::invalid_argument("tracker initial pose must be finite");
    }
}

TrackerStep ScanTracker::update(const std::vector<Point2D>& scan) {
    TrackerStep step;
    ++scans_;
    if (previous_.empty()) {  // first scan: nothing to register against yet
        previous_ = scan;
        step.pose_in_world = pose_;
        return step;
    }

    // Point-to-line needs enough points for line fits and the minimum pairs.
    const std::size_t needed =
        std::max(options_.icp.min_pairs, options_.icp.normal_neighbors);
    bool registered = false;
    if (scan.size() >= needed && previous_.size() >= needed) {
        // Previous scan is "first", new scan is "second": the result is the new
        // sensor frame expressed in the previous sensor frame.
        const IcpResult result = alignIcp(previous_, scan, last_motion_, options_.icp);
        step.status = result.status;
        step.iterations = result.iterations;
        if (!result.pairs_kept_per_pass.empty()) {
            step.pairs_kept = result.pairs_kept_per_pass.back();
            step.residual_rmse = result.nearest_pair_rmse.back();
        }
        if (result.status == IcpStatus::Converged ||
            result.status == IcpStatus::HitIterationLimit) {
            step.motion = result.second_in_first;
            registered = true;
        }
    } else {
        step.status = IcpStatus::TooFewPairs;
    }

    if (!registered) {
        step.lost = true;
        step.motion = last_motion_;  // predict: keep moving as before
    }
    last_motion_ = step.motion;
    pose_ = composePoses(pose_, step.motion);
    previous_ = scan;
    step.pose_in_world = pose_;
    return step;
}
}  // namespace atlas
