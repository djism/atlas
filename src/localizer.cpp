#include "atlas/localizer.hpp"
#include "atlas/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace atlas {
namespace {
// Pose b expressed in pose a's frame: the motion that takes a to b.
Pose2D relativePose(const Pose2D& a, const Pose2D& b) {
    const Point2D offset = inverseTransformPoint({b.x, b.y}, a);
    return {offset.x, offset.y, std::remainder(b.theta - a.theta, 2 * std::acos(-1.0))};
}
}  // namespace

MapLocalizer::MapLocalizer(std::vector<Point2D> map_points, const Pose2D& initial_pose_in_map,
                           MapLocalizerOptions options)
    : options_(options), map_(std::move(map_points)), pose_(initial_pose_in_map) {
    if (!std::isfinite(pose_.x) || !std::isfinite(pose_.y) || !std::isfinite(pose_.theta))
        throw std::invalid_argument("localizer initial pose must be finite");
    if (map_.size() < std::max(options_.icp.min_pairs, options_.icp.normal_neighbors))
        throw std::invalid_argument("localizer map has too few points");
    if (options_.icp.method == IcpMethod::PointToLine)
        map_normals_ = estimateNormals(map_, options_.icp.normal_neighbors, options_.icp.max_spread_ratio);
}

LocalizerStep MapLocalizer::update(const std::vector<Point2D>& scan) {
    LocalizerStep step;
    ++scans_;
    // The first scan is registered too: unlike scan-to-scan tracking, there is
    // already something to compare against.
    const Pose2D guess = scans_ == 1 ? pose_ : composePoses(pose_, last_motion_);
    bool registered = false;
    if (scan.size() >= options_.icp.min_pairs) {
        // Map is "first", scan is "second": second_in_first is the sensor in the map.
        const IcpResult result = alignIcp(map_, map_normals_, scan, guess, options_.icp);
        step.status = result.status;
        step.iterations = result.iterations;
        if (!result.pairs_kept_per_pass.empty()) {
            step.pairs_kept = result.pairs_kept_per_pass.back();
            step.residual_rmse = result.nearest_pair_rmse.back();
            step.inlier_fraction = static_cast<double>(step.pairs_kept) / scan.size();
        }
        if (result.status == IcpStatus::Converged ||
            result.status == IcpStatus::HitIterationLimit) {
            Pose2D measured = result.second_in_first;
            step.observability = result.observability;
            if (step.observability.weak < options_.min_weak_observability) {
                // Replace the position component along the weak direction w with
                // the guess's: measured += ((guess - measured) . w) w. Across w,
                // and in heading, the scan's answer stands.
                const Point2D& w = step.observability.weak_direction;
                const double along = (guess.x - measured.x) * w.x + (guess.y - measured.y) * w.y;
                measured.x += along * w.x;
                measured.y += along * w.y;
                step.weakly_constrained = true;
            }
            const Pose2D correction = relativePose(guess, measured);
            step.jump_rejected =
                scans_ > 1 &&
                (std::hypot(correction.x, correction.y) > options_.max_correction_translation ||
                 std::abs(correction.theta) > options_.max_correction_rotation);
            if (!step.jump_rejected) {
                step.pose_in_map = measured;
                registered = true;
            }
        }
    } else {
        step.status = IcpStatus::TooFewPairs;
    }
    if (!registered) {
        step.lost = true;
        step.pose_in_map = guess;  // predict: keep moving as before
    }
    if (scans_ > 1) last_motion_ = relativePose(pose_, step.pose_in_map);
    pose_ = step.pose_in_map;
    return step;
}
}  // namespace atlas
