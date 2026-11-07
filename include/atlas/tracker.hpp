#pragma once
#include "atlas/icp.hpp"
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
struct TrackerOptions {
    // Scan-to-scan registration settings. Point-to-line with a moderate gate,
    // which suits a good constant-velocity guess (see docs/icp.md).
    IcpOptions icp = [] {
        IcpOptions o;
        o.method = IcpMethod::PointToLine;
        o.max_pair_distance = 0.5;  // meters
        o.min_pairs = 10;
        o.max_iterations = 20;
        o.translation_tolerance = 1e-6;
        o.rotation_tolerance = 1e-6;
        return o;
    }();
};

struct TrackerStep {
    Pose2D pose_in_world{0, 0, 0};  // estimated sensor pose after this scan
    Pose2D motion{0, 0, 0};         // new sensor frame in the previous one
    IcpStatus status = IcpStatus::Converged;
    bool lost = false;  // registration failed; pose predicted from previous motion
    std::size_t pairs_kept = 0;
    std::size_t iterations = 0;
    double residual_rmse = 0.0;  // of the kept pairs, meters
};

// Scan-to-scan odometry: registers each new scan against the previous one with
// ICP, starting from a constant-velocity guess, and chains the motions with
// composePoses. Only the initial pose is given; there is no map, so errors
// accumulate (drift) and are never corrected. When registration fails
// (TooFewPairs, Degenerate, too few points), the step is marked lost and the
// pose is predicted from the previous motion. Scans are in current sensor axes.
class ScanTracker {
public:
    explicit ScanTracker(const Pose2D& initial_pose_in_world, TrackerOptions options = {});

    TrackerStep update(const std::vector<Point2D>& scan);
    const Pose2D& pose() const { return pose_; }
    std::size_t scansProcessed() const { return scans_; }

private:
    TrackerOptions options_;
    Pose2D pose_;
    Pose2D last_motion_{0, 0, 0};
    std::vector<Point2D> previous_;
    std::size_t scans_ = 0;
};
}  // namespace atlas
