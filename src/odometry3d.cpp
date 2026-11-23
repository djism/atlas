#include "atlas/odometry3d.hpp"
#include "atlas/voxel.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace atlas {
ScanOdometry3D::ScanOdometry3D(ScanOdometryOptions options) : options_(options) {}

ScanOdometryStep ScanOdometry3D::update(const std::vector<Point3D>& scan) {
    ScanOdometryStep step;
    // Deskew with the prediction: the sensor moves about as much during this
    // sweep as it did between the last two scans.
    const std::vector<Point3D> deskewed =
        options_.deskew && started_ ? deskewScan(scan, last_motion_, options_.sweep_direction) : scan;
    const std::vector<Point3D> source = voxelDownsample(deskewed, options_.source_voxel);
    step.source_points = source.size();
    if (started_) {
        Pose3D motion = last_motion_;  // constant-velocity guess
        if (options_.use_icp) {
            VoxelGrid3D grid(options_.icp.max_correspondence_distance);
            grid.insert(voxelDownsample(previous_, options_.reference_voxel));
            const Icp3DResult r = alignIcp3D(grid, source, motion, options_.icp);
            step.status = r.status;
            step.iterations = r.iterations;
            step.pairs = r.pairs;
            step.rmse = r.rmse;
            // A failed registration keeps the guess (dead reckoning for this scan).
            if (r.status == Icp3DStatus::Converged || r.status == Icp3DStatus::HitIterationLimit)
                motion = r.source_in_reference;
        }
        step.motion = motion;
        pose_ = composePoses3D(pose_, motion);
        last_motion_ = motion;
    }
    started_ = true;
    // Kept for the next match, deskewed with the motion just estimated (for the
    // first scan that motion is zero, which leaves the scan unchanged).
    previous_ = options_.deskew ? deskewScan(scan, step.motion, options_.sweep_direction) : scan;
    step.pose = pose_;
    return step;
}

TrajectoryComparison compareTrajectories(const std::vector<Pose3D>& estimate, const std::vector<Pose3D>& truth) {
    if (estimate.size() != truth.size() || truth.size() < 2)
        throw std::invalid_argument("compareTrajectories: need two equally long trajectories of 2+ poses");
    TrajectoryComparison c;
    const auto dist = [](const Point3D& a, const Point3D& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    };
    for (std::size_t i = 1; i < truth.size(); ++i) {
        const Pose3D t = relativePose3D(truth[i - 1], truth[i]);
        const Pose3D e = relativePose3D(estimate[i - 1], estimate[i]);
        c.step_translation_error.push_back(dist(t.t, e.t));
        c.step_rotation_error.push_back(rotationAngle(relativePose3D(t, e)));
        const double length = std::sqrt(t.t.x * t.t.x + t.t.y * t.t.y + t.t.z * t.t.z);
        const double along = length > 0 ? (e.t.x * t.t.x + e.t.y * t.t.y + e.t.z * t.t.z) / length : 0;
        c.step_forward_error.push_back(along - length);
        c.length += length;
        c.worst_error = std::max(c.worst_error, dist(estimate[i].t, truth[i].t));
    }
    c.end_error = dist(estimate.back().t, truth.back().t);
    return c;
}
}  // namespace atlas
