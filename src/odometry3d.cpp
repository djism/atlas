#include "atlas/odometry3d.hpp"
#include "atlas/matching3d.hpp"
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

MapOdometry3D::MapOdometry3D(MapOdometryOptions options)
    : options_(options), map_(options.voxel_size, options.max_points_per_voxel) {}

double MapOdometry3D::threshold() const {
    return samples_ == 0 ? options_.initial_threshold : std::sqrt(model_sse_ / samples_);
}

MapOdometryStep MapOdometry3D::update(const std::vector<Point3D>& scan) {
    MapOdometryStep step;
    // 1. Range filter, then deskew with the last motion (constant velocity).
    std::vector<Point3D> frame;
    frame.reserve(scan.size());
    for (const Point3D& p : scan) {
        const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (r >= options_.min_range && r <= options_.max_range) frame.push_back(p);
    }
    if (options_.deskew) frame = deskewScan(frame, last_motion_, options_.sweep_direction);
    // 2. Two thinnings: for the map, and for registration.
    const std::vector<Point3D> for_map = voxelDownsample(frame, 0.5 * options_.voxel_size);
    const std::vector<Point3D> source = voxelDownsample(for_map, 1.5 * options_.voxel_size);
    step.source_points = source.size();
    // 3. Register against the map from the constant-velocity guess.
    const double sigma = threshold();
    step.sigma = sigma;
    const Pose3D guess = composePoses3D(pose_, last_motion_);
    Pose3D pose = guess;
    if (map_.size() > 0) {
        MapIcpOptions o;
        o.max_correspondence_distance = 3 * sigma;
        o.kernel = sigma;
        o.max_iterations = options_.max_iterations;
        const Icp3DResult r = alignToMap(map_, source, guess, o);
        step.status = r.status;
        step.iterations = r.iterations;
        step.pairs = r.pairs;
        if (r.status == Icp3DStatus::Converged || r.status == Icp3DStatus::HitIterationLimit) pose = r.source_in_reference;
    }
    // Adaptive threshold: how far the estimate moved from the prediction,
    // counting rotation as the motion of a point at max_range.
    const Pose3D deviation = relativePose3D(guess, pose);
    const Pose3D motion = relativePose3D(pose_, pose);
    const double moved = std::sqrt(motion.t.x * motion.t.x + motion.t.y * motion.t.y + motion.t.z * motion.t.z);
    if (moved > options_.min_motion) {
        const double error = std::sqrt(deviation.t.x * deviation.t.x + deviation.t.y * deviation.t.y +
                                       deviation.t.z * deviation.t.z) +
                             2 * options_.max_range * std::sin(rotationAngle(deviation) / 2);
        model_sse_ += error * error;
        ++samples_;
    }
    // 4. Update the map at the new pose; forget what is out of range.
    map_.insert(transformPoints(for_map, pose));
    map_.removeFarFrom(pose.t, options_.max_range);
    step.map_points = map_.size();
    // Snap both back to proper rotations: the motion is computed from two poses
    // and fed into the next guess, and without this, rounding error grew about
    // 2.5x per frame (orthonormality error 0.01 by frame 36, divergence by 44).
    last_motion_ = orthonormalized(motion);
    pose_ = orthonormalized(pose);
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

namespace atlas {
KittiMetric kittiMetric(const std::vector<Pose3D>& estimate, const std::vector<Pose3D>& truth) {
    if (estimate.size() != truth.size() || truth.size() < 2)
        throw std::invalid_argument("kittiMetric: need two equally long trajectories of 2+ poses");
    // Cumulative ground-truth path length (devkit: trajectoryDistances).
    std::vector<double> dist{0};
    for (std::size_t i = 1; i < truth.size(); ++i) {
        const Point3D a = truth[i - 1].t, b = truth[i].t;
        dist.push_back(dist.back() + std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)));
    }
    KittiMetric m;
    double t_sum = 0, r_sum = 0;
    for (std::size_t first = 0; first < truth.size(); first += 10) {
        for (double len = 100; len <= 800; len += 100) {
            std::size_t last = first;
            while (last < truth.size() && !(dist[last] > dist[first] + len)) ++last;  // lastFrameFromSegmentLength
            if (last == truth.size()) continue;
            const Pose3D delta_truth = relativePose3D(truth[first], truth[last]);
            const Pose3D delta_estimate = relativePose3D(estimate[first], estimate[last]);
            const Pose3D error = relativePose3D(delta_estimate, delta_truth);  // inv(estimate) * truth
            t_sum += std::sqrt(error.t.x * error.t.x + error.t.y * error.t.y + error.t.z * error.t.z) / len;
            r_sum += rotationAngle(error) / len;
            ++m.segments;
        }
    }
    if (m.segments) {
        m.translation_percent = 100 * t_sum / m.segments;
        m.rotation_deg_per_100m = 100 * (r_sum / m.segments) * 180 / std::acos(-1.0);
    }
    return m;
}
}  // namespace atlas
