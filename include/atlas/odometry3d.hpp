#pragma once
#include "atlas/geometry3d.hpp"
#include "atlas/icp3d.hpp"
#include "atlas/kitti.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
struct ScanOdometryOptions {
    // The previous scan is thinned to one point per `reference_voxel` cube and
    // filed in a grid of `max_correspondence_distance` cubes; the new scan is
    // thinned to one point per `source_voxel` cube before matching.
    double reference_voxel = 0.25;  // meters
    double source_voxel = 1.0;      // meters
    Icp3DOptions icp;
    bool use_icp = true;  // false: constant velocity only
    // Remove the sweep smear before matching: the new scan is deskewed with the
    // predicted motion (the previous estimate), and stored for the next match
    // deskewed with the motion just estimated. Needs the scan's points in
    // measurement-time order around the sensor, as KITTI raw sweeps are.
    bool deskew = false;
    SweepDirection sweep_direction = SweepDirection::Clockwise;  // KITTI HDL-64E (measured)
};

struct ScanOdometryStep {
    Pose3D pose;    // sensor at this scan, in the sensor frame of the first scan
    Pose3D motion;  // this scan's frame in the previous scan's frame
    Icp3DStatus status = Icp3DStatus::Converged;
    std::size_t iterations = 0;
    std::size_t pairs = 0;
    std::size_t source_points = 0;  // after thinning
    double rmse = 0;
};

// Scan-to-scan LiDAR odometry: registers each scan against the previous one,
// starting from a constant-velocity guess (repeat the previous motion), and
// chains the motions. Like the 2D ScanTracker, errors accumulate (drift).
// Points: the sensor frame of each scan (KITTI: Velodyne, x forward, y left, z up).
class ScanOdometry3D {
public:
    explicit ScanOdometry3D(ScanOdometryOptions options = {});
    ScanOdometryStep update(const std::vector<Point3D>& scan);

private:
    ScanOdometryOptions options_;
    Pose3D pose_{};
    Pose3D last_motion_{};
    std::vector<Point3D> previous_;
    bool started_ = false;
};

// Estimated vs answer-key trajectory (same frames, same first pose).
struct TrajectoryComparison {
    std::vector<double> step_translation_error;  // |estimated motion - true motion|, meters, per frame
    std::vector<double> step_rotation_error;     // angle between estimated and true motion, radians
    std::vector<double> step_forward_error;      // estimated - true motion along the true motion's x, meters
    double length = 0;       // of the true path, meters
    double end_error = 0;    // distance between final positions, meters
    double worst_error = 0;  // largest position error over the run, meters
};

// Throws std::invalid_argument for different lengths or fewer than 2 poses.
TrajectoryComparison compareTrajectories(const std::vector<Pose3D>& estimate, const std::vector<Pose3D>& truth);
}  // namespace atlas
