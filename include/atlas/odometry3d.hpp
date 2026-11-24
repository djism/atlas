#pragma once
#include "atlas/geometry3d.hpp"
#include "atlas/icp3d.hpp"
#include "atlas/kitti.hpp"
#include "atlas/voxel.hpp"
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

// Scan-to-MAP LiDAR odometry, following KISS-ICP's design (Vizzo et al.,
// "KISS-ICP: In Defense of Point-to-Point ICP", RA-L 2023), implemented here:
//   1. drop points outside [min_range, max_range]; deskew with the last motion;
//   2. thin to one point per voxel/2 cube (kept for the map) and again per
//      1.5*voxel cube (used to register);
//   3. register against the local map from the constant-velocity guess, pairs
//      within 3 sigma, robust kernel sigma, where sigma is learned from how far
//      past estimates moved from their predictions (adaptive threshold);
//   4. add the voxel/2 points to the map at the new pose; drop map cubes
//      farther than max_range.
struct MapOdometryOptions {
    double voxel_size = 1.0;            // meters (KISS-ICP default: max_range / 100)
    double max_range = 100.0;           // meters
    double min_range = 0.0;
    std::size_t max_points_per_voxel = 20;
    double initial_threshold = 2.0;     // sigma before any motion was seen, meters
    double min_motion = 0.1;            // frames moving less than this do not update sigma
    bool deskew = true;
    SweepDirection sweep_direction = SweepDirection::Clockwise;
    std::size_t max_iterations = 500;
};

struct MapOdometryStep {
    Pose3D pose;     // sensor at this scan, in the first scan's sensor frame
    Icp3DStatus status = Icp3DStatus::Converged;
    std::size_t iterations = 0;
    std::size_t pairs = 0;
    std::size_t source_points = 0;
    std::size_t map_points = 0;
    double sigma = 0;  // adaptive threshold used for this scan, meters
};

class MapOdometry3D {
public:
    explicit MapOdometry3D(MapOdometryOptions options = {});
    MapOdometryStep update(const std::vector<Point3D>& scan);
    const LocalMap3D& map() const { return map_; }

private:
    double threshold() const;
    MapOdometryOptions options_;
    LocalMap3D map_;
    Pose3D pose_{};
    Pose3D last_motion_{};
    double model_sse_ = 0;
    std::size_t samples_ = 0;
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

// The KITTI odometry benchmark's metric, ported from the devkit's
// evaluate_odometry.cpp: for every 10th frame and every segment length of
// 100, 200, ..., 800 m (by ground-truth path length), the error of the
// estimated relative pose over that segment, divided by the length. Both
// trajectories must be in the devkit's frame (left camera; see cameraPoses).
struct KittiMetric {
    double translation_percent = 0;     // mean over segments, % of distance
    double rotation_deg_per_100m = 0;   // mean over segments
    std::size_t segments = 0;           // 0 if the drive is shorter than 100 m
};
KittiMetric kittiMetric(const std::vector<Pose3D>& estimate, const std::vector<Pose3D>& truth);

// Throws std::invalid_argument for different lengths or fewer than 2 poses.
TrajectoryComparison compareTrajectories(const std::vector<Pose3D>& estimate, const std::vector<Pose3D>& truth);
}  // namespace atlas
