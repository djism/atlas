#pragma once
#include "atlas/geometry3d.hpp"
#include "atlas/types.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace atlas {
// One KITTI Velodyne scan (HDL-64E, one 360-degree sweep, ~120k points).
// Points are in the VELODYNE frame: origin at the sensor, x forward, y left,
// z up, meters. Raw-data scans are NOT motion-compensated: the sensor moves
// during the 0.1 s sweep. (The odometry benchmark's scans are compensated.)
struct KittiScan {
    std::vector<Point3D> points;
    std::vector<float> reflectance;  // 0 to 1, one per point, same order
};

// Reads a KITTI .bin scan: consecutive little-endian float32 records
// (x, y, z, reflectance), 16 bytes per point, count = file size / 16 (KITTI
// devkit readme). An empty file gives an empty scan.
// Throws std::runtime_error if the file cannot be read, its size is not a
// multiple of 16 bytes (truncated or not a scan), a value is not finite, or
// the machine is not little-endian.
KittiScan readKittiScan(const std::string& path);

// Ground truth (odometry dataset/poses/NN.txt): one line per frame, 12 numbers,
// the row-major 3x4 matrix [R | t] of the LEFT CAMERA at frame i expressed in
// the left camera at frame 0. Camera axes: x right, y down, z forward.
// Throws std::runtime_error for an unreadable file, a line without exactly 12
// finite numbers, or a rotation that is not orthonormal within 1e-4.
std::vector<Pose3D> readKittiPoses(const std::string& path);

// `Tr` from an odometry calib.txt: the Velodyne frame in the left camera frame,
// i.e. camera_point = Tr * velodyne_point. Throws std::runtime_error if the
// file or a valid Tr line is missing.
Pose3D readKittiVelodyneInCamera(const std::string& calib_path);

// Frame times in seconds from an odometry times.txt, one per line.
// Throws std::runtime_error for an unreadable file or a non-number.
std::vector<double> readKittiTimes(const std::string& path);

// Ground truth re-expressed for the Velodyne: frame i in Velodyne frame 0,
//   velodyne_i_in_velodyne_0 = Tr^-1 * camera_i_in_camera_0 * Tr
// (read right to left: Velodyne i -> camera i -> camera 0 -> Velodyne 0).
std::vector<Pose3D> velodynePoses(const std::vector<Pose3D>& camera_poses, const Pose3D& velodyne_in_camera);

// Which way the laser head turns, seen from above (+z): Clockwise means the
// azimuth atan2(y, x) DEcreases with time.
enum class SweepDirection { Clockwise, Counterclockwise };

// Time of a point within its sweep, as a fraction 0..1, from its azimuth. KITTI
// raw sweeps start and end facing backwards and face forward (+x) at the
// middle, when the cameras fire and the ground-truth pose is taken (raw
// timestamps: the scan time is the mean of start and end).
double sweepFraction(const Point3D& point, SweepDirection direction);

// Removes the motion smear from one sweep. `sweep_motion` is the sensor's
// motion over one whole sweep (the sensor frame at the sweep's end, in the
// frame at its start); constant velocity is assumed within the sweep. Each
// point, measured from the sensor at fraction s, is re-expressed from the
// sensor at the middle (s = 0.5): p_mid = scaleMotion(sweep_motion, s - 0.5) * p.
std::vector<Point3D> deskewScan(const std::vector<Point3D>& points, const Pose3D& sweep_motion,
                                SweepDirection direction);

// File of frame `index` in a KITTI velodyne data folder: dir/0000000042.bin.
std::string kittiScanPath(const std::string& dir, std::size_t index);

// Quick facts about a scan, for sanity checks against the sensor's geometry.
struct ScanSummary {
    std::size_t points = 0;
    double min_range = 0, median_range = 0, max_range = 0;  // meters from the sensor
    double median_z = 0;          // meters; about -1.7 when most points hit the road
    double forward_share = 0;     // share of points with x > 0
    float max_reflectance = 0;
};

// Throws std::invalid_argument for an empty scan.
ScanSummary summarizeScan(const KittiScan& scan);
}  // namespace atlas
