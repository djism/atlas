#pragma once
#include "atlas/geometry3d.hpp"
#include "atlas/icp3d.hpp"
#include "atlas/posegraph3d.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
// Keyframes: the poses kept as pose-graph nodes. A frame becomes a keyframe
// when the sensor has moved at least `min_distance` meters along its path, or
// turned at least `min_angle` radians, since the last keyframe. Frame 0 is
// always the first keyframe. Standing still adds none.
struct KeyframeOptions {
    double min_distance = 5.0;
    double min_angle = 0.26;  // about 15 degrees
};
std::vector<std::size_t> selectKeyframes(const std::vector<Pose3D>& poses, const KeyframeOptions& options = {});

// Distance travelled along the poses from the first one, per pose (meters).
std::vector<double> pathLengths(const std::vector<Pose3D>& poses);

// A loop candidate: keyframe `later` may be a return to the place of keyframe
// `earlier` (indices into the keyframe list).
struct LoopCandidate {
    std::size_t earlier = 0;
    std::size_t later = 0;
    double distance = 0;  // between their ESTIMATED positions, meters
    double path_gap = 0;  // distance travelled from earlier to later, meters
};

// Candidates by proximity in the estimated map: for each keyframe, the
// nearest earlier keyframe within `max_distance` meters whose path gap is at
// least `min_path_gap` meters (so it is a return, not the stretch just
// driven). This only works while drift stays well below max_distance; with
// larger drift a real system needs place recognition (matching what the
// places look like), which Atlas does not have.
struct LoopSearchOptions {
    double max_distance = 10.0;
    double min_path_gap = 50.0;
};
std::vector<LoopCandidate> findLoopCandidates(const std::vector<Pose3D>& keyframes, const std::vector<double>& path_lengths,
                                              const LoopSearchOptions& options = {});

// Points of several scans, each placed with its pose (all poses in one world
// frame), expressed in the frame of `center` and thinned to one point per
// `voxel`-sized cube: a local map around one place. Throws
// std::invalid_argument if the counts differ or voxel is not positive.
std::vector<Point3D> buildSubmap(const std::vector<std::vector<Point3D>>& scans, const std::vector<Pose3D>& poses,
                                 const Pose3D& center, double voxel);

// Geometric verification of a loop candidate: align the later scan to a
// local map around the earlier place, starting from the odometry's guess of
// their relative pose, and accept only a strong fit. Two ICP passes (a
// coarse one with a wide gate for metre-level drift, then a fine one), then
// the fit is measured on STRUCTURE only (scan points higher than
// `structure_min_z` in the scan's own frame: walls, poles, buildings): the
// share with a map point within `inlier_distance`, and their RMS distance.
// The road surface is left out of the fit because flat ground matches flat
// ground on any street; it still helps ICP pin height and tilt.
struct LoopVerifyOptions {
    double source_voxel = 0.5;      // thin the scan to one point per cube this size (1.0 m: 8 cm error on the synthetic test, 0.5 m: 3.6 cm)
    double coarse_gate = 2.0;       // meters
    double fine_gate = 0.5;         // meters
    double inlier_distance = 0.3;   // meters
    double structure_min_z = -1.3;  // meters in the scan frame (KITTI: the road is 1.73 m below the sensor)
    double min_inlier_fraction = 0.6;
    double max_rmse = 0.15;         // meters, of the inliers
    double max_correction = 5.0;    // meters: a larger jump from the guess is not trusted
    std::size_t min_points = 200;   // of the thinned scan, and of its structure points
};

struct LoopVerification {
    bool accepted = false;
    Pose3D relative;  // later scan in the frame of the earlier place (meaningful only if accepted)
    double inlier_fraction = 0;  // of the structure points
    double rmse = 0;             // of the structure inliers, meters
    std::size_t structure_points = 0;
    double correction = 0;  // meters moved from the guess
    Icp3DStatus status = Icp3DStatus::TooFewPairs;
    const char* reason = "";  // why it was rejected ("" if accepted)
};

LoopVerification verifyLoop(const std::vector<Point3D>& submap, const std::vector<Point3D>& scan, const Pose3D& guess,
                            const LoopVerifyOptions& options = {});

// A verified loop between two keyframes (frame indices into the trajectory).
struct LoopEdge {
    std::size_t earlier_frame = 0;
    std::size_t later_frame = 0;
    Pose3D relative;  // later in earlier's frame, as verified
};

// Edge uncertainties (standard deviations; weights are 1 / sigma^2).
// Odometry between keyframes: grows with the distance d between them, at a
// drift rate measured on sequence 04 (0.48 % translation, 0.12 deg / 100 m),
// with floors so short edges are not treated as perfect. Loops: from how well
// the verified loops of sequence 07 agree with each other (2-9 cm).
struct LoopClosureOptions {
    double odometry_sigma_per_m = 0.0048;
    double odometry_sigma_floor = 0.02;         // m
    double odometry_rot_sigma_per_m = 2.1e-5;   // rad per m (0.12 deg / 100 m)
    double odometry_rot_sigma_floor = 0.002;    // rad
    double loop_sigma = 0.10;                   // m
    double loop_rot_sigma = 0.005;              // rad
    PoseGraphOptions graph;
};

struct LoopClosureResult {
    std::vector<Pose3D> poses;  // every frame, corrected
    PoseGraphResult graph;      // over the keyframes
    std::size_t loops = 0;
};

// Pose-graph loop closure over keyframes: one odometry edge between each pair
// of consecutive keyframes (their relative pose in the estimate), one edge per
// loop, optimized with keyframe 0 fixed. Every frame then moves rigidly with
// the keyframe at or before it: corrected_i = optimized_k * (estimate_k^-1 *
// estimate_i). With no loops the trajectory is returned unchanged. Throws
// std::invalid_argument if a loop's frames are not keyframes, keyframes are
// not increasing from frame 0, or a sigma is not positive.
LoopClosureResult closeLoops(const std::vector<Pose3D>& estimate, const std::vector<std::size_t>& keyframes,
                             const std::vector<LoopEdge>& loops, const LoopClosureOptions& options = {});
}  // namespace atlas
