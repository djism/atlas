#pragma once
#include "atlas/icp.hpp"
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
struct MapLocalizerOptions {
    // Scan-to-map registration. Point-to-line: map points lie along walls.
    IcpOptions icp = [] {
        IcpOptions o;
        o.method = IcpMethod::PointToLine;
        o.normal_neighbors = 8;     // map walls are 1-2 cells thick; fit over more points
        o.max_spread_ratio = 0.1;
        o.max_pair_distance = 0.3;  // meters
        o.min_pairs = 20;
        o.max_iterations = 20;
        o.translation_tolerance = 1e-5;
        o.rotation_tolerance = 1e-5;
        return o;
    }();
    // Motion sanity check: a result farther than this from the guess is treated
    // as a failed registration (the robot cannot jump). It stops one bad slide
    // (e.g. along parallel walls) from becoming the next guess's velocity.
    double max_correction_translation = 0.15;  // meters
    double max_correction_rotation = 0.2;      // radians
    // Degeneracy handling: if the weakest translation direction gets less than
    // this share of the evidence (TranslationObservability::weak, 0 to 0.5), the
    // scan cannot fix position along it. ICP's answer is then kept only ACROSS
    // that direction; along it, the prediction (guess) is kept. 0 disables.
    double min_weak_observability = 0.05;  // chosen from two 3 m-range runs; see docs
};

struct LocalizerStep {
    Pose2D pose_in_map{0, 0, 0};  // estimated sensor pose after this scan
    IcpStatus status = IcpStatus::Converged;
    bool lost = false;  // registration failed or rejected; pose predicted
    bool jump_rejected = false;  // ICP answered, but too far from the guess
    // The scan did not constrain position along observability.weak_direction
    // (map axes); the pose along it is the prediction, not a measurement.
    bool weakly_constrained = false;
    TranslationObservability observability;
    std::size_t pairs_kept = 0;
    std::size_t iterations = 0;
    double residual_rmse = 0.0;  // of the kept pairs, meters
    // Share of scan points that found a map partner within the gate. A drop
    // means the scan disagrees with the map (world changed, or a wrong pose).
    double inlier_fraction = 0.0;
};

// Scan-to-map localization (pose tracking). Each scan, in current sensor axes,
// is registered against fixed map points (e.g. OccupancyMap::occupiedCenters()),
// in map coordinates. Because the reference is the map, ICP returns the sensor
// pose IN THE MAP directly: errors do not chain from scan to scan.
// The guess is the previous estimate moved by the previous motion (constant
// velocity). Needs the initial pose; it does not find itself from nowhere. On
// failure, or a result too far from the guess, the step is marked lost and the
// pose is predicted. The first scan skips the jump check (no velocity yet).
// "Converged" still means stable, not correct: in a view of parallel walls ICP
// can slide along them with no rise in residual. So when the scan's weakest
// translation direction is too weak, the position along it is taken from the
// prediction instead (see min_weak_observability).
class MapLocalizer {
public:
    // Throws std::invalid_argument for a nonfinite initial pose or a map with
    // fewer points than the line fits and minimum pairs need.
    MapLocalizer(std::vector<Point2D> map_points, const Pose2D& initial_pose_in_map,
                 MapLocalizerOptions options = {});

    LocalizerStep update(const std::vector<Point2D>& scan);
    const Pose2D& pose() const { return pose_; }
    std::size_t scansProcessed() const { return scans_; }

private:
    MapLocalizerOptions options_;
    std::vector<Point2D> map_;
    std::vector<NormalEstimate> map_normals_;  // fitted once: the map does not change
    Pose2D pose_;
    Pose2D last_motion_{0, 0, 0};  // newest sensor frame in the one before
    std::size_t scans_ = 0;
};
}  // namespace atlas
