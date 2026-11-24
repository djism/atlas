#pragma once
#include "atlas/geometry3d.hpp"
#include "atlas/voxel.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
struct Icp3DOptions {
    // Pairs farther apart than this are ignored (outliers, unmatched parts).
    // Must not exceed the reference grid's voxel size (see VoxelGrid3D::nearest).
    double max_correspondence_distance = 1.0;  // meters
    std::size_t max_iterations = 50;
    double translation_tolerance = 1e-4;  // meters: stop when an update is smaller
    double rotation_tolerance = 1e-5;     // radians
    std::size_t min_pairs = 50;
};

enum class Icp3DStatus { Converged, HitIterationLimit, TooFewPairs, Degenerate };

struct Icp3DResult {
    Pose3D source_in_reference;  // maps source points into the reference frame
    Icp3DStatus status = Icp3DStatus::Converged;
    std::size_t iterations = 0;
    std::size_t pairs = 0;       // kept pairs in the last iteration
    double rmse = 0;             // of the kept pairs, meters, before the last update
};

// Point-to-point ICP, solved by Gauss-Newton. Each iteration: move every source
// point x by the current estimate, pair it with its nearest reference point q
// within the gate, and minimize sum |x + dt + dtheta x x - q|^2 over the small
// correction (dt, dtheta): a 6x6 linear system (dtheta x x ~ rotation of x by
// the small angle dtheta). Then T <- (rotation(dtheta), dt) * T. "Converged"
// means the correction became small: stable, not necessarily correct.
// Throws std::invalid_argument if the gate exceeds the grid's voxel size.
Icp3DResult alignIcp3D(const VoxelGrid3D& reference, const std::vector<Point3D>& source, const Pose3D& guess,
                       const Icp3DOptions& options = {});

const char* icp3DStatusName(Icp3DStatus status);

// Point-to-point ICP against a local map with robust (Geman-McClure) weights:
// each pair with squared distance r2 counts w = kernel^2 / (kernel + r2)^2, so
// far pairs (moving objects, wrong matches) pull less. Pairs farther than
// max_correspondence_distance are ignored. Gauss-Newton as in alignIcp3D.
struct MapIcpOptions {
    double max_correspondence_distance = 1.0;  // meters
    double kernel = 0.3;                       // Geman-McClure scale (KISS-ICP 1.3.0 convention)
    std::size_t max_iterations = 500;
    double tolerance = 1e-4;  // stop when the update (translation m + rotation rad) is smaller
    std::size_t min_pairs = 50;
};
Icp3DResult alignToMap(const LocalMap3D& map, const std::vector<Point3D>& source, const Pose3D& guess,
                       const MapIcpOptions& options);
}  // namespace atlas
