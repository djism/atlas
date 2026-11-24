#pragma once
#include "atlas/geometry3d.hpp"
#include <array>
#include <cstddef>
#include <vector>

namespace atlas {
// A pose graph: nodes are poses (each one a sensor frame in the world frame of
// pose 0), edges are measured relative poses between two nodes, such as one
// odometry step or one loop closure. Optimizing finds the poses that disagree
// least, in the weighted least-squares sense, with every measurement at once:
// like springs pulling the trajectory into the shape that stretches them least.
struct PoseGraphEdge3D {
    std::size_t from = 0;
    std::size_t to = 0;
    Pose3D measured;  // pose `to` expressed in the frame of pose `from`, as measured
    // Weights: 1 / sigma^2 for each translation component (1/m^2) and each
    // rotation-vector component (1/rad^2). A trusted edge gets large weights.
    double translation_weight = 1;
    double rotation_weight = 1;
};

// The 6-vector by which an edge's measurement disagrees with two poses:
// E = measured^-1 * (from^-1 * to), residual = [translation of E, rotation
// vector of E]. Zero when the poses agree exactly with the measurement. If
// `to` = from * measured * D for a small motion D, the residual is D.
std::array<double, 6> edgeResidual3D(const Pose3D& from, const Pose3D& to, const Pose3D& measured);

struct PoseGraphOptions {
    std::size_t max_iterations = 50;
    double tolerance = 1e-10;  // stop when the cost improves by less than this fraction
};

struct PoseGraphResult {
    std::vector<Pose3D> poses;
    double initial_cost = 0;  // sum over edges of weighted squared residuals
    double final_cost = 0;
    std::size_t iterations = 0;
    bool converged = false;
};

// Levenberg-Marquardt on all poses but the first, which stays fixed (it
// anchors the map; without it the whole graph could slide freely). Each pose
// is updated by a small motion applied on the left (in the world frame) and
// re-orthonormalized. Jacobians are by central differences (simple and
// verifiable; fine for a few hundred poses). The normal equations are dense:
// memory and time grow as (6 n)^2 and (6 n)^3, so use keyframes, not every
// scan. Throws std::invalid_argument for no poses, an edge index out of
// range, an edge from a pose to itself, or a negative or non-finite weight.
PoseGraphResult optimizePoseGraph3D(std::vector<Pose3D> poses, const std::vector<PoseGraphEdge3D>& edges,
                                    const PoseGraphOptions& options = {});

// Solves A x = b for a symmetric positive-definite n x n matrix A (row-major)
// by Cholesky factorization. Returns false (x untouched) if A is not positive
// definite.
bool solveCholesky(std::vector<double> a, std::vector<double> b, std::size_t n, std::vector<double>& x);
}  // namespace atlas
