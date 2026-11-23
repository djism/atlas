#include "atlas/icp3d.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace atlas {
namespace {
// Solves A x = b for a symmetric 6x6 A by Gaussian elimination with partial
// pivoting. Returns false if A is (numerically) singular.
bool solve6(double a[6][6], double b[6], double x[6]) {
    double scale = 0;
    for (int i = 0; i < 6; ++i) scale = std::max(scale, std::abs(a[i][i]));
    if (scale == 0) return false;
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        for (int r = col + 1; r < 6; ++r)
            if (std::abs(a[r][col]) > std::abs(a[pivot][col])) pivot = r;
        if (std::abs(a[pivot][col]) < 1e-12 * scale) return false;
        if (pivot != col) {
            for (int c = 0; c < 6; ++c) std::swap(a[col][c], a[pivot][c]);
            std::swap(b[col], b[pivot]);
        }
        for (int r = col + 1; r < 6; ++r) {
            const double f = a[r][col] / a[col][col];
            for (int c = col; c < 6; ++c) a[r][c] -= f * a[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int r = 5; r >= 0; --r) {
        double s = b[r];
        for (int c = r + 1; c < 6; ++c) s -= a[r][c] * x[c];
        x[r] = s / a[r][r];
    }
    return true;
}
}  // namespace

const char* icp3DStatusName(Icp3DStatus status) {
    switch (status) {
        case Icp3DStatus::Converged: return "converged";
        case Icp3DStatus::HitIterationLimit: return "iteration-limit";
        case Icp3DStatus::TooFewPairs: return "too-few-pairs";
        case Icp3DStatus::Degenerate: return "degenerate";
    }
    return "unknown";
}

Icp3DResult alignIcp3D(const VoxelGrid3D& reference, const std::vector<Point3D>& source, const Pose3D& guess,
                       const Icp3DOptions& options) {
    if (options.max_correspondence_distance > reference.voxelSize())
        throw std::invalid_argument("alignIcp3D: correspondence gate exceeds the reference voxel size");
    Icp3DResult result;
    result.source_in_reference = guess;
    for (std::size_t it = 0; it < options.max_iterations; ++it) {
        result.iterations = it + 1;
        // Normal equations of the linearized residuals r = (x - q) + dt - [x]x dtheta,
        // Jacobian J = [I | -[x]x]  (rows: residual x, y, z; columns: dt, dtheta).
        double h[6][6] = {}, g[6] = {};
        double sum_sq = 0;
        std::size_t pairs = 0;
        for (const Point3D& p : source) {
            const Point3D x = transformPoint3D(p, result.source_in_reference);
            Point3D q;
            double d;
            if (!reference.nearest(x, options.max_correspondence_distance, q, d)) continue;
            const double r[3] = {x.x - q.x, x.y - q.y, x.z - q.z};
            // -[x]x = [[0, z, -y], [-z, 0, x], [y, -x, 0]]
            const double j[3][6] = {{1, 0, 0, 0, x.z, -x.y}, {0, 1, 0, -x.z, 0, x.x}, {0, 0, 1, x.y, -x.x, 0}};
            for (int row = 0; row < 3; ++row)
                for (int a = 0; a < 6; ++a) {
                    if (j[row][a] == 0) continue;
                    g[a] += j[row][a] * r[row];
                    for (int b = 0; b < 6; ++b) h[a][b] += j[row][a] * j[row][b];
                }
            sum_sq += d * d;
            ++pairs;
        }
        result.pairs = pairs;
        if (pairs < options.min_pairs) {
            result.status = Icp3DStatus::TooFewPairs;
            return result;
        }
        result.rmse = std::sqrt(sum_sq / pairs);
        double minus_g[6], delta[6];
        for (int a = 0; a < 6; ++a) minus_g[a] = -g[a];
        if (!solve6(h, minus_g, delta)) {
            result.status = Icp3DStatus::Degenerate;
            return result;
        }
        // Left update: T <- (rotation(dtheta), dt) * T.
        Pose3D step = rotationFromVector({delta[3], delta[4], delta[5]});
        step.t = {delta[0], delta[1], delta[2]};
        result.source_in_reference = composePoses3D(step, result.source_in_reference);
        const double moved = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        const double turned = std::sqrt(delta[3] * delta[3] + delta[4] * delta[4] + delta[5] * delta[5]);
        if (moved < options.translation_tolerance && turned < options.rotation_tolerance) {
            result.status = Icp3DStatus::Converged;
            return result;
        }
    }
    result.status = Icp3DStatus::HitIterationLimit;
    return result;
}
}  // namespace atlas
