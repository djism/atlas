#include "atlas/posegraph3d.hpp"
#include <cmath>
#include <stdexcept>

namespace atlas {
std::array<double, 6> edgeResidual3D(const Pose3D& from, const Pose3D& to, const Pose3D& measured) {
    const Pose3D e = composePoses3D(inversePose3D(measured), relativePose3D(from, to));
    const Point3D w = rotationVector(e);
    return {e.t.x, e.t.y, e.t.z, w.x, w.y, w.z};
}

namespace {
// Small motion applied on the left, in the world frame: rotate by phi about
// the world origin, then shift by rho. delta = [rho, phi].
Pose3D perturbed(const Pose3D& pose, const double delta[6]) {
    Pose3D step = rotationFromVector({delta[3], delta[4], delta[5]});
    step.t = {delta[0], delta[1], delta[2]};
    return orthonormalized(composePoses3D(step, pose));
}

double edgeCost(const PoseGraphEdge3D& e, const std::array<double, 6>& r) {
    return e.translation_weight * (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) +
           e.rotation_weight * (r[3] * r[3] + r[4] * r[4] + r[5] * r[5]);
}

double totalCost(const std::vector<Pose3D>& poses, const std::vector<PoseGraphEdge3D>& edges) {
    double c = 0;
    for (const PoseGraphEdge3D& e : edges) c += edgeCost(e, edgeResidual3D(poses[e.from], poses[e.to], e.measured));
    return c;
}
}  // namespace

bool solveCholesky(std::vector<double> a, std::vector<double> b, std::size_t n, std::vector<double>& x) {
    // A = L L^T, L stored in the lower triangle of a.
    for (std::size_t j = 0; j < n; ++j) {
        double d = a[j * n + j];
        for (std::size_t k = 0; k < j; ++k) d -= a[j * n + k] * a[j * n + k];
        if (!(d > 0) || !std::isfinite(d)) return false;
        const double ljj = std::sqrt(d);
        a[j * n + j] = ljj;
        for (std::size_t i = j + 1; i < n; ++i) {
            double s = a[i * n + j];
            for (std::size_t k = 0; k < j; ++k) s -= a[i * n + k] * a[j * n + k];
            a[i * n + j] = s / ljj;
        }
    }
    for (std::size_t i = 0; i < n; ++i) {  // L y = b
        double s = b[i];
        for (std::size_t k = 0; k < i; ++k) s -= a[i * n + k] * b[k];
        b[i] = s / a[i * n + i];
    }
    for (std::size_t i = n; i-- > 0;) {  // L^T x = y
        double s = b[i];
        for (std::size_t k = i + 1; k < n; ++k) s -= a[k * n + i] * b[k];
        b[i] = s / a[i * n + i];
    }
    x = std::move(b);
    return true;
}

PoseGraphResult optimizePoseGraph3D(std::vector<Pose3D> poses, const std::vector<PoseGraphEdge3D>& edges,
                                    const PoseGraphOptions& options) {
    if (poses.empty()) throw std::invalid_argument("optimizePoseGraph3D: no poses");
    for (const PoseGraphEdge3D& e : edges) {
        if (e.from >= poses.size() || e.to >= poses.size()) throw std::invalid_argument("optimizePoseGraph3D: edge index out of range");
        if (e.from == e.to) throw std::invalid_argument("optimizePoseGraph3D: edge from a pose to itself");
        if (!(e.translation_weight >= 0) || !(e.rotation_weight >= 0) || !std::isfinite(e.translation_weight) ||
            !std::isfinite(e.rotation_weight))
            throw std::invalid_argument("optimizePoseGraph3D: weights must be finite and non-negative");
    }
    PoseGraphResult result;
    result.initial_cost = result.final_cost = totalCost(poses, edges);
    const std::size_t free_poses = poses.size() - 1;  // pose 0 is fixed
    const std::size_t n = 6 * free_poses;
    if (n == 0 || edges.empty()) {
        result.poses = std::move(poses);
        result.converged = true;
        return result;
    }
    // Column of pose k's first variable, or -1 for the fixed pose 0.
    const auto col = [](std::size_t k) { return k == 0 ? -1L : static_cast<long>(6 * (k - 1)); };
    double lambda = 1e-4;
    const double h = 1e-6;
    for (std::size_t it = 0; it < options.max_iterations; ++it) {
        std::vector<double> H(n * n, 0.0), g(n, 0.0);
        for (const PoseGraphEdge3D& e : edges) {
            const std::array<double, 6> r = edgeResidual3D(poses[e.from], poses[e.to], e.measured);
            const double w[6] = {e.translation_weight, e.translation_weight, e.translation_weight,
                                 e.rotation_weight, e.rotation_weight, e.rotation_weight};
            // J[a]: 6x6 Jacobian of r with respect to the left perturbation of pose a.
            double J[2][6][6];
            const std::size_t ends[2] = {e.from, e.to};
            for (int a = 0; a < 2; ++a) {
                for (int d = 0; d < 6; ++d) {
                    double plus[6] = {0, 0, 0, 0, 0, 0}, minus[6] = {0, 0, 0, 0, 0, 0};
                    plus[d] = h;
                    minus[d] = -h;
                    Pose3D p_from = poses[e.from], p_to = poses[e.to], m_from = p_from, m_to = p_to;
                    (a == 0 ? p_from : p_to) = perturbed(poses[ends[a]], plus);
                    (a == 0 ? m_from : m_to) = perturbed(poses[ends[a]], minus);
                    const std::array<double, 6> rp = edgeResidual3D(p_from, p_to, e.measured);
                    const std::array<double, 6> rm = edgeResidual3D(m_from, m_to, e.measured);
                    for (int i = 0; i < 6; ++i) J[a][i][d] = (rp[i] - rm[i]) / (2 * h);
                }
            }
            // H += J^T W J and g += J^T W r for both ends (skipping the fixed pose).
            for (int a = 0; a < 2; ++a) {
                const long ca = col(ends[a]);
                if (ca < 0) continue;
                for (int p = 0; p < 6; ++p) {
                    double gp = 0;
                    for (int i = 0; i < 6; ++i) gp += J[a][i][p] * w[i] * r[i];
                    g[static_cast<std::size_t>(ca + p)] += gp;
                }
                for (int b = 0; b < 2; ++b) {
                    const long cb = col(ends[b]);
                    if (cb < 0) continue;
                    for (int p = 0; p < 6; ++p)
                        for (int q = 0; q < 6; ++q) {
                            double s = 0;
                            for (int i = 0; i < 6; ++i) s += J[a][i][p] * w[i] * J[b][i][q];
                            H[static_cast<std::size_t>(ca + p) * n + static_cast<std::size_t>(cb + q)] += s;
                        }
                }
            }
        }
        // Levenberg-Marquardt: damp, try the step, keep it only if the cost falls.
        bool improved = false;
        for (int attempt = 0; attempt < 10 && !improved; ++attempt) {
            std::vector<double> A = H, minus_g(n), delta;
            for (std::size_t i = 0; i < n; ++i) {
                A[i * n + i] += lambda * (H[i * n + i] + 1e-9);
                minus_g[i] = -g[i];
            }
            if (!solveCholesky(A, minus_g, n, delta)) {
                lambda *= 10;
                continue;
            }
            std::vector<Pose3D> trial = poses;
            for (std::size_t k = 1; k < poses.size(); ++k) trial[k] = perturbed(poses[k], &delta[6 * (k - 1)]);
            const double cost = totalCost(trial, edges);
            if (cost < result.final_cost) {
                const double gain = (result.final_cost - cost) / std::max(result.final_cost, 1e-300);
                poses = std::move(trial);
                result.final_cost = cost;
                lambda = std::max(1e-9, lambda / 3);
                improved = true;
                if (gain < options.tolerance) result.converged = true;
            } else {
                lambda *= 4;
            }
        }
        result.iterations = it + 1;
        if (!improved || result.converged || result.final_cost < 1e-20) {
            result.converged = true;  // no step lowers the cost further: a (local) minimum
            break;
        }
    }
    result.poses = std::move(poses);
    return result;
}
}  // namespace atlas
