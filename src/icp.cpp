#include "atlas/icp.hpp"
#include "atlas/geometry.hpp"
#include "atlas/matching.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace atlas {
namespace {
// Solves a 3x3 system A x = b (Gaussian elimination, partial pivoting).
// Returns false when a pivot is negligible relative to A's largest diagonal.
bool solve3x3(double a[3][3], double b[3], double x[3]) {
    const double scale = std::max({std::abs(a[0][0]), std::abs(a[1][1]), std::abs(a[2][2])});
    if (!(scale > 0.0)) return false;
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int r = col + 1; r < 3; ++r) {
            if (std::abs(a[r][col]) > std::abs(a[pivot][col])) pivot = r;
        }
        if (std::abs(a[pivot][col]) <= 1e-12 * scale) return false;
        for (int c = 0; c < 3; ++c) std::swap(a[col][c], a[pivot][c]);
        std::swap(b[col], b[pivot]);
        for (int r = col + 1; r < 3; ++r) {
            const double factor = a[r][col] / a[col][col];
            for (int c = col; c < 3; ++c) a[r][c] -= factor * a[col][c];
            b[r] -= factor * b[col];
        }
    }
    for (int r = 2; r >= 0; --r) {
        double sum = b[r];
        for (int c = r + 1; c < 3; ++c) sum -= a[r][c] * x[c];
        x[r] = sum / a[r][r];
    }
    return true;
}

// Signed distance from second point q, moved by pose, to the line through p
// with unit normal n (first-sensor axes).
double lineResidual(const Point2D& p, const Point2D& n, const Point2D& q, const Pose2D& pose) {
    const Point2D moved = transformPoint(q, pose);
    return n.x * (moved.x - p.x) + n.y * (moved.y - p.y);
}

// One Gauss-Newton step of point-to-line error. A small correction (dx, dy, dtheta)
// applied in first-sensor axes moves a point m by about (dx, dy) + dtheta*(-m.y, m.x),
// so each residual changes by J . (dx, dy, dtheta) with J = (n.x, n.y, cross(m, n)).
// Returns false when the system is singular (some motion unconstrained).
bool pointToLineStep(const std::vector<Point2D>& first, const std::vector<Point2D>& normals,
                     const std::vector<Point2D>& second, const Pose2D& current, Pose2D& next) {
    double h[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double g[3] = {0, 0, 0};
    for (std::size_t i = 0; i < second.size(); ++i) {
        const Point2D m = transformPoint(second[i], current);
        const Point2D& n = normals[i];
        const double r = lineResidual(first[i], n, second[i], current);
        const double j[3] = {n.x, n.y, m.x * n.y - m.y * n.x};
        for (int row = 0; row < 3; ++row) {
            g[row] += j[row] * r;
            for (int col = 0; col < 3; ++col) h[row][col] += j[row] * j[col];
        }
    }
    double minus_g[3] = {-g[0], -g[1], -g[2]};
    double delta[3] = {0, 0, 0};
    if (!solve3x3(h, minus_g, delta)) return false;
    // Compose: new = correction applied after current (both map into first axes).
    const double c = std::cos(delta[2]);
    const double s = std::sin(delta[2]);
    const double theta = current.theta + delta[2];
    next = {c * current.x - s * current.y + delta[0],
            s * current.x + c * current.y + delta[1],
            std::atan2(std::sin(theta), std::cos(theta))};
    return true;
}
}  // namespace

IcpResult alignIcp(const std::vector<Point2D>& first,
                   const std::vector<Point2D>& second,
                   const Pose2D& initial_guess,
                   const IcpOptions& options) {
    std::vector<NormalEstimate> none;
    return alignIcp(first, none, second, initial_guess, options);
}

IcpResult alignIcp(const std::vector<Point2D>& first,
                   const std::vector<NormalEstimate>& precomputed_normals,
                   const std::vector<Point2D>& second,
                   const Pose2D& initial_guess,
                   const IcpOptions& options) {
    if (options.max_iterations == 0) {
        throw std::invalid_argument("ICP needs at least one iteration");
    }
    if (!(options.translation_tolerance >= 0.0) || !std::isfinite(options.translation_tolerance) ||
        !(options.rotation_tolerance >= 0.0) || !std::isfinite(options.rotation_tolerance)) {
        throw std::invalid_argument("ICP tolerances must be finite and nonnegative");
    }
    // Infinity is allowed (gate off); NaN and nonpositive values are not.
    if (!(options.max_pair_distance > 0.0)) {
        throw std::invalid_argument("ICP max_pair_distance must be positive");
    }
    if (options.min_pairs < 2) {
        throw std::invalid_argument("ICP min_pairs must be at least 2");
    }
    if (second.size() < options.min_pairs) {
        throw std::invalid_argument("second scan has fewer points than ICP min_pairs");
    }

    const bool to_line = options.method == IcpMethod::PointToLine;
    if (!precomputed_normals.empty() && precomputed_normals.size() != first.size()) {
        throw std::invalid_argument("ICP needs one precomputed normal per first point");
    }
    // Normals depend only on the first scan, so fit them once (or reuse the caller's).
    std::vector<NormalEstimate> fitted;
    if (to_line && precomputed_normals.empty()) {
        fitted = estimateNormals(first, options.normal_neighbors, options.max_spread_ratio);
    }
    const std::vector<NormalEstimate>& first_normals =
        precomputed_normals.empty() ? fitted : precomputed_normals;

    IcpResult result;
    result.second_in_first = initial_guess;
    std::vector<Point2D> kept_first;
    std::vector<Point2D> kept_second;
    std::vector<Point2D> kept_normals;
    kept_first.reserve(second.size());
    kept_second.reserve(second.size());
    kept_normals.reserve(second.size());
    for (std::size_t pass = 0; pass < options.max_iterations; ++pass) {
        result.iterations = pass + 1;

        // 1. Guess pairs using the current pose (validates scans and pose).
        const auto matches = findNearestCorrespondences(first, second, result.second_in_first);

        // 2. Gate: keep pairs whose gap, in first-sensor axes, is within the limit.
        //    PointToLine also drops pairs whose partner has no valid line fit.
        kept_first.clear();
        kept_second.clear();
        kept_normals.clear();
        for (std::size_t i = 0; i < second.size(); ++i) {
            if (to_line && !first_normals[matches[i]].valid) continue;
            const Point2D moved = transformPoint(second[i], result.second_in_first);
            const Point2D& partner = first[matches[i]];
            if (std::hypot(moved.x - partner.x, moved.y - partner.y) <= options.max_pair_distance) {
                kept_first.push_back(partner);
                kept_second.push_back(second[i]);
                if (to_line) kept_normals.push_back(first_normals[matches[i]].normal);
            }
        }
        if (kept_second.size() < options.min_pairs) {
            result.status = IcpStatus::TooFewPairs;  // keep the last good pose
            return result;
        }

        // 3. New pose for the kept pairs, and the RMSE of the error being minimized.
        Pose2D estimate{0, 0, 0};
        if (to_line) {
            if (!pointToLineStep(kept_first, kept_normals, kept_second, result.second_in_first,
                                 estimate)) {
                result.status = IcpStatus::Degenerate;  // keep the last good pose
                return result;
            }
            double squared = 0.0;
            for (std::size_t i = 0; i < kept_second.size(); ++i) {
                const double r = lineResidual(kept_first[i], kept_normals[i], kept_second[i], estimate);
                squared += r * r;
            }
            result.nearest_pair_rmse.push_back(
                std::sqrt(squared / static_cast<double>(kept_second.size())));
            result.observability = translationObservability(kept_normals);
        } else {
            estimate = estimateFromMatchedPoints(kept_first, kept_second);
            result.nearest_pair_rmse.push_back(computeMatchedRmse(kept_first, kept_second, estimate));
        }
        result.pose_per_pass.push_back(estimate);
        result.pairs_kept_per_pass.push_back(kept_second.size());

        // 4. Stop once a pass no longer changes the pose.
        const double moved = std::hypot(estimate.x - result.second_in_first.x,
                                        estimate.y - result.second_in_first.y);
        const double turn_change = estimate.theta - result.second_in_first.theta;
        const double turned = std::abs(std::atan2(std::sin(turn_change), std::cos(turn_change)));
        result.second_in_first = estimate;
        if (moved <= options.translation_tolerance && turned <= options.rotation_tolerance) {
            result.status = IcpStatus::Converged;
            return result;
        }
    }
    result.status = IcpStatus::HitIterationLimit;
    return result;
}

AmbiguityResult checkAmbiguity(const std::vector<Point2D>& first,
                               const std::vector<Point2D>& second,
                               const Pose2D& solution,
                               const AmbiguityOptions& options) {
    if (options.offsets.empty()) {
        throw std::invalid_argument("ambiguity check needs at least one offset");
    }
    if (!(options.rival_fraction > 0.0 && options.rival_fraction <= 1.0)) {
        throw std::invalid_argument("rival_fraction must be in (0, 1]");
    }
    AmbiguityResult result;
    // Also validates scans, solution, and inlier_distance.
    result.solution_inliers = countInliers(first, second, solution, options.inlier_distance);

    for (const auto& offset : options.offsets) {
        const Pose2D start{solution.x + offset.x, solution.y + offset.y,
                           solution.theta + offset.theta};
        IcpResult restart;
        try {
            restart = alignIcp(first, second, start, options.icp);
        } catch (const std::invalid_argument&) {
            continue;  // estimator could not use this restart's pairs
        }
        if (restart.status == IcpStatus::TooFewPairs) continue;
        const Pose2D& candidate = restart.second_in_first;

        // Different only if some point lands noticeably elsewhere.
        double separation = 0.0;
        for (const auto& point : second) {
            const Point2D a = transformPoint(point, solution);
            const Point2D b = transformPoint(point, candidate);
            separation = std::max(separation, std::hypot(a.x - b.x, a.y - b.y));
        }
        if (separation <= options.inlier_distance) continue;

        const std::size_t inliers = countInliers(first, second, candidate, options.inlier_distance);
        if (!result.rival_found || inliers > result.rival_inliers) {
            result.rival_found = true;
            result.rival = candidate;
            result.rival_inliers = inliers;
        }
    }
    result.ambiguous = result.rival_found &&
                       static_cast<double>(result.rival_inliers) >=
                           options.rival_fraction * static_cast<double>(result.solution_inliers);
    return result;
}
}  // namespace atlas
