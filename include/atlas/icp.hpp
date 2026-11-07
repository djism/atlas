#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <limits>
#include <vector>

namespace atlas {
enum class IcpMethod {
    PointToPoint,  // closed-form solve of point-to-point error each pass
    PointToLine,   // one Gauss-Newton step of point-to-line error each pass
};

struct IcpOptions {
    IcpMethod method = IcpMethod::PointToPoint;
    // PointToLine only: line fit per first-scan point (see estimateNormals).
    // Pairs whose first-scan partner has no valid normal are dropped.
    std::size_t normal_neighbors = 5;
    double max_spread_ratio = 0.05;
    std::size_t max_iterations = 30;
    // Stop when one pass changes the pose by less than both tolerances.
    double translation_tolerance = 1e-9;  // meters
    double rotation_tolerance = 1e-9;     // radians
    // Distance gate (meters, first-sensor axes): before solving, drop pairs whose
    // points are farther apart than this under the CURRENT guess. Infinity = off.
    // Too tight early on rejects true pairs while the guess is still poor.
    double max_pair_distance = std::numeric_limits<double>::infinity();
    // Fewer pairs surviving the gate than this ends ICP with status TooFewPairs.
    // A second scan smaller than this is invalid input (exception), not "lost".
    std::size_t min_pairs = 2;
};

enum class IcpStatus {
    Converged,          // pose (and so pairing) stopped changing: stable, not correct
    HitIterationLimit,  // still changing after max_iterations
    TooFewPairs,        // gate left fewer than min_pairs: treat as localization lost
    Degenerate,         // PointToLine: scene does not constrain some motion (e.g. hallway)
};

struct IcpResult {
    // Pose of the second sensor frame in the first frame. For TooFewPairs and
    // Degenerate, the last estimate before the failing pass (possibly the guess).
    Pose2D second_in_first;
    IcpStatus status = IcpStatus::HitIterationLimit;
    std::size_t iterations = 0;  // passes started, including a failing one
    // Per completed pass: RMSE over KEPT nearest-neighbor pairs (may be wrong
    // pairs; a gate lowers this by construction), pose, and kept-pair count.
    // The RMSE is of the error the method minimizes: point-to-point distance,
    // or for PointToLine the distance from each point to its partner's line.
    std::vector<double> nearest_pair_rmse;
    std::vector<Pose2D> pose_per_pass;
    std::vector<std::size_t> pairs_kept_per_pass;

    bool converged() const { return status == IcpStatus::Converged; }
};

// 2D ICP. Each pass pairs every second-scan point with its nearest first-scan
// point under the current guess and drops pairs beyond the distance gate.
// PointToPoint then solves the full pose with estimateFromMatchedPoints.
// PointToLine minimizes each point's distance to its partner's local line
// (normals fitted once on the first scan) with one linearized step per pass;
// sliding along a wall costs nothing, so wall scans need no exact point matches.
// Local method: a poor initial guess or a repetitive scene can converge to a
// wrong pose. For PointToLine, a singular linear system (scene does not
// constrain some motion) ends with status Degenerate. Throws
// std::invalid_argument for invalid input or options, or when point-to-point
// pairs are unusable (e.g. all to one point).
IcpResult alignIcp(const std::vector<Point2D>& first,
                   const std::vector<Point2D>& second,
                   const Pose2D& initial_guess,
                   const IcpOptions& options = {});

struct AmbiguityOptions {
    // Restart offsets added to the solution's (x, y, theta), first-sensor axes.
    std::vector<Pose2D> offsets{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                                {0, 0, 0.3}, {0, 0, -0.3}};
    // Inlier distance (meters). Also the separation test: a rival is "different"
    // if some second-scan point lands farther than this from the solution's mapping.
    double inlier_distance = 0.2;
    // Ambiguous if a different rival has at least this fraction of the
    // solution's inliers.
    double rival_fraction = 0.8;
    IcpOptions icp;  // used for each restart
};

struct AmbiguityResult {
    bool ambiguous = false;
    std::size_t solution_inliers = 0;
    bool rival_found = false;  // a different pose was reached from some offset
    Pose2D rival{0, 0, 0};     // the best-scoring different pose, if found
    std::size_t rival_inliers = 0;
};

// Alias/ambiguity check (e.g. repeating structure): restarts ICP from the
// solution plus each offset and asks whether a clearly different pose explains
// almost as many points. Only rivals reachable from the given offsets are found,
// so "not ambiguous" does not prove uniqueness. Costs one ICP run per offset.
// Restarts that end TooFewPairs, or that the estimator rejects, are skipped.
// Throws std::invalid_argument for empty offsets, nonpositive inlier_distance,
// rival_fraction outside (0, 1], or invalid scans.
AmbiguityResult checkAmbiguity(const std::vector<Point2D>& first,
                               const std::vector<Point2D>& second,
                               const Pose2D& solution,
                               const AmbiguityOptions& options = {});
}  // namespace atlas
