#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
// source -> destination: R(theta) * point + translation.
Point2D transformPoint(const Point2D& point, const Pose2D& source_in_destination);
// destination -> source: R(theta)^T * (point - translation).
Point2D inverseTransformPoint(const Point2D& point, const Pose2D& source_in_destination);

// Chains frames: given frame B in frame A and frame C in frame B, returns
// frame C in frame A. Translation: R(a.theta) * c.xy + a.xy; heading: a.theta + c.theta
// (wrapped to (-pi, pi]).
Pose2D composePoses(const Pose2D& b_in_a, const Pose2D& c_in_b);

// The reverse of composePoses: given frames B and C both in frame A, returns
// frame C in frame B (the motion that takes pose b to pose c). Heading wrapped
// to (-pi, pi]. composePoses(b, relativePose(b, c)) equals c.
Pose2D relativePose(const Pose2D& b_in_a, const Pose2D& c_in_a);

// Mean of a nonempty point set, in the same frame as the inputs.
// Throws std::invalid_argument for an empty set.
Point2D computeCentroid(const std::vector<Point2D>& points);

struct NormalEstimate {
    Point2D normal{0, 0};  // unit length when valid; sign is arbitrary
    bool valid = false;
};

// Local line fit per point: takes the point and its nearest neighbors
// (`neighbors` points in total, including itself) and fits a line through them.
// The normal is perpendicular to that line, in the points' own frame. A
// neighborhood is "line-like" when its spread across the line divided by its
// spread along it is at most max_spread_ratio; otherwise (corners, clutter,
// coincident points) valid is false. Brute-force neighbor search: O(n^2).
// Throws std::invalid_argument for neighbors < 3, fewer points than neighbors,
// a negative or NaN ratio, or nonfinite points.
std::vector<NormalEstimate> estimateNormals(const std::vector<Point2D>& points,
                                            std::size_t neighbors = 5,
                                            double max_spread_ratio = 0.05);

// How well a set of point-to-line constraints pins down TRANSLATION. Each pair
// can only correct position along its normal n, so the evidence per direction u
// is the average of (n . u)^2, i.e. u^T A u with A = mean(n n^T). A is 2x2,
// symmetric, and (for unit normals) has eigenvalues summing to 1:
//   strong = most evidence in any direction, weak = least (0 <= weak <= 0.5).
// weak = 0: all normals parallel (hallway), nothing fixes motion along weak_direction.
// weak = 0.5: normals evenly spread (e.g. a corner seen equally on both walls).
// Rotation is not analyzed here.
struct TranslationObservability {
    double weak = 0.0;
    double strong = 0.0;
    Point2D weak_direction{1, 0};  // unit, in the normals' frame; sign arbitrary
};

// Throws std::invalid_argument for an empty set or a non-unit or nonfinite normal.
TranslationObservability translationObservability(const std::vector<Point2D>& unit_normals);
}  // namespace atlas
