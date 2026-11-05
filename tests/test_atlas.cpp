#include "atlas/geometry.hpp"
#include "atlas/matching.hpp"
#include "atlas/simulation.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(bool passed, const std::string& name) {
    if (!passed) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}
void near(double actual, double expected, const std::string& name) {
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, name);
}
template <typename F> void rejects(F operation, const std::string& name) {
    try { operation(); check(false, name); }
    catch (const std::invalid_argument&) {}
}
}

int main() {
    using namespace atlas;
    const double pi = std::acos(-1.0);
    const auto world = transformPoint({2, 1}, {5, 3, pi / 2});
    near(world.x, 4, "90-degree transform x");
    near(world.y, 5, "90-degree transform y");
    for (const auto& pose : std::vector<Pose2D>{{0,0,0}, {5,3,pi/2}, {-2,7,-0.7}}) {
        const auto recovered = inverseTransformPoint(transformPoint({2, -3}, pose), pose);
        near(recovered.x, 2, "transform inverse x");
        near(recovered.y, -3, "transform inverse y");
    }
    // Centroid: mean of the points, in the same frame as the input.
    const auto center = computeCentroid({{2,1}, {4,-1}});
    near(center.x, 3, "centroid x");
    near(center.y, 0, "centroid y");
    const auto single = computeCentroid({{-2.5, 7}});
    near(single.x, -2.5, "single-point centroid x");
    near(single.y, 7, "single-point centroid y");
    rejects([] { computeCentroid({}); }, "empty centroid");
    // Translating every point shifts the centroid equally, so centered points
    // are unchanged. This is why centering removes translation before rotation.
    {
        const std::vector<Point2D> original{{1,2}, {4,-1}, {-3,0.5}};
        const Point2D shift{-7, 2.5};
        std::vector<Point2D> shifted;
        for (const auto& p : original) shifted.push_back({p.x + shift.x, p.y + shift.y});
        const auto c0 = computeCentroid(original);
        const auto c1 = computeCentroid(shifted);
        near(c1.x, c0.x + shift.x, "centroid follows translation x");
        near(c1.y, c0.y + shift.y, "centroid follows translation y");
        for (std::size_t i = 0; i < original.size(); ++i) {
            near(shifted[i].x - c1.x, original[i].x - c0.x, "centered point invariant x");
            near(shifted[i].y - c1.y, original[i].y - c0.y, "centered point invariant y");
        }
    }
    const auto scan = simulateScan({{3,4}, {6,0}, {0,0}}, {0,0,0}, 5);
    check(scan.size() == 2, "range includes boundary and rejects far point");
    const auto local = simulateScan({{4,5}}, {5,3,pi/2}, 5);
    near(local[0].x, 2, "scan returns sensor x, not world x");
    near(local[0].y, 1, "scan returns sensor y, not world y");
    check(findNearestPoint({3,-1}, {{2,1},{4,-1}}) == 1, "nearest search");
    check(findNearestPoint({0,0}, {{-1,0},{1,0}}) == 0, "first tie wins");
    rejects([] { findNearestPoint({0,0}, {}); }, "empty nearest search");
    rejects([] { simulateScan({}, {0,0,0}, -1); }, "negative range");
    rejects([] { estimateFromTwoPairs({}, {}); }, "empty pairs");
    rejects([] { estimateFromTwoPairs({{0,0},{1,0}}, {{0,0}}); }, "unequal pairs");
    rejects([] { estimateFromTwoPairs({{0,0},{0,0}}, {{0,0},{1,0}}); }, "degenerate pair");

    // Exercise positive/negative translation and rotation, including angle wrapping.
    const std::vector<Point2D> second{{1,2},{4,-1}};
    for (const auto& expected : std::vector<Pose2D>{{1,0,pi/18}, {-2,3,-0.4}, {0,0,2.9}}) {
        std::vector<Point2D> first;
        for (const auto& p : second) first.push_back(transformPoint(p, expected));
        const auto actual = estimateFromTwoPairs(first, second);
        near(actual.x, expected.x, "recover translation x");
        near(actual.y, expected.y, "recover translation y");
        near(actual.theta, expected.theta, "recover rotation");
        for (std::size_t i = 0; i < second.size(); ++i) {
            const auto aligned = transformPoint(second[i], actual);
            near(aligned.x, first[i].x, "alignment x");
            near(aligned.y, first[i].y, "alignment y");
        }
    }
    // Independent end-to-end driving example with known expected relative motion.
    const std::vector<Point2D> landmarks{{4,5},{6,7},{20,20}};
    const auto before = simulateScan(landmarks, {5,3,pi/2}, 5);
    const auto after = simulateScan(landmarks, {5,4,pi/2+pi/18}, 5);
    const auto motion = estimateFromTwoPairs(before, after);
    near(motion.x, 1, "driving forward x");
    near(motion.y, 0, "driving forward y");
    near(motion.theta, pi/18, "driving turn");

    // ---- Multi-point known-correspondence alignment ----
    // first[i] = R * second[i] + t for a chosen pose; estimator sees points only.
    const auto applyPose = [](const std::vector<Point2D>& points, const Pose2D& pose) {
        std::vector<Point2D> out;
        for (const auto& p : points) out.push_back(transformPoint(p, pose));
        return out;
    };
    const std::vector<Point2D> cloud{{1,2}, {4,-1}, {-2,3.5}, {0.5,-4}, {6,6}};
    // Identity, positive/negative translation and rotation, near +/-pi.
    // 3.5 rad is outside (-pi, pi], so the estimator reports 3.5 - 2*pi.
    for (const auto& expected : std::vector<Pose2D>{
             {0,0,0}, {1,0,pi/18}, {-2,3,-0.4}, {0.3,-5,2.9}, {4,4,-3.0}, {-1,2,3.5}}) {
        const auto first = applyPose(cloud, expected);
        const auto actual = estimateFromMatchedPoints(first, cloud);
        const double wrapped = std::atan2(std::sin(expected.theta), std::cos(expected.theta));
        near(actual.x, expected.x, "multi recover translation x");
        near(actual.y, expected.y, "multi recover translation y");
        near(actual.theta, wrapped, "multi recover rotation (wrapped)");
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            const auto aligned = transformPoint(cloud[i], actual);
            near(aligned.x, first[i].x, "multi known-pair alignment x");
            near(aligned.y, first[i].y, "multi known-pair alignment y");
        }
    }
    // Reordering BOTH sets identically keeps pairs intact, so the pose is unchanged.
    {
        const Pose2D expected{-2, 3, -0.4};
        const auto first = applyPose(cloud, expected);
        const std::vector<std::size_t> order{3, 0, 4, 2, 1};
        std::vector<Point2D> first_reordered, second_reordered;
        for (auto i : order) { first_reordered.push_back(first[i]); second_reordered.push_back(cloud[i]); }
        const auto actual = estimateFromMatchedPoints(first_reordered, second_reordered);
        near(actual.x, expected.x, "reordered pairs x");
        near(actual.y, expected.y, "reordered pairs y");
        near(actual.theta, expected.theta, "reordered pairs rotation");
    }
    // Collinear points still define a direction in 2D, so they are accepted.
    {
        const std::vector<Point2D> line{{0,0}, {1,1}, {3,3}, {-2,-2}};
        const Pose2D expected{2, -1, 0.7};
        const auto actual = estimateFromMatchedPoints(applyPose(line, expected), line);
        near(actual.x, expected.x, "collinear x");
        near(actual.y, expected.y, "collinear y");
        near(actual.theta, expected.theta, "collinear rotation");
    }
    // Same driving scans: the general estimator agrees with the two-pair baseline.
    {
        const auto general = estimateFromMatchedPoints(before, after);
        near(general.x, 1, "multi driving forward x");
        near(general.y, 0, "multi driving forward y");
        near(general.theta, pi/18, "multi driving turn");
    }
    // Fixed noisy example: exact recovery is NOT expected.
    {
        const Pose2D truth{1, 0, pi/18};
        const std::vector<Point2D> noise{{0.02,-0.01}, {-0.015,0.02}, {0.01,0.015},
                                         {-0.02,-0.005}, {0.005,-0.02}};
        auto first = applyPose(cloud, truth);
        for (std::size_t i = 0; i < first.size(); ++i) {
            first[i].x += noise[i].x;
            first[i].y += noise[i].y;
        }
        const auto estimate = estimateFromMatchedPoints(first, cloud);
        const double estimate_rmse = computeMatchedRmse(first, cloud, estimate);
        // Least-squares guarantee: no pose, including the truth, fits these pairs better.
        check(estimate_rmse <= computeMatchedRmse(first, cloud, truth),
              "noisy estimate fits data at least as well as truth");
        for (const auto& nudge : std::vector<Pose2D>{
                 {1e-3,0,0}, {-1e-3,0,0}, {0,1e-3,0}, {0,-1e-3,0}, {0,0,1e-3}, {0,0,-1e-3}}) {
            const Pose2D nearby{estimate.x + nudge.x, estimate.y + nudge.y,
                                estimate.theta + nudge.theta};
            check(estimate_rmse <= computeMatchedRmse(first, cloud, nearby),
                  "noisy estimate is a local RMSE minimum");
        }
        // Example-specific closeness, not a general accuracy guarantee.
        check(std::abs(estimate.x - truth.x) < 0.02, "noisy translation x close");
        check(std::abs(estimate.y - truth.y) < 0.02, "noisy translation y close");
        check(std::abs(estimate.theta - truth.theta) < 0.01, "noisy rotation close");
    }
    near(computeMatchedRmse({{3,4}}, {{0,0}}, {0,0,0}), 5, "rmse single pair 3-4-5");
    rejects([] { computeMatchedRmse({}, {}, {0,0,0}); }, "rmse empty");
    rejects([] { computeMatchedRmse({{0,0}}, {{0,0},{1,1}}, {0,0,0}); }, "rmse unequal");
    const double nan = std::nan("");
    const double inf = std::numeric_limits<double>::infinity();
    rejects([] { estimateFromMatchedPoints({}, {}); }, "multi empty");
    rejects([] { estimateFromMatchedPoints({{0,0},{1,0},{2,1}}, {{0,0},{1,0}}); }, "multi unequal");
    rejects([] { estimateFromMatchedPoints({{1,1}}, {{2,2}}); }, "multi single pair");
    rejects([] { estimateFromMatchedPoints({{3,3},{3,3},{3,3}}, {{0,0},{1,0},{2,0}}); },
            "multi coincident first scan");
    rejects([] { estimateFromMatchedPoints({{0,0},{1,0},{2,0}}, {{5,5},{5,5},{5,5}}); },
            "multi coincident second scan");
    rejects([] { estimateFromMatchedPoints({{0,0},{0,0}}, {{0,0},{0,0}}); }, "multi all zero");
    rejects([&] { estimateFromMatchedPoints({{0,0},{nan,1}}, {{0,0},{1,1}}); }, "multi NaN");
    rejects([&] { estimateFromMatchedPoints({{0,0},{1,1}}, {{inf,0},{1,1}}); }, "multi infinity");
    // A mirrored symmetric shape: dot and cross cancel, so no rotation is preferred.
    rejects([] { estimateFromMatchedPoints({{1,0},{-1,0},{0,1},{0,-1}},
                                           {{1,0},{-1,0},{0,-1},{0,1}}); },
            "multi ambiguous rotation");
    if (failures) return EXIT_FAILURE;
    std::cout << "All geometry, simulation, matching, and motion checks passed.\n";
    return EXIT_SUCCESS;
}
