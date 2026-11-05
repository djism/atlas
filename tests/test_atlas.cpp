#include "atlas/geometry.hpp"
#include "atlas/icp.hpp"
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

        // One wrong pair: second[2] paired with the observation of landmark 4,
        // a many-to-one mistake nearest-neighbor matching can make. The estimator
        // cannot detect it and returns a confident but badly wrong pose.
        // Thresholds below describe THIS example, not general guarantees.
        auto mispaired = first;
        mispaired[2] = first[4];
        const auto corrupted = estimateFromMatchedPoints(mispaired, cloud);
        check(std::abs(corrupted.theta - truth.theta) > 0.17, "one wrong pair: rotation off by >10 deg");
        check(std::hypot(corrupted.x - truth.x, corrupted.y - truth.y) > 1.0,
              "one wrong pair: translation off by >1 m");
        check(computeMatchedRmse(mispaired, cloud, corrupted) > 10 * estimate_rmse,
              "one wrong pair: RMSE grows far above the noise level");
        // Least squares spreads the damage: correct pairs are now misaligned too.
        double worst_residual = 0.0;
        std::size_t worst_index = 0;
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            const auto aligned = transformPoint(cloud[i], corrupted);
            const double residual = std::hypot(aligned.x - mispaired[i].x,
                                               aligned.y - mispaired[i].y);
            if (i != 2) check(residual > 0.1, "one wrong pair contaminates correct pairs");
            if (residual > worst_residual) { worst_residual = residual; worst_index = i; }
        }
        check(worst_index == 2, "here the wrong pair has the largest residual");
        // Removing the wrong pair restores a good estimate from the remaining four.
        std::vector<Point2D> first_kept, second_kept;
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            if (i == 2) continue;
            first_kept.push_back(first[i]);
            second_kept.push_back(cloud[i]);
        }
        const auto recovered = estimateFromMatchedPoints(first_kept, second_kept);
        check(std::abs(recovered.x - truth.x) < 0.02, "wrong pair removed: x close");
        check(std::abs(recovered.y - truth.y) < 0.02, "wrong pair removed: y close");
        check(std::abs(recovered.theta - truth.theta) < 0.01, "wrong pair removed: rotation close");
    }
    // ---- Correspondence guesses (one ICP pairing step) ----
    {
        // With the true pose as the guess, nearest neighbors find the true pairs.
        const Pose2D truth{-2, 3, -0.4};
        const auto matches = findNearestCorrespondences(applyPose(cloud, truth), cloud, truth);
        check(matches == std::vector<std::size_t>({0, 1, 2, 3, 4}), "good guess finds true pairs");
    }
    {
        // Points on the first-sensor x-axis; the sensor moved 0.6 m forward.
        const std::vector<Point2D> first{{0,0}, {1,0}, {5,0}};
        const std::vector<Point2D> second{{-0.6,0}, {0.4,0}, {4.4,0}};
        // Guess "no motion": 0.4 is nearer 0 than 1, so two points share first[0].
        check(findNearestCorrespondences(first, second, {0,0,0}) ==
                  std::vector<std::size_t>({0, 0, 2}),
              "zero guess gives many-to-one wrong pairing");
        // Partially corrected guess (mean shift of those pairs) fixes the pairing.
        check(findNearestCorrespondences(first, second, {0.8/3, 0, 0}) ==
                  std::vector<std::size_t>({0, 1, 2}),
              "improved guess gives correct pairing");
    }
    rejects([] { findNearestCorrespondences({}, {{0,0}}, {0,0,0}); }, "correspondence empty first");
    rejects([] { findNearestCorrespondences({{0,0}}, {}, {0,0,0}); }, "correspondence empty second");
    rejects([] { findNearestCorrespondences({{0,0}}, {{0,0}}, {0, 0, std::nan("")}); },
            "correspondence nonfinite pose");
    rejects([] { findNearestCorrespondences({{0,0}}, {{std::nan(""),0}}, {0,0,0}); },
            "correspondence nonfinite point");
    // ---- ICP: repeat pair -> solve until the pose stops changing ----
    {
        // x-axis example: pass 1 pairs wrongly (many-to-one), later passes fix it.
        const std::vector<Point2D> first{{0,0}, {1,0}, {5,0}};
        const std::vector<Point2D> second{{-0.6,0}, {0.4,0}, {4.4,0}};
        const auto result = alignIcp(first, second, {0,0,0});
        check(result.converged, "icp x-axis converges");
        check(result.iterations == 3, "icp x-axis takes three passes");
        near(result.second_in_first.x, 0.6, "icp x-axis translation");
        near(result.second_in_first.y, 0, "icp x-axis lateral");
        near(result.second_in_first.theta, 0, "icp x-axis rotation");
        check(result.nearest_pair_rmse.size() == result.iterations, "icp one rmse per pass");
        check(result.pose_per_pass.size() == result.iterations, "icp one pose per pass");
        near(result.pose_per_pass.front().x, 0.8 / 3, "icp pass 1 is the partial mean shift");
        near(result.pose_per_pass.back().x, result.second_in_first.x, "icp last pass is the result");
        check(result.nearest_pair_rmse.front() > 0.1, "icp first pass has a wrong pair");
        near(result.nearest_pair_rmse.back(), 0, "icp final pairs fit exactly");
        // Stopping at the pass limit is reported, not hidden.
        IcpOptions one_pass;
        one_pass.max_iterations = 1;
        const auto limited = alignIcp(first, second, {0,0,0}, one_pass);
        check(!limited.converged && limited.iterations == 1, "icp reports hitting pass limit");
    }
    {
        // Moderate motion from a zero guess: recovers the truth exactly.
        const Pose2D truth{1, 0.5, 0.5};
        const auto result = alignIcp(applyPose(cloud, truth), cloud, {0,0,0});
        check(result.converged, "icp cloud converges");
        near(result.second_in_first.x, truth.x, "icp cloud x");
        near(result.second_in_first.y, truth.y, "icp cloud y");
        near(result.second_in_first.theta, truth.theta, "icp cloud rotation");
    }
    {
        // FAILURE: 162-degree turn from a zero guess. Pairing stabilizes on a wrong
        // pose (local minimum). "converged" means stable, not correct.
        const Pose2D truth{0, 0, 0.9 * pi};
        const auto result = alignIcp(applyPose(cloud, truth), cloud, {0,0,0});
        check(result.converged, "icp large turn still reports stable");
        check(std::abs(result.second_in_first.theta - truth.theta) > pi / 2,
              "icp large turn converges to wrong rotation");
        check(result.nearest_pair_rmse.back() > 1.0, "icp large turn leaves high residual");
    }
    {
        // FAILURE: identical-looking corridor. Poles at world x = 0..10; the first
        // scan sees x = 0..9, then the car moves 1 m and sees x = 1..10, which in
        // its own frame is again 0..9. The scans are identical, so ICP reports no
        // motion with ZERO residual. Low residual does not prove correct pose.
        std::vector<Point2D> window;
        for (int i = 0; i < 10; ++i) {
            window.push_back({static_cast<double>(i), 2});
            window.push_back({static_cast<double>(i), -2});
        }
        const auto result = alignIcp(window, window, {0,0,0});  // truth: x = 1
        check(result.converged, "icp corridor reports stable");
        near(result.second_in_first.x, 0, "icp corridor misses the 1 m motion");
        near(result.nearest_pair_rmse.back(), 0, "icp corridor fits perfectly anyway");
    }
    rejects([] { alignIcp({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, IcpOptions{0, 1e-9, 1e-9}); },
            "icp zero iterations");
    rejects([] { alignIcp({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, IcpOptions{10, -1, 1e-9}); },
            "icp negative tolerance");
    rejects([] { alignIcp({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, IcpOptions{10, 1e-9, std::nan("")}); },
            "icp NaN tolerance");
    rejects([] { alignIcp({}, {{0,0},{1,0}}, {0,0,0}); }, "icp empty first");
    rejects([] { alignIcp({{0,0},{1,0}}, {{0,0}}, {0,0,0}); }, "icp single second point");
    // Every point pairs with the only first point: estimator cannot use the pairs.
    rejects([] { alignIcp({{0,0}}, {{0,0},{1,0}}, {0,0,0}); }, "icp all pairs to one point");
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
