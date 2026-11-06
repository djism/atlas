#include "atlas/geometry.hpp"
#include "atlas/icp.hpp"
#include "atlas/matching.hpp"
#include "atlas/simulation.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {
// Printing only; the algorithm lives in src/icp.cpp.
void printIcp(const char* title, const atlas::IcpResult& result,
              const atlas::Pose2D& answer_key) {
    const double deg = 180.0 / std::acos(-1.0);
    std::cout << "\nICP: " << title << '\n';
    for (std::size_t i = 0; i < result.iterations; ++i) {
        const auto& pose = result.pose_per_pass[i];
        std::cout << "  pass " << i + 1 << ": pose (" << pose.x << ", " << pose.y << ", "
                  << pose.theta * deg << " deg) | nearest-pair RMSE (m): "
                  << result.nearest_pair_rmse[i] << '\n';
    }
    const auto& final_pose = result.second_in_first;
    const char* status = result.status == atlas::IcpStatus::Converged ? "converged (pairs stable)"
                       : result.status == atlas::IcpStatus::TooFewPairs ? "LOST: too few pairs"
                                                                        : "hit pass limit";
    std::cout << "  " << status
              << " | answer key (" << answer_key.x << ", " << answer_key.y << ", "
              << answer_key.theta * deg << " deg) | position error (m): "
              << std::hypot(final_pose.x - answer_key.x, final_pose.y - answer_key.y) << '\n';
}
}  // namespace

int main() {
    using namespace atlas;
    const double pi = std::acos(-1.0);
    const std::vector<Point2D> landmarks{{4.0, 5.0}, {6.0, 7.0}, {20.0, 20.0}};
    const Pose2D first_pose{5.0, 3.0, pi / 2.0};
    const Pose2D second_pose{5.0, 4.0, pi / 2.0 + pi / 18.0};

    // Ground truth is used only to simulate observations and check the answer.
    const auto first_scan = simulateScan(landmarks, first_pose, 5.0);
    const auto second_scan = simulateScan(landmarks, second_pose, 5.0);

    // These scans happen to contain the same two landmarks in matching order.
    // The estimator receives scans only, never the world poses or landmarks.
    const auto estimated = estimateFromMatchedPoints(first_scan, second_scan);
    // Answer key only: the true second pose expressed in the first sensor frame.
    const auto expected_translation = inverseTransformPoint(
        {second_pose.x, second_pose.y}, first_pose);
    std::cout << std::fixed << std::setprecision(6)
              << "Atlas | ideal 2D simulation (no noise)\n"
              << "\nKnown pairs: least-squares matched-point alignment\n"
              << "Expected translation in first sensor frame: "
              << expected_translation.x << ", " << expected_translation.y << '\n'
              << "Estimated translation: " << estimated.x << ", " << estimated.y << '\n'
              << "Estimated rotation (degrees): " << estimated.theta * 180.0 / pi << '\n';
    for (std::size_t i = 0; i < second_scan.size(); ++i) {
        const auto aligned = transformPoint(second_scan[i], estimated);
        std::cout << "Aligned: " << aligned.x << ", " << aligned.y
                  << " | matched-point error (m): "
                  << std::hypot(aligned.x - first_scan[i].x, aligned.y - first_scan[i].y)
                  << '\n';
    }
    std::cout << "Matched-point RMSE (m): "
              << computeMatchedRmse(first_scan, second_scan, estimated) << '\n';

    // ICP receives NO pairing: reverse the second scan so index order cannot help.
    // The answer key is printed for comparison only; ICP never sees it.
    const Pose2D driving_answer{expected_translation.x, expected_translation.y, pi / 18.0};
    const std::vector<Point2D> second_reversed(second_scan.rbegin(), second_scan.rend());
    printIcp("driving scene, unknown pairing, zero initial guess",
             alignIcp(first_scan, second_reversed, {0.0, 0.0, 0.0}), driving_answer);

    // Teaching example: pass 1 pairs 0.4 with 0 (wrong); later passes repair it.
    printIcp("x-axis example, sensor moved 0.6 m",
             alignIcp({{0, 0}, {1, 0}, {5, 0}}, {{-0.6, 0}, {0.4, 0}, {4.4, 0}}, {0, 0, 0}),
             {0.6, 0.0, 0.0});

    // Failure: after moving 1 m along evenly spaced poles, the scan looks identical.
    std::vector<Point2D> corridor;
    for (int i = 0; i < 10; ++i) {
        corridor.push_back({static_cast<double>(i), 2.0});
        corridor.push_back({static_cast<double>(i), -2.0});
    }
    printIcp("FAILURE - identical-looking corridor, moved 1 m",
             alignIcp(corridor, corridor, {0.0, 0.0, 0.0}), {1.0, 0.0, 0.0});
    std::cout << "  -> zero residual, wrong pose: a good fit does not prove correct localization.\n";
}
