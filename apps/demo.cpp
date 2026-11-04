#include "atlas/geometry.hpp"
#include "atlas/matching.hpp"
#include "atlas/simulation.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

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
    const auto estimated = estimateFromTwoPairs(first_scan, second_scan);
    const auto expected_translation = inverseTransformPoint(
        {second_pose.x, second_pose.y}, first_pose);
    std::cout << std::fixed << std::setprecision(6)
              << "Atlas v0.1.0 | ideal 2D simulation, known correspondences\n"
              << "Expected translation in first sensor frame: "
              << expected_translation.x << ", " << expected_translation.y << '\n'
              << "Estimated translation: " << estimated.x << ", " << estimated.y << '\n'
              << "Estimated rotation (degrees): " << estimated.theta * 180.0 / pi << '\n';
    double squared_error = 0.0;
    for (std::size_t i = 0; i < second_scan.size(); ++i) {
        const auto aligned = transformPoint(second_scan[i], estimated);
        const double error = std::hypot(aligned.x - first_scan[i].x,
                                        aligned.y - first_scan[i].y);
        squared_error += error * error;
        std::cout << "Aligned: " << aligned.x << ", " << aligned.y
                  << " | matched-point error (m): " << error << '\n';
    }
    std::cout << "Matched-point RMSE (m): "
              << std::sqrt(squared_error / second_scan.size()) << '\n';
}
