#include "atlas/geometry.hpp"
#include "atlas/matching.hpp"
#include "atlas/simulation.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
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
    if (failures) return EXIT_FAILURE;
    std::cout << "All geometry, simulation, matching, and motion checks passed.\n";
    return EXIT_SUCCESS;
}
