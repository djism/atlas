#include "atlas/geometry.hpp"
#include "atlas/experiment.hpp"
#include "atlas/icp.hpp"
#include "atlas/localizer.hpp"
#include "atlas/mapping.hpp"
#include "atlas/matching.hpp"
#include "atlas/planner.hpp"
#include "atlas/simulation.hpp"
#include "atlas/tracker.hpp"
#include <algorithm>
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
    // ---- Seeded Gaussian noise ----
    {
        const std::vector<Point2D> clean{{1,2}, {-3,0.5}, {4,-4}};
        const auto a = addGaussianNoise(clean, 0.05, 42);
        const auto b = addGaussianNoise(clean, 0.05, 42);
        const auto c = addGaussianNoise(clean, 0.05, 43);
        bool same = true, differs = false, moved = false;
        for (std::size_t i = 0; i < clean.size(); ++i) {
            same = same && a[i].x == b[i].x && a[i].y == b[i].y;
            differs = differs || a[i].x != c[i].x || a[i].y != c[i].y;
            moved = moved || a[i].x != clean[i].x;
        }
        check(same, "noise: same seed reproduces exactly");
        check(differs, "noise: different seed gives different noise");
        check(moved, "noise: nonzero stddev changes points");
        check(clean[0].x == 1 && clean[0].y == 2, "noise: input left unchanged");
        const auto unchanged = addGaussianNoise(clean, 0.0, 42);
        for (std::size_t i = 0; i < clean.size(); ++i) {
            check(unchanged[i].x == clean[i].x && unchanged[i].y == clean[i].y,
                  "noise: zero stddev returns points unchanged");
        }
        rejects([] { addGaussianNoise({{0,0}}, -0.1, 1); }, "noise: negative stddev");
        rejects([] { addGaussianNoise({{0,0}}, std::nan(""), 1); }, "noise: NaN stddev");

        // Statistics over 20000 points (40000 values) at the origin, sigma = 0.05 m.
        // Bounds are several standard errors wide; a fixed seed makes this deterministic.
        const double sigma = 0.05;
        const auto samples = addGaussianNoise(std::vector<Point2D>(20000, Point2D{0, 0}), sigma, 7);
        double sum = 0, sum_sq = 0;
        std::size_t within_one_sigma = 0;
        for (const auto& p : samples) {
            for (double v : {p.x, p.y}) {
                sum += v;
                sum_sq += v * v;
                if (std::abs(v) <= sigma) ++within_one_sigma;
            }
        }
        const double n = 2.0 * samples.size();
        const double mean = sum / n;
        const double sample_sigma = std::sqrt(sum_sq / n - mean * mean);
        check(std::abs(mean) < 5 * sigma / std::sqrt(n), "noise: mean near zero");
        check(std::abs(sample_sigma - sigma) < 0.03 * sigma, "noise: stddev within 3%");
        // Gaussian: ~68.3% within one sigma. A uniform distribution with the same
        // stddev gives ~57.7%, so this checks the shape, not just the spread.
        check(std::abs(within_one_sigma / n - 0.6827) < 0.01, "noise: Gaussian shape");
    }
    // ---- Ray-cast LiDAR ----
    {
        // 4 beams: 0 deg (forward), 90, 180, 270. Sensor at origin facing +x.
        LidarConfig four;
        four.beam_count = 4;
        four.max_range = 10;
        near(beamAngle(1, four), pi / 2, "beam 1 at 90 degrees");
        const Segment2D wall{{3,-1}, {3,2}};  // worked example: hit at t = 3, u = 1/3
        const auto ranges = simulateLidarRanges({wall}, {0,0,0}, four);
        near(ranges[0], 3, "forward beam hits wall at 3 m");
        check(std::isinf(ranges[1]) && std::isinf(ranges[2]) && std::isinf(ranges[3]),
              "other beams miss");
        // Nearest wall wins: a wall at x = 3 hides the one at x = 5 (occlusion).
        // Checked in both list orders, so "last hit wins" cannot pass by luck.
        near(simulateLidarRanges({{{5,-1},{5,1}}, wall}, {0,0,0}, four)[0], 3, "nearer wall occludes (far first)");
        near(simulateLidarRanges({wall, {{5,-1},{5,1}}}, {0,0,0}, four)[0], 3, "nearer wall occludes (near first)");
        // Range limit, with the boundary counted as a hit.
        LidarConfig short_range = four;
        short_range.max_range = 2.5;
        check(std::isinf(simulateLidarRanges({wall}, {0,0,0}, short_range)[0]), "beyond max range: no return");
        short_range.max_range = 3;
        near(simulateLidarRanges({wall}, {0,0,0}, short_range)[0], 3, "exactly max range: hit");
        // Endpoints count; just past an end misses.
        near(simulateLidarRanges({{{3,0},{3,2}}}, {0,0,0}, four)[0], 3, "hit at wall endpoint");
        check(std::isinf(simulateLidarRanges({{{3,0.001},{3,2}}}, {0,0,0}, four)[0]), "just past endpoint misses");
        // Wall behind the sensor: only the 180-degree beam sees it.
        const auto behind = simulateLidarRanges({{{-3,-1},{-3,1}}}, {0,0,0}, four);
        check(std::isinf(behind[0]), "forward beam ignores wall behind");
        near(behind[2], 3, "backward beam hits wall behind");
        // Parallel wall along y = 1: beam 0 runs alongside it; beam 1 (90 deg) hits it.
        const auto parallel = simulateLidarRanges({{{-5,1},{5,1}}}, {0,0,0}, four);
        check(std::isinf(parallel[0]), "parallel beam does not hit");
        near(parallel[1], 1, "perpendicular beam hits");
        // Frames: sensor at (5,3) heading 90 deg; forward is world +y. A wall at
        // world y = 6 is 3 m ahead and appears at (3, 0) in SENSOR axes.
        const Pose2D pose{5, 3, pi / 2};
        const auto world_ranges = simulateLidarRanges({{{0,6},{10,6}}}, pose, four);
        near(world_ranges[0], 3, "rotated sensor: forward hit at 3 m");
        const auto points = rangesToPoints(world_ranges, four);
        const auto expected = inverseTransformPoint({5, 6}, pose);
        check(points.size() == 1, "rangesToPoints skips beams without return");
        near(points[0].x, expected.x, "lidar point in sensor axes x");
        near(points[0].y, expected.y, "lidar point in sensor axes y");
        // Property: in a 10 m square room, all 360 beams hit, and every point,
        // mapped back to world, lies on a wall.
        const std::vector<Segment2D> room{{{0,0},{10,0}}, {{10,0},{10,10}},
                                          {{10,10},{0,10}}, {{0,10},{0,0}}};
        const Pose2D inside{4, 6, 0.3};
        LidarConfig dense;  // 360 beams, 10 m
        const auto room_ranges = simulateLidarRanges(room, inside, dense);
        const auto room_points = rangesToPoints(room_ranges, dense);
        check(room_points.size() == 360, "closed room: every beam returns");
        bool on_walls = true;
        for (const auto& p : room_points) {
            const auto w = transformPoint(p, inside);
            const double to_wall = std::min({std::abs(w.x), std::abs(w.x - 10),
                                             std::abs(w.y), std::abs(w.y - 10)});
            on_walls = on_walls && to_wall < 1e-9;
        }
        check(on_walls, "every room point lies on a wall");
        LidarConfig no_beams = four;
        no_beams.beam_count = 0;
        rejects([&] { simulateLidarRanges({wall}, {0,0,0}, no_beams); }, "lidar zero beams");
        LidarConfig bad_range = four;
        bad_range.max_range = -1;
        rejects([&] { simulateLidarRanges({wall}, {0,0,0}, bad_range); }, "lidar negative range");
        rejects([&] { simulateLidarRanges({{{1,1},{1,1}}}, {0,0,0}, four); }, "lidar zero-length wall");
        rejects([&] { simulateLidarRanges({{{1,std::nan("")},{1,1}}}, {0,0,0}, four); }, "lidar NaN wall");
        rejects([&] { simulateLidarRanges({wall}, {0,0,std::nan("")}, four); }, "lidar NaN pose");
        rejects([&] { rangesToPoints({1.0, 2.0}, four); }, "ranges size must match beams");
    }
    // ---- Composing poses ----
    {
        // B at (1,0) facing 90 deg in A; C is 1 m forward of B. In A, forward for
        // B is +y, so C is at (1,1) facing 90 deg.
        const auto c = composePoses({1, 0, pi / 2}, {1, 0, 0});
        near(c.x, 1, "compose x");
        near(c.y, 1, "compose y");
        near(c.theta, pi / 2, "compose heading");
        // A pose composed with its inverse is the identity.
        const Pose2D p{2, -3, 2.5};
        const auto back = inverseTransformPoint({0, 0}, p);  // A's origin in p's axes
        const auto identity = composePoses(p, {back.x, back.y, -p.theta});
        near(identity.x, 0, "compose with inverse x");
        near(identity.y, 0, "compose with inverse y");
        near(identity.theta, 0, "compose with inverse heading");
        near(composePoses({0, 0, 3}, {0, 0, 0.5}).theta, 3.5 - 2 * pi, "compose wraps heading");
    }
    // ---- Normals: local line fits ----
    {
        // Points on the line y = 2x: normal is perpendicular to direction (1, 2).
        std::vector<Point2D> line;
        for (int i = 0; i < 8; ++i) line.push_back({0.5 * i, 1.0 * i});
        const auto normals = estimateNormals(line);
        bool all_valid = true, perpendicular = true, unit = true;
        for (const auto& n : normals) {
            all_valid = all_valid && n.valid;
            perpendicular = perpendicular && std::abs(n.normal.x * 1 + n.normal.y * 2) < 1e-9;
            unit = unit && std::abs(std::hypot(n.normal.x, n.normal.y) - 1) < 1e-9;
        }
        check(all_valid && perpendicular && unit, "normals on a line: valid, unit, perpendicular");
        // Corner: the point at the bend has neighbors on two walls, so no line fits.
        const std::vector<Point2D> corner{{-2,0}, {-1,0}, {0,0}, {0,1}, {0,2}};
        check(!estimateNormals(corner)[2].valid, "corner point has no valid normal");
        check(!estimateNormals({{1,1}, {1,1}, {1,1}}, 3)[0].valid, "coincident points: no normal");
        rejects([] { estimateNormals({{0,0},{1,0},{2,0}}, 2); }, "normals need 3 neighbors");
        rejects([] { estimateNormals({{0,0},{1,0}}, 3); }, "normals need enough points");
        rejects([] { estimateNormals({{0,0},{1,0},{std::nan(""),0}}, 3); }, "normals NaN point");
    }
    // ---- Point-to-line ICP on ray-cast room scans ----
    {
        const std::vector<Segment2D> room{
            {{0,0},{8,0}}, {{8,0},{8,2}}, {{8,2},{9.5,2}}, {{9.5,2},{9.5,4}}, {{9.5,4},{8,4}},
            {{8,4},{8,6}}, {{8,6},{0,6}}, {{0,6},{0,0}},
            {{2,4},{3,4}}, {{3,4},{3,5}}, {{3,5},{2,5}}, {{2,5},{2,4}}};  // alcove + box
        const Pose2D before{4, 3, 0};
        const Pose2D after{4.3, 3.05, 5 * pi / 180};
        LidarConfig lidar;  // 360 beams, 10 m
        const auto scan1 = rangesToPoints(simulateLidarRanges(room, before, lidar), lidar);
        const auto scan2 = rangesToPoints(simulateLidarRanges(room, after, lidar), lidar);
        const auto offset = inverseTransformPoint({after.x, after.y}, before);
        const Pose2D truth{offset.x, offset.y, after.theta - before.theta};
        const auto errorOf = [&](const IcpResult& r) {
            return std::hypot(r.second_in_first.x - truth.x, r.second_in_first.y - truth.y);
        };
        const auto to_point = alignIcp(scan1, scan2, {0,0,0});
        IcpOptions line_options;
        line_options.method = IcpMethod::PointToLine;
        const auto to_line = alignIcp(scan1, scan2, {0,0,0}, line_options);
        // No noise, yet point-to-point is not exact: beams hit different wall spots.
        check(errorOf(to_point) > 1e-3, "room point-to-point: millimeter-level error without noise");
        check(to_line.converged(), "room point-to-line converges");
        check(to_line.observability.weak > 0.2, "full room view: every direction well constrained");
        check(errorOf(to_line) < errorOf(to_point) / 3, "room point-to-line at least 3x more accurate");
        check(errorOf(to_line) < 1e-3, "room point-to-line within 1 mm here");
        check(std::abs(to_line.second_in_first.theta - truth.theta) < 0.001, "room point-to-line rotation");
        check(to_line.iterations < to_point.iterations / 2, "room point-to-line needs far fewer passes");
        check(to_line.pairs_kept_per_pass.back() < scan2.size(), "corner pairs without normals dropped");

        // Hallway: two long parallel walls, ends out of range. Every normal is
        // across the hallway, so nothing constrains motion along it: the solve refuses.
        const std::vector<Segment2D> hallway{{{-50,-1},{50,-1}}, {{-50,1},{50,1}}};
        const auto h1 = rangesToPoints(simulateLidarRanges(hallway, {0,0,0}, lidar), lidar);
        const auto h2 = rangesToPoints(simulateLidarRanges(hallway, {0.5,0,0}, lidar), lidar);
        // ---- Scan-to-scan tracking along a circle in the room ----
        {
            const auto truthAt = [&](int k) {
                const double a = k * 0.05 / 1.5;  // 5 cm per scan on a 1.5 m radius
                return Pose2D{5 + 1.5 * std::cos(a), 2.5 + 1.5 * std::sin(a), a + pi / 2};
            };
            ScanTracker tracker(truthAt(0));  // only the initial pose is given
            const auto first_step = tracker.update(
                rangesToPoints(simulateLidarRanges(room, truthAt(0), lidar), lidar));
            check(!first_step.lost && first_step.pose_in_world.x == truthAt(0).x,
                  "tracker first scan keeps initial pose");
            std::size_t lost = 0;
            TrackerStep last;
            const int steps = 120;  // 6 m of travel
            for (int k = 1; k <= steps; ++k) {
                const auto scan = addGaussianNoise(
                    rangesToPoints(simulateLidarRanges(room, truthAt(k), lidar), lidar), 0.01, 100 + k);
                last = tracker.update(scan);
                if (last.lost) ++lost;
            }
            const auto end = truthAt(steps);
            const double heading_diff = last.pose_in_world.theta - end.theta;
            check(lost == 0, "tracker loop: no lost steps");
            check(tracker.scansProcessed() == steps + 1, "tracker counts scans");
            // Drift accumulates; bounds describe this scene (measured ~2-4 cm).
            check(std::hypot(last.pose_in_world.x - end.x, last.pose_in_world.y - end.y) < 0.1,
                  "tracker loop: drift under 10 cm over 6 m here");
            check(std::abs(std::atan2(std::sin(heading_diff), std::cos(heading_diff))) < 0.02,
                  "tracker loop: heading drift under ~1 deg here");
        }
        {
            // Hallway: every registration is degenerate, so every step is lost.
            ScanTracker tracker({0, 0, 0});
            std::size_t lost = 0;
            for (int k = 0; k < 10; ++k) {
                const auto step = tracker.update(
                    rangesToPoints(simulateLidarRanges(hallway, {0.1 * k, 0, 0}, lidar), lidar));
                if (step.lost) ++lost;
            }
            check(lost == 9, "tracker hallway: every registration flagged lost");
        }
        rejects([] { ScanTracker t({0, 0, std::nan("")}); }, "tracker NaN initial pose");

        const auto hall = alignIcp(h1, h2, {0,0,0}, line_options);
        check(hall.status == IcpStatus::Degenerate, "hallway point-to-line reports Degenerate");
        check(hall.pose_per_pass.empty() && hall.second_in_first.x == 0, "degenerate keeps the guess");
    }
    // ---- Occupancy mapping: one beam at a time, 1 m cells ----
    {
        LidarConfig one{1, 10.0};  // a single beam, straight ahead
        OccupancyMap map({0, 0}, 6, 4, 1.0);
        // Sensor in cell (0,0) facing +x; wall 3 m ahead ends the beam at (3.5, 0.5).
        map.integrateScan({3.0}, one, {0.5, 0.5, 0});
        check(map.columns() == 6 && map.rows() == 4, "map size in cells");
        check(map.state(0, 0) == CellState::Free && map.state(1, 0) == CellState::Free &&
              map.state(2, 0) == CellState::Free, "beam path cells free");
        check(map.state(3, 0) == CellState::Occupied, "beam end cell occupied");
        check(map.state(4, 0) == CellState::Unknown, "cell behind the hit stays unknown");
        check(map.state(0, 1) == CellState::Unknown, "cell beside the beam stays unknown");
        near(map.logOdds(3, 0), 0.85, "one hit adds the hit weight");
        near(map.logOdds(1, 0), -0.4, "one pass adds the miss weight");
        check(std::abs(map.probability(3, 0) - 0.7) < 0.01, "one hit is about 70% occupied");
        check(map.stateAt({-1, 0}) == CellState::Unknown && map.logOdds(9, 9) == 0,
              "outside the map is unknown");

        OccupancyMap clear({0, 0}, 6, 4, 1.0);
        clear.integrateScan({std::numeric_limits<double>::infinity()}, {1, 2.2}, {0.5, 0.5, 0});
        check(clear.state(2, 0) == CellState::Free && clear.state(3, 0) == CellState::Unknown,
              "no return clears up to max range, no further");

        // 30-degree beam ending at (2.92, 1.90): crosses x = 1 first, then y = 1, then x = 2.
        OccupancyMap slanted({0, 0}, 6, 4, 1.0);
        std::vector<double> twelve(12, std::numeric_limits<double>::infinity());
        twelve[1] = 2.8;  // beam 1 of 12 points at 30 degrees
        slanted.integrateScan(twelve, {12, 0.4}, {0.5, 0.5, 0});  // other beams: 0.4 m, stay in cell
        OccupancyMap ray({0, 0}, 6, 4, 1.0);
        std::vector<double> only(12, 0.0);  // zero-length beams touch only the sensor cell
        only[1] = 2.8;
        ray.integrateScan(only, {12, 10.0}, {0.5, 0.5, 0});
        check(ray.state(1, 0) == CellState::Free && ray.state(1, 1) == CellState::Free,
              "slanted beam visits each crossed cell");
        check(ray.state(2, 1) == CellState::Occupied, "slanted beam end cell occupied");
        check(ray.state(0, 1) == CellState::Unknown && ray.state(2, 0) == CellState::Unknown,
              "slanted beam skips cells it does not cross");
        check(slanted.state(1, 0) == CellState::Unknown, "no-return beam shorter than a cell stays home");

        for (int k = 0; k < 20; ++k) map.integrateScan({3.0}, one, {0.5, 0.5, 0});
        near(map.logOdds(3, 0), 4.0, "repeated hits clamp at max log-odds");
        near(map.logOdds(1, 0), -4.0, "repeated passes clamp at min log-odds");

        rejects([] { OccupancyMap m({0, 0}, 0, 4, 1.0); }, "map zero width");
        rejects([] { OccupancyMap m({0, 0}, 4, 4, -1); }, "map negative resolution");
        rejects([] { OccupancyMap m({std::nan(""), 0}, 4, 4, 1); }, "map NaN origin");
        rejects([] { OccupancyMapOptions o; o.hit = -1; OccupancyMap m({0, 0}, 4, 4, 1, o); },
                "map hit weight must be positive");
        rejects([&] { map.integrateScan({3.0, 3.0}, one, {0.5, 0.5, 0}); }, "map range count");
        rejects([&] { map.integrateScan({-1.0}, one, {0.5, 0.5, 0}); }, "map negative range");
        rejects([&] { map.integrateScan({std::nan("")}, one, {0.5, 0.5, 0}); }, "map NaN range");
        rejects([&] { map.integrateScan({3.0}, one, {7, 0.5, 0}); }, "map sensor outside");
        rejects([&] { map.integrateScan({3.0}, one, {0.5, 0.5, std::nan("")}); }, "map NaN pose");
    }
    // ---- Occupancy mapping: a ray-cast room, and a box that is later removed ----
    {
        const std::vector<Segment2D> walls{
            {{0,0},{8,0}}, {{8,0},{8,2}}, {{8,2},{9.5,2}}, {{9.5,2},{9.5,4}}, {{9.5,4},{8,4}},
            {{8,4},{8,6}}, {{8,6},{0,6}}, {{0,6},{0,0}}};
        const std::vector<Segment2D> box{{{2,4},{3,4}}, {{3,4},{3,5}}, {{3,5},{2,5}}, {{2,5},{2,4}}};
        std::vector<Segment2D> with_box = walls;
        with_box.insert(with_box.end(), box.begin(), box.end());
        const auto distanceToSegments = [](const Point2D& p, const std::vector<Segment2D>& segs) {
            double best = std::numeric_limits<double>::infinity();
            for (const auto& s : segs) {
                const double ex = s.end.x - s.start.x, ey = s.end.y - s.start.y;
                const double u = std::clamp(((p.x - s.start.x) * ex + (p.y - s.start.y) * ey) /
                                            (ex * ex + ey * ey), 0.0, 1.0);
                best = std::min(best, std::hypot(p.x - s.start.x - u * ex, p.y - s.start.y - u * ey));
            }
            return best;
        };
        LidarConfig lidar;  // 360 beams, 10 m
        const double res = 0.05;
        OccupancyMap map({-0.5, -0.5}, 10.5, 7, res);
        // Mapping with KNOWN poses: the answer key gives each scan's pose. This is
        // building a map, not localizing; localization against it is the next step.
        const std::vector<Pose2D> survey{{1.5,1.5,0}, {4,1.5,0.5}, {6.5,1.5,1}, {8.7,3,2},
                                         {6.5,4.5,2.5}, {4,3,3}, {1.2,4.8,-1}};
        for (const auto& pose : survey) map.integrateScan(simulateLidarRanges(with_box, pose, lidar), lidar, pose);

        const auto occupied = map.occupiedCenters();
        double worst = 0;
        for (const auto& c : occupied) worst = std::max(worst, distanceToSegments(c, with_box));
        check(occupied.size() > 400, "room map has occupied cells");
        check(worst <= res, "every occupied cell is within one cell of a real wall");
        std::size_t samples = 0, found = 0;
        for (const auto& s : with_box) {
            const double len = std::hypot(s.end.x - s.start.x, s.end.y - s.start.y);
            for (double d = 0.025; d < len; d += 0.05, ++samples) {
                const Point2D p{s.start.x + (s.end.x - s.start.x) * d / len,
                                s.start.y + (s.end.y - s.start.y) * d / len};
                bool near_occupied = false;  // the hit cell may be on either side of the wall line
                for (long dc = -1; dc <= 1; ++dc)
                    for (long dr = -1; dr <= 1; ++dr)
                        near_occupied = near_occupied || map.state(map.columnOf(p.x) + dc, map.rowOf(p.y) + dr) == CellState::Occupied;
                found += near_occupied;
            }
        }
        check(found >= samples * 95 / 100, "at least 95% of wall length is mapped");
        check(map.stateAt({5, 1}) == CellState::Free, "open floor is free");
        check(map.stateAt({2.5, 4.5}) == CellState::Unknown, "inside the box: never seen");
        check(map.stateAt({-0.3, 3}) == CellState::Unknown, "outside the room: never seen");

        // The world changes: the box is removed. Its front face (x = 3, seen from
        // (4, 3)) was mapped as occupied; new scans see the wall behind it instead.
        const Pose2D spot{4, 3, 0};
        OccupancyMap changing({-0.5, -0.5}, 10.5, 7, res);
        for (int k = 0; k < 3; ++k) changing.integrateScan(simulateLidarRanges(with_box, spot, lidar), lidar, spot);
        // A hit exactly on x = 3 falls in the cell that STARTS at 3 (the robot's side).
        const Point2D face{3.01, 4.5};
        check(changing.stateAt(face) == CellState::Occupied, "box face mapped while present");
        int scans_to_clear = 0;
        while (changing.stateAt(face) != CellState::Free && scans_to_clear < 50) {
            changing.integrateScan(simulateLidarRanges(walls, spot, lidar), lidar, spot);
            ++scans_to_clear;
        }
        check(changing.stateAt(face) == CellState::Free, "removed box face becomes free");
        check(scans_to_clear > 1, "one scan does not erase repeated evidence");

        // Mean hit points sit on the walls; cell centers can be half a cell off.
        const auto hit_points = map.occupiedPoints();
        // Exception: a corner cell averages hits from two walls, landing between them.
        double worst_point = 0, mean_center = 0;
        std::size_t off_wall = 0;
        for (const auto& p : hit_points) {
            const double d = distanceToSegments(p, with_box);
            worst_point = std::max(worst_point, d);
            off_wall += d > 1e-6;
        }
        for (const auto& c : occupied) mean_center += distanceToSegments(c, with_box) / occupied.size();
        check(hit_points.size() == occupied.size(), "one mean hit point per occupied cell");
        check(off_wall * 100 < hit_points.size(), "noise-free mean hit points lie on the walls (corners aside)");
        check(worst_point < res / 2, "corner mean points stay within half a cell");
        check(mean_center > 0.01, "cell centers are biased off the walls");

        // ---- Scan-to-map localization vs scan-to-scan tracking ----
        // Three laps of a 1.5 m circle, 10 cm per scan, 3 cm scan noise. The map
        // (built above from answer-key poses) and the scans are all the localizer
        // gets, plus the starting pose. The answer key is used only to score it.
        const auto truthAt = [&](int k) {
            const double a = k * 0.1 / 1.5;
            return Pose2D{4.5 + 1.5 * std::cos(a), 2.8 + 1.5 * std::sin(a), a + pi / 2};
        };
        const int scans = static_cast<int>(3 * 2 * pi * 1.5 / 0.1);
        ScanTracker tracker(truthAt(0));
        MapLocalizer localizer(hit_points, truthAt(0));
        double tracker_worst = 0, localizer_worst = 0, first_lap_worst = 0, last_lap_worst = 0;
        int localizer_lost = 0;
        double min_inliers = 1;
        for (int k = 0; k <= scans; ++k) {
            const Pose2D truth = truthAt(k);
            const auto scan = addGaussianNoise(
                rangesToPoints(simulateLidarRanges(with_box, truth, lidar), lidar), 0.03, 1000 + k);
            const auto t = tracker.update(scan).pose_in_world;
            const auto step = localizer.update(scan);
            const double e = std::hypot(step.pose_in_map.x - truth.x, step.pose_in_map.y - truth.y);
            tracker_worst = std::max(tracker_worst, std::hypot(t.x - truth.x, t.y - truth.y));
            localizer_worst = std::max(localizer_worst, e);
            if (k < scans / 3) first_lap_worst = std::max(first_lap_worst, e);
            if (k > 2 * scans / 3) last_lap_worst = std::max(last_lap_worst, e);
            localizer_lost += step.lost;
            min_inliers = std::min(min_inliers, step.inlier_fraction);
        }
        check(localizer_lost == 0, "map localizer: no lost scans on three laps");
        check(localizer_worst < 0.02, "map localizer stays within 2 cm (3 cm noise)");
        check(tracker_worst > 4 * localizer_worst, "scan-to-scan tracker drifts several times farther");
        check(last_lap_worst < 2 * first_lap_worst, "map localizer error does not grow lap to lap");
        check(min_inliers > 0.85, "map localizer: most scan points match the map");

        // A wrong starting pose (10 cm, 2 degrees off) is corrected by the first
        // scan; scan-to-scan tracking would keep that offset forever.
        MapLocalizer corrected(hit_points, {4.1, 3.05, 2 * pi / 180});
        const auto first = corrected.update(rangesToPoints(simulateLidarRanges(with_box, spot, lidar), lidar));
        check(std::hypot(first.pose_in_map.x - 4, first.pose_in_map.y - 3) < 0.005 &&
              std::abs(first.pose_in_map.theta) < 0.002, "first scan corrects a wrong starting pose");

        // The world changed: a box the map never saw, 1 m ahead. Fewer points
        // match the map, but enough walls do to keep the pose.
        auto changed = with_box;
        changed.insert(changed.end(), {{{5,2.5},{5,3.5}}, {{5,3.5},{5.5,3.5}}, {{5.5,3.5},{5.5,2.5}}, {{5.5,2.5},{5,2.5}}});
        MapLocalizer seen(hit_points, spot), surprised(hit_points, spot);
        const auto usual = seen.update(rangesToPoints(simulateLidarRanges(with_box, spot, lidar), lidar));
        const auto unusual = surprised.update(rangesToPoints(simulateLidarRanges(changed, spot, lidar), lidar));
        check(std::hypot(unusual.pose_in_map.x - 4, unusual.pose_in_map.y - 3) < 0.005, "unmapped box: pose kept");
        check(unusual.inlier_fraction < usual.inlier_fraction - 0.1, "unmapped box lowers inlier fraction");

        // Motion sanity check: the robot "jumps" 0.5 m between scans. ICP finds
        // the new pose, but it is too far from the guess, so the step is rejected.
        MapLocalizer jumpy(hit_points, spot);
        jumpy.update(rangesToPoints(simulateLidarRanges(with_box, spot, lidar), lidar));
        const auto jump = jumpy.update(rangesToPoints(simulateLidarRanges(with_box, {4.5, 3, 0}, lidar), lidar));
        check(jump.jump_rejected && jump.lost, "0.5 m jump rejected and flagged lost");
        check(std::hypot(jump.pose_in_map.x - 4, jump.pose_in_map.y - 3) < 0.005, "rejected jump keeps the prediction");

        // A robot that starts turning in place at full rate (0.32 rad per scan, as
        // in the sandbox) after driving straight. A 0.2 rad jump limit rejected
        // the correct answers here forever; the limit must cover this.
        {
            MapLocalizer turning(hit_points, {2, 2.5, 0});
            Pose2D p{2, 2.5, 0};
            int turn_lost = 0;
            double turn_worst = 0;
            for (int k = 0; k < 20; ++k) {
                const auto st = turning.update(rangesToPoints(simulateLidarRanges(with_box, p, lidar), lidar));
                turn_lost += st.lost;
                turn_worst = std::max(turn_worst, std::abs(std::remainder(st.pose_in_map.theta - p.theta, 2 * pi)));
                if (k < 10) p.x += 0.06; else p.theta += 0.32;
            }
            check(turn_lost == 0 && turn_worst < 0.01, "sudden in-place turn: tracked, not rejected as a jump");
        }

        // The sandbox house (12 x 8 m, two rooms, furniture). The robot drives to
        // the doorway, spins in place from 0.99 to -0.73 rad at 3.2 rad/s with a
        // scan every 0.18 s, then leaves. Far walls make a 0.58 rad heading error
        // push most points outside the pair gate, so ICP from the guess converges
        // wrongly with a poor fit (about a third of the scan). The fit check
        // rejects that, and the heading search recovers.
        {
            std::vector<Segment2D> house{{{0,0},{12,0}}, {{12,0},{12,8}}, {{12,8},{0,8}}, {{0,8},{0,0}},
                                         {{6,0},{6,3}}, {{6,4.5},{6,8}}};
            for (const auto& f : std::vector<std::vector<double>>{{0.5,4,0.8,2.2}, {3,4,1.2,0.7}, {3,7.75,2,0.4},
                                                                  {11.1,4.25,1.6,2.1}, {8,7.6,1.6,0.6}, {8.5,0.45,1.4,0.6}}) {
                const double x0 = f[0] - f[2] / 2, x1 = f[0] + f[2] / 2, y0 = f[1] - f[3] / 2, y1 = f[1] + f[3] / 2;
                house.insert(house.end(), {{{x0,y0},{x1,y0}}, {{x1,y0},{x1,y1}}, {{x1,y1},{x0,y1}}, {{x0,y1},{x0,y0}}});
            }
            std::vector<Pose2D> stops;
            for (const Point2D& p : std::vector<Point2D>{{1.5,1.5}, {4.5,1.5}, {6,3.75}, {8,2}, {10,2}, {10,6.5},
                                                          {8,6.5}, {4.5,6.5}, {1.5,6.5}})
                stops.push_back({p.x, p.y, 0});
            const auto house_map = surveyMap(house, stops, {360, 20}, 0.05).occupiedPoints();
            std::vector<Pose2D> route;
            Pose2D p{5.4, 2.85, 0.99};
            int phase = 0;
            for (double t = 0; t < 4; t += 0.18) {
                route.push_back(p);
                if (phase == 0) {
                    p.x += 0.108 * std::cos(p.theta); p.y += 0.108 * std::sin(p.theta);
                    if (std::hypot(p.x - 6, p.y - 3.74) < 0.06) phase = 1;
                } else if (phase == 1) {
                    p.theta = std::max(-0.73, p.theta - 0.576);
                    if (p.theta <= -0.73) phase = 2;
                } else { p.x += 0.108 * std::cos(p.theta); p.y += 0.108 * std::sin(p.theta); }
            }
            const LidarConfig house_lidar{360, 8};
            const auto runRoute = [&](const MapLocalizerOptions& o, int& recovered, int& poor) {
                MapLocalizer loc(house_map, route.front(), o);
                double worst = 0;
                for (std::size_t k = 0; k < route.size(); ++k) {
                    const auto st = loc.update(addGaussianNoise(
                        rangesToPoints(simulateLidarRanges(house, route[k], house_lidar), house_lidar), 0.01, 300 + k));
                    worst = std::max(worst, std::hypot(st.pose_in_map.x - route[k].x, st.pose_in_map.y - route[k].y));
                    recovered += st.recovered;
                    poor += st.poor_fit;
                }
                return worst;
            };
            int recovered = 0, poor = 0, unused = 0;
            const double with_recovery = runRoute({}, recovered, poor);
            MapLocalizerOptions no_recovery;
            no_recovery.recovery_heading_range = 0;
            no_recovery.min_inlier_fraction = 0;
            const double without = runRoute(no_recovery, unused, unused);
            check(poor > 0 && recovered > 0, "doorway spin: poor fit detected and recovered");
            check(with_recovery < 0.05, "doorway spin: stays within 5 cm with fit check and recovery");
            check(without > 0.5, "doorway spin: without them the estimate runs away");

            // A start 1.5 m and 1 rad wrong: the result fits too little of the scan,
            // so it is flagged lost instead of being accepted as the pose.
            MapLocalizer bad_start(house_map, {route.front().x + 1.5, route.front().y, route.front().theta + 1});
            const auto first_bad = bad_start.update(
                rangesToPoints(simulateLidarRanges(house, route.front(), house_lidar), house_lidar));
            check(first_bad.lost && first_bad.poor_fit, "badly wrong start: poor fit flagged, not accepted");
        }

        rejects([&] { MapLocalizer m({{0, 0}, {1, 0}}, spot); }, "localizer map too small");
        rejects([&] { MapLocalizer m(hit_points, {0, std::nan(""), 0}); }, "localizer NaN pose");

        // ---- Degeneracy: a 3 m sensor sometimes sees only parallel walls ----
        // Circle of 1.8 m around (4.6, 2.6), 1 cm noise. Near k = 66 the scan holds
        // only the bottom wall and the box face parallel to it: x is unobservable.
        const auto aroundAt = [&](int k) {
            const double a = k * 0.1 / 1.8;
            return Pose2D{4.6 + 1.8 * std::cos(a), 2.6 + 1.8 * std::sin(a), a + pi / 2};
        };
        const LidarConfig short_lidar{360, 3.0};
        MapLocalizerOptions blind_to_degeneracy;
        blind_to_degeneracy.min_weak_observability = 0;
        MapLocalizer handled(hit_points, aroundAt(0)), unhandled(hit_points, aroundAt(0), blind_to_degeneracy);
        double handled_worst = 0, unhandled_worst = 0, weakest = 1;
        int weak_steps = 0;
        for (int k = 0; k <= 100; ++k) {
            const Pose2D truth = aroundAt(k);
            const auto scan = addGaussianNoise(
                rangesToPoints(simulateLidarRanges(with_box, truth, short_lidar), short_lidar), 0.01, 7 + k);
            const auto h = handled.update(scan);
            const auto u = unhandled.update(scan);
            handled_worst = std::max(handled_worst, std::hypot(h.pose_in_map.x - truth.x, h.pose_in_map.y - truth.y));
            unhandled_worst = std::max(unhandled_worst, std::hypot(u.pose_in_map.x - truth.x, u.pose_in_map.y - truth.y));
            weak_steps += h.weakly_constrained;
            if (!h.lost) weakest = std::min(weakest, h.observability.weak);
        }
        check(weakest < 0.01, "short range: some scans see only parallel walls");
        check(weak_steps > 0, "weak scans are flagged");
        check(unhandled_worst > 1.0, "ignoring degeneracy: slides compound past 1 m");
        check(handled_worst < 0.05, "keeping the prediction along the weak direction: within 5 cm");

        // One scan of that parallel-wall view, from a guess 10 cm off along the
        // walls (x) and 5 cm off across them (y). The scan fixes y; it cannot fix x,
        // so x keeps the guess, even though the guess is wrong there.
        const Pose2D view = aroundAt(66);
        MapLocalizer one_view(hit_points, {view.x + 0.10, view.y + 0.05, view.theta});
        const auto v = one_view.update(addGaussianNoise(
            rangesToPoints(simulateLidarRanges(with_box, view, short_lidar), short_lidar), 0.01, 73));
        check(v.weakly_constrained && std::abs(v.observability.weak_direction.x) > 0.99,
              "parallel-wall view: weak direction is x");
        check(std::abs(v.pose_in_map.y - view.y) < 0.005, "across the walls: scan corrects y");
        const Point2D& w = v.observability.weak_direction;  // nearly, not exactly, x
        check(std::abs((v.pose_in_map.x - (view.x + 0.10)) * w.x + (v.pose_in_map.y - (view.y + 0.05)) * w.y) < 1e-9,
              "along the weak direction: no change from the guess");
        check(std::abs(v.pose_in_map.x - (view.x + 0.10)) < 0.002, "so the 10 cm error along the walls remains");
    }
    // ---- Scripted experiment harness ----
    {
        const auto circle = circlePath({0, 0}, 1, 0.1, 1);
        check(circle.size() == 63, "circle path: one pose per 10 cm, both ends included");
        near(circle[0].x, 1, "circle path starts at angle 0");
        near(circle[0].theta, pi / 2, "circle path heading along travel");
        rejects([] { circlePath({0, 0}, 0, 0.1, 1); }, "circle path zero radius");

        ExperimentConfig config;
        config.walls = {{{0,0},{6,0}}, {{6,0},{6,4}}, {{6,4},{0,4}}, {{0,4},{0,0}}, {{1,1},{1.5,1}}};
        config.survey = {{3, 2, 0}, {1, 3, 1}};
        config.trajectory = circlePath({3, 2}, 1, 0.1, 0.5);
        config.noise = 0.0;
        const auto report = runExperiment(config);
        check(report.scans == config.trajectory.size(), "experiment: one result per pose");
        check(report.tracker_errors.size() == report.scans && report.localizer_errors.size() == report.scans,
              "experiment: per-scan errors recorded");
        check(std::abs(report.driven - 0.1 * (report.scans - 1)) < 0.01, "experiment: distance driven");
        check(report.map_points > 100 && report.sparse_scans == 0, "experiment: map built, scans not sparse");
        check(report.localizer.lost == 0 && report.localizer.worst_error < 0.01,
              "experiment: noise-free localizer within 1 cm");
        check(std::abs(report.localizer.final_error - report.localizer_errors.back()) < 1e-12,
              "experiment: final error is the last scan's");
        const auto surveyed = surveyMap(config.walls, config.survey, {}, 0.05);
        near(surveyed.origin().x, -0.5, "survey map: origin is the walls' corner minus the margin");
        check(surveyed.columns() == 140 && surveyed.rows() == 100, "survey map: walls' box plus margins");
        rejects([&] { surveyMap({}, config.survey, {}, 0.05); }, "survey map without walls");
        rejects([&] { surveyMap(config.walls, {}, {}, 0.05); }, "survey map without poses");
        ExperimentConfig empty = config;
        empty.trajectory.clear();
        rejects([&] { runExperiment(empty); }, "experiment without trajectory");
    }
    // ---- Translation observability of line constraints ----
    {
        const auto all_up = translationObservability({{0, 1}, {0, -1}, {0, 1}});
        near(all_up.weak, 0, "parallel normals: weak 0");
        near(all_up.strong, 1, "parallel normals: strong 1");
        near(std::abs(all_up.weak_direction.x), 1, "parallel normals: weak direction along the walls");
        const auto corner = translationObservability({{0, 1}, {1, 0}});
        near(corner.weak, 0.5, "corner: weak 0.5");
        near(corner.strong, 0.5, "corner: strong 0.5");
        std::vector<Point2D> mostly_up(9, {0, 1});
        mostly_up.push_back({1, 0});
        const auto ninety_ten = translationObservability(mostly_up);
        near(ninety_ten.weak, 0.1, "90/10 normals: weak 0.1");
        near(std::abs(ninety_ten.weak_direction.x), 1, "90/10 normals: weak along x");
        const double h = std::sqrt(0.5);
        const auto diagonal = translationObservability({{h, h}, {-h, -h}});
        near(diagonal.weak, 0, "diagonal walls: weak 0");
        near(std::abs(diagonal.weak_direction.x * h + diagonal.weak_direction.y * h), 0,
             "diagonal walls: weak direction perpendicular to normals");
        near(std::hypot(diagonal.weak_direction.x, diagonal.weak_direction.y), 1, "weak direction is unit");
        rejects([] { translationObservability({}); }, "observability empty");
        rejects([] { translationObservability({{0, 2}}); }, "observability non-unit normal");
    }
    // ---- Grid planning (A*) ----
    {
        const auto pathIsFree = [](const OccupancyGrid& g, const PlanResult& r) {
            for (std::size_t i = 1; i < r.path.size(); ++i) {
                if (!segmentIsFree(g, r.path[i - 1], r.path[i])) return false;
            }
            return true;
        };
        // Inflation: cells near a wall are blocked, far cells are free, outside is blocked.
        const auto walled = buildPlanningGrid({{{5, 0}, {5, 3}}}, 10, 6, 0.05, 0.18);
        check(walled.blockedAt({5.2, 1}), "cell within robot radius + margin of wall is blocked");
        check(!walled.blockedAt({5.3, 1}), "cell beyond radius + margin is free");
        check(walled.blocked(-1, 0) && walled.blocked(0, 1000), "outside the grid counts as blocked");

        // Free space: straight line, smoothed to just start and goal.
        const OccupancyGrid empty(10, 6, 0.05);
        const auto straight = planPath(empty, {1, 1}, {8, 1});
        check(straight.status == PlanStatus::Found && straight.path.size() == 2, "free space: straight path");
        near(straight.length, 7, "free space: length equals distance");

        // Wall x = 5 from y = 0 to 3: go around its top end.
        const auto around = planPath(walled, {2, 1}, {8, 1});
        check(around.status == PlanStatus::Found, "around wall: found");
        check(pathIsFree(walled, around), "around wall: every segment collision-free");
        check(around.length > 6 && around.length < 9.5, "around wall: plausible detour length");
        check(around.path.front().x == 2 && around.path.back().x == 8, "path starts and ends exactly");

        // Doorway: wall at x = 5 with a 1 m gap from y = 2.5 to 3.5. Open, then closed.
        const std::vector<Segment2D> door{{{5, 0}, {5, 2.5}}, {{5, 3.5}, {5, 6}}};
        const auto through = planPath(buildPlanningGrid(door, 10, 6, 0.05, 0.18), {2, 1}, {8, 1});
        check(through.status == PlanStatus::Found, "open doorway: found");
        bool used_gap = false;
        for (const auto& p : through.path) used_gap = used_gap || (p.y > 2.5 && p.y < 3.5);
        check(used_gap, "open doorway: path passes through the gap");
        auto closed_door = door;
        closed_door.push_back({{5, 2.5}, {5, 3.5}});
        check(planPath(buildPlanningGrid(closed_door, 10, 6, 0.05, 0.18), {2, 1}, {8, 1}).status ==
                  PlanStatus::NoPath, "closed doorway: no path");
        // A gap narrower than the robot plus margin is closed for planning.
        const std::vector<Segment2D> narrow{{{5, 0}, {5, 2.9}}, {{5, 3.1}, {5, 6}}};
        check(planPath(buildPlanningGrid(narrow, 10, 6, 0.05, 0.18), {2, 1}, {8, 1}).status ==
                  PlanStatus::NoPath, "20 cm gap too narrow for 18 cm robot");

        // Shortest-path check in a U-shaped trap open toward the start. Going around
        // the U's inflated corners (inflation 0.23 m) along straight lines is
        // 2*hypot(1.77, 2.23) + 2.46 = 8.154 m; rounded inflation lets a shortest
        // path cut slightly inside that, so it must not be longer. An overestimating
        // heuristic (not admissible) produced 8.20 m here.
        const std::vector<Segment2D> u{{{4, 1}, {6, 1}}, {{6, 1}, {6, 5}}, {{6, 5}, {4, 5}}};
        const auto u_grid = buildPlanningGrid(u, 10, 6, 0.05, 0.18);
        const auto trap = planPath(u_grid, {2, 3}, {8, 3});
        check(trap.status == PlanStatus::Found && pathIsFree(u_grid, trap), "U trap: found, collision-free");
        check(trap.length <= 2 * std::hypot(1.77, 2.23) + 2.46, "U trap: no longer than corner detour");

        check(planPath(walled, {5, 1}, {8, 1}).status == PlanStatus::StartBlocked, "start on obstacle");
        check(planPath(walled, {2, 1}, {5.1, 1}).status == PlanStatus::GoalBlocked, "goal on obstacle");

        // No corner cutting: two blocked cells touching only at a corner.
        OccupancyGrid corner(0.3, 0.3, 0.1);
        corner.addSegment({{0.15, 0.05}, {0.15, 0.05}}, 0.01);  // blocks cell (1, 0)
        corner.addSegment({{0.05, 0.15}, {0.05, 0.15}}, 0.01);  // blocks cell (0, 1)
        const auto squeeze = planPath(corner, {0.05, 0.05}, {0.15, 0.15});
        check(squeeze.status == PlanStatus::NoPath, "diagonal squeeze between corners refused");
        rejects([] { OccupancyGrid g(0, 1, 0.1); }, "grid zero width");
        rejects([&] { planPath(empty, {std::nan(""), 1}, {2, 2}); }, "plan NaN start");
    }
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
        check(result.converged(), "icp x-axis converges");
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
        check(!limited.converged() && limited.iterations == 1, "icp reports hitting pass limit");
    }
    {
        // Moderate motion from a zero guess: recovers the truth exactly.
        const Pose2D truth{1, 0.5, 0.5};
        const auto result = alignIcp(applyPose(cloud, truth), cloud, {0,0,0});
        check(result.converged(), "icp cloud converges");
        near(result.second_in_first.x, truth.x, "icp cloud x");
        near(result.second_in_first.y, truth.y, "icp cloud y");
        near(result.second_in_first.theta, truth.theta, "icp cloud rotation");
    }
    {
        // Seeded noise on BOTH scans (sigma 2 cm): ICP still converges, near but
        // not at the truth. Bounds describe this seed, not a general accuracy claim.
        const Pose2D truth{1, 0.5, 0.5};
        const auto first = addGaussianNoise(applyPose(cloud, truth), 0.02, 1);
        const auto second = addGaussianNoise(cloud, 0.02, 2);
        const auto result = alignIcp(first, second, {0,0,0});
        const double position_error = std::hypot(result.second_in_first.x - truth.x,
                                                 result.second_in_first.y - truth.y);
        check(result.converged(), "icp noisy converges");
        check(position_error > 1e-6, "icp noisy is not exact");
        check(position_error < 0.1, "icp noisy position within 10 cm here");
        check(std::abs(result.second_in_first.theta - truth.theta) < 0.01,
              "icp noisy rotation within 0.01 rad here");
    }
    {
        // Partial overlap. ICP pairs EVERY second-scan point, so a point seen only in
        // the second scan is forced onto some first-scan point and biases the pose.
        // Noise-free; numbers describe this scene, not a general claim.
        const Pose2D truth{1, 0.5, 0.5};
        const auto first = applyPose(cloud, truth);
        auto second = cloud;
        second.push_back({8, -5});  // new pole, visible only after moving
        const auto biased = alignIcp(first, second, {0,0,0});
        const auto& pose = biased.second_in_first;
        check(biased.converged(), "overlap: still reports stable");
        check(std::hypot(pose.x - truth.x, pose.y - truth.y) > 1.0, "overlap: position off by >1 m");
        check(std::abs(pose.theta - truth.theta) > 0.17, "overlap: rotation off by >10 deg");
        // The unpartnered point leaves the largest residual: what a gate can exploit.
        const auto matches = findNearestCorrespondences(first, second, pose);
        std::size_t worst = 0;
        double worst_residual = 0;
        for (std::size_t i = 0; i < second.size(); ++i) {
            const auto aligned = transformPoint(second[i], pose);
            const double r = std::hypot(aligned.x - first[matches[i]].x,
                                        aligned.y - first[matches[i]].y);
            if (r > worst_residual) { worst_residual = r; worst = i; }
        }
        check(worst == 5, "overlap: unpartnered point has largest residual here");
        // Even a lone point close to the others biases the estimate.
        auto second_near = cloud;
        second_near.push_back({3, 1});
        const auto near_result = alignIcp(first, second_near, {0,0,0}).second_in_first;
        check(std::hypot(near_result.x - truth.x, near_result.y - truth.y) > 0.1,
              "overlap: nearby lone point still biases by >10 cm");
        // An extra point only in the FIRST scan is harmless here: nothing is forced
        // to pair with it (unless it is nearer than a true partner).
        auto first_extra = first;
        first_extra.push_back(transformPoint({8, -5}, truth));
        const auto clean = alignIcp(first_extra, cloud, {0,0,0}).second_in_first;
        near(clean.x, truth.x, "overlap: extra first-scan point harmless x");
        near(clean.y, truth.y, "overlap: extra first-scan point harmless y");
        near(clean.theta, truth.theta, "overlap: extra first-scan point harmless rotation");

        // ---- Distance gate on the same lone-pole scene ----
        const auto gated = [&](double gate, const Pose2D& guess,
                               const std::vector<Point2D>& f, const std::vector<Point2D>& s) {
            IcpOptions options;
            options.max_pair_distance = gate;
            return alignIcp(f, s, guess, options);
        };
        const auto errorOf = [&](const IcpResult& r) {
            return std::hypot(r.second_in_first.x - truth.x, r.second_in_first.y - truth.y);
        };
        // Right size (4 m) from a zero guess: lone pole dropped, exact recovery.
        const auto right = gated(4.0, {0,0,0}, first, second);
        check(right.converged() && right.pairs_kept_per_pass.back() == 5, "gate 4 m keeps 5 true pairs");
        near(errorOf(right), 0, "gate 4 m recovers exact position");
        near(right.second_in_first.theta, truth.theta, "gate 4 m recovers exact rotation");
        // Too tight for a poor early guess: true pairs dropped on pass 1, and the
        // result is WORSE than no gate at all. Stable, but wrong.
        const auto tight = gated(2.0, {0,0,0}, first, second);
        check(tight.converged() && tight.pairs_kept_per_pass.front() == 3, "gate 2 m keeps only 3 pairs");
        check(errorOf(tight) > errorOf(biased), "gate 2 m from zero guess is worse than no gate");
        check(std::abs(tight.second_in_first.theta - truth.theta) > 0.5, "gate 2 m rotation off by >28 deg");
        // Not monotonic: a TIGHTER 1 m gate succeeds here. Only 2 pairs survive pass 1,
        // they happen to be correct, and as the guess improves all 5 true pairs pass.
        // Growth across passes requires gating with the CURRENT guess.
        const auto bootstrap = gated(1.0, {0,0,0}, first, second);
        check(bootstrap.converged() && bootstrap.pairs_kept_per_pass.front() == 2 &&
                  bootstrap.pairs_kept_per_pass.back() == 5,
              "gate 1 m: kept pairs grow 2 -> 5 as the guess improves");
        near(errorOf(bootstrap), 0, "gate 1 m: recovers exact position");
        // Far too tight: localization lost, reported clearly, last pose kept.
        const auto lost = gated(0.5, {0,0,0}, first, second);
        check(lost.status == IcpStatus::TooFewPairs, "gate 0.5 m from zero guess: too few pairs");
        check(lost.iterations == 1 && lost.pose_per_pass.empty(), "lost on pass 1, no completed pass");
        check(lost.second_in_first.x == 0 && lost.second_in_first.theta == 0, "lost keeps initial guess");
        // With a good guess (as from tracking), a tight gate works even with noise.
        const Pose2D good_guess{0.9, 0.4, 0.45};
        const auto noisy_first = addGaussianNoise(first, 0.02, 1);
        const auto noisy_second = addGaussianNoise(second, 0.02, 2);
        const auto ungated_noisy = gated(std::numeric_limits<double>::infinity(), good_guess,
                                         noisy_first, noisy_second);
        const auto gated_noisy = gated(1.0, good_guess, noisy_first, noisy_second);
        check(errorOf(ungated_noisy) > 0.5, "no gate, good guess: lone pole still biases >50 cm");
        check(gated_noisy.converged() && gated_noisy.pairs_kept_per_pass.back() == 5,
              "gate 1 m, good guess: keeps 5 true pairs under noise");
        check(errorOf(gated_noisy) < 0.05, "gate 1 m, good guess: error near noise level");
    }
    rejects([] { IcpOptions o; o.max_pair_distance = 0; alignIcp({{0,0},{1,0},{0,1}}, {{0,0},{1,0},{0,1}}, {0,0,0}, o); },
            "icp zero gate");
    rejects([] { IcpOptions o; o.max_pair_distance = std::nan(""); alignIcp({{0,0},{1,0},{0,1}}, {{0,0},{1,0},{0,1}}, {0,0,0}, o); },
            "icp NaN gate");
    rejects([] { IcpOptions o; o.min_pairs = 1; alignIcp({{0,0},{1,0},{0,1}}, {{0,0},{1,0},{0,1}}, {0,0,0}, o); },
            "icp min_pairs below 2");
    {
        // FAILURE: 162-degree turn from a zero guess. Pairing stabilizes on a wrong
        // pose (local minimum). "converged" means stable, not correct.
        const Pose2D truth{0, 0, 0.9 * pi};
        const auto result = alignIcp(applyPose(cloud, truth), cloud, {0,0,0});
        check(result.converged(), "icp large turn still reports stable");
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
        check(result.converged(), "icp corridor reports stable");
        near(result.second_in_first.x, 0, "icp corridor misses the 1 m motion");
        near(result.nearest_pair_rmse.back(), 0, "icp corridor fits perfectly anyway");

        // Ambiguity check: restarting 1 m forward/back reaches a different pose
        // (about +/-0.9 m) that still explains 18 of 20 poles: flagged ambiguous.
        const auto report = checkAmbiguity(window, window, result.second_in_first);
        check(report.ambiguous && report.rival_found, "corridor flagged ambiguous");
        check(report.solution_inliers == 20 && report.rival_inliers == 18, "corridor rival explains 18/20");
        near(std::abs(report.rival.x), 0.9, "corridor rival about one pole spacing away");
        // Only turn restarts: they reach a different pose that fits badly (0 inliers).
        // A weak rival is not ambiguity, and the check misses the slide rival
        // because no offset reaches it: "not ambiguous" does not prove uniqueness.
        AmbiguityOptions turns_only;
        turns_only.offsets = {{0, 0, 0.3}, {0, 0, -0.3}};
        const auto weak = checkAmbiguity(window, window, result.second_in_first, turns_only);
        check(weak.rival_found && weak.rival_inliers < 5 && !weak.ambiguous,
              "corridor turn-only restarts: weak rival, not ambiguous");
    }
    {
        // Well-spread noisy cloud: every restart returns to the same pose.
        const Pose2D truth{1, 0.5, 0.5};
        const auto first = addGaussianNoise(applyPose(cloud, truth), 0.02, 1);
        const auto second = addGaussianNoise(cloud, 0.02, 2);
        const auto solution = alignIcp(first, second, {0,0,0}).second_in_first;
        const auto report = checkAmbiguity(first, second, solution);
        check(!report.ambiguous && !report.rival_found, "spread cloud not ambiguous");
        check(report.solution_inliers == 5, "spread cloud solution explains all points");
    }
    {
        // Wrong local minimum (162-degree turn from zero guess). With a half-turn
        // restart offset, the check finds a rival that explains MORE points:
        // evidence the reported solution is wrong, not just uncertain.
        const Pose2D truth{0, 0, 0.9 * pi};
        const auto first = applyPose(cloud, truth);
        const auto solution = alignIcp(first, cloud, {0,0,0}).second_in_first;
        AmbiguityOptions options;
        options.offsets.push_back({0, 0, pi});
        const auto report = checkAmbiguity(first, cloud, solution, options);
        check(report.ambiguous && report.rival_inliers > report.solution_inliers,
              "local minimum: rival explains more points");
        near(report.rival.theta, truth.theta, "local minimum: rival is the true rotation");
    }
    check(countInliers({{0,0}, {1,0}}, {{0,0}, {1,0.5}}, {0,0,0}, 0.2) == 1, "count inliers");
    rejects([] { countInliers({{0,0}}, {{0,0}}, {0,0,0}, 0); }, "inliers zero distance");
    rejects([] { AmbiguityOptions o; o.offsets.clear();
                 checkAmbiguity({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, o); }, "ambiguity no offsets");
    rejects([] { AmbiguityOptions o; o.rival_fraction = 0;
                 checkAmbiguity({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, o); }, "ambiguity zero fraction");
    rejects([] { AmbiguityOptions o; o.rival_fraction = 1.5;
                 checkAmbiguity({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, o); }, "ambiguity fraction above 1");
    // Options are set by field name: positional {a, b, c} silently shifts when fields are added.
    rejects([] { IcpOptions o; o.max_iterations = 0;
                 alignIcp({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, o); }, "icp zero iterations");
    rejects([] { IcpOptions o; o.translation_tolerance = -1;
                 alignIcp({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, o); }, "icp negative tolerance");
    rejects([] { IcpOptions o; o.rotation_tolerance = std::nan("");
                 alignIcp({{0,0},{1,0}}, {{0,0},{1,0}}, {0,0,0}, o); }, "icp NaN tolerance");
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
