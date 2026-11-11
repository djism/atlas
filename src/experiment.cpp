#include "atlas/experiment.hpp"
#include "atlas/geometry.hpp"
#include "atlas/mapping.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace atlas {
namespace {
void score(MethodReport& report, std::vector<double>& errors, const Pose2D& estimate,
           const Pose2D& truth, bool lost) {
    const double e = std::hypot(estimate.x - truth.x, estimate.y - truth.y);
    const double heading = std::abs(std::remainder(estimate.theta - truth.theta, 2 * std::acos(-1.0)));
    report.worst_error = std::max(report.worst_error, e);
    report.mean_error += e;  // divided by the scan count at the end
    report.final_error = e;
    report.worst_heading_error = std::max(report.worst_heading_error, heading);
    report.lost += lost;
    errors.push_back(e);
}
}  // namespace

std::vector<Pose2D> circlePath(const Point2D& center, double radius, double step, double laps) {
    if (!(radius > 0) || !(step > 0) || !(laps > 0) || !std::isfinite(radius + step + laps))
        throw std::invalid_argument("circlePath: radius, step, and laps must be positive");
    const double pi = std::acos(-1.0);
    const auto count = static_cast<std::size_t>(laps * 2 * pi * radius / step);
    std::vector<Pose2D> path;
    path.reserve(count + 1);
    for (std::size_t k = 0; k <= count; ++k) {
        const double a = k * step / radius;
        path.push_back({center.x + radius * std::cos(a), center.y + radius * std::sin(a), a + pi / 2});
    }
    return path;
}

std::vector<Pose2D> waypointPath(const std::vector<Point2D>& waypoints, double step, double turn_step) {
    if (waypoints.size() < 2) throw std::invalid_argument("waypointPath needs at least two waypoints");
    if (!(step > 0) || !(turn_step > 0) || !std::isfinite(step + turn_step))
        throw std::invalid_argument("waypointPath: steps must be positive");
    const auto headingTo = [](const Point2D& a, const Point2D& b) { return std::atan2(b.y - a.y, b.x - a.x); };
    Pose2D p{waypoints[0].x, waypoints[0].y, headingTo(waypoints[0], waypoints[1])};
    std::vector<Pose2D> path{p};
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const Point2D& a = waypoints[i];
        const Point2D& b = waypoints[i + 1];
        // Turn in place toward b.
        const double target = headingTo(a, b);
        double remaining = std::remainder(target - p.theta, 2 * std::acos(-1.0));
        while (std::abs(remaining) > 1e-12) {
            const double turn = std::clamp(remaining, -turn_step, turn_step);
            p.theta = std::remainder(p.theta + turn, 2 * std::acos(-1.0));
            remaining -= turn;
            path.push_back(p);
        }
        // Drive straight to b; the last step may be shorter. Steps are counted,
        // not summed: adding 0.1 ten times gives 0.9999..., an extra tiny step.
        const double length = std::hypot(b.x - a.x, b.y - a.y);
        const auto steps = static_cast<std::size_t>(std::ceil(length / step - 1e-9));
        for (std::size_t k = 1; k <= steps; ++k) {
            const double s = std::min(k * step, length) / length;
            p.x = a.x + s * (b.x - a.x);
            p.y = a.y + s * (b.y - a.y);
            path.push_back(p);
        }
    }
    return path;
}

ExperimentReport runExperiment(const ExperimentConfig& config) {
    if (config.trajectory.empty()) throw std::invalid_argument("experiment needs a trajectory");
    if (config.survey.empty()) throw std::invalid_argument("experiment needs survey poses");

    // 1. Map from survey scans at answer-key poses.
    const OccupancyMap map = surveyMap(config.walls, config.survey, config.survey_lidar, config.map_resolution);

    ExperimentReport report;
    const auto map_points = map.occupiedPoints();
    report.map_points = map_points.size();

    // 2. Run both estimators on identical scans.
    ScanTracker tracker(config.trajectory.front(), config.tracker);
    MapLocalizer localizer(map_points, config.trajectory.front(), config.localizer);
    Pose2D dead_reckoning = config.trajectory.front();
    for (std::size_t k = 0; k < config.trajectory.size(); ++k) {
        const Pose2D& truth = config.trajectory[k];
        // Wheel odometry for the motion since the previous scan (none for the
        // first). Its seed stream is offset so it never reuses a scan's seed.
        Pose2D odometry{0, 0, 0};
        if (k > 0) {
            const Pose2D& before = config.trajectory[k - 1];
            report.driven += std::hypot(truth.x - before.x, truth.y - before.y);
            odometry = simulateOdometry(relativePose(before, truth), config.odometry,
                                        (config.seed << 20) + 0x0D0 + k);
        }
        dead_reckoning = composePoses(dead_reckoning, odometry);
        score(report.odometry, report.odometry_errors, dead_reckoning, truth, false);
        const auto scan = addGaussianNoise(
            rangesToPoints(simulateLidarRanges(config.walls, truth, config.lidar), config.lidar),
            config.noise, config.seed + k);
        report.sparse_scans += scan.size() < config.localizer.icp.min_pairs;

        const TrackerStep t = tracker.update(scan);
        score(report.tracker, report.tracker_errors, t.pose_in_world, truth, t.lost);
        const LocalizerStep l = config.localizer_uses_odometry ? localizer.update(scan, odometry)
                                                               : localizer.update(scan);
        score(report.localizer, report.localizer_errors, l.pose_in_map, truth, l.lost);
        report.localizer_weak += l.weakly_constrained;
        report.localizer_jumps += l.jump_rejected;
        if (!l.lost) report.localizer_min_inliers = std::min(report.localizer_min_inliers, l.inlier_fraction);
    }
    report.scans = config.trajectory.size();
    report.tracker.mean_error /= report.scans;
    report.localizer.mean_error /= report.scans;
    report.odometry.mean_error /= report.scans;
    return report;
}
}  // namespace atlas
