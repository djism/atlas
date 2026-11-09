#include "atlas/experiment.hpp"
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

ExperimentReport runExperiment(const ExperimentConfig& config) {
    if (config.trajectory.empty()) throw std::invalid_argument("experiment needs a trajectory");
    if (config.survey.empty()) throw std::invalid_argument("experiment needs survey poses");

    // 1. Map from survey scans at answer-key poses, sized to the walls plus a margin.
    double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
    double max_x = -min_x, max_y = -min_x;
    for (const auto& w : config.walls)
        for (const auto& p : {w.start, w.end}) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        }
    const double margin = 0.5;
    OccupancyMap map({min_x - margin, min_y - margin}, max_x - min_x + 2 * margin,
                     max_y - min_y + 2 * margin, config.map_resolution);
    for (const auto& pose : config.survey)
        map.integrateScan(simulateLidarRanges(config.walls, pose, config.survey_lidar), config.survey_lidar, pose);

    ExperimentReport report;
    const auto map_points = map.occupiedPoints();
    report.map_points = map_points.size();

    // 2. Run both estimators on identical scans.
    ScanTracker tracker(config.trajectory.front(), config.tracker);
    MapLocalizer localizer(map_points, config.trajectory.front(), config.localizer);
    for (std::size_t k = 0; k < config.trajectory.size(); ++k) {
        const Pose2D& truth = config.trajectory[k];
        if (k > 0) {
            const Pose2D& before = config.trajectory[k - 1];
            report.driven += std::hypot(truth.x - before.x, truth.y - before.y);
        }
        const auto scan = addGaussianNoise(
            rangesToPoints(simulateLidarRanges(config.walls, truth, config.lidar), config.lidar),
            config.noise, config.seed + k);
        report.sparse_scans += scan.size() < config.localizer.icp.min_pairs;

        const TrackerStep t = tracker.update(scan);
        score(report.tracker, report.tracker_errors, t.pose_in_world, truth, t.lost);
        const LocalizerStep l = localizer.update(scan);
        score(report.localizer, report.localizer_errors, l.pose_in_map, truth, l.lost);
        report.localizer_weak += l.weakly_constrained;
        report.localizer_jumps += l.jump_rejected;
        if (!l.lost) report.localizer_min_inliers = std::min(report.localizer_min_inliers, l.inlier_fraction);
    }
    report.scans = config.trajectory.size();
    report.tracker.mean_error /= report.scans;
    report.localizer.mean_error /= report.scans;
    return report;
}
}  // namespace atlas
