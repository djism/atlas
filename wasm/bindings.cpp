// JavaScript bindings for the Atlas core, compiled only by Emscripten.
// The core library (src/, include/) has no browser dependencies; this file
// exposes a small, explicit surface: scan simulation, planning, one ICP run
// replayed pass by pass, and a tracker.
#include "atlas/icp.hpp"
#include "atlas/matching.hpp"
#include "atlas/planner.hpp"
#include "atlas/simulation.hpp"
#include "atlas/tracker.hpp"
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <cstdint>
#include <exception>
#include <string>
#include <vector>

using emscripten::val;

namespace {
val toFloat64Array(const std::vector<double>& values) {
    val array = val::global("Float64Array").new_(values.size());
    array.call<void>("set", val(emscripten::typed_memory_view(values.size(), values.data())));
    return array;
}

val errorObject(const std::exception& e) {
    val result = val::object();
    result.set("error", std::string(e.what()));
    return result;
}

const char* statusName(atlas::IcpStatus status) {
    switch (status) {
        case atlas::IcpStatus::Converged: return "converged";
        case atlas::IcpStatus::HitIterationLimit: return "iteration-limit";
        case atlas::IcpStatus::TooFewPairs: return "too-few-pairs";
        case atlas::IcpStatus::Degenerate: return "degenerate";
    }
    return "unknown";
}

std::vector<atlas::Segment2D> segmentsFromFlat(const std::vector<double>& flat) {
    std::vector<atlas::Segment2D> segments;
    for (std::size_t i = 0; i + 3 < flat.size(); i += 4) {
        segments.push_back({{flat[i], flat[i + 1]}, {flat[i + 2], flat[i + 3]}});
    }
    return segments;
}

// walls: flat [x0, y0, x1, y1, ...] in world meters. Returns
// { ranges: Float64Array (per beam, Infinity = no return),
//   points: Float64Array (flat x, y in SENSOR axes, noisy) } or { error }.
val simulateScan(val walls_flat, double x, double y, double theta, int beams,
                 double max_range, double noise_stddev, double seed) {
    try {
        const auto walls = segmentsFromFlat(emscripten::convertJSArrayToNumberVector<double>(walls_flat));
        atlas::LidarConfig config;
        config.beam_count = beams > 0 ? static_cast<std::size_t>(beams) : 0;
        config.max_range = max_range;
        const auto ranges = atlas::simulateLidarRanges(walls, {x, y, theta}, config);
        const auto points = atlas::addGaussianNoise(atlas::rangesToPoints(ranges, config),
                                                    noise_stddev, static_cast<std::uint64_t>(seed));
        std::vector<double> points_flat;
        points_flat.reserve(points.size() * 2);
        for (const auto& p : points) {
            points_flat.push_back(p.x);
            points_flat.push_back(p.y);
        }
        val result = val::object();
        result.set("ranges", toFloat64Array(ranges));
        result.set("points", toFloat64Array(points_flat));
        return result;
    } catch (const std::exception& e) {
        return errorObject(e);
    }
}

// Plans on a grid over [0, width] x [0, height] built from obstacle segments
// inflated by robot_radius (+ 5 cm margin). Returns
// { status: "found" | "start-blocked" | "goal-blocked" | "no-path",
//   path: Float64Array (flat x, y, world frame), length, expanded } or { error }.
val planRoute(val obstacles_flat, double width, double height, double resolution,
              double robot_radius, double sx, double sy, double gx, double gy) {
    try {
        const auto obstacles =
            segmentsFromFlat(emscripten::convertJSArrayToNumberVector<double>(obstacles_flat));
        const auto grid = atlas::buildPlanningGrid(obstacles, width, height, resolution, robot_radius);
        const auto plan = atlas::planPath(grid, {sx, sy}, {gx, gy});
        const char* status = "no-path";
        switch (plan.status) {
            case atlas::PlanStatus::Found: status = "found"; break;
            case atlas::PlanStatus::StartBlocked: status = "start-blocked"; break;
            case atlas::PlanStatus::GoalBlocked: status = "goal-blocked"; break;
            case atlas::PlanStatus::NoPath: status = "no-path"; break;
        }
        std::vector<double> flat;
        for (const auto& p : plan.path) {
            flat.push_back(p.x);
            flat.push_back(p.y);
        }
        val result = val::object();
        result.set("status", std::string(status));
        result.set("path", toFloat64Array(flat));
        result.set("length", plan.length);
        result.set("expanded", static_cast<double>(plan.expanded));
        return result;
    } catch (const std::exception& e) {
        return errorObject(e);
    }
}

std::vector<atlas::Point2D> pointsFromFlat(const std::vector<double>& flat) {
    std::vector<atlas::Point2D> points;
    points.reserve(flat.size() / 2);
    for (std::size_t i = 0; i + 1 < flat.size(); i += 2) points.push_back({flat[i], flat[i + 1]});
    return points;
}

// One ICP run, returned pass by pass so the page can replay what the engine did.
// first_flat, second_flat: flat x, y, each in its own sensor axes.
// (gx, gy, gtheta): initial guess of the second sensor's pose in the first frame.
// Returns { status, iterations,
//   poses: Float64Array (x, y, theta per step; step 0 is the guess, then one per pass),
//   rmse: Float64Array (per completed pass),
//   pairs: Float64Array (per step, second.size() indices into first: the nearest-
//          neighbor pairs used at the START of the next pass) } or { error }.
val alignScans(val first_flat, val second_flat, double gx, double gy, double gtheta,
               bool point_to_line) {
    try {
        const auto first = pointsFromFlat(emscripten::convertJSArrayToNumberVector<double>(first_flat));
        const auto second = pointsFromFlat(emscripten::convertJSArrayToNumberVector<double>(second_flat));
        atlas::IcpOptions options;
        options.method = point_to_line ? atlas::IcpMethod::PointToLine : atlas::IcpMethod::PointToPoint;
        // "Stopped changing" at a visible scale: with centimeter sensor noise the
        // nearest-neighbor pairing keeps jittering below 0.1 mm / 0.1 mrad.
        options.max_iterations = 40;
        options.translation_tolerance = 1e-4;
        options.rotation_tolerance = 1e-4;
        const atlas::Pose2D guess{gx, gy, gtheta};
        const atlas::IcpResult result = atlas::alignIcp(first, second, guess, options);

        std::vector<atlas::Pose2D> steps{guess};
        steps.insert(steps.end(), result.pose_per_pass.begin(), result.pose_per_pass.end());
        std::vector<double> poses;
        std::vector<double> pairs;
        for (const auto& pose : steps) {
            poses.insert(poses.end(), {pose.x, pose.y, pose.theta});
            for (std::size_t index : atlas::findNearestCorrespondences(first, second, pose)) {
                pairs.push_back(static_cast<double>(index));
            }
        }
        val out = val::object();
        out.set("status", std::string(statusName(result.status)));
        out.set("iterations", static_cast<double>(result.iterations));
        out.set("poses", toFloat64Array(poses));
        out.set("rmse", toFloat64Array(result.nearest_pair_rmse));
        out.set("pairs", toFloat64Array(pairs));
        return out;
    } catch (const std::exception& e) {
        return errorObject(e);
    }
}

// Scan-to-scan tracker. Only the initial pose is provided by the caller.
class Tracker {
public:
    Tracker(double x, double y, double theta) : tracker_({x, y, theta}) {}

    // points_flat: flat x, y in current sensor axes. Returns the estimate and
    // diagnostics, or { error }.
    val update(val points_flat) {
        try {
            const auto scan = pointsFromFlat(emscripten::convertJSArrayToNumberVector<double>(points_flat));
            const atlas::TrackerStep step = tracker_.update(scan);
            val result = val::object();
            result.set("x", step.pose_in_world.x);
            result.set("y", step.pose_in_world.y);
            result.set("theta", step.pose_in_world.theta);
            result.set("lost", step.lost);
            result.set("status", std::string(statusName(step.status)));
            result.set("pairsKept", static_cast<double>(step.pairs_kept));
            result.set("iterations", static_cast<double>(step.iterations));
            result.set("residualRmse", step.residual_rmse);
            return result;
        } catch (const std::exception& e) {
            return errorObject(e);
        }
    }

private:
    atlas::ScanTracker tracker_;
};
}  // namespace

EMSCRIPTEN_BINDINGS(atlas_engine) {
    emscripten::function("simulateScan", &simulateScan);
    emscripten::function("planRoute", &planRoute);
    emscripten::function("alignScans", &alignScans);
    emscripten::class_<Tracker>("Tracker")
        .constructor<double, double, double>()
        .function("update", &Tracker::update);
}
