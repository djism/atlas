#pragma once
#include "atlas/localizer.hpp"
#include "atlas/simulation.hpp"
#include "atlas/tracker.hpp"
#include "atlas/types.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace atlas {
// Answer-key sensor poses (world frame) along a circle, heading along the
// direction of travel (counterclockwise). `step` is meters between scans.
std::vector<Pose2D> circlePath(const Point2D& center, double radius, double step, double laps);

// Answer-key sensor poses for a robot that drives straight between waypoints
// (world frame) at `step` meters per scan, stops at each waypoint, and turns in
// place toward the next at up to `turn_step` radians per scan. The start
// heading faces the second waypoint. Unlike a circle, speed and turn rate
// change suddenly, which is what a constant-velocity guess handles worst.
// Throws std::invalid_argument for fewer than two waypoints or nonpositive steps.
std::vector<Pose2D> waypointPath(const std::vector<Point2D>& waypoints, double step, double turn_step);

// One scripted run: simulate scans along a known trajectory, feed the same
// scans to the scan-to-scan tracker and the map localizer, and score both (and
// odometry alone) against the answer key. The estimators receive only scans, the map, and the
// first pose; the trajectory is used to simulate and to score.
struct ExperimentConfig {
    std::vector<Segment2D> walls;        // the world, world frame
    std::vector<Pose2D> survey;          // answer-key poses for building the map
    double map_resolution = 0.05;        // meters
    std::vector<Pose2D> trajectory;      // answer-key poses, one scan each
    LidarConfig survey_lidar;            // sensor for building the map (default 10 m)
    LidarConfig lidar;                   // sensor during the run
    double noise = 0.01;                 // meters, Gaussian per point coordinate
    std::uint64_t seed = 1;              // scan k uses seed + k
    TrackerOptions tracker;
    MapLocalizerOptions localizer;
    // Simulated wheel odometry, one reading per scan (separate seed stream).
    // Always scored on its own as dead reckoning; the localizer uses it as its
    // guess only when localizer_uses_odometry is set (else constant velocity).
    OdometryNoise odometry;
    bool localizer_uses_odometry = false;
};

struct MethodReport {
    double worst_error = 0;      // meters, position vs answer key
    double mean_error = 0;
    double final_error = 0;
    double worst_heading_error = 0;  // radians
    std::size_t lost = 0;            // scans where registration failed or was rejected
};

struct ExperimentReport {
    std::size_t scans = 0;
    double driven = 0;               // meters along the trajectory
    std::size_t map_points = 0;
    std::size_t sparse_scans = 0;    // scans with fewer points than the localizer's min_pairs
    MethodReport tracker;
    MethodReport localizer;
    MethodReport odometry;           // dead reckoning: odometry readings chained, no scans
    std::size_t localizer_weak = 0;  // scans flagged weakly_constrained
    std::size_t localizer_jumps = 0; // scans with a rejected jump
    double localizer_min_inliers = 1;
    // Per-scan position errors, for plots (meters).
    std::vector<double> tracker_errors;
    std::vector<double> localizer_errors;
    std::vector<double> odometry_errors;
};

// Throws std::invalid_argument for an empty trajectory or survey, or invalid
// inputs to the underlying functions.
ExperimentReport runExperiment(const ExperimentConfig& config);
}  // namespace atlas
