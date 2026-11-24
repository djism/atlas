// KITTI checks. Setup and printing only; the work is in src/kitti.cpp,
// src/geometry3d.cpp, src/matching3d.cpp and src/image.cpp.
//   atlas_kitti scan    <scan.bin> [out.bmp]
//   atlas_kitti truth   <odometry_dir> <seq>
//   atlas_kitti overlap <odometry_dir> <seq> <velodyne_dir> <frame>
//   atlas_kitti map     <odometry_dir> <seq> <velodyne_dir> <out.bmp> [every_nth_scan]
//   atlas_kitti odometry <odometry_dir> <seq> <velodyne_dir> [source_voxel_m] [frames]
//   atlas_kitti deskew-check <odometry_dir> <seq> <velodyne_dir>
//   atlas_kitti icp-from-truth <odometry_dir> <seq> <velodyne_dir>
//   atlas_kitti score   <odometry_dir> <seq> <estimate.txt> [name]   (estimate: Velodyne frame, frame 0 = identity)
//   atlas_kitti plot    <odometry_dir> <seq> <out.svg> <estimate.txt> <name> [<estimate.txt> <name> ...]
//   atlas_kitti run     <odometry_dir> <seq> <velodyne_dir> <out_poses.txt> [key=value ...]
//       One scan-to-map run, streaming (read a scan, process it, discard it); prints one JSON line.
//       Keys: voxel (m, 1.0), max_range (m, 100), deskew (0/1, 1), correct (0/1, 1), frames (all),
//       diag (path: per-frame CSV of ICP diagnostics for the replay explorer; off by default).
//   atlas_kitti loops   <odometry_dir> <seq> <estimate.txt> [keyframe_m=5] [radius_m=10] [min_gap_m=50]
//       Keyframes and loop candidates found in the ESTIMATE; each graded against the answer key.
//   atlas_kitti verify-loops <odometry_dir> <seq> <velodyne_dir> <estimate.txt> [radius_m=10] [half_window=10]
//       Verifies each loop candidate (and a decoy for each) with ICP; grades against the answer key.
//   atlas_kitti close-loops <odometry_dir> <seq> <estimate.txt> <loops.txt>
//       Pose-graph loop closure with verified loops; scores before/after; saves results/atlas_<seq>_slam.txt.
//   atlas_kitti export-replay <velodyne_dir> <estimate.txt> <out_dir> [scan_points=1000] [map_voxel=0.4] [map_range=50]
//       Point data for the web replay explorer (see docs/replay.md): map.bin, Atlas's map built from
//       its OWN estimated poses; scans.bin, a thinned copy of every scan in its sensor frame.
// odometry_dir is .../odometry/dataset (poses/, sequences/); velodyne_dir holds 0000000000.bin ...
#include "atlas/image.hpp"
#include "atlas/kitti.hpp"
#include "atlas/loopclosure3d.hpp"
#include "atlas/matching3d.hpp"
#include "atlas/odometry3d.hpp"
#include "atlas/voxel.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace {
using namespace atlas;

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return NAN;
    const auto k = static_cast<std::size_t>(q * (v.size() - 1));
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
    return v[k];
}

struct Sequence {
    std::vector<Pose3D> camera;    // left camera i in camera 0 (answer key as published)
    std::vector<Pose3D> velodyne;  // Velodyne i in Velodyne 0
    std::vector<double> times;
};

Sequence loadSequence(const std::string& odometry, const std::string& seq) {
    Sequence s;
    s.camera = readKittiPoses(odometry + "/poses/" + seq + ".txt");
    s.velodyne = velodynePoses(s.camera, readKittiVelodyneInCamera(odometry + "/sequences/" + seq + "/calib.txt"));
    s.times = readKittiTimes(odometry + "/sequences/" + seq + "/times.txt");
    if (s.times.size() != s.camera.size())
        throw std::runtime_error("poses and times differ in length: " + std::to_string(s.camera.size()) + " vs " +
                                 std::to_string(s.times.size()));
    return s;
}

int scanCommand(int argc, char** argv) {
    const KittiScan scan = readKittiScan(argv[2]);
    const ScanSummary s = summarizeScan(scan);
    std::printf("KITTI scan %s\n", argv[2]);
    std::printf("  points            %zu\n", s.points);
    std::printf("  range (m)         min %.2f, median %.2f, max %.2f\n", s.min_range, s.median_range, s.max_range);
    std::printf("  median z (m)      %.2f  (sensor ~1.73 m above the road: most points are road)\n", s.median_z);
    std::printf("  forward share     %.2f  (x > 0)\n", s.forward_share);
    std::printf("  max reflectance   %.2f\n", s.max_reflectance);
    std::printf("  first point (m)   x %.3f, y %.3f, z %.3f  (Velodyne frame: x forward, y left, z up)\n",
                scan.points.front().x, scan.points.front().y, scan.points.front().z);
    if (argc == 4) {
        writeBmp(birdsEyeView(scan.points, 50, 0.1), argv[3]);
        std::printf("Wrote %s: top-down, +/-50 m, 10 cm per pixel, forward is up, sensor at the center\n", argv[3]);
    }
    return EXIT_SUCCESS;
}

int truthCommand(char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    double worst_rotation = 0, length = 0, turned = 0;
    std::vector<double> dts, speeds, forward, sideways, vertical;
    for (std::size_t i = 0; i < s.camera.size(); ++i) {
        worst_rotation = std::max(worst_rotation, orthonormalityError(s.camera[i]));
        if (i == 0) continue;
        const Pose3D step = relativePose3D(s.velodyne[i - 1], s.velodyne[i]);  // Velodyne i in Velodyne i-1
        const double d = std::sqrt(step.t.x * step.t.x + step.t.y * step.t.y + step.t.z * step.t.z);
        const double dt = s.times[i] - s.times[i - 1];
        length += d;
        turned += rotationAngle(step);
        dts.push_back(dt);
        speeds.push_back(d / dt);
        forward.push_back(step.t.x);
        sideways.push_back(step.t.y);
        vertical.push_back(step.t.z);
    }
    const Point3D end = s.velodyne.back().t;
    std::printf("KITTI sequence %s ground truth, converted to the Velodyne frame with Tr\n", argv[3]);
    std::printf("  frames               %zu poses, %zu times\n", s.camera.size(), s.times.size());
    std::printf("  time between frames  %.4f to %.4f s\n", percentile(dts, 0), percentile(dts, 1));
    std::printf("  path length          %.1f m; speed min %.2f, median %.2f, max %.2f m/s\n", length,
                percentile(speeds, 0), percentile(speeds, 0.5), percentile(speeds, 1));
    std::printf("  motion per frame     median forward %.3f, left %.3f, up %.3f m (Velodyne axes of the previous frame)\n",
                percentile(forward, 0.5), percentile(sideways, 0.5), percentile(vertical, 0.5));
    std::printf("  total turning        %.1f deg (sum of per-frame rotation angles)\n", turned * 180 / std::acos(-1.0));
    std::printf("  end position         x %.1f forward, y %.1f left, z %.1f up (m, in Velodyne frame 0)\n", end.x, end.y, end.z);
    std::printf("  worst rotation error %.1e (orthonormality of the published matrices)\n", worst_rotation);
    return EXIT_SUCCESS;
}

int overlapCommand(char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::string dir = argv[4];
    const std::size_t k = std::stoul(argv[5]);
    if (k + 1 >= s.velodyne.size()) throw std::runtime_error("frame must be below " + std::to_string(s.velodyne.size() - 1));
    const KittiScan first = readKittiScan(kittiScanPath(dir, k));
    const std::vector<Point3D> second = takeEvery(readKittiScan(kittiScanPath(dir, k + 1)).points, 60);

    // Three guesses for "scan k+1's frame in scan k's frame":
    const Pose3D none{};                                                  // the car did not move
    const Pose3D camera_axes = relativePose3D(s.camera[k], s.camera[k + 1]);  // WRONG: camera motion used as if Velodyne
    const Pose3D velodyne = relativePose3D(s.velodyne[k], s.velodyne[k + 1]); // right: converted with Tr
    std::printf("Scans %zu and %zu of sequence %s: %zu points vs %zu sampled points (every 60th)\n", k, k + 1, argv[3],
                first.points.size(), second.size());
    std::printf("Distance from each sampled point of scan %zu, placed by the guess, to its nearest point of scan %zu:\n", k + 1, k);
    std::printf("  %-44s %8s %8s\n", "guess", "median", "90th %");
    const auto row = [&](const char* name, const Pose3D& guess) {
        const auto d = nearestDistances(first.points, second, guess);
        std::printf("  %-44s %6.3f m %6.3f m\n", name, percentile(d, 0.5), percentile(d, 0.9));
        return d;
    };
    row("no motion", none);
    row("ground truth in CAMERA axes (wrong frame)", camera_axes);
    const auto d = row("ground truth in Velodyne axes (Tr applied)", velodyne);
    std::printf("  (true motion: %.3f m forward, %.3f m left, %.3f m up, %.2f deg)\n", velodyne.t.x, velodyne.t.y,
                velodyne.t.z, rotationAngle(velodyne) * 180 / std::acos(-1.0));

    // The road is flat: a road point moved forward still lands next to road, so
    // road points cannot show forward motion. Repeat with points well above it.
    std::vector<Point3D> raised;
    for (const Point3D& p : second)
        if (p.z > -1.2) raised.push_back(p);  // sensor ~1.73 m above the road: >= ~0.5 m above it
    std::printf("Only the %zu sampled points at least ~0.5 m above the road (trees, poles, cars):\n", raised.size());
    const auto raisedRow = [&](const char* name, const Pose3D& guess) {
        const auto r = nearestDistances(first.points, raised, guess);
        std::printf("  %-44s %6.3f m %6.3f m\n", name, percentile(r, 0.5), percentile(r, 0.9));
    };
    raisedRow("no motion", none);
    raisedRow("ground truth in CAMERA axes (wrong frame)", camera_axes);
    raisedRow("ground truth in Velodyne axes (Tr applied)", velodyne);

    // Where around the car the leftover distance is (raw scans are not motion-compensated).
    const char* names[] = {"front (|azimuth| < 45 deg)", "left", "rear", "right"};
    std::vector<double> sector[4];
    for (std::size_t i = 0; i < second.size(); ++i) {
        const double az = std::atan2(second[i].y, second[i].x) * 180 / std::acos(-1.0);
        const int s4 = std::abs(az) < 45 ? 0 : az >= 45 && az < 135 ? 1 : std::abs(az) >= 135 ? 2 : 3;
        sector[s4].push_back(d[i]);
    }
    std::printf("With the Velodyne ground truth, by direction from the car:\n");
    for (int i = 0; i < 4; ++i)
        std::printf("  %-28s median %.3f m over %zu points\n", names[i], percentile(sector[i], 0.5), sector[i].size());
    return EXIT_SUCCESS;
}

int mapCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::string dir = argv[4];
    const std::size_t every = argc == 7 ? std::max(1ul, std::stoul(argv[6])) : 5;
    std::vector<Point3D> cloud;
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    std::size_t used = 0;
    for (std::size_t i = 0; i < s.velodyne.size(); i += every, ++used) {
        // Every 10th point of the scan, placed in Velodyne frame 0 by the answer key.
        const auto placed = transformPoints(takeEvery(readKittiScan(kittiScanPath(dir, i)).points, 10), s.velodyne[i]);
        cloud.insert(cloud.end(), placed.begin(), placed.end());
        const Point3D& t = s.velodyne[i].t;
        xmin = std::min(xmin, t.x); xmax = std::max(xmax, t.x);
        ymin = std::min(ymin, t.y); ymax = std::max(ymax, t.y);
    }
    const double margin = 40, resolution = 0.2;
    const double half = std::max(xmax - xmin, ymax - ymin) / 2 + margin;
    writeBmp(birdsEyeView(cloud, half, resolution, {(xmin + xmax) / 2, (ymin + ymax) / 2}), argv[5]);
    std::printf("Wrote %s: %zu scans (every %zu), %zu points placed by the ground truth in Velodyne frame 0;\n"
                "top-down, forward (+x of frame 0) is up, %.0f cm per pixel, %.0f m wide\n",
                argv[5], used, every, cloud.size(), resolution * 100, 2 * half);
    return EXIT_SUCCESS;
}
int odometryCommand(int argc, char** argv);
// Which deskew model makes scans agree under the ground truth? Pairs of scans
// (k, k+gap) are deskewed with the TRUE per-sweep motion (an oracle: this tests
// the timing model, not an estimator), placed with the true relative pose, and
// scored by the distance from raised points of scan k+gap to scan k.
int deskewCheckCommand(char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::string dir = argv[4];
    const char* names[] = {"no deskew", "deskew, clockwise", "deskew, counterclockwise"};
    std::printf("Sequence %s: raised points (>= ~0.5 m above the road) of scan k+gap vs scan k, both deskewed\n"
                "with the true sweep motion, placed by the true relative pose. Median distance per pair (m),\n"
                "capped at 1 m (pairs farther apart count as 1 m). k = 0, 20, ..., 260.\n\n", argv[3]);
    for (std::size_t gap : {1u, 5u}) {
        std::vector<double> per_model[3], rear[3];
        for (std::size_t k = 0; k + gap + 1 < s.velodyne.size(); k += 20) {
            const auto a = readKittiScan(kittiScanPath(dir, k)).points;
            const auto b = takeEvery(readKittiScan(kittiScanPath(dir, k + gap)).points, 20);
            const Pose3D sweep_a = relativePose3D(s.velodyne[k], s.velodyne[k + 1]);
            const Pose3D sweep_b = relativePose3D(s.velodyne[k + gap], s.velodyne[k + gap + 1]);
            const Pose3D b_in_a = relativePose3D(s.velodyne[k], s.velodyne[k + gap]);
            for (int m = 0; m < 3; ++m) {
                const auto da = m == 0 ? a : deskewScan(a, sweep_a, m == 1 ? SweepDirection::Clockwise : SweepDirection::Counterclockwise);
                const auto db = m == 0 ? b : deskewScan(b, sweep_b, m == 1 ? SweepDirection::Clockwise : SweepDirection::Counterclockwise);
                VoxelGrid3D grid(1.0);
                grid.insert(da);
                std::vector<double> d, d_rear;
                for (std::size_t i = 0; i < db.size(); ++i) {
                    if (b[i].z <= -1.2) continue;  // raised points only (judged before deskew)
                    Point3D q;
                    double dist = 1.0;
                    if (!grid.nearest(transformPoint3D(db[i], b_in_a), 1.0, q, dist)) dist = 1.0;
                    d.push_back(dist);
                    if (std::abs(std::atan2(b[i].y, b[i].x)) > 2.356) d_rear.push_back(dist);  // behind: |azimuth| > 135 deg
                }
                per_model[m].push_back(percentile(d, 0.5));
                rear[m].push_back(percentile(d_rear, 0.5));
            }
        }
        std::printf("gap %zu frame%s (%zu pairs):\n", gap, gap > 1 ? "s" : "", per_model[0].size());
        for (int m = 0; m < 3; ++m) {
            int wins = 0;
            for (std::size_t i = 0; i < per_model[m].size(); ++i) wins += per_model[m][i] <= per_model[0][i];
            std::printf("  %-26s median of pair medians %.3f m | behind the car %.3f m | at or below 'no deskew' in %d/%zu pairs\n",
                        names[m], percentile(per_model[m], 0.5), percentile(rear[m], 0.5), wins, per_model[m].size());
        }
    }
    return EXIT_SUCCESS;
}
// Diagnostic: ICP started AT the true motion (scans deskewed with the truth).
// If it walks away, the matching itself prefers a wrong answer; splitting the
// source points by height shows which points pull it.
int icpFromTruthCommand(char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::string dir = argv[4];
    const char* names[] = {"all points", "road only (z <= -1.2 m)", "raised only (z > -1.2 m)"};
    std::printf("Sequence %s, pairs (k, k+1), k = 0, 20, ...: ICP from the true motion; source thinned to 1 m,\n"
                "reference to 0.25 m, pairs within 1 m. Where it ends, relative to the truth (medians):\n", argv[3]);
    for (int set = 0; set < 3; ++set) {
        std::vector<double> forward, yaw;
        for (std::size_t k = 0; k + 2 < s.velodyne.size(); k += 20) {
            const Pose3D sweep = relativePose3D(s.velodyne[k], s.velodyne[k + 1]);
            const Pose3D next = relativePose3D(s.velodyne[k + 1], s.velodyne[k + 2]);
            const auto a = deskewScan(readKittiScan(kittiScanPath(dir, k)).points, sweep, SweepDirection::Clockwise);
            const auto b = deskewScan(readKittiScan(kittiScanPath(dir, k + 1)).points, next, SweepDirection::Clockwise);
            std::vector<Point3D> source;
            for (const Point3D& p : voxelDownsample(b, 1.0))
                if (set == 0 || (set == 1) == (p.z <= -1.2)) source.push_back(p);
            VoxelGrid3D grid(1.0);
            grid.insert(voxelDownsample(a, 0.25));
            const Pose3D off = relativePose3D(sweep, alignIcp3D(grid, source, sweep).source_in_reference);
            forward.push_back(off.t.x);
            yaw.push_back(std::atan2(off.r[3], off.r[0]) * 180 / std::acos(-1.0));
        }
        std::size_t ahead = 0;
        for (double f : forward) ahead += f > 0;
        std::printf("  %-26s forward %+.4f m (%zu of %zu pairs forward), heading %+.4f deg\n", names[set],
                    percentile(forward, 0.5), ahead, forward.size(), percentile(yaw, 0.5));
    }
    return EXIT_SUCCESS;
}
struct Score {
    KittiMetric metric;
    TrajectoryComparison comparison;
    double heading_deg = 0;
};

// Scores Velodyne-frame poses against the ground truth: the KITTI metric in the
// camera frame (as the devkit does), plus end error and final heading error.
Score scoreTrajectory(const Sequence& s, const Pose3D& tr, const std::vector<Pose3D>& estimate_velodyne) {
    Score out;
    const std::vector<Pose3D> truth(s.velodyne.begin(), s.velodyne.begin() + static_cast<std::ptrdiff_t>(estimate_velodyne.size()));
    const std::vector<Pose3D> truth_camera(s.camera.begin(), s.camera.begin() + static_cast<std::ptrdiff_t>(estimate_velodyne.size()));
    out.metric = kittiMetric(cameraPoses(estimate_velodyne, tr), truth_camera);
    out.comparison = compareTrajectories(estimate_velodyne, truth);
    const Pose3D last = relativePose3D(truth.back(), estimate_velodyne.back());
    out.heading_deg = std::atan2(last.r[3], last.r[0]) * 180 / std::acos(-1.0);
    return out;
}

void printScoreHeader() {
    std::printf("%-34s | %-14s | %-12s | %-17s | %-9s | %s\n", "method", "KITTI t_err", "r_err", "end error (drift)",
                "heading", "per-frame error med");
}
void printScore(const char* name, const Score& sc) {
    char end[32];
    std::snprintf(end, sizeof end, "%.2f m (%.2f%%)", sc.comparison.end_error, 100 * sc.comparison.end_error / sc.comparison.length);
    std::printf("%-34s | %6.2f %%       | %5.2f d/100m | %-17s | %+6.2f d  | %.3f m\n", name, sc.metric.translation_percent,
                sc.metric.rotation_deg_per_100m, end, sc.heading_deg, percentile(sc.comparison.step_translation_error, 0.5));
}

// Runs Atlas's odometry variants on one sequence, scores each against the
// ground truth (KITTI metric in the camera frame), and saves the poses
// (Velodyne frame) to results/atlas_<seq>_<variant>.txt for `score` and plots.
int odometryCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const Pose3D tr = readKittiVelodyneInCamera(std::string(argv[2]) + "/sequences/" + argv[3] + "/calib.txt");
    const std::string dir = argv[4];
    const double source_voxel = argc >= 6 ? std::stod(argv[5]) : 1.0;
    const std::size_t frames = std::min(s.velodyne.size(), argc >= 7 ? std::stoul(argv[6]) : s.velodyne.size());
    std::vector<std::vector<Point3D>> raw, corrected;
    for (std::size_t i = 0; i < frames; ++i) {
        raw.push_back(readKittiScan(kittiScanPath(dir, i)).points);
        corrected.push_back(correctKittiScan(raw.back()));
    }
    std::printf("Sequence %s, frames 0-%zu, raw scans. One thread, Apple M4; ms excludes file reading.\n\n", argv[3], frames - 1);
    printScoreHeader();
    struct Variant { const char* name; const char* file; int kind; };
    const Variant variants[] = {
        {"scan-to-scan, raw", "scan_raw", 0},
        {"scan-to-scan, deskewed", "scan_deskew", 1},
        {"scan-to-map (KISS design), corr.", "map", 2},
    };
    for (const Variant& v : variants) {
        std::vector<Pose3D> estimate;
        std::vector<double> ms;
        std::size_t not_converged = 0, source_points = 0, map_points = 0;
        ScanOdometryOptions so;
        so.source_voxel = source_voxel;
        so.deskew = v.kind == 1;
        ScanOdometry3D scan_odometry(so);
        MapOdometry3D map_odometry;
        for (std::size_t i = 0; i < frames; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            if (v.kind < 2) {
                const ScanOdometryStep st = scan_odometry.update(raw[i]);
                estimate.push_back(st.pose);
                if (i > 0) not_converged += st.status != Icp3DStatus::Converged;
                source_points += st.source_points;
            } else {
                const MapOdometryStep st = map_odometry.update(corrected[i]);
                estimate.push_back(st.pose);
                if (i > 0) not_converged += st.status != Icp3DStatus::Converged;
                source_points += st.source_points;
                map_points = st.map_points;
            }
            ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        }
        const Score sc = scoreTrajectory(s, tr, estimate);
        char name[64];
        std::snprintf(name, sizeof name, "%s, %.0f ms", v.name, percentile(ms, 0.5));
        printScore(name, sc);
        std::printf("    %zu of %zu frames not 'converged'; %.0f source points per scan%s\n", not_converged, frames - 1,
                    static_cast<double>(source_points) / frames,
                    map_points ? (", map " + std::to_string(map_points) + " points at the end").c_str() : "");
        writeKittiPoses("results/atlas_" + std::string(argv[3]) + "_" + v.file + ".txt", estimate);
    }
    std::printf("\nKITTI t_err / r_err: the devkit metric (segments of 100-800 m, every 10th frame), camera frame.\n"
                "Poses saved to results/atlas_%s_<variant>.txt (Velodyne frame).\n", argv[3]);
    return EXIT_SUCCESS;
}

int scoreCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const Pose3D tr = readKittiVelodyneInCamera(std::string(argv[2]) + "/sequences/" + argv[3] + "/calib.txt");
    const std::vector<Pose3D> estimate = readKittiPoses(argv[4]);
    if (estimate.size() > s.velodyne.size()) throw std::runtime_error("estimate has more poses than the ground truth");
    const Score sc = scoreTrajectory(s, tr, estimate);
    std::printf("Scored %zu frames (%zu KITTI segments of 100-800 m)\n", estimate.size(), sc.metric.segments);
    printScoreHeader();
    printScore(argc == 6 ? argv[5] : argv[4], sc);
    return EXIT_SUCCESS;
}
// Loop candidates in an estimated trajectory. The search sees only the
// estimate; the answer key is used afterwards to show how far apart the two
// places really are ("same place": within 10 m, a fixed bar independent of the
// search radius). Whether a candidate is a GOOD loop is decided later, by
// whether the scans can be aligned correctly (verification).
int loopsCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::vector<Pose3D> estimate = readKittiPoses(argv[4]);
    if (estimate.size() > s.velodyne.size()) throw std::runtime_error("estimate has more poses than the ground truth");
    KeyframeOptions ko;
    LoopSearchOptions lo;
    if (argc >= 6) ko.min_distance = std::stod(argv[5]);
    if (argc >= 7) lo.max_distance = std::stod(argv[6]);
    if (argc >= 8) lo.min_path_gap = std::stod(argv[7]);
    const std::vector<std::size_t> keys = selectKeyframes(estimate, ko);
    const std::vector<double> lengths = pathLengths(estimate);
    std::vector<Pose3D> key_poses;
    std::vector<double> key_lengths;
    for (std::size_t k : keys) {
        key_poses.push_back(estimate[k]);
        key_lengths.push_back(lengths[k]);
    }
    const std::vector<LoopCandidate> found = findLoopCandidates(key_poses, key_lengths, lo);
    std::printf("Sequence %s: %zu frames, %.0f m estimated path, %zu keyframes (every %.1f m or %.0f deg).\n"
                "Loop candidates: earlier keyframe within %.1f m in the ESTIMATE, at least %.0f m back along the path.\n\n",
                argv[3], estimate.size(), lengths.back(), keys.size(), ko.min_distance, ko.min_angle * 180 / std::acos(-1.0),
                lo.max_distance, lo.min_path_gap);
    std::printf("%-15s | %-10s | %-9s | %-26s\n", "frames", "estimated", "path gap", "true distance (answer key)");
    const double same_place = 10.0;
    std::size_t real = 0;
    for (const LoopCandidate& c : found) {
        const std::size_t a = keys[c.earlier], b = keys[c.later];
        const Point3D ta = s.velodyne[a].t, tb = s.velodyne[b].t;
        const double truth = std::sqrt((ta.x - tb.x) * (ta.x - tb.x) + (ta.y - tb.y) * (ta.y - tb.y) + (ta.z - tb.z) * (ta.z - tb.z));
        real += truth <= same_place;
        std::printf("%5zu -> %5zu  | %6.2f m   | %6.0f m  | %6.2f m%s\n", a, b, c.distance, c.path_gap, truth,
                    truth <= same_place ? "   same place" : "");
    }
    std::printf("\n%zu candidates; %zu are the same place (within %.0f m) by the answer key.\n", found.size(), real, same_place);
    return EXIT_SUCCESS;
}

// A scan as the odometry saw it: KITTI-corrected, then deskewed with the
// estimated motion into it (frame 0 uses the motion out of it).
std::vector<Point3D> preparedScan(const std::string& dir, const std::vector<Pose3D>& estimate, std::size_t i) {
    const std::vector<Point3D> raw = correctKittiScan(readKittiScan(kittiScanPath(dir, i)).points);
    const Pose3D motion = i > 0 ? relativePose3D(estimate[i - 1], estimate[i]) : relativePose3D(estimate[0], estimate[1]);
    return deskewScan(raw, motion, SweepDirection::Clockwise);
}

// Verifies loop candidates on real scans and grades the relative poses against
// the answer key. For each candidate it also verifies a decoy: the same later
// scan paired with a far-away place (300 frames off), with the same guess, as
// if the search had been fooled. Every decoy should be rejected.
int verifyLoopsCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::string dir = argv[4];
    const std::vector<Pose3D> estimate = readKittiPoses(argv[5]);
    LoopSearchOptions lo;
    if (argc >= 7) lo.max_distance = std::stod(argv[6]);
    const std::size_t half = argc >= 8 ? std::stoul(argv[7]) : 10;
    const std::vector<std::size_t> keys = selectKeyframes(estimate);
    const std::vector<double> lengths = pathLengths(estimate);
    std::vector<Pose3D> key_poses;
    std::vector<double> key_lengths;
    for (std::size_t k : keys) {
        key_poses.push_back(estimate[k]);
        key_lengths.push_back(lengths[k]);
    }
    const auto submapAround = [&](std::size_t a) {
        std::vector<std::vector<Point3D>> scans;
        std::vector<Pose3D> poses;
        for (std::size_t i = a > half ? a - half : 0; i <= std::min(estimate.size() - 1, a + half); ++i) {
            scans.push_back(preparedScan(dir, estimate, i));
            poses.push_back(estimate[i]);
        }
        return buildSubmap(scans, poses, estimate[a], 0.25);
    };
    const auto poseError = [](const Pose3D& a, const Pose3D& b) {
        const Pose3D d = relativePose3D(a, b);
        return std::make_pair(std::sqrt(d.t.x * d.t.x + d.t.y * d.t.y + d.t.z * d.t.z), rotationAngle(d) * 180 / std::acos(-1.0));
    };
    std::printf("Sequence %s: loop candidates within %.0f m (estimate), local maps of %zu scans each side.\n"
                "Errors are against the answer key's relative pose: the odometry guess, then the verified result.\n\n",
                argv[3], lo.max_distance, half);
    std::printf("%-12s | %-8s | %-6s | %-7s | %-18s | %-18s | %s\n", "frames", "verdict", "fit", "rms", "guess error",
                "verified error", "reason");
    std::size_t accepted = 0, decoys_accepted = 0, n = 0, bad_accepted = 0;
    // Accepted loops, for the pose graph: earlier later, then 12 numbers (later in earlier's frame), fit.
    const std::string loops_path = "results/loops_" + std::string(argv[3]) + ".txt";
    std::FILE* loops_file = std::fopen(loops_path.c_str(), "w");
    if (!loops_file) throw std::runtime_error("cannot write " + loops_path);
    for (const LoopCandidate& c : findLoopCandidates(key_poses, key_lengths, lo)) {
        const std::size_t later = keys[c.later];
        const std::vector<Point3D> scan = preparedScan(dir, estimate, later);
        for (int decoy = 0; decoy < 2; ++decoy) {
            const std::size_t earlier = decoy ? (keys[c.earlier] + 300) % later : keys[c.earlier];
            const Pose3D guess = relativePose3D(estimate[keys[c.earlier]], estimate[later]);  // decoys keep the same guess
            const auto t0 = std::chrono::steady_clock::now();
            const LoopVerification v = verifyLoop(submapAround(earlier), scan, guess);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            const Pose3D truth = relativePose3D(s.velodyne[earlier], s.velodyne[later]);
            const auto ge = poseError(truth, guess), ve = poseError(truth, v.relative);
            char frames[32];
            std::snprintf(frames, sizeof frames, "%zu->%zu%s", earlier, later, decoy ? "*" : "");
            const Pose3D d = relativePose3D(truth, v.relative);  // verified error, in the later scan's own axes
            char axes[64] = "";
            if (!decoy) std::snprintf(axes, sizeof axes, " [fwd %+.2f left %+.2f up %+.2f]", d.t.x, d.t.y, d.t.z);
            std::printf("%-12s | %-8s | %5.1f%% | %5.3f m | %5.2f m %6.2f deg | %5.2f m %6.2f deg%s | %s%s\n", frames,
                        v.accepted ? "accept" : "reject", 100 * v.inlier_fraction, v.rmse, ge.first, ge.second, ve.first,
                        ve.second, axes, v.accepted ? "" : v.reason, decoy ? "  (decoy)" : "");
            (void)ms;
            if (decoy) decoys_accepted += v.accepted;
            else {
                if (v.accepted) {
                    const Pose3D& r = v.relative;
                    std::fprintf(loops_file, "%zu %zu %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.4f\n", earlier,
                                 later, r.r[0], r.r[1], r.r[2], r.t.x, r.r[3], r.r[4], r.r[5], r.t.y, r.r[6], r.r[7], r.r[8], r.t.z,
                                 v.inlier_fraction);
                }
                ++n;
                accepted += v.accepted;
                bad_accepted += v.accepted && (ve.first > 0.3 || ve.second > 1.0);
            }
        }
    }
    std::fclose(loops_file);
    std::printf("\nAccepted loops saved to %s.\n", loops_path.c_str());
    std::printf("\n%zu of %zu candidates accepted (%zu of them more than 0.3 m or 1 deg from the truth); %zu of %zu decoys "
                "accepted.\n* = decoy: the later scan paired with a place 300 frames away, with the real candidate's guess.\n",
                accepted, n, bad_accepted, decoys_accepted, n);
    return EXIT_SUCCESS;
}

// Loop closure with verified loops (from verify-loops), scored against the
// answer key before and after. Position error is measured in the frame of
// scan 0, where estimate and answer key both start (no alignment).
int closeLoopsCommand(char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const Pose3D tr = readKittiVelodyneInCamera(std::string(argv[2]) + "/sequences/" + argv[3] + "/calib.txt");
    const std::vector<Pose3D> estimate = readKittiPoses(argv[4]);
    if (estimate.size() > s.velodyne.size()) throw std::runtime_error("estimate has more poses than the ground truth");
    std::vector<LoopEdge> loops;
    std::ifstream in(argv[5]);
    if (!in) throw std::runtime_error(std::string("cannot read ") + argv[5]);
    for (std::string line; std::getline(in, line);) {
        if (line.empty()) continue;
        std::istringstream row(line);
        LoopEdge l;
        double v[12], fit;
        row >> l.earlier_frame >> l.later_frame;
        for (double& x : v) row >> x;
        row >> fit;
        if (!row) throw std::runtime_error("bad loop line: " + line);
        l.relative.r = {v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10]};
        l.relative.t = {v[3], v[7], v[11]};
        loops.push_back(l);
    }
    const std::vector<std::size_t> keys = selectKeyframes(estimate);
    const auto t0 = std::chrono::steady_clock::now();
    const LoopClosureResult closed = closeLoops(estimate, keys, loops);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const auto positions = [&](const std::vector<Pose3D>& e) {
        double s2 = 0, worst = 0;
        std::size_t at = 0;
        for (std::size_t i = 0; i < e.size(); ++i) {
            const Point3D t = s.velodyne[i].t;
            const double d2 = (e[i].t.x - t.x) * (e[i].t.x - t.x) + (e[i].t.y - t.y) * (e[i].t.y - t.y) + (e[i].t.z - t.z) * (e[i].t.z - t.z);
            s2 += d2;
            if (d2 > worst * worst) { worst = std::sqrt(d2); at = i; }
        }
        return std::make_tuple(std::sqrt(s2 / static_cast<double>(e.size())), worst, at);
    };
    std::printf("Sequence %s: %zu keyframes, %zu verified loops; pose graph %zu iterations, cost %.1f -> %.1f, %.0f ms.\n\n",
                argv[3], keys.size(), loops.size(), closed.graph.iterations, closed.graph.initial_cost, closed.graph.final_cost, ms);
    std::printf("%-26s | %-12s | %-17s | %-10s | %-12s | %s\n", "", "position RMS", "worst (frame)", "end error", "KITTI t_err", "r_err");
    for (int k = 0; k < 2; ++k) {
        const std::vector<Pose3D>& e = k ? closed.poses : estimate;
        const auto [rms, worst, at] = positions(e);
        const Score sc = scoreTrajectory(s, tr, e);
        std::printf("%-26s | %8.3f m   | %6.2f m (%4zu)    | %6.2f m   | %8.2f %%   | %.2f d/100m\n",
                    k ? "after loop closure" : "odometry (before)", rms, worst, at, sc.comparison.end_error,
                    sc.metric.translation_percent, sc.metric.rotation_deg_per_100m);
    }
    const std::string out = "results/atlas_" + std::string(argv[3]) + "_slam.txt";
    writeKittiPoses(out, closed.poses);
    std::printf("\nPosition error: against the answer key in scan 0's frame (both start there), every frame.\nSaved %s.\n", out.c_str());
    return EXIT_SUCCESS;
}

// One configured scan-to-map run for the evaluation workers (eval/). The
// output is a single JSON line so a program can read it; poses go to a file.
int runCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const Pose3D tr = readKittiVelodyneInCamera(std::string(argv[2]) + "/sequences/" + argv[3] + "/calib.txt");
    MapOdometryOptions options;
    bool correct = true;
    std::size_t frames = s.velodyne.size();
    std::string diag_path;
    for (int i = 6; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto eq = arg.find('=');
        if (eq == std::string::npos) throw std::runtime_error("expected key=value, got " + arg);
        const std::string key = arg.substr(0, eq), value = arg.substr(eq + 1);
        const auto flag = [&] {
            if (value != "0" && value != "1") throw std::runtime_error(key + " must be 0 or 1");
            return value == "1";
        };
        if (key == "voxel") options.voxel_size = std::stod(value);
        else if (key == "max_range") options.max_range = std::stod(value);
        else if (key == "deskew") options.deskew = flag();
        else if (key == "correct") correct = flag();
        else if (key == "diag") diag_path = value;
        else if (key == "frames") frames = std::min(frames, static_cast<std::size_t>(std::stoul(value)));
        else throw std::runtime_error("unknown key " + key);
    }
    if (!(options.voxel_size > 0) || !(options.max_range > 0) || frames < 2)
        throw std::runtime_error("voxel and max_range must be positive; need at least 2 frames");
    MapOdometry3D odometry(options);
    std::vector<Pose3D> estimate;
    std::vector<MapOdometryStep> steps;
    std::vector<double> ms;
    std::size_t not_converged = 0, map_points = 0;
    for (std::size_t i = 0; i < frames; ++i) {
        std::vector<Point3D> points = readKittiScan(kittiScanPath(argv[4], i)).points;
        if (correct) points = correctKittiScan(points);
        const auto t0 = std::chrono::steady_clock::now();
        const MapOdometryStep st = odometry.update(points);
        ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        estimate.push_back(st.pose);
        steps.push_back(st);
        if (i > 0) not_converged += st.status != Icp3DStatus::Converged;
        map_points = std::max(map_points, st.map_points);
    }
    writeKittiPoses(argv[5], estimate);
    if (!diag_path.empty()) {
        // One row per frame: what the engine itself saw (no ground truth here).
        std::FILE* f = std::fopen(diag_path.c_str(), "w");
        if (!f) throw std::runtime_error("cannot write " + diag_path);
        std::fprintf(f, "frame,ms,status,iterations,pairs,source_points,map_points,sigma_m\n");
        for (std::size_t i = 0; i < steps.size(); ++i)
            std::fprintf(f, "%zu,%.2f,%s,%zu,%zu,%zu,%zu,%.4f\n", i, ms[i], icp3DStatusName(steps[i].status), steps[i].iterations,
                         steps[i].pairs, steps[i].source_points, steps[i].map_points, steps[i].sigma);
        if (std::fclose(f) != 0) throw std::runtime_error("cannot write " + diag_path);
    }
    const Score sc = scoreTrajectory(s, tr, estimate);
    std::printf("{\"sequence\":\"%s\",\"frames\":%zu,\"t_err_percent\":%.4f,\"r_err_deg_per_100m\":%.4f,"
                "\"segments\":%zu,\"end_error_m\":%.3f,\"path_length_m\":%.1f,\"step_error_median_m\":%.4f,"
                "\"ms_median\":%.1f,\"ms_p90\":%.1f,\"ms_max\":%.1f,\"not_converged\":%zu,\"map_points_max\":%zu}\n",
                argv[3], frames, sc.metric.translation_percent, sc.metric.rotation_deg_per_100m, sc.metric.segments,
                sc.comparison.end_error, sc.comparison.length, percentile(sc.comparison.step_translation_error, 0.5),
                percentile(ms, 0.5), percentile(ms, 0.9), percentile(ms, 1.0), not_converged, map_points);
    return EXIT_SUCCESS;
}

void putInt16(std::vector<unsigned char>& out, double value) {
    const long v = std::lround(value);
    if (v < INT16_MIN || v > INT16_MAX) throw std::runtime_error("value out of int16 range in export");
    const auto u = static_cast<std::uint16_t>(static_cast<std::int16_t>(v));
    out.push_back(static_cast<unsigned char>(u & 0xff));  // little-endian, whatever the machine
    out.push_back(static_cast<unsigned char>(u >> 8));
}

void writeBytes(const std::string& path, const std::vector<unsigned char>& bytes) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!f) throw std::runtime_error("cannot write " + path);
}

// Exports what the replay explorer draws. Formats (all little-endian int16 x, y, z triples):
//   map.bin   decimeters, in the first scan's sensor frame: one point per map_voxel cube, from
//             every scan (KITTI-corrected, within map_range) placed with ATLAS'S ESTIMATED pose,
//             so drift shows up in the map exactly as the estimator built it.
//   scans.bin centimeters, each scan in its own sensor frame (KITTI-corrected), thinned to exactly
//             scan_points points (every k-th point within map_range; short scans repeat the last
//             point). Frame i starts at byte i * scan_points * 6.
int exportReplayCommand(int argc, char** argv) {
    const std::string dir = argv[2];
    const std::vector<Pose3D> estimate = readKittiPoses(argv[3]);
    const std::string out = argv[4];
    std::size_t scan_points = 1000;
    double voxel = 0.4, range = 50;
    for (int i = 5; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto eq = arg.find('=');
        const std::string key = arg.substr(0, eq), value = eq == std::string::npos ? "" : arg.substr(eq + 1);
        if (key == "scan_points") scan_points = std::stoul(value);
        else if (key == "map_voxel") voxel = std::stod(value);
        else if (key == "map_range") range = std::stod(value);
        else throw std::runtime_error("unknown key " + arg);
    }
    if (scan_points == 0 || !(voxel > 0) || !(range > 0)) throw std::runtime_error("bad export parameters");
    std::unordered_set<std::uint64_t> filled;
    std::vector<unsigned char> map_bytes, scan_bytes;
    const auto cell = [&](double v) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v / voxel)) + (1 << 20)) & 0x1fffff; };
    for (std::size_t i = 0; i < estimate.size(); ++i) {
        const std::vector<Point3D> points = correctKittiScan(readKittiScan(kittiScanPath(dir, i)).points);
        std::vector<Point3D> near;
        near.reserve(points.size());
        for (const Point3D& p : points)
            if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) <= range) near.push_back(p);
        if (near.empty()) throw std::runtime_error("scan " + std::to_string(i) + " has no points in range");
        for (const Point3D& p : near) {
            const Point3D w = transformPoint3D(p, estimate[i]);
            if (filled.insert((cell(w.x) << 42) | (cell(w.y) << 21) | cell(w.z)).second) {
                putInt16(map_bytes, w.x * 10);
                putInt16(map_bytes, w.y * 10);
                putInt16(map_bytes, w.z * 10);
            }
        }
        const std::size_t step = std::max<std::size_t>(1, near.size() / scan_points);
        for (std::size_t k = 0; k < scan_points; ++k) {
            const Point3D& p = near[std::min(k * step, near.size() - 1)];
            putInt16(scan_bytes, p.x * 100);
            putInt16(scan_bytes, p.y * 100);
            putInt16(scan_bytes, p.z * 100);
        }
    }
    writeBytes(out + "/map.bin", map_bytes);
    writeBytes(out + "/scans.bin", scan_bytes);
    std::printf("{\"frames\":%zu,\"scan_points\":%zu,\"map_points\":%zu,\"map_voxel\":%g,\"map_range\":%g}\n",
                estimate.size(), scan_points, map_bytes.size() / 6, voxel, range);
    return EXIT_SUCCESS;
}

// Top-down SVG: the ground truth (gray) under each estimate, all in the
// first scan's Velodyne frame (the frame odometry estimates live in).
int plotCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const auto positions = [](const std::vector<Pose3D>& poses) {
        std::vector<Point2D> out;
        out.reserve(poses.size());
        for (const Pose3D& p : poses) out.push_back({p.t.x, p.t.y});
        return out;
    };
    const char* colors[] = {"#1f5fbf", "#d9480f", "#2b8a3e", "#862e9c"};
    std::vector<PlotSeries> series{{"ground truth (KITTI poses, converted to Velodyne)", "#a8a8a8", positions(s.velodyne)}};
    for (int i = 5; i + 1 < argc; i += 2) {
        const std::vector<Pose3D> estimate = readKittiPoses(argv[i]);
        if (estimate.size() > s.velodyne.size()) throw std::runtime_error("estimate has more poses than the ground truth");
        series.push_back({argv[i + 1], colors[(i - 5) / 2 % 4], positions(estimate)});
    }
    const std::string title = "KITTI sequence " + std::string(argv[3]) + ": estimated vs true path, seen from above";
    std::ofstream file(argv[4]);
    file << trajectorySvg(series, title);
    if (!file) throw std::runtime_error(std::string("cannot write ") + argv[4]);
    std::printf("Wrote %s (%zu paths)\n", argv[4], series.size());
    return EXIT_SUCCESS;
}
}  // namespace

int main(int argc, char** argv) {
    const std::string cmd = argc > 1 ? argv[1] : "";
    try {
        if (cmd == "scan" && (argc == 3 || argc == 4)) return scanCommand(argc, argv);
        if (cmd == "truth" && argc == 4) return truthCommand(argv);
        if (cmd == "overlap" && argc == 6) return overlapCommand(argv);
        if (cmd == "map" && (argc == 6 || argc == 7)) return mapCommand(argc, argv);
        if (cmd == "odometry" && argc >= 5 && argc <= 7) return odometryCommand(argc, argv);
        if (cmd == "deskew-check" && argc == 5) return deskewCheckCommand(argv);
        if (cmd == "icp-from-truth" && argc == 5) return icpFromTruthCommand(argv);
        if (cmd == "score" && (argc == 5 || argc == 6)) return scoreCommand(argc, argv);
        if (cmd == "plot" && argc >= 7 && argc % 2 == 1) return plotCommand(argc, argv);
        if (cmd == "run" && argc >= 6) return runCommand(argc, argv);
        if (cmd == "export-replay" && argc >= 5) return exportReplayCommand(argc, argv);
        if (cmd == "loops" && argc >= 5 && argc <= 8) return loopsCommand(argc, argv);
        if (cmd == "verify-loops" && argc >= 6 && argc <= 8) return verifyLoopsCommand(argc, argv);
        if (cmd == "close-loops" && argc == 6) return closeLoopsCommand(argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cerr << "usage:\n  atlas_kitti scan <scan.bin> [out.bmp]\n  atlas_kitti truth <odometry_dir> <seq>\n"
                 "  atlas_kitti overlap <odometry_dir> <seq> <velodyne_dir> <frame>\n"
                 "  atlas_kitti map <odometry_dir> <seq> <velodyne_dir> <out.bmp> [every_nth_scan]\n"
                 "  atlas_kitti odometry <odometry_dir> <seq> <velodyne_dir> [source_voxel_m] [frames]\n"
                 "  atlas_kitti deskew-check <odometry_dir> <seq> <velodyne_dir>\n"
                 "  atlas_kitti icp-from-truth <odometry_dir> <seq> <velodyne_dir>\n"
                 "  atlas_kitti score <odometry_dir> <seq> <estimate_velodyne.txt> [name]\n"
                 "  atlas_kitti plot <odometry_dir> <seq> <out.svg> <estimate.txt> <name> [<estimate.txt> <name> ...]\n"
                 "  atlas_kitti run <odometry_dir> <seq> <velodyne_dir> <out_poses.txt> [voxel=1 max_range=100 deskew=1 correct=1 frames=N diag=file]\n"
                 "  atlas_kitti loops <odometry_dir> <seq> <estimate.txt> [keyframe_m=5 radius_m=10 min_gap_m=50]\n"
                 "  atlas_kitti verify-loops <odometry_dir> <seq> <velodyne_dir> <estimate.txt> [radius_m=10 half_window=10]\n"
                 "  atlas_kitti close-loops <odometry_dir> <seq> <estimate.txt> <loops.txt>\n"
                 "  atlas_kitti export-replay <velodyne_dir> <estimate.txt> <out_dir> [scan_points=1000 map_voxel=0.4 map_range=50]\n";
    return EXIT_FAILURE;
}
