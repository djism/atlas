// KITTI checks. Setup and printing only; the work is in src/kitti.cpp,
// src/geometry3d.cpp, src/matching3d.cpp and src/image.cpp.
//   atlas_kitti scan    <scan.bin> [out.bmp]
//   atlas_kitti truth   <odometry_dir> <seq>
//   atlas_kitti overlap <odometry_dir> <seq> <velodyne_dir> <frame>
//   atlas_kitti map     <odometry_dir> <seq> <velodyne_dir> <out.bmp> [every_nth_scan]
//   atlas_kitti odometry <odometry_dir> <seq> <velodyne_dir> [source_voxel_m] [frames]
//   atlas_kitti deskew-check <odometry_dir> <seq> <velodyne_dir>
//   atlas_kitti icp-from-truth <odometry_dir> <seq> <velodyne_dir>
// odometry_dir is .../odometry/dataset (poses/, sequences/); velodyne_dir holds 0000000000.bin ...
#include "atlas/image.hpp"
#include "atlas/kitti.hpp"
#include "atlas/matching3d.hpp"
#include "atlas/odometry3d.hpp"
#include "atlas/voxel.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
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
int odometryCommand(int argc, char** argv) {
    const Sequence s = loadSequence(argv[2], argv[3]);
    const std::string dir = argv[4];
    const double source_voxel = argc >= 6 ? std::stod(argv[5]) : 1.0;
    const std::size_t frames = std::min(s.velodyne.size(), argc >= 7 ? std::stoul(argv[6]) : s.velodyne.size());
    std::vector<KittiScan> scans;
    for (std::size_t i = 0; i < frames; ++i) scans.push_back(readKittiScan(kittiScanPath(dir, i)));
    const std::vector<Pose3D> truth(s.velodyne.begin(), s.velodyne.begin() + static_cast<std::ptrdiff_t>(frames));
    std::printf("Sequence %s, frames 0-%zu, raw scans; scored against the ground truth. Source thinned to one point\n"
                "per %.2f m cube; previous scan to 0.25 m; pairs within 1 m. One thread; ms excludes file reading.\n\n",
                argv[3], frames - 1, source_voxel);
    std::printf("%-30s | %-24s | %-13s | %-19s | %-13s | %s\n", "method", "per-frame error med/90%", "forward bias",
                "end error / drift", "heading drift", "ms/scan");
    // Rows: raw scans; deskewed with the estimator's own prediction; and an ORACLE
    // row deskewed beforehand with the true motion (uses the answer key: an upper
    // bound for what deskewing can give, not a result).
    for (int mode = 0; mode < 3; ++mode) {
        ScanOdometryOptions o;
        o.source_voxel = source_voxel;
        o.deskew = mode == 1;
        ScanOdometry3D odometry(o);
        std::vector<Pose3D> estimate;
        std::vector<double> ms;
        std::size_t not_converged = 0;
        for (std::size_t i = 0; i < scans.size(); ++i) {
            std::vector<Point3D> points = scans[i].points;
            if (mode == 2 && i + 1 < truth.size())  // oracle: true motion over this sweep
                points = deskewScan(points, relativePose3D(truth[i], truth[i + 1]), SweepDirection::Clockwise);
            const auto t0 = std::chrono::steady_clock::now();
            const ScanOdometryStep step = odometry.update(points);
            ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
            estimate.push_back(step.pose);
            if (i > 0) not_converged += step.status != Icp3DStatus::Converged;
        }
        const TrajectoryComparison c = compareTrajectories(estimate, truth);
        const Pose3D last = relativePose3D(truth.back(), estimate.back());
        char err[32], end[32];
        std::snprintf(err, sizeof err, "%.3f / %.3f m", percentile(c.step_translation_error, 0.5),
                      percentile(c.step_translation_error, 0.9));
        std::snprintf(end, sizeof end, "%.2f m / %.2f%%", c.end_error, 100 * c.end_error / c.length);
        const char* names[] = {"raw scans", "deskewed (own prediction)", "ORACLE: deskewed with truth"};
        std::printf("%-30s | %-24s | %+7.4f m/frm | %-19s | %+8.2f deg  | %.1f\n", names[mode], err,
                    percentile(c.step_forward_error, 0.5), end,
                    std::atan2(last.r[3], last.r[0]) * 180 / std::acos(-1.0), percentile(ms, 0.5));
        if (not_converged) std::printf("  (%zu of %zu frames not 'converged')\n", not_converged, frames - 1);
    }
    std::printf("\nForward bias: median of (estimated - true) motion along the true direction, per frame.\n"
                "Heading drift: yaw of the final estimated orientation relative to the true one.\n");
    return EXIT_SUCCESS;
}

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
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cerr << "usage:\n  atlas_kitti scan <scan.bin> [out.bmp]\n  atlas_kitti truth <odometry_dir> <seq>\n"
                 "  atlas_kitti overlap <odometry_dir> <seq> <velodyne_dir> <frame>\n"
                 "  atlas_kitti map <odometry_dir> <seq> <velodyne_dir> <out.bmp> [every_nth_scan]\n"
                 "  atlas_kitti odometry <odometry_dir> <seq> <velodyne_dir> [source_voxel_m] [frames]\n"
                 "  atlas_kitti deskew-check <odometry_dir> <seq> <velodyne_dir>\n"
                 "  atlas_kitti icp-from-truth <odometry_dir> <seq> <velodyne_dir>\n";
    return EXIT_FAILURE;
}
