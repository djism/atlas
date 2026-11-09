// Scripted localization experiment: scan-to-scan tracker vs scan-to-map
// localizer over fixed trajectories, sensor ranges, noise levels, and seeds.
// Usage: atlas_experiment [--seeds N] [--csv path]
// Setup and printing only; the run loop is runExperiment() in src/experiment.cpp.
#include "atlas/experiment.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
struct Path {
    const char* name;
    atlas::Point2D center;
    double radius;
};

// A localizer run counts as failed when its worst error exceeds this.
constexpr double kFailure = 0.10;  // meters

[[noreturn]] void fileError(const std::string& message) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
}
}  // namespace

int main(int argc, char** argv) {
    using namespace atlas;
    int seeds = 3;
    std::string csv_path;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc) seeds = std::max(1, std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csv_path = argv[++i];
        else { std::cerr << "usage: atlas_experiment [--seeds N] [--csv path]\n"; return EXIT_FAILURE; }
    }

    // Scenario: 8 x 6 m room with an alcove and a 1 m box (world frame, meters).
    ExperimentConfig base;
    base.walls = {{{0,0},{8,0}}, {{8,0},{8,2}}, {{8,2},{9.5,2}}, {{9.5,2},{9.5,4}}, {{9.5,4},{8,4}},
                  {{8,4},{8,6}}, {{8,6},{0,6}}, {{0,6},{0,0}},
                  {{2,4},{3,4}}, {{3,4},{3,5}}, {{3,5},{2,5}}, {{2,5},{2,4}}};
    base.survey = {{1.5,1.5,0}, {4,1.5,0.5}, {6.5,1.5,1}, {8.7,3,2}, {6.5,4.5,2.5}, {4,3,3}, {1.2,4.8,-1}};
    const std::vector<Path> paths{{"A", {4.6, 2.6}, 1.8}, {"B", {4.5, 2.8}, 1.5}};
    const std::vector<double> ranges{10.0, 3.0};
    const std::vector<double> noises{0.01, 0.03};

    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path);
        if (!csv) fileError("cannot write " + csv_path);
        csv << "path,range_m,noise_m,seed,method,worst_m,mean_m,final_m,lost,weak,jumps,scans,driven_m\n";
    }

    std::printf("Atlas localization experiment: 2 laps per run, 10 cm per scan, 360 beams,\n"
                "map from 7 noise-free survey scans at 5 cm. Errors vs answer key; %d seed(s) per row.\n"
                "Failed = localizer worst error above %.0f cm.\n\n", seeds, kFailure * 100);
    std::printf("%-4s %6s %6s | %-24s | %-24s | %5s %5s %5s %6s\n", "path", "range", "noise",
                "tracker worst / drift", "localizer worst / mean", "lost", "weak", "jumps", "failed");
    for (const auto& path : paths)
        for (double range : ranges)
            for (double noise : noises) {
                double tracker_worst = 0, tracker_drift = 0, localizer_worst = 0, localizer_mean = 0;
                std::size_t lost = 0, weak = 0, jumps = 0;
                int failed = 0;
                for (int s = 0; s < seeds; ++s) {
                    ExperimentConfig config = base;
                    config.trajectory = circlePath(path.center, path.radius, 0.1, 2);
                    config.lidar = {360, range};
                    config.noise = noise;
                    config.seed = 1000 * (s + 1);
                    const ExperimentReport r = runExperiment(config);
                    tracker_worst = std::max(tracker_worst, r.tracker.worst_error);
                    tracker_drift = std::max(tracker_drift, r.tracker.final_error / r.driven);
                    localizer_worst = std::max(localizer_worst, r.localizer.worst_error);
                    localizer_mean += r.localizer.mean_error / seeds;
                    lost += r.localizer.lost;
                    weak += r.localizer_weak;
                    jumps += r.localizer_jumps;
                    failed += r.localizer.worst_error > kFailure;
                    if (csv) {
                        for (int m = 0; m < 2; ++m) {
                            const MethodReport& mr = m ? r.localizer : r.tracker;
                            csv << path.name << ',' << range << ',' << noise << ',' << config.seed << ','
                                << (m ? "localizer" : "tracker") << ',' << mr.worst_error << ','
                                << mr.mean_error << ',' << mr.final_error << ',' << mr.lost << ','
                                << (m ? r.localizer_weak : 0) << ',' << (m ? r.localizer_jumps : 0) << ','
                                << r.scans << ',' << r.driven << '\n';
                        }
                    }
                }
                char tracker_text[32], localizer_text[32];
                std::snprintf(tracker_text, sizeof tracker_text, "%7.3f m / %5.2f%%", tracker_worst, 100 * tracker_drift);
                std::snprintf(localizer_text, sizeof localizer_text, "%7.3f m / %7.4f m", localizer_worst, localizer_mean);
                std::printf("%-4s %4.0f m %3.0f cm | %-24s | %-24s | %5zu %5zu %5zu %3d/%d\n", path.name, range,
                            noise * 100, tracker_text, localizer_text, lost, weak, jumps, failed, seeds);
            }
    std::printf("\nTracker drift = final error / distance driven (worst seed). Lost, weak, jumps:\n"
                "localizer scans summed over seeds (lost: failed or rejected; weak: position along\n"
                "one direction taken from the prediction; jumps: answer too far from the guess).\n");
    if (csv) std::printf("Per-run rows written to %s\n", csv_path.c_str());
    return EXIT_SUCCESS;
}
