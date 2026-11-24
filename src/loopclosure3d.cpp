#include "atlas/loopclosure3d.hpp"
#include "atlas/matching3d.hpp"
#include "atlas/voxel.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace atlas {
namespace {
double distance3D(const Point3D& a, const Point3D& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
}  // namespace

std::vector<double> pathLengths(const std::vector<Pose3D>& poses) {
    std::vector<double> out;
    out.reserve(poses.size());
    double s = 0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        if (i > 0) s += distance3D(poses[i - 1].t, poses[i].t);
        out.push_back(s);
    }
    return out;
}

std::vector<std::size_t> selectKeyframes(const std::vector<Pose3D>& poses, const KeyframeOptions& options) {
    if (!(options.min_distance > 0) || !(options.min_angle > 0))
        throw std::invalid_argument("selectKeyframes: thresholds must be positive");
    std::vector<std::size_t> keys;
    if (poses.empty()) return keys;
    keys.push_back(0);
    double travelled = 0;
    for (std::size_t i = 1; i < poses.size(); ++i) {
        travelled += distance3D(poses[i - 1].t, poses[i].t);
        const double turned = rotationAngle(relativePose3D(poses[keys.back()], poses[i]));
        if (travelled >= options.min_distance || turned >= options.min_angle) {
            keys.push_back(i);
            travelled = 0;
        }
    }
    return keys;
}

std::vector<LoopCandidate> findLoopCandidates(const std::vector<Pose3D>& keyframes, const std::vector<double>& path_lengths,
                                              const LoopSearchOptions& options) {
    if (keyframes.size() != path_lengths.size())
        throw std::invalid_argument("findLoopCandidates: one path length per keyframe");
    if (!(options.max_distance > 0) || !(options.min_path_gap > 0))
        throw std::invalid_argument("findLoopCandidates: thresholds must be positive");
    std::vector<LoopCandidate> out;
    for (std::size_t j = 0; j < keyframes.size(); ++j) {
        LoopCandidate best;
        bool found = false;
        for (std::size_t i = 0; i < j; ++i) {
            const double gap = path_lengths[j] - path_lengths[i];
            if (gap < options.min_path_gap) break;  // path lengths only grow: later i are closer along the path
            const double d = distance3D(keyframes[i].t, keyframes[j].t);
            if (d <= options.max_distance && (!found || d < best.distance)) {
                best = {i, j, d, gap};
                found = true;
            }
        }
        if (found) out.push_back(best);
    }
    return out;
}

std::vector<Point3D> buildSubmap(const std::vector<std::vector<Point3D>>& scans, const std::vector<Pose3D>& poses,
                                 const Pose3D& center, double voxel) {
    if (scans.size() != poses.size()) throw std::invalid_argument("buildSubmap: one pose per scan");
    if (!(voxel > 0)) throw std::invalid_argument("buildSubmap: voxel must be positive");
    std::vector<Point3D> all;
    for (std::size_t i = 0; i < scans.size(); ++i) {
        const std::vector<Point3D> placed = transformPoints(scans[i], relativePose3D(center, poses[i]));
        all.insert(all.end(), placed.begin(), placed.end());
    }
    return voxelDownsample(all, voxel);
}

LoopVerification verifyLoop(const std::vector<Point3D>& submap, const std::vector<Point3D>& scan, const Pose3D& guess,
                            const LoopVerifyOptions& options) {
    LoopVerification out;
    const std::vector<Point3D> source = voxelDownsample(scan, options.source_voxel);
    if (source.size() < options.min_points || submap.size() < options.min_points) {
        out.reason = "too few points";
        return out;
    }
    VoxelGrid3D coarse(options.coarse_gate), fine(std::max(options.fine_gate, options.inlier_distance));
    coarse.insert(submap);
    fine.insert(submap);
    Icp3DOptions c;
    c.max_correspondence_distance = options.coarse_gate;
    const Icp3DResult first = alignIcp3D(coarse, source, guess, c);
    Icp3DOptions f;
    f.max_correspondence_distance = options.fine_gate;
    const Icp3DResult second = alignIcp3D(fine, source, first.source_in_reference, f);
    out.status = second.status;
    out.relative = second.source_in_reference;
    const Pose3D moved = relativePose3D(guess, out.relative);
    out.correction = std::sqrt(moved.t.x * moved.t.x + moved.t.y * moved.t.y + moved.t.z * moved.t.z);
    // Fit: how much of the scan's structure (not its road surface) lands on the map.
    std::size_t inliers = 0;
    double sum2 = 0;
    for (const Point3D& raw : source) {
        if (raw.z <= options.structure_min_z) continue;
        ++out.structure_points;
        Point3D q;
        double d;
        if (fine.nearest(transformPoint3D(raw, out.relative), options.inlier_distance, q, d)) {
            ++inliers;
            sum2 += d * d;
        }
    }
    if (out.structure_points < options.min_points / 4) {
        out.reason = "too little structure to judge";
        out.status = second.status;
        return out;
    }
    out.inlier_fraction = static_cast<double>(inliers) / static_cast<double>(out.structure_points);
    out.rmse = inliers ? std::sqrt(sum2 / static_cast<double>(inliers)) : 0;
    if (second.status != Icp3DStatus::Converged) out.reason = "ICP did not converge";
    else if (out.inlier_fraction < options.min_inlier_fraction) out.reason = "too little of the scan fits the map";
    else if (out.rmse > options.max_rmse) out.reason = "fit too loose";
    else if (out.correction > options.max_correction) out.reason = "moved too far from the guess";
    else out.accepted = true;
    return out;
}

LoopClosureResult closeLoops(const std::vector<Pose3D>& estimate, const std::vector<std::size_t>& keyframes,
                             const std::vector<LoopEdge>& loops, const LoopClosureOptions& o) {
    if (keyframes.empty() || keyframes[0] != 0) throw std::invalid_argument("closeLoops: keyframes must start at frame 0");
    for (std::size_t k = 1; k < keyframes.size(); ++k)
        if (keyframes[k] <= keyframes[k - 1] || keyframes[k] >= estimate.size())
            throw std::invalid_argument("closeLoops: keyframes must increase and lie in the trajectory");
    for (double v : {o.odometry_sigma_per_m, o.odometry_sigma_floor, o.odometry_rot_sigma_per_m, o.odometry_rot_sigma_floor,
                     o.loop_sigma, o.loop_rot_sigma})
        if (!(v > 0)) throw std::invalid_argument("closeLoops: sigmas must be positive");
    const auto keyIndex = [&](std::size_t frame) {
        const auto it = std::lower_bound(keyframes.begin(), keyframes.end(), frame);
        if (it == keyframes.end() || *it != frame) throw std::invalid_argument("closeLoops: a loop frame is not a keyframe");
        return static_cast<std::size_t>(it - keyframes.begin());
    };
    std::vector<Pose3D> nodes;
    for (std::size_t f : keyframes) nodes.push_back(estimate[f]);
    std::vector<PoseGraphEdge3D> edges;
    for (std::size_t k = 1; k < nodes.size(); ++k) {
        const Pose3D step = relativePose3D(nodes[k - 1], nodes[k]);
        const double d = std::sqrt(step.t.x * step.t.x + step.t.y * step.t.y + step.t.z * step.t.z);
        const double st = std::max(o.odometry_sigma_floor, o.odometry_sigma_per_m * d);
        const double sr = std::max(o.odometry_rot_sigma_floor, o.odometry_rot_sigma_per_m * d);
        edges.push_back({k - 1, k, step, 1 / (st * st), 1 / (sr * sr)});
    }
    for (const LoopEdge& l : loops)
        edges.push_back({keyIndex(l.earlier_frame), keyIndex(l.later_frame), l.relative, 1 / (o.loop_sigma * o.loop_sigma),
                         1 / (o.loop_rot_sigma * o.loop_rot_sigma)});
    LoopClosureResult out;
    out.loops = loops.size();
    if (loops.empty()) {  // nothing to correct: odometry edges alone agree with the estimate exactly
        out.poses = estimate;
        out.graph.poses = nodes;
        out.graph.converged = true;
        return out;
    }
    out.graph = optimizePoseGraph3D(nodes, edges, o.graph);
    out.poses.reserve(estimate.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < estimate.size(); ++i) {
        while (k + 1 < keyframes.size() && keyframes[k + 1] <= i) ++k;
        out.poses.push_back(orthonormalized(composePoses3D(out.graph.poses[k], relativePose3D(nodes[k], estimate[i]))));
    }
    return out;
}
}  // namespace atlas
