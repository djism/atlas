#include "atlas/kitti.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace atlas {
namespace {
bool littleEndian() {
    const std::uint16_t probe = 1;
    unsigned char first = 0;
    std::memcpy(&first, &probe, 1);
    return first == 1;
}

double medianOf(std::vector<double> values) {
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}
}  // namespace

KittiScan readKittiScan(const std::string& path) {
    static_assert(sizeof(float) == 4, "KITTI scans store 32-bit floats");
    if (!littleEndian()) throw std::runtime_error("KITTI scans are little-endian; this machine is not");
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot open KITTI scan: " + path);
    const std::vector<char> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (file.bad()) throw std::runtime_error("cannot read KITTI scan: " + path);
    constexpr std::size_t record = 4 * sizeof(float);
    if (bytes.size() % record != 0)
        throw std::runtime_error("KITTI scan size " + std::to_string(bytes.size()) +
                                 " bytes is not a multiple of 16 (truncated?): " + path);
    const std::size_t count = bytes.size() / record;
    KittiScan scan;
    scan.points.reserve(count);
    scan.reflectance.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        float v[4];
        std::memcpy(v, bytes.data() + i * record, record);  // memcpy: no alignment assumptions
        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]) || !std::isfinite(v[3]))
            throw std::runtime_error("KITTI scan point " + std::to_string(i) + " is not finite: " + path);
        scan.points.push_back({v[0], v[1], v[2]});
        scan.reflectance.push_back(v[3]);
    }
    return scan;
}

namespace {
// 12 numbers -> [R | t]; false if the text has a different count or a non-finite value.
bool parsePose(const std::string& text, Pose3D& pose) {
    std::istringstream in(text);
    double v[12];
    for (double& x : v)
        if (!(in >> x) || !std::isfinite(x)) return false;
    std::string extra;
    if (in >> extra) return false;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) pose.r[3 * row + col] = v[4 * row + col];
    }
    pose.t = {v[3], v[7], v[11]};
    return true;
}
}  // namespace

std::vector<Pose3D> readKittiPoses(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open KITTI poses: " + path);
    std::vector<Pose3D> poses;
    std::string line;
    for (std::size_t n = 1; std::getline(file, line); ++n) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;  // blank line
        Pose3D pose;
        if (!parsePose(line, pose))
            throw std::runtime_error("KITTI poses line " + std::to_string(n) + " is not 12 finite numbers: " + path);
        if (orthonormalityError(pose) > 1e-4)
            throw std::runtime_error("KITTI poses line " + std::to_string(n) + " is not a rotation: " + path);
        poses.push_back(pose);
    }
    return poses;
}

Pose3D readKittiVelodyneInCamera(const std::string& calib_path) {
    std::ifstream file(calib_path);
    if (!file) throw std::runtime_error("cannot open KITTI calibration: " + calib_path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.rfind("Tr:", 0) != 0) continue;
        Pose3D tr;
        if (!parsePose(line.substr(3), tr) || orthonormalityError(tr) > 1e-4)
            throw std::runtime_error("KITTI calibration Tr is not a valid rigid transform: " + calib_path);
        return tr;
    }
    throw std::runtime_error("KITTI calibration has no Tr line: " + calib_path);
}

std::vector<double> readKittiTimes(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open KITTI times: " + path);
    std::vector<double> times;
    std::string token;
    while (file >> token) {
        std::size_t used = 0;
        double t = 0;
        try { t = std::stod(token, &used); } catch (const std::exception&) { used = 0; }
        if (used != token.size() || !std::isfinite(t)) throw std::runtime_error("KITTI times: not a number '" + token + "': " + path);
        times.push_back(t);
    }
    return times;
}

std::vector<Pose3D> velodynePoses(const std::vector<Pose3D>& camera_poses, const Pose3D& velodyne_in_camera) {
    const Pose3D camera_in_velodyne = inversePose3D(velodyne_in_camera);
    std::vector<Pose3D> out;
    out.reserve(camera_poses.size());
    for (const Pose3D& c : camera_poses)
        out.push_back(composePoses3D(composePoses3D(camera_in_velodyne, c), velodyne_in_camera));
    return out;
}

double sweepFraction(const Point3D& p, SweepDirection direction) {
    const double pi = std::acos(-1.0);
    const double azimuth = std::atan2(p.y, p.x);  // (-pi, pi], 0 = forward, +pi/2 = left
    // Clockwise: starts at +pi (behind, approached from the left), forward at 0.5, ends at -pi.
    return direction == SweepDirection::Clockwise ? 0.5 * (1 - azimuth / pi) : 0.5 * (1 + azimuth / pi);
}

std::vector<Point3D> deskewScan(const std::vector<Point3D>& points, const Pose3D& sweep_motion,
                                SweepDirection direction) {
    std::vector<Point3D> out;
    out.reserve(points.size());
    for (const Point3D& p : points)
        out.push_back(transformPoint3D(p, scaleMotion(sweep_motion, sweepFraction(p, direction) - 0.5)));
    return out;
}

std::string kittiScanPath(const std::string& dir, std::size_t index) {
    char name[32];
    std::snprintf(name, sizeof name, "%010zu.bin", index);
    return dir.empty() || dir.back() == '/' ? dir + name : dir + "/" + name;
}

ScanSummary summarizeScan(const KittiScan& scan) {
    if (scan.points.empty()) throw std::invalid_argument("cannot summarize an empty scan");
    ScanSummary s;
    s.points = scan.points.size();
    std::vector<double> ranges, heights;
    ranges.reserve(s.points);
    heights.reserve(s.points);
    std::size_t forward = 0;
    for (const Point3D& p : scan.points) {
        ranges.push_back(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
        heights.push_back(p.z);
        forward += p.x > 0;
    }
    const auto [lo, hi] = std::minmax_element(ranges.begin(), ranges.end());
    s.min_range = *lo;
    s.max_range = *hi;
    s.median_range = medianOf(std::move(ranges));
    s.median_z = medianOf(std::move(heights));
    s.forward_share = static_cast<double>(forward) / s.points;
    s.max_reflectance = *std::max_element(scan.reflectance.begin(), scan.reflectance.end());
    return s;
}
}  // namespace atlas
