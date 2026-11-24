#include "atlas/voxel.hpp"
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace atlas {
namespace {
void checkSize(double s) {
    if (!(s > 0) || !std::isfinite(s)) throw std::invalid_argument("voxel size must be positive and finite");
}
// 21 bits per axis (about +/- 1 million cubes): +/- 100 km at 0.1 m cubes.
std::int64_t pack(std::int64_t ix, std::int64_t iy, std::int64_t iz) {
    constexpr std::int64_t off = 1 << 20, mask = (1 << 21) - 1;
    return ((ix + off) & mask) << 42 | ((iy + off) & mask) << 21 | ((iz + off) & mask);
}
}  // namespace

std::vector<Point3D> voxelDownsample(const std::vector<Point3D>& points, double voxel_size) {
    checkSize(voxel_size);
    std::unordered_set<std::int64_t> seen;
    seen.reserve(points.size());
    std::vector<Point3D> out;
    for (const Point3D& p : points) {
        const auto c = [&](double v) { return static_cast<std::int64_t>(std::floor(v / voxel_size)); };
        if (seen.insert(pack(c(p.x), c(p.y), c(p.z))).second) out.push_back(p);
    }
    return out;
}

VoxelGrid3D::VoxelGrid3D(double voxel_size) : voxel_(voxel_size) { checkSize(voxel_size); }

std::int64_t VoxelGrid3D::cell(double v) const { return static_cast<std::int64_t>(std::floor(v / voxel_)); }
std::int64_t VoxelGrid3D::key(std::int64_t ix, std::int64_t iy, std::int64_t iz) const { return pack(ix, iy, iz); }

void VoxelGrid3D::insert(const std::vector<Point3D>& points) {
    for (const Point3D& p : points) {
        cells_[key(cell(p.x), cell(p.y), cell(p.z))].push_back(points_.size());
        points_.push_back(p);
    }
}

bool VoxelGrid3D::nearestIndex(const Point3D& q, double max_distance, std::size_t& index, double& distance) const {
    if (!(max_distance > 0) || max_distance > voxel_)
        throw std::invalid_argument("VoxelGrid3D::nearest: max_distance must be in (0, voxel size]");
    const std::int64_t cx = cell(q.x), cy = cell(q.y), cz = cell(q.z);
    double best = max_distance * max_distance;
    bool any = false;
    for (std::int64_t dx = -1; dx <= 1; ++dx)
        for (std::int64_t dy = -1; dy <= 1; ++dy)
            for (std::int64_t dz = -1; dz <= 1; ++dz) {
                const auto it = cells_.find(key(cx + dx, cy + dy, cz + dz));
                if (it == cells_.end()) continue;
                for (std::size_t i : it->second) {
                    const Point3D& p = points_[i];
                    const double ex = p.x - q.x, ey = p.y - q.y, ez = p.z - q.z;
                    const double d2 = ex * ex + ey * ey + ez * ez;
                    if (d2 <= best) { best = d2; index = i; any = true; }
                }
            }
    if (any) distance = std::sqrt(best);
    return any;
}

bool VoxelGrid3D::nearest(const Point3D& q, double max_distance, Point3D& found, double& distance) const {
    std::size_t i = 0;
    if (!nearestIndex(q, max_distance, i, distance)) return false;
    found = points_[i];
    return true;
}

void VoxelGrid3D::neighbors(const Point3D& q, double radius, std::vector<std::size_t>& out) const {
    if (!(radius > 0) || radius > voxel_)
        throw std::invalid_argument("VoxelGrid3D::neighbors: radius must be in (0, voxel size]");
    out.clear();
    const std::int64_t cx = cell(q.x), cy = cell(q.y), cz = cell(q.z);
    const double r2 = radius * radius;
    for (std::int64_t dx = -1; dx <= 1; ++dx)
        for (std::int64_t dy = -1; dy <= 1; ++dy)
            for (std::int64_t dz = -1; dz <= 1; ++dz) {
                const auto it = cells_.find(key(cx + dx, cy + dy, cz + dz));
                if (it == cells_.end()) continue;
                for (std::size_t i : it->second) {
                    const Point3D& p = points_[i];
                    const double ex = p.x - q.x, ey = p.y - q.y, ez = p.z - q.z;
                    if (ex * ex + ey * ey + ez * ez <= r2) out.push_back(i);
                }
            }
}
LocalMap3D::LocalMap3D(double voxel_size, std::size_t max_points_per_voxel)
    : voxel_(voxel_size), max_points_(max_points_per_voxel) {
    checkSize(voxel_size);
    if (max_points_per_voxel == 0) throw std::invalid_argument("LocalMap3D: max_points_per_voxel must be positive");
}

void LocalMap3D::insert(const std::vector<Point3D>& points) {
    for (const Point3D& p : points) {
        const auto c = [&](double v) { return static_cast<std::int64_t>(std::floor(v / voxel_)); };
        const std::int64_t ix = c(p.x), iy = c(p.y), iz = c(p.z);
        Cell& cell = cells_[pack(ix, iy, iz)];
        if (cell.points.empty()) { cell.ix = ix; cell.iy = iy; cell.iz = iz; }
        if (cell.points.size() < max_points_) cell.points.push_back(p);
    }
}

void LocalMap3D::removeFarFrom(const Point3D& position, double max_distance) {
    const double limit = max_distance * max_distance;
    for (auto it = cells_.begin(); it != cells_.end();) {
        const Point3D& p = it->second.points.front();  // a cube's first point stands for the cube
        const double dx = p.x - position.x, dy = p.y - position.y, dz = p.z - position.z;
        if (dx * dx + dy * dy + dz * dz > limit) it = cells_.erase(it);
        else ++it;
    }
}

bool LocalMap3D::nearest(const Point3D& q, double max_distance, Point3D& found, double& distance) const {
    const auto c = [&](double v) { return static_cast<std::int64_t>(std::floor(v / voxel_)); };
    const std::int64_t cx = c(q.x), cy = c(q.y), cz = c(q.z);
    double best = max_distance * max_distance;
    bool any = false;
    for (std::int64_t dx = -1; dx <= 1; ++dx)
        for (std::int64_t dy = -1; dy <= 1; ++dy)
            for (std::int64_t dz = -1; dz <= 1; ++dz) {
                const auto it = cells_.find(pack(cx + dx, cy + dy, cz + dz));
                if (it == cells_.end()) continue;
                for (const Point3D& p : it->second.points) {
                    const double ex = p.x - q.x, ey = p.y - q.y, ez = p.z - q.z;
                    const double d2 = ex * ex + ey * ey + ez * ez;
                    if (d2 <= best) { best = d2; found = p; any = true; }
                }
            }
    if (any) distance = std::sqrt(best);
    return any;
}

std::size_t LocalMap3D::size() const {
    std::size_t n = 0;
    for (const auto& kv : cells_) n += kv.second.points.size();
    return n;
}

std::vector<Point3D> LocalMap3D::points() const {
    std::vector<Point3D> out;
    for (const auto& kv : cells_) out.insert(out.end(), kv.second.points.begin(), kv.second.points.end());
    return out;
}
}  // namespace atlas
