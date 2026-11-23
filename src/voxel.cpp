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
    for (const Point3D& p : points) cells_[key(cell(p.x), cell(p.y), cell(p.z))].push_back(p);
    count_ += points.size();
}

bool VoxelGrid3D::nearest(const Point3D& q, double max_distance, Point3D& found, double& distance) const {
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
                for (const Point3D& p : it->second) {
                    const double ex = p.x - q.x, ey = p.y - q.y, ez = p.z - q.z;
                    const double d2 = ex * ex + ey * ey + ez * ez;
                    if (d2 <= best) { best = d2; found = p; any = true; }
                }
            }
    if (any) distance = std::sqrt(best);
    return any;
}
}  // namespace atlas
