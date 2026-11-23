#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace atlas {
// Keeps the first point that falls into each cube of side `voxel_size`
// (meters), in input order. Thins dense nearby returns far more than sparse
// distant ones. Throws std::invalid_argument for a nonpositive or nonfinite size.
std::vector<Point3D> voxelDownsample(const std::vector<Point3D>& points, double voxel_size);

// Points filed by cube (hash map from integer cube coordinates). A query looks
// in its own cube and the 26 around it, so with voxel_size >= max_distance it
// returns EXACTLY the nearest point within max_distance (same as brute force),
// at a cost that depends on local density, not on the total point count.
class VoxelGrid3D {
public:
    // Throws std::invalid_argument for a nonpositive or nonfinite voxel size.
    explicit VoxelGrid3D(double voxel_size);
    void insert(const std::vector<Point3D>& points);
    std::size_t size() const { return count_; }
    double voxelSize() const { return voxel_; }

    // Nearest stored point within max_distance of q. Returns false if none.
    // Throws std::invalid_argument if max_distance > voxel size (the 27-cube
    // search would then miss points) or is not positive.
    bool nearest(const Point3D& q, double max_distance, Point3D& found, double& distance) const;

private:
    std::int64_t key(std::int64_t ix, std::int64_t iy, std::int64_t iz) const;
    std::int64_t cell(double v) const;
    double voxel_;
    std::size_t count_ = 0;
    std::unordered_map<std::int64_t, std::vector<Point3D>> cells_;
};
}  // namespace atlas
