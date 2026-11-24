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
    // Appends points; their indices continue from size().
    void insert(const std::vector<Point3D>& points);
    std::size_t size() const { return points_.size(); }
    double voxelSize() const { return voxel_; }
    const std::vector<Point3D>& points() const { return points_; }

    // Nearest stored point within max_distance of q. Returns false if none.
    // Throws std::invalid_argument if max_distance > voxel size (the 27-cube
    // search would then miss points) or is not positive.
    bool nearest(const Point3D& q, double max_distance, Point3D& found, double& distance) const;
    // Same, giving the point's index (into points()).
    bool nearestIndex(const Point3D& q, double max_distance, std::size_t& index, double& distance) const;
    // Indices of all stored points within radius of q (radius <= voxel size).
    void neighbors(const Point3D& q, double radius, std::vector<std::size_t>& out) const;

private:
    std::int64_t key(std::int64_t ix, std::int64_t iy, std::int64_t iz) const;
    std::int64_t cell(double v) const;
    double voxel_;
    std::vector<Point3D> points_;
    std::unordered_map<std::int64_t, std::vector<std::size_t>> cells_;
};
// A local map for odometry: points in a fixed (odometry) frame, filed by cube,
// at most `max_points_per_voxel` per cube (new points into a full cube are
// dropped, so the map stops growing where it is already dense). Cubes farther
// than `max_distance` from a given position can be removed as the sensor moves
// on. Nearest-neighbor queries search the 27 cubes around the query: exact
// within one voxel size; beyond that they can miss closer points two cubes away
// (an approximation, as in KISS-ICP).
class LocalMap3D {
public:
    // Throws std::invalid_argument for a nonpositive voxel size or zero points per voxel.
    LocalMap3D(double voxel_size, std::size_t max_points_per_voxel);
    void insert(const std::vector<Point3D>& points);
    void removeFarFrom(const Point3D& position, double max_distance);
    bool nearest(const Point3D& q, double max_distance, Point3D& found, double& distance) const;
    std::size_t size() const;
    std::size_t voxels() const { return cells_.size(); }
    std::vector<Point3D> points() const;

private:
    double voxel_;
    std::size_t max_points_;
    struct Cell {
        std::int64_t ix, iy, iz;
        std::vector<Point3D> points;
    };
    std::unordered_map<std::int64_t, Cell> cells_;
};
}  // namespace atlas
