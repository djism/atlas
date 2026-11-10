#pragma once
#include "atlas/simulation.hpp"
#include "atlas/types.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace atlas {
// What the map currently believes about a cell.
enum class CellState { Unknown, Free, Occupied };

// Evidence weights in log-odds, l = ln(p / (1 - p)), where p is the probability
// that the cell is occupied. l = 0 means p = 0.5 (no evidence). Each observation
// ADDS its weight, so repeated agreement builds confidence and disagreement
// cancels it. Clamping keeps the map able to change its mind.
struct OccupancyMapOptions {
    double hit = 0.85;            // beam ended in the cell (p = 0.70 alone)
    double miss = -0.4;           // beam passed through the cell (p = 0.40 alone)
    double min_log_odds = -4.0;   // clamp: p never below ~0.018
    double max_log_odds = 4.0;    // clamp: p never above ~0.982
    double occupied_above = 0.62; // log-odds; p >= 0.65 reads as Occupied
    double free_below = -0.2;     // log-odds; p <= 0.45 reads as Free
};

// Probabilistic occupancy grid over the rectangle [origin, origin + (width, height)]
// in the MAP frame. Cell (column, row) covers
// x in [origin.x + column * resolution, + resolution), same for y with row.
// The map frame here is the world frame, because scans are integrated at poses
// expressed in world coordinates.
class OccupancyMap {
public:
    // Throws std::invalid_argument for nonpositive or nonfinite sizes or
    // resolution, a nonfinite origin, or inconsistent options.
    OccupancyMap(const Point2D& origin, double width, double height, double resolution,
                 const OccupancyMapOptions& options = {});

    // Adds one scan taken at sensor_in_map. ranges[i] belongs to beam i of config
    // (as from simulateLidarRanges); infinity = no return. Every cell a beam
    // passes through gets a miss; the cell where it ends gets a hit. A beam with no
    // return clears cells up to max_range (assumes nothing was in range there).
    // Beams stop at the map edge. Throws std::invalid_argument if ranges.size()
    // != beam_count, a range is negative or NaN, the pose is nonfinite, or the
    // sensor is outside the map.
    void integrateScan(const std::vector<double>& ranges, const LidarConfig& config,
                       const Pose2D& sensor_in_map);

    std::size_t columns() const { return columns_; }
    std::size_t rows() const { return rows_; }
    double resolution() const { return resolution_; }
    const Point2D& origin() const { return origin_; }
    bool inside(long column, long row) const;
    long columnOf(double x) const;
    long rowOf(double y) const;
    Point2D centerOf(long column, long row) const;

    // Outside the map: log-odds 0 and Unknown.
    double logOdds(long column, long row) const;
    double probability(long column, long row) const;  // 1 - 1 / (1 + e^l)
    CellState state(long column, long row) const;
    CellState stateAt(const Point2D& p_in_map) const;

    // Centers of all Occupied cells, in map coordinates.
    std::vector<Point2D> occupiedCenters() const;
    // For each Occupied cell, the mean of the beam end points recorded in it, in
    // map coordinates. Unlike cell centers (up to half a cell from the wall),
    // these keep sub-cell accuracy; use them as localization reference points.
    std::vector<Point2D> occupiedPoints() const;

private:
    void add(long column, long row, double weight);
    std::size_t index(long column, long row) const;

    Point2D origin_;
    double resolution_;
    std::size_t columns_;
    std::size_t rows_;
    OccupancyMapOptions options_;
    std::vector<double> log_odds_;
    std::vector<Point2D> hit_sum_;            // sum of beam end points per cell
    std::vector<std::uint32_t> hit_count_;
};

// Builds a map from one scan per survey pose (answer-key poses, world frame),
// sized to the walls' bounding box plus `margin` meters. This is "mapping with
// known poses", like a surveyed mapping drive. Throws std::invalid_argument for
// no walls or no survey poses, or invalid inputs to the functions it calls.
OccupancyMap surveyMap(const std::vector<Segment2D>& walls, const std::vector<Pose2D>& survey,
                       const LidarConfig& lidar, double resolution, double margin = 0.5);
}  // namespace atlas
