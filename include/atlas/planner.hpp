#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <vector>

namespace atlas {
// Square-cell grid over [0, width] x [0, height] meters (world frame). A cell is
// "blocked" when its center is within `inflation` of any obstacle segment, so a
// round robot of radius <= inflation can be planned as a single point.
class OccupancyGrid {
public:
    OccupancyGrid(double width, double height, double resolution);

    // Blocks every cell whose center lies within `inflation` meters of the segment.
    void addSegment(const Segment2D& segment, double inflation);

    std::size_t columns() const { return columns_; }
    std::size_t rows() const { return rows_; }
    double resolution() const { return resolution_; }
    bool inside(long column, long row) const;
    // Outside the grid counts as blocked.
    bool blocked(long column, long row) const;
    bool blockedAt(const Point2D& p) const;
    long columnOf(double x) const;
    long rowOf(double y) const;
    Point2D centerOf(long column, long row) const;

private:
    double resolution_;
    std::size_t columns_;
    std::size_t rows_;
    std::vector<unsigned char> blocked_;
};

// Builds the planning grid from obstacle segments, inflated by robot_radius + margin.
OccupancyGrid buildPlanningGrid(const std::vector<Segment2D>& obstacles, double width,
                                double height, double resolution, double robot_radius,
                                double margin = 0.05);

enum class PlanStatus { Found, StartBlocked, GoalBlocked, NoPath };

struct PlanResult {
    PlanStatus status = PlanStatus::NoPath;
    // World points from start to goal, smoothed (straight runs between turns).
    std::vector<Point2D> path;
    double length = 0.0;          // meters along the smoothed path
    std::size_t expanded = 0;     // cells A* expanded (search effort)
};

// A* over 8-connected cells (diagonal moves cost sqrt(2) cells and may not cut
// blocked corners), octile-distance heuristic, then line-of-sight smoothing.
// The grid path is shortest among grid paths; smoothing only shortens it.
// Throws std::invalid_argument for nonfinite start or goal.
PlanResult planPath(const OccupancyGrid& grid, const Point2D& start, const Point2D& goal);

// True if the straight segment a-b stays in free cells (sampled at resolution / 4).
bool segmentIsFree(const OccupancyGrid& grid, const Point2D& a, const Point2D& b);
}  // namespace atlas
