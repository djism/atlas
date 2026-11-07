#include "atlas/planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace atlas {
namespace {
double distanceToSegment(const Point2D& p, const Segment2D& s) {
    const double ex = s.end.x - s.start.x;
    const double ey = s.end.y - s.start.y;
    const double length_squared = ex * ex + ey * ey;
    double t = 0.0;
    if (length_squared > 0.0) {
        t = ((p.x - s.start.x) * ex + (p.y - s.start.y) * ey) / length_squared;
        t = std::min(1.0, std::max(0.0, t));
    }
    return std::hypot(p.x - (s.start.x + t * ex), p.y - (s.start.y + t * ey));
}
}  // namespace

OccupancyGrid::OccupancyGrid(double width, double height, double resolution)
    : resolution_(resolution) {
    if (!(width > 0.0) || !(height > 0.0) || !(resolution > 0.0) || !std::isfinite(width) ||
        !std::isfinite(height) || !std::isfinite(resolution)) {
        throw std::invalid_argument("grid size and resolution must be finite and positive");
    }
    columns_ = static_cast<std::size_t>(std::ceil(width / resolution));
    rows_ = static_cast<std::size_t>(std::ceil(height / resolution));
    if (columns_ * rows_ > 4'000'000) {
        throw std::invalid_argument("grid too large; increase resolution");
    }
    blocked_.assign(columns_ * rows_, 0);
}

bool OccupancyGrid::inside(long column, long row) const {
    return column >= 0 && row >= 0 && static_cast<std::size_t>(column) < columns_ &&
           static_cast<std::size_t>(row) < rows_;
}

bool OccupancyGrid::blocked(long column, long row) const {
    if (!inside(column, row)) return true;
    return blocked_[static_cast<std::size_t>(row) * columns_ + static_cast<std::size_t>(column)] != 0;
}

long OccupancyGrid::columnOf(double x) const { return static_cast<long>(std::floor(x / resolution_)); }
long OccupancyGrid::rowOf(double y) const { return static_cast<long>(std::floor(y / resolution_)); }

bool OccupancyGrid::blockedAt(const Point2D& p) const { return blocked(columnOf(p.x), rowOf(p.y)); }

Point2D OccupancyGrid::centerOf(long column, long row) const {
    return {(static_cast<double>(column) + 0.5) * resolution_, (static_cast<double>(row) + 0.5) * resolution_};
}

void OccupancyGrid::addSegment(const Segment2D& s, double inflation) {
    if (!(inflation >= 0.0) || !std::isfinite(inflation) || !std::isfinite(s.start.x) ||
        !std::isfinite(s.start.y) || !std::isfinite(s.end.x) || !std::isfinite(s.end.y)) {
        throw std::invalid_argument("segment and inflation must be finite (inflation >= 0)");
    }
    // Only cells in the segment's bounding box, grown by the inflation, can be affected.
    const long c0 = std::max(0L, columnOf(std::min(s.start.x, s.end.x) - inflation));
    const long c1 = std::min(static_cast<long>(columns_) - 1, columnOf(std::max(s.start.x, s.end.x) + inflation));
    const long r0 = std::max(0L, rowOf(std::min(s.start.y, s.end.y) - inflation));
    const long r1 = std::min(static_cast<long>(rows_) - 1, rowOf(std::max(s.start.y, s.end.y) + inflation));
    for (long r = r0; r <= r1; ++r) {
        for (long c = c0; c <= c1; ++c) {
            if (distanceToSegment(centerOf(c, r), s) <= inflation) {
                blocked_[static_cast<std::size_t>(r) * columns_ + static_cast<std::size_t>(c)] = 1;
            }
        }
    }
}

OccupancyGrid buildPlanningGrid(const std::vector<Segment2D>& obstacles, double width,
                                double height, double resolution, double robot_radius,
                                double margin) {
    if (!(robot_radius >= 0.0) || !(margin >= 0.0)) {
        throw std::invalid_argument("robot radius and margin must be nonnegative");
    }
    OccupancyGrid grid(width, height, resolution);
    for (const auto& s : obstacles) grid.addSegment(s, robot_radius + margin);
    return grid;
}

bool segmentIsFree(const OccupancyGrid& grid, const Point2D& a, const Point2D& b) {
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const std::size_t samples =
        static_cast<std::size_t>(std::ceil(length / (grid.resolution() / 4.0))) + 1;
    for (std::size_t i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        if (grid.blockedAt({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)})) return false;
    }
    return true;
}

PlanResult planPath(const OccupancyGrid& grid, const Point2D& start, const Point2D& goal) {
    if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(goal.x) ||
        !std::isfinite(goal.y)) {
        throw std::invalid_argument("start and goal must be finite");
    }
    PlanResult result;
    const long sc = grid.columnOf(start.x), sr = grid.rowOf(start.y);
    const long gc = grid.columnOf(goal.x), gr = grid.rowOf(goal.y);
    if (grid.blocked(sc, sr)) { result.status = PlanStatus::StartBlocked; return result; }
    if (grid.blocked(gc, gr)) { result.status = PlanStatus::GoalBlocked; return result; }

    const long columns = static_cast<long>(grid.columns());
    const auto index = [&](long c, long r) { return static_cast<std::size_t>(r * columns + c); };
    const std::size_t cells = grid.columns() * grid.rows();
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> cost(cells, inf);        // best known distance from start (cells)
    std::vector<long> parent(cells, -1);
    std::vector<unsigned char> closed(cells, 0);
    // Octile distance: exact shortest 8-connected distance with no obstacles,
    // so it never overestimates (admissible) and A* returns a shortest grid path.
    const auto heuristic = [&](long c, long r) {
        const double dx = std::abs(static_cast<double>(c - gc));
        const double dy = std::abs(static_cast<double>(r - gr));
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    };
    using Entry = std::pair<double, long>;  // (cost + heuristic, cell index)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    cost[index(sc, sr)] = 0.0;
    open.push({heuristic(sc, sr), static_cast<long>(index(sc, sr))});

    const long dc[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const long dr[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    bool reached = false;
    while (!open.empty()) {
        const long current = open.top().second;
        open.pop();
        if (closed[static_cast<std::size_t>(current)]) continue;  // stale queue entry
        closed[static_cast<std::size_t>(current)] = 1;
        ++result.expanded;
        const long c = current % columns;
        const long r = current / columns;
        if (c == gc && r == gr) { reached = true; break; }
        for (int k = 0; k < 8; ++k) {
            const long nc = c + dc[k], nr = r + dr[k];
            if (grid.blocked(nc, nr)) continue;
            const bool diagonal = dc[k] != 0 && dr[k] != 0;
            // No corner cutting: a diagonal step needs both side cells free.
            if (diagonal && (grid.blocked(c + dc[k], r) || grid.blocked(c, r + dr[k]))) continue;
            const double next = cost[static_cast<std::size_t>(current)] + (diagonal ? std::sqrt(2.0) : 1.0);
            const std::size_t n = index(nc, nr);
            if (next < cost[n]) {
                cost[n] = next;
                parent[n] = current;
                open.push({next + heuristic(nc, nr), static_cast<long>(n)});
            }
        }
    }
    if (!reached) { result.status = PlanStatus::NoPath; return result; }

    // Cells goal -> start, as world points; exact start and goal at the ends.
    std::vector<Point2D> raw{goal};
    for (long n = parent[index(gc, gr)]; n != -1 && n != static_cast<long>(index(sc, sr));
         n = parent[static_cast<std::size_t>(n)]) {
        raw.push_back(grid.centerOf(n % columns, n / columns));
    }
    raw.push_back(start);
    std::reverse(raw.begin(), raw.end());

    // String pulling: from each kept point, jump to the farthest visible point.
    result.path.push_back(raw.front());
    std::size_t anchor = 0;
    while (anchor + 1 < raw.size()) {
        std::size_t farthest = anchor + 1;
        for (std::size_t j = raw.size() - 1; j > anchor + 1; --j) {
            if (segmentIsFree(grid, raw[anchor], raw[j])) { farthest = j; break; }
        }
        result.path.push_back(raw[farthest]);
        anchor = farthest;
    }
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        result.length += std::hypot(result.path[i].x - result.path[i - 1].x,
                                    result.path[i].y - result.path[i - 1].y);
    }
    result.status = PlanStatus::Found;
    return result;
}
}  // namespace atlas
