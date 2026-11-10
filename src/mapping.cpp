#include "atlas/mapping.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace atlas {
namespace {
bool finite(double v) { return std::isfinite(v); }
}  // namespace

OccupancyMap::OccupancyMap(const Point2D& origin, double width, double height,
                           double resolution, const OccupancyMapOptions& options)
    : origin_(origin), resolution_(resolution), columns_(0), rows_(0), options_(options) {
    if (!finite(origin.x) || !finite(origin.y)) throw std::invalid_argument("map origin must be finite");
    if (!finite(width) || !finite(height) || width <= 0 || height <= 0)
        throw std::invalid_argument("map width and height must be positive and finite");
    if (!finite(resolution) || resolution <= 0)
        throw std::invalid_argument("map resolution must be positive and finite");
    const OccupancyMapOptions& o = options;
    if (!(o.hit > 0) || !(o.miss < 0) || !(o.min_log_odds < o.free_below) ||
        !(o.free_below < o.occupied_above) || !(o.occupied_above < o.max_log_odds) ||
        !finite(o.min_log_odds) || !finite(o.max_log_odds))
        throw std::invalid_argument("map options: need hit > 0 > miss and "
                                    "min < free_below < occupied_above < max");
    columns_ = static_cast<std::size_t>(std::ceil(width / resolution));
    rows_ = static_cast<std::size_t>(std::ceil(height / resolution));
    log_odds_.assign(columns_ * rows_, 0.0);
    hit_sum_.assign(columns_ * rows_, Point2D{0, 0});
    hit_count_.assign(columns_ * rows_, 0);
}

std::size_t OccupancyMap::index(long column, long row) const {
    return static_cast<std::size_t>(row) * columns_ + static_cast<std::size_t>(column);
}

bool OccupancyMap::inside(long column, long row) const {
    return column >= 0 && row >= 0 && static_cast<std::size_t>(column) < columns_ &&
           static_cast<std::size_t>(row) < rows_;
}

long OccupancyMap::columnOf(double x) const {
    return static_cast<long>(std::floor((x - origin_.x) / resolution_));
}

long OccupancyMap::rowOf(double y) const {
    return static_cast<long>(std::floor((y - origin_.y) / resolution_));
}

Point2D OccupancyMap::centerOf(long column, long row) const {
    return {origin_.x + (column + 0.5) * resolution_, origin_.y + (row + 0.5) * resolution_};
}

double OccupancyMap::logOdds(long column, long row) const {
    if (!inside(column, row)) return 0.0;
    return log_odds_[index(column, row)];
}

double OccupancyMap::probability(long column, long row) const {
    return 1.0 - 1.0 / (1.0 + std::exp(logOdds(column, row)));
}

CellState OccupancyMap::state(long column, long row) const {
    const double l = logOdds(column, row);
    if (l >= options_.occupied_above) return CellState::Occupied;
    if (l <= options_.free_below) return CellState::Free;
    return CellState::Unknown;
}

CellState OccupancyMap::stateAt(const Point2D& p) const {
    return state(columnOf(p.x), rowOf(p.y));
}

std::vector<Point2D> OccupancyMap::occupiedCenters() const {
    std::vector<Point2D> centers;
    for (std::size_t row = 0; row < rows_; ++row)
        for (std::size_t column = 0; column < columns_; ++column)
            if (state(static_cast<long>(column), static_cast<long>(row)) == CellState::Occupied)
                centers.push_back(centerOf(static_cast<long>(column), static_cast<long>(row)));
    return centers;
}

OccupancyMap surveyMap(const std::vector<Segment2D>& walls, const std::vector<Pose2D>& survey,
                       const LidarConfig& lidar, double resolution, double margin) {
    if (walls.empty()) throw std::invalid_argument("surveyMap needs walls");
    if (survey.empty()) throw std::invalid_argument("surveyMap needs survey poses");
    double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
    double max_x = -min_x, max_y = -min_x;
    for (const auto& w : walls)
        for (const auto& p : {w.start, w.end}) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        }
    OccupancyMap map({min_x - margin, min_y - margin}, max_x - min_x + 2 * margin,
                     max_y - min_y + 2 * margin, resolution);
    for (const auto& pose : survey)
        map.integrateScan(simulateLidarRanges(walls, pose, lidar), lidar, pose);
    return map;
}

std::vector<Point2D> OccupancyMap::occupiedPoints() const {
    std::vector<Point2D> points;
    for (std::size_t row = 0; row < rows_; ++row)
        for (std::size_t column = 0; column < columns_; ++column) {
            const long c = static_cast<long>(column), r = static_cast<long>(row);
            if (state(c, r) != CellState::Occupied) continue;
            const std::size_t i = index(c, r);
            // Only hits raise log-odds, so an Occupied cell always has a hit.
            points.push_back({hit_sum_[i].x / hit_count_[i], hit_sum_[i].y / hit_count_[i]});
        }
    return points;
}

void OccupancyMap::add(long column, long row, double weight) {
    double& l = log_odds_[index(column, row)];
    l = std::clamp(l + weight, options_.min_log_odds, options_.max_log_odds);
}

void OccupancyMap::integrateScan(const std::vector<double>& ranges, const LidarConfig& config,
                                 const Pose2D& sensor) {
    if (ranges.size() != config.beam_count)
        throw std::invalid_argument("integrateScan: one range per beam is required");
    if (!finite(sensor.x) || !finite(sensor.y) || !finite(sensor.theta))
        throw std::invalid_argument("integrateScan: sensor pose must be finite");
    if (!finite(config.max_range) || config.max_range <= 0)
        throw std::invalid_argument("integrateScan: max_range must be positive and finite");
    const long start_column = columnOf(sensor.x);
    const long start_row = rowOf(sensor.y);
    if (!inside(start_column, start_row))
        throw std::invalid_argument("integrateScan: sensor is outside the map");
    for (double r : ranges)
        if (std::isnan(r) || r < 0) throw std::invalid_argument("integrateScan: ranges must be >= 0");

    const double inf = std::numeric_limits<double>::infinity();
    for (std::size_t beam = 0; beam < ranges.size(); ++beam) {
        const bool hit = std::isfinite(ranges[beam]) && ranges[beam] <= config.max_range;
        const double length = hit ? ranges[beam] : config.max_range;
        // Beam direction in MAP axes: sensor heading plus the beam's angle in sensor axes.
        const double angle = sensor.theta + beamAngle(beam, config);
        const double dx = std::cos(angle), dy = std::sin(angle);

        // Grid traversal (Amanatides & Woo): walk every cell the beam crosses, in
        // order. t is meters along the beam. next_x is the t where the beam crosses
        // the next vertical cell border; step_x is the t between vertical borders.
        long column = start_column, row = start_row;
        const int dir_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
        const int dir_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
        double next_x = inf, next_y = inf, step_x = inf, step_y = inf;
        if (dir_x != 0) {
            const double border = origin_.x + (column + (dir_x > 0 ? 1 : 0)) * resolution_;
            next_x = (border - sensor.x) / dx;
            step_x = resolution_ / std::abs(dx);
        }
        if (dir_y != 0) {
            const double border = origin_.y + (row + (dir_y > 0 ? 1 : 0)) * resolution_;
            next_y = (border - sensor.y) / dy;
            step_y = resolution_ / std::abs(dy);
        }
        bool left_map = false;
        // Every cell before the one containing the beam's end point: the beam passed through.
        while (std::min(next_x, next_y) < length) {
            add(column, row, options_.miss);
            if (next_x < next_y) { column += dir_x; next_x += step_x; }
            else { row += dir_y; next_y += step_y; }
            if (!inside(column, row)) { left_map = true; break; }
        }
        if (left_map) continue;
        // The end cell: a wall was seen here, or (no return) it was clear too.
        add(column, row, hit ? options_.hit : options_.miss);
        if (hit) {
            const std::size_t i = index(column, row);
            hit_sum_[i].x += sensor.x + length * dx;  // the beam end point, map frame
            hit_sum_[i].y += sensor.y + length * dy;
            ++hit_count_[i];
        }
    }
}
}  // namespace atlas
