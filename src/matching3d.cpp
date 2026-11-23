#include "atlas/matching3d.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace atlas {
std::vector<Point3D> takeEvery(const std::vector<Point3D>& points, std::size_t step) {
    if (step == 0) throw std::invalid_argument("takeEvery: step must be at least 1");
    std::vector<Point3D> out;
    out.reserve(points.size() / step + 1);
    for (std::size_t i = 0; i < points.size(); i += step) out.push_back(points[i]);
    return out;
}

std::vector<Point3D> transformPoints(const std::vector<Point3D>& points, const Pose3D& source_in_destination) {
    std::vector<Point3D> out;
    out.reserve(points.size());
    for (const Point3D& p : points) out.push_back(transformPoint3D(p, source_in_destination));
    return out;
}

std::vector<double> nearestDistances(const std::vector<Point3D>& reference, const std::vector<Point3D>& queries,
                                     const Pose3D& query_in_reference) {
    if (reference.empty()) throw std::invalid_argument("nearestDistances: empty reference");
    std::vector<double> out;
    out.reserve(queries.size());
    for (const Point3D& q : queries) {
        const Point3D p = transformPoint3D(q, query_in_reference);
        double best = std::numeric_limits<double>::infinity();
        for (const Point3D& r : reference) {
            const double dx = r.x - p.x, dy = r.y - p.y, dz = r.z - p.z;
            best = std::min(best, dx * dx + dy * dy + dz * dz);  // compare squared; one sqrt per query
        }
        out.push_back(std::sqrt(best));
    }
    return out;
}
}  // namespace atlas
