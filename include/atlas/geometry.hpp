#pragma once
#include "atlas/types.hpp"

namespace atlas {
// source -> destination: R(theta) * point + translation.
Point2D transformPoint(const Point2D& point, const Pose2D& source_in_destination);
// destination -> source: R(theta)^T * (point - translation).
Point2D inverseTransformPoint(const Point2D& point, const Pose2D& source_in_destination);
}  // namespace atlas
