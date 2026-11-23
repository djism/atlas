#include "atlas/geometry3d.hpp"
#include <algorithm>
#include <cmath>

namespace atlas {
Point3D transformPoint3D(const Point3D& p, const Pose3D& a) {
    const auto& r = a.r;
    return {r[0] * p.x + r[1] * p.y + r[2] * p.z + a.t.x,
            r[3] * p.x + r[4] * p.y + r[5] * p.z + a.t.y,
            r[6] * p.x + r[7] * p.y + r[8] * p.z + a.t.z};
}

Pose3D composePoses3D(const Pose3D& b_in_a, const Pose3D& c_in_b) {
    Pose3D c_in_a;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += b_in_a.r[3 * i + k] * c_in_b.r[3 * k + j];
            c_in_a.r[3 * i + j] = sum;
        }
    c_in_a.t = transformPoint3D(c_in_b.t, b_in_a);
    return c_in_a;
}

Pose3D inversePose3D(const Pose3D& b_in_a) {
    Pose3D a_in_b;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a_in_b.r[3 * i + j] = b_in_a.r[3 * j + i];  // transpose
    const Point3D minus = transformPoint3D(b_in_a.t, {a_in_b.r, {0, 0, 0}});
    a_in_b.t = {-minus.x, -minus.y, -minus.z};
    return a_in_b;
}

Pose3D relativePose3D(const Pose3D& b_in_a, const Pose3D& c_in_a) {
    return composePoses3D(inversePose3D(b_in_a), c_in_a);
}

Pose3D rotationFromVector(const Point3D& v) {
    const double angle = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    Pose3D out;
    if (angle < 1e-15) return out;
    const double x = v.x / angle, y = v.y / angle, z = v.z / angle;
    const double c = std::cos(angle), s = std::sin(angle), k = 1 - c;
    // R = c I + s [a]x + (1 - c) a a^T, a the unit axis.
    out.r = {c + k * x * x,     k * x * y - s * z, k * x * z + s * y,
             k * y * x + s * z, c + k * y * y,     k * y * z - s * x,
             k * z * x - s * y, k * z * y + s * x, c + k * z * z};
    return out;
}

Point3D rotationVector(const Pose3D& pose) {
    const auto& r = pose.r;
    const double angle = rotationAngle(pose);
    // Skew-symmetric part of R is sin(angle) [axis]x.
    const Point3D w{(r[7] - r[5]) / 2, (r[2] - r[6]) / 2, (r[3] - r[1]) / 2};
    const double s = std::sin(angle);
    if (angle < 1e-12) return {w.x, w.y, w.z};  // first order: R ~ I + [w]x
    if (s < 1e-9) {
        // angle ~ pi: axis from the diagonal, R ~ 2 a a^T - I.
        const Point3D a{std::sqrt(std::max(0.0, (r[0] + 1) / 2)), std::sqrt(std::max(0.0, (r[4] + 1) / 2)),
                        std::sqrt(std::max(0.0, (r[8] + 1) / 2))};
        return {a.x * angle, std::copysign(a.y, r[1] + r[3]) * angle, std::copysign(a.z, r[2] + r[6]) * angle};
    }
    return {w.x * angle / s, w.y * angle / s, w.z * angle / s};
}

Pose3D scaleMotion(const Pose3D& motion, double s) {
    const Point3D v = rotationVector(motion);
    Pose3D out = rotationFromVector({v.x * s, v.y * s, v.z * s});
    out.t = {motion.t.x * s, motion.t.y * s, motion.t.z * s};
    return out;
}

double rotationAngle(const Pose3D& pose) {
    const double c = (pose.r[0] + pose.r[4] + pose.r[8] - 1) / 2;
    return std::acos(std::clamp(c, -1.0, 1.0));  // clamp: rounding can push |c| past 1
}

double orthonormalityError(const Pose3D& pose) {
    const auto& r = pose.r;
    double worst = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double dot = 0;
            for (int k = 0; k < 3; ++k) dot += r[3 * i + k] * r[3 * j + k];
            worst = std::max(worst, std::abs(dot - (i == j ? 1.0 : 0.0)));
        }
    const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                       r[2] * (r[3] * r[7] - r[4] * r[6]);
    return std::max(worst, std::abs(det - 1));
}
}  // namespace atlas
