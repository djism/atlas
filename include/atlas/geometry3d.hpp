#pragma once
#include "atlas/types.hpp"
#include <array>

namespace atlas {
// Rigid 3D transform: a SOURCE frame's orientation and origin in a
// DESTINATION frame, as Pose2D in 2D. destination_point = R * source_point + t.
// R is row-major: r[3 * row + column].
struct Pose3D {
    std::array<double, 9> r{1, 0, 0, 0, 1, 0, 0, 0, 1};
    Point3D t{0, 0, 0};
};

// source -> destination: R * p + t.
Point3D transformPoint3D(const Point3D& point, const Pose3D& source_in_destination);
// Frame C in frame A, from B in A and C in B (same convention as 2D).
Pose3D composePoses3D(const Pose3D& b_in_a, const Pose3D& c_in_b);
// A in B, from B in A: R^T and -R^T t.
Pose3D inversePose3D(const Pose3D& b_in_a);
// C in B, from B and C both in A: the motion that takes pose b to pose c.
Pose3D relativePose3D(const Pose3D& b_in_a, const Pose3D& c_in_a);
// Rotation by |v| radians about the axis v / |v| (Rodrigues' formula); the
// zero vector gives the identity. Translation is zero.
Pose3D rotationFromVector(const Point3D& v);

// The inverse of rotationFromVector: axis * angle of R, angle in [0, pi].
// (Near pi the axis sign is ambiguous; the motions here are small.)
Point3D rotationVector(const Pose3D& pose);

// A fraction s of a motion, at constant velocity: rotation by s times its
// axis-angle vector, translation scaled by s (exact for pure translation;
// for the small rotations between scans the difference from a screw motion
// is negligible). s may be negative (moving backwards in time).
Pose3D scaleMotion(const Pose3D& motion, double s);

// Angle of the rotation, radians in [0, pi]: acos((trace(R) - 1) / 2).
double rotationAngle(const Pose3D& pose);
// Largest |(R R^T - I)| entry and |det(R) - 1|: 0 for an exact rotation.
double orthonormalityError(const Pose3D& pose);
}  // namespace atlas
