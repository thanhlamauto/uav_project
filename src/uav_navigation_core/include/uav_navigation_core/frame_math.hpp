#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {
// Quaternion order follows ROS: x, y, z, w. Odometry twist is expressed in
// child_frame_id (body); the velocity-level planner operates in world ENU.
inline std::array<double, 4> NormalizeQuaternion(std::array<double, 4> q) {
  double norm2 = 0;
  for (double component : q) {
    if (!std::isfinite(component)) throw std::invalid_argument("non-finite orientation");
    norm2 += component * component;
  }
  if (!std::isfinite(norm2) || norm2 < 1e-12) throw std::invalid_argument("invalid quaternion norm");
  for (double &component : q) component /= std::sqrt(norm2);
  return q;
}
inline Vec3 RotateBodyToEnu(Vec3 v, std::array<double, 4> orientation) {
  if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
    throw std::invalid_argument("non-finite velocity");
  const auto q = NormalizeQuaternion(orientation);
  const Vec3 u{q[0], q[1], q[2]};
  const double dot = u.x*v.x + u.y*v.y + u.z*v.z;
  const double uu = u.x*u.x + u.y*u.y + u.z*u.z;
  const Vec3 cross{u.y*v.z-u.z*v.y, u.z*v.x-u.x*v.z, u.x*v.y-u.y*v.x};
  return {2*dot*u.x + (q[3]*q[3]-uu)*v.x + 2*q[3]*cross.x,
          2*dot*u.y + (q[3]*q[3]-uu)*v.y + 2*q[3]*cross.y,
          2*dot*u.z + (q[3]*q[3]-uu)*v.z + 2*q[3]*cross.z};
}
// Heading derivative from world angular velocity. At nonzero roll/pitch,
// body r and world omega_z are generally NOT the Euler yaw derivative.
inline double YawRateFromAngularVelocityEnu(Vec3 omega, std::array<double, 4> q) {
  q = NormalizeQuaternion(q);
  if (!std::isfinite(omega.x) || !std::isfinite(omega.y) || !std::isfinite(omega.z))
    throw std::invalid_argument("non-finite angular velocity");
  const Vec3 heading = RotateBodyToEnu({1,0,0}, q);
  const double horizontal2 = heading.x*heading.x + heading.y*heading.y;
  if (horizontal2 < 1e-6) throw std::invalid_argument("yaw undefined near vertical pitch");
  const double rate = omega.z - heading.z *
      (heading.x*omega.x + heading.y*omega.y) / horizontal2;
  if (!std::isfinite(rate)) throw std::invalid_argument("invalid yaw rate");
  return rate;
}
} // namespace uav_navigation_core
