#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "uav_navigation_core/frame_math.hpp"

namespace uav_navigation_ros {
// Both linear and angular twist follow child_frame_id. Derive Euler yaw rate
// from the full orientation/angular velocity, not blindly from body omega_z.
inline uav_navigation_core::State StateFromOdometry(
    const nav_msgs::msg::Odometry& message, const std::string& planning_frame,
    const std::string& body_frame) {
  namespace core = uav_navigation_core;
  if (message.header.frame_id != planning_frame)
    throw std::invalid_argument("odometry pose frame != planning frame");
  const auto& p=message.pose.pose.position;
  const auto& q=message.pose.pose.orientation;
  const auto& v=message.twist.twist.linear;
  const auto& w=message.twist.twist.angular;
  for (double value : {p.x,p.y,p.z,v.x,v.y,v.z,w.x,w.y,w.z})
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite odometry twist/pose");
  const auto orientation=core::NormalizeQuaternion({q.x,q.y,q.z,q.w});
  core::Vec3 velocity{v.x,v.y,v.z},omega{w.x,w.y,w.z};
  if (message.child_frame_id==body_frame) {
    velocity=core::RotateBodyToEnu(velocity,orientation);
    omega=core::RotateBodyToEnu(omega,orientation);
  } else if (message.child_frame_id!=planning_frame) {
    throw std::invalid_argument("odometry twist frame must be body or planning frame");
  }
  const auto heading=core::RotateBodyToEnu({1,0,0},orientation);
  core::State state;
  state.stamp_ns=core::TimeNs(message.header.stamp.sec)*1000000000LL+
                 message.header.stamp.nanosec;
  state.position_enu_m={p.x,p.y,p.z};
  state.velocity_enu_m_s=velocity;
  state.yaw_enu_rad=std::atan2(heading.y,heading.x);
  state.yaw_rate_enu_rad_s=core::YawRateFromAngularVelocityEnu(omega,orientation);
  return state;
}
} // namespace uav_navigation_ros
