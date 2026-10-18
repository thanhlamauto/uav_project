#include "uav_navigation_ros/navigation_helpers.hpp"
namespace uav_navigation_ros::navigation_detail {
const char *ModeName(Mode mode) {
  switch (mode) {
  case Mode::kWaitingForState:
    return "WAITING_FOR_STATE";
  case Mode::kWaitingForPath:
    return "WAITING_FOR_PATH";
  case Mode::kWaitingForObstacles:
    return "WAITING_FOR_OBSTACLES";
  case Mode::kActive:
    return "ACTIVE";
  case Mode::kHoldStale:
    return "HOLD_STALE";
  case Mode::kNoSafeTrajectory:
    return "NO_SAFE_TRAJECTORY";
  case Mode::kRouteBlocked:
    return "ROUTE_BLOCKED";
  case Mode::kStoppingRecovery:
    return "STOPPING_RECOVERY";
  case Mode::kPlannerTimeout:
    return "PLANNER_TIMEOUT";
  case Mode::kGoalReached:
    return "GOAL_REACHED";
  case Mode::kInvalidInput:
    return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

const char *ProposalSourceName(mppi::ProposalSource source) {
  switch (source) {
  case mppi::ProposalSource::kGaussian:
    return "gaussian";
  case mppi::ProposalSource::kReferenceProposal:
    return "reference";
  case mppi::ProposalSource::kBrakingProposal:
    return "braking";
  case mppi::ProposalSource::kRecoveryProposal:
    return "recovery";
  case mppi::ProposalSource::kSpecificAction:
    return "specific_action";
  case mppi::ProposalSource::kCount:
    return "unknown";
  }
  return "unknown";
}

bool Finite(double value) { return std::isfinite(value); }
bool Finite(const core::Vec3 &value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}
double Norm(const core::Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}
double Distance(const core::Vec3 &a, const core::Vec3 &b) {
  return Norm({a.x - b.x, a.y - b.y, a.z - b.z});
}
double Yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
core::Vec3 TransformPoint(const core::Vec3 &point,
                          const geometry_msgs::msg::Transform &transform) {
  const auto &q = transform.rotation;
  const core::Vec3 u{q.x, q.y, q.z};
  const double dot_uv = u.x * point.x + u.y * point.y + u.z * point.z;
  const double dot_uu = u.x * u.x + u.y * u.y + u.z * u.z;
  const core::Vec3 cross{u.y * point.z - u.z * point.y,
                         u.z * point.x - u.x * point.z,
                         u.x * point.y - u.y * point.x};
  return {2.0 * dot_uv * u.x + (q.w * q.w - dot_uu) * point.x +
              2.0 * q.w * cross.x + transform.translation.x,
          2.0 * dot_uv * u.y + (q.w * q.w - dot_uu) * point.y +
              2.0 * q.w * cross.y + transform.translation.y,
          2.0 * dot_uv * u.z + (q.w * q.w - dot_uu) * point.z +
              2.0 * q.w * cross.z + transform.translation.z};
}
core::State ToCoreState(const mppi::MppiState &state) {
  core::State output;
  output.stamp_ns = state.stamp_ns;
  output.position_enu_m = state.position_enu_m;
  output.velocity_enu_m_s = state.velocity_enu_m_s;
  output.acceleration_enu_m_s2 = state.acceleration_memory_enu_m_s2;
  output.yaw_enu_rad = state.yaw_enu_rad;
  output.yaw_rate_enu_rad_s = state.yaw_rate_enu_rad_s;
  return output;
}
core::Trajectory ToCoreTrajectory(const mppi::MppiTrajectory &trajectory) {
  core::Trajectory output;
  output.points.reserve(trajectory.points.size());
  for (const auto &point : trajectory.points) {
    core::TrajectoryPoint converted;
    converted.time_from_start_s = point.time_from_start_s;
    converted.state = ToCoreState(point.state);
    converted.control = point.requested_control;
    output.points.push_back(converted);
  }
  return output;
}
diagnostic_msgs::msg::KeyValue KeyValue(const std::string &key,
                                        const std::string &value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}
}
