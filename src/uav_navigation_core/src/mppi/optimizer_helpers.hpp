#pragma once
#include "uav_navigation_core/mppi/optimizer.hpp"
#include <algorithm>
#include <cmath>
namespace uav_navigation_core::mppi::optimizer_detail {

inline std::array<double, 4> Components(const Control &control) {
  return {control.velocity_enu_m_s.x, control.velocity_enu_m_s.y,
          control.velocity_enu_m_s.z, control.yaw_rate_enu_rad_s};
}
inline Control FromComponents(const std::array<double, 4> &values) {
  return {{values[0], values[1], values[2]}, values[3]};
}
inline Control Add(const Control &a, const Control &b) {
  const auto av = Components(a);
  const auto bv = Components(b);
  std::array<double, 4> result{};
  for (std::size_t i = 0; i < 4; ++i)
    result[i] = av[i] + bv[i];
  return FromComponents(result);
}
inline Control Sub(const Control &a, const Control &b) {
  const auto av = Components(a);
  const auto bv = Components(b);
  std::array<double, 4> result{};
  for (std::size_t i = 0; i < 4; ++i)
    result[i] = av[i] - bv[i];
  return FromComponents(result);
}
inline Control Clamp(const Control &control, const std::array<double, 4> &minimum,
              const std::array<double, 4> &maximum) {
  auto values = Components(control);
  for (std::size_t i = 0; i < 4; ++i)
    values[i] = std::clamp(values[i], minimum[i], maximum[i]);
  return FromComponents(values);
}
inline bool Finite(const Control &control) {
  const auto values = Components(control);
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}
inline Trajectory ToSafetyTrajectory(const MppiTrajectory &input) {
  Trajectory output;
  output.points.reserve(input.points.size());
  for (const auto &point : input.points) {
    TrajectoryPoint converted;
    converted.time_from_start_s = point.time_from_start_s;
    converted.state.stamp_ns = point.state.stamp_ns;
    converted.state.position_enu_m = point.state.position_enu_m;
    converted.state.velocity_enu_m_s = point.state.velocity_enu_m_s;
    converted.state.acceleration_enu_m_s2 =
        point.state.acceleration_memory_enu_m_s2;
    converted.state.yaw_enu_rad = point.state.yaw_enu_rad;
    converted.state.yaw_rate_enu_rad_s = point.state.yaw_rate_enu_rad_s;
    converted.control = point.requested_control;
    output.points.push_back(converted);
  }
  return output;
}
inline State ToSafetyState(const MppiState &input) {
  State output;
  output.stamp_ns = input.stamp_ns;
  output.position_enu_m = input.position_enu_m;
  output.velocity_enu_m_s = input.velocity_enu_m_s;
  output.acceleration_enu_m_s2 = input.acceleration_memory_enu_m_s2;
  output.yaw_enu_rad = input.yaw_enu_rad;
  output.yaw_rate_enu_rad_s = input.yaw_rate_enu_rad_s;
  return output;
}
// Maps the safety checker verdict onto the diagnostic rejection categories.
// The checker already reports the first failing predicate, so this mapping
// preserves the deterministic "first failure" policy and never double counts.
inline RejectionReason ClassifyRejection(const SafetyResult &safety) {
  if (safety.safe || safety.reason == "clear_trajectory")
    return RejectionReason::kSafe;
  if (safety.reason == "swept_collision")
    return RejectionReason::kSweptCollision;
  if ((safety.reason == "known_map_collision" || safety.reason == "prior_region_collision"))
    return RejectionReason::kStaticCollision;
  if ((safety.reason == "predicted_stopping_clearance" ||
      safety.reason == "prior_region_stopping_clearance"))
    return RejectionReason::kStoppingDistance;
  if (safety.reason.rfind("invalid", 0) == 0)
    return RejectionReason::kNonFinite;
  return RejectionReason::kOther;
}
inline void AccumulateRejection(RejectionCounts &counts, RejectionReason reason) {
  counts.total += 1;
  switch (reason) {
  case RejectionReason::kSafe:
    counts.safe += 1;
    break;
  case RejectionReason::kNonFinite:
    counts.reject_non_finite += 1;
    break;
  case RejectionReason::kSweptCollision:
    counts.reject_swept_collision += 1;
    break;
  case RejectionReason::kStaticCollision:
    counts.reject_static_collision += 1;
    break;
  case RejectionReason::kDynamicCollision:
    counts.reject_dynamic_collision += 1;
    break;
  case RejectionReason::kStoppingDistance:
    counts.reject_stopping_distance += 1;
    break;
  case RejectionReason::kOther:
    counts.reject_other += 1;
    break;
  }
}
}
