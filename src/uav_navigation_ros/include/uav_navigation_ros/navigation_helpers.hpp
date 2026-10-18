#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "uav_navigation_core/command_conditioner.hpp"
#include "uav_navigation_core/mppi/cost_evaluator.hpp"
#include "uav_navigation_core/mppi/dynamics.hpp"
#include "uav_navigation_core/mppi/optimizer.hpp"
#include "uav_navigation_core/mppi/path_reference.hpp"
#include "uav_navigation_core/mppi/rollout.hpp"
#include "uav_navigation_core/mppi/route_blockage.hpp"
#include "uav_navigation_core/stabilization.hpp"
#include "uav_navigation_core/trajectory_safety_checker.hpp"


namespace core = uav_navigation_core;
namespace mppi = uav_navigation_core::mppi;
namespace uav_navigation_ros::navigation_detail {
constexpr double kPi = 3.14159265358979323846;
// Diagnostic-only threshold: a path-heading change above 15 degrees marks the
// start of a "major turn" for the distance-to-next-turn metric. It does not
// alter path geometry or controller behavior.
constexpr double kTurnAngle = 0.2617993877991494;

enum class Mode {
  kWaitingForState,
  kWaitingForPath,
  kWaitingForObstacles,
  kActive,
  kHoldStale,
  kNoSafeTrajectory,
  kRouteBlocked,
  kStoppingRecovery,
  kPlannerTimeout,
  kGoalReached,
  kInvalidInput,
};

const char *ModeName(Mode mode);
const char *ProposalSourceName(mppi::ProposalSource source);
bool Finite(double value);
bool Finite(const core::Vec3 &value);
double Norm(const core::Vec3 &value);
double Distance(const core::Vec3 &a, const core::Vec3 &b);
double Yaw(const geometry_msgs::msg::Quaternion &q);
core::Vec3 TransformPoint(const core::Vec3 &point,
                          const geometry_msgs::msg::Transform &transform);
core::State ToCoreState(const mppi::MppiState &state);
core::Trajectory ToCoreTrajectory(const mppi::MppiTrajectory &trajectory);
diagnostic_msgs::msg::KeyValue KeyValue(const std::string &key,
                                        const std::string &value);
}
