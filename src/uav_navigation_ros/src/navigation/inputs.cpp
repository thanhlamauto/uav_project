#include "uav_navigation_ros/local_navigation_node.hpp"
#include "uav_navigation_core/frame_math.hpp"
#include "uav_navigation_ros/odometry_state.hpp"

using namespace std::chrono_literals;
namespace uav_navigation_ros {
using namespace navigation_detail;

void LocalNavigationNode::OnOdometry(const nav_msgs::msg::Odometry &message){
    const std::string frame = message.header.frame_id.empty()
                                  ? planning_frame_
                                  : message.header.frame_id;
    if (frame != planning_frame_) {
      odometry_error_ = "odometry frame " + frame + " != " + planning_frame_;
      return;
    }
    core::State measured;
    try { measured = StateFromOdometry(message, planning_frame_, body_frame_); }
    catch (const std::exception& error) {odometry_error_=error.what();return;}
    const auto velocity=measured.velocity_enu_m_s;
    const auto stamp=measured.stamp_ns;
    core::Vec3 acceleration{};
    if (have_state_ && stamp > state_.stamp_ns) {
      const double elapsed =
          static_cast<double>(stamp - state_.stamp_ns) * 1e-9;
      if (elapsed <= 0.5) {
        acceleration = {(velocity.x - state_.velocity_enu_m_s.x) / elapsed,
                        (velocity.y - state_.velocity_enu_m_s.y) / elapsed,
                        (velocity.z - state_.velocity_enu_m_s.z) / elapsed};
      }
    }
    state_.stamp_ns = stamp;
    state_.position_enu_m = measured.position_enu_m;
    state_.velocity_enu_m_s = velocity;
    state_.acceleration_memory_enu_m_s2 = acceleration;
    state_.yaw_enu_rad = measured.yaw_enu_rad;
    state_.yaw_rate_enu_rad_s = measured.yaw_rate_enu_rad_s;
    last_state_received_ = std::chrono::steady_clock::now();
    have_state_ = true;
    odometry_error_.clear();
  }

void LocalNavigationNode::OnPath(const nav_msgs::msg::Path &message){
    const std::string frame = message.header.frame_id.empty()
                                  ? planning_frame_
                                  : message.header.frame_id;
    if (frame != planning_frame_) {
      path_error_ = "path frame " + frame + " != " + planning_frame_;
      return;
    }
    std::vector<core::Vec3> points;
    points.reserve(message.poses.size());
    for (const auto &pose : message.poses) {
      const auto &p = pose.pose.position;
      if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z)) {
        path_error_ = "non-finite global path";
        return;
      }
      points.push_back({p.x, p.y, p.z});
    }
    if (points.size() < 2) {
      reference_path_.reset();
      ResetController();
      route_blocked_latched_ = false;
      detour_side_ = 0;
      path_error_ = "global path has fewer than two points";
      return;
    }
    try {
      reference_path_ =
          std::make_shared<mppi::PathReference>(std::move(points));
      goal_ = reference_path_->points().back();
      // Diagnostic path versioning: a new accepted path gets a monotonic local
      // id and its originating message stamp so the analyzer can join it to
      // the global planner goal_id/global_path_id. This does not change which
      // path the controller uses.
      previous_path_id_ = local_active_path_id_;
      ++local_active_path_id_;
      active_path_stamp_ns_ =
          rclcpp::Time(message.header.stamp).nanoseconds();
      path_received_monotonic_ = std::chrono::steady_clock::now();
      solves_on_active_path_ = 0;
      have_first_solve_on_path_ = false;
      have_first_safe_on_path_ = false;
      // The global path is a replacement plan, so the MPPI warm start and the
      // command conditioner are reset rather than carried across the handoff.
      ResetController();
      route_blocked_latched_ = false;
      detour_side_ = 0;
      path_error_.clear();
        RCLCPP_INFO(get_logger(),
                  "accepted global path id=%llu previous_id=%llu stamp_ns=%lld",
                  static_cast<unsigned long long>(local_active_path_id_),
                  static_cast<unsigned long long>(previous_path_id_),
                  static_cast<long long>(active_path_stamp_ns_));
    } catch (const std::exception &error) {
      reference_path_.reset();
      path_error_ = error.what();
    }
  }

void LocalNavigationNode::OnObstacles(const sensor_msgs::msg::PointCloud2 &message){
    geometry_msgs::msg::Transform transform;
    const std::string source = message.header.frame_id.empty()
                                   ? planning_frame_
                                   : message.header.frame_id;
    if (source != planning_frame_) {
      try {
        transform = tf_buffer_
                        .lookupTransform(planning_frame_, source,
                                         message.header.stamp, 50ms)
                        .transform;
      } catch (const std::exception &error) {
        obstacle_error_ =
            std::string("obstacle TF unavailable: ") + error.what();
        return;
      }
    } else {
      transform.rotation.w = 1.0;
    }
    std::vector<core::Vec3> points;
    try {
      const std::size_t total =
          static_cast<std::size_t>(message.width) * message.height;
      const std::size_t stride = std::max<std::size_t>(
          1, (total + obstacle_max_points_ - 1) / obstacle_max_points_);
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      std::size_t index = 0;
      for (; x != x.end(); ++x, ++y, ++z, ++index) {
        if (index % stride != 0)
          continue;
        const core::Vec3 point{*x, *y, *z};
        if (Finite(point))
          points.push_back(TransformPoint(point, transform));
      }
    } catch (const std::exception &error) {
      obstacle_error_ = std::string("invalid obstacle cloud: ") + error.what();
      return;
    }
    obstacles_.stamp_ns = rclcpp::Time(message.header.stamp).nanoseconds();
    obstacles_.observation_valid = true;
    obstacles_.points_enu_m = std::move(points);
    last_obstacles_received_ = std::chrono::steady_clock::now();
    have_obstacles_ = true;
    obstacle_error_.clear();
  }
}
