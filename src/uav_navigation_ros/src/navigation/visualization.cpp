#include "uav_navigation_ros/local_navigation_node.hpp"

using namespace std::chrono_literals;
namespace uav_navigation_ros {
using namespace navigation_detail;

void LocalNavigationNode::PublishVisualization(){
    const auto stamp = now();
    if (selected_trajectory_) {
      nav_msgs::msg::Path path;
      path.header.stamp = stamp;
      path.header.frame_id = planning_frame_;
      for (const auto &point : selected_trajectory_->points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.state.position_enu_m.x;
        pose.pose.position.y = point.state.position_enu_m.y;
        pose.pose.position.z = point.state.position_enu_m.z;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(std::move(pose));
      }
      predicted_path_publisher_->publish(path);
    }
    if (!last_result_)
      return;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < last_result_->total_costs.size(); ++i)
      if (last_result_->safe[i]) {
        minimum = std::min(minimum, last_result_->total_costs[i]);
        maximum = std::max(maximum, last_result_->total_costs[i]);
      }
    const auto count = std::min<std::size_t>(last_result_->trajectories.size(),
                                             visualization_max_samples_);
    for (std::size_t i = 0; i < count; ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = planning_frame_;
      marker.ns = last_result_->safe[i] ? "mppi_feasible" : "mppi_rejected";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.025;
      marker.pose.orientation.w = 1.0;
      marker.color.a = 0.75;
      if (!last_result_->safe[i]) {
        marker.color.r = 0.45;
        marker.color.g = 0.02;
        marker.color.b = 0.02;
      } else {
        const double ratio =
            maximum > minimum
                ? (last_result_->total_costs[i] - minimum) / (maximum - minimum)
                : 0.0;
        marker.color.r = static_cast<float>(ratio);
        marker.color.g = static_cast<float>(1.0 - 0.5 * ratio);
        marker.color.b = 0.0;
      }
      for (const auto &point : last_result_->trajectories[i].points) {
        geometry_msgs::msg::Point position;
        position.x = point.state.position_enu_m.x;
        position.y = point.state.position_enu_m.y;
        position.z = point.state.position_enu_m.z;
        marker.points.push_back(position);
      }
      array.markers.push_back(std::move(marker));
    }
    sample_publisher_->publish(array);
  }
}
