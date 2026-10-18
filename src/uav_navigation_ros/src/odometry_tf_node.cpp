#include <cmath>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "uav_navigation_core/frame_math.hpp"

// Normalize an explicitly selected localization source; never switch on failure.
// Both sources have world ENU pose and body FLU twist. The configured source
// frame and planning frame must share origin/axes (identity registration).
class OdometryTfNode final : public rclcpp::Node {
public:
  OdometryTfNode() : Node("odometry_tf"), broadcaster_(*this), static_broadcaster_(*this) {
    frame_ = declare_parameter<std::string>("planning_frame", "map");
    body_ = declare_parameter<std::string>("body_frame", "base_link");
    max_age_ = declare_parameter<double>("max_state_age_s", 0.5);
    const auto source = declare_parameter<std::string>("localization_source", "gazebo");
    if (source != "gazebo" && source != "mavros")
      throw std::invalid_argument("localization_source must be gazebo or mavros");
    source_frame_ = declare_parameter<std::string>(
        "source_frame", source == "gazebo" ? "odom" : frame_);
    if (frame_.empty() || body_.empty() || source_frame_.empty() ||
        !std::isfinite(max_age_) || max_age_ <= 0)
      throw std::invalid_argument("invalid localization frames or age limit");
    const auto camera_frame=declare_parameter<std::string>("camera_frame","camera_optical_frame");
    const auto translation=declare_parameter<std::vector<double>>("camera_translation_m",{0,0,0});
    const auto rotation=declare_parameter<std::vector<double>>("camera_quaternion_xyzw",{-.5,.5,-.5,.5});
    if(translation.size()!=3 || rotation.size()!=4 || camera_frame.empty())
      throw std::invalid_argument("invalid camera extrinsics");
    for(double value:translation) if(!std::isfinite(value))
      throw std::invalid_argument("non-finite camera translation");
    const auto q=uav_navigation_core::NormalizeQuaternion({rotation[0],rotation[1],rotation[2],rotation[3]});
    geometry_msgs::msg::TransformStamped mount;
    mount.header.stamp=now();mount.header.frame_id=body_;mount.child_frame_id=camera_frame;
    mount.transform.translation.x=translation[0];mount.transform.translation.y=translation[1];
    mount.transform.translation.z=translation[2];
    mount.transform.rotation.x=q[0];mount.transform.rotation.y=q[1];
    mount.transform.rotation.z=q[2];mount.transform.rotation.w=q[3];
    static_broadcaster_.sendTransform(mount);
    const auto topic = declare_parameter<std::string>("input_odometry_topic",
        source == "gazebo" ? "/iris/odometry" : "/mavros/local_position/odom");
    const auto output = declare_parameter<std::string>(
        "odometry_topic", "/localization/odometry");
    if (topic == output) throw std::invalid_argument("localization input/output must differ");
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "Localization source=%s: %s [%s] -> %s [%s]; "
        "identity ENU registration, no fallback", source.c_str(), topic.c_str(),
        source_frame_.c_str(), output.c_str(), frame_.c_str());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { OnOdometry(*message); });
  }
private:
  void OnOdometry(const nav_msgs::msg::Odometry &message) {
    if (message.header.frame_id != source_frame_ || message.child_frame_id != body_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Localization frame mismatch; state rejected");
      return;
    }
    const double age = (now() - rclcpp::Time(message.header.stamp)).seconds();
    if (age < -0.05 || age > max_age_) return;
    const auto &p = message.pose.pose.position;
    const auto &q = message.pose.pose.orientation;
    const auto& linear = message.twist.twist.linear;
    const auto& angular = message.twist.twist.angular;
    const double values[] = {p.x,p.y,p.z,linear.x,linear.y,linear.z,
                             angular.x,angular.y,angular.z};
    for (double value : values) if (!std::isfinite(value)) return;
    try {
      const auto normalized = uav_navigation_core::NormalizeQuaternion({q.x,q.y,q.z,q.w});
      geometry_msgs::msg::TransformStamped tf;
      auto state = message;
      state.header.frame_id = frame_;
      state.pose.pose.orientation.x = normalized[0];
      state.pose.pose.orientation.y = normalized[1];
      state.pose.pose.orientation.z = normalized[2];
      state.pose.pose.orientation.w = normalized[3];
      // Preserve capture stamp, world position, body twist and covariance.
      // In Gazebo mode source_frame=odom and map are the same world ENU origin.
      publisher_->publish(state);
      tf.header = state.header;
      tf.child_frame_id = body_;
      tf.transform.translation.x = p.x;
      tf.transform.translation.y = p.y;
      tf.transform.translation.z = p.z;
      tf.transform.rotation.x = normalized[0]; tf.transform.rotation.y = normalized[1];
      tf.transform.rotation.z = normalized[2]; tf.transform.rotation.w = normalized[3];
      broadcaster_.sendTransform(tf);
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"Invalid localization orientation: %s",error.what());
    }
  }
  std::string frame_, body_, source_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  double max_age_{};
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTfNode>());
  rclcpp::shutdown();
}
