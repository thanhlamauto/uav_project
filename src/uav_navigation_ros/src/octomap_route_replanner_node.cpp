#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <octomap/OcTree.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uav_navigation_core/static_octomap_planner.hpp"
#include "uav_navigation_core/static_route.hpp"

namespace core = uav_navigation_core;

// Experimental replacement: immutable prior, no camera cloud subscription.
class StaticMapReplanner final : public rclcpp::Node {
 public:
  StaticMapReplanner() : Node("octomap_route_replanner") {
    frame_ = declare_parameter<std::string>("planning_frame", "map");
    const auto prior = declare_parameter<std::string>("prior_map_file", "");
    if (prior.empty() || !map_.readBinary(prior))
      throw std::runtime_error("cannot read static prior map");
    bounds_.min = Parse(declare_parameter<std::vector<double>>("map_min", {-4,-12,1}));
    bounds_.max = Parse(declare_parameter<std::vector<double>>("map_max", {26,12,9}));
    config_.clearance_m = declare_parameter<double>("clearance_m", 1.0);
    config_.max_expansions = declare_parameter<int>("max_expansions", 250000);
    max_connector_m_=declare_parameter<double>("route_retention_distance_m", .75);
    if(!std::isfinite(max_connector_m_) || max_connector_m_<=0)
      throw std::invalid_argument("route_retention_distance_m must be positive");
    max_age_s_ = declare_parameter<double>("max_cloud_age_s", .5);
    const double hz = declare_parameter<double>("replan_rate_hz", 5.0);
    if (!std::isfinite(hz) || hz <= 0 || hz > 20)
      throw std::invalid_argument("replan_rate_hz must be in (0,20]");
    const auto cache_start=std::chrono::steady_clock::now();
    planner_=std::make_unique<core::StaticOctomapPlanner>(map_,bounds_,config_);
    RCLCPP_INFO(get_logger(),"Static clearance cache ready in %.2f ms",
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-cache_start).count());
    pub_ = create_publisher<nav_msgs::msg::Path>(
        "/planning/global_path", rclcpp::QoS(1).reliable().transient_local());
    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odometry_topic", "/localization/odometry"),
        rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          if (m->header.frame_id != frame_) return;
          const auto& p = m->pose.pose.position;
          state_ = {p.x,p.y,p.z}; stamp_ = rclcpp::Time(m->header.stamp);
          have_state_ = Finite(state_);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("goal_topic", "/goal_pose"),
        rclcpp::QoS(1).reliable(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          const auto& p = m->pose.position;
          const core::Vec3 g{p.x,p.y,p.z};
          if (m->header.frame_id != frame_ || !Finite(g) || !bounds_.Contains(g)) return;
          goal_ = g; have_goal_ = true; last_route_ = {}; published_ = false;
          Publish({});  // Invalidate the previous goal exactly once.
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/hz), [this]{Replan();});
    RCLCPP_INFO(get_logger(), "Static prior only; A* requested %.1f Hz wall time; camera map updates disabled", hz);
  }

 private:
  static core::Vec3 Parse(const std::vector<double>& p) {
    if (p.size()!=3) throw std::invalid_argument("expected xyz bounds");
    return {p[0],p[1],p[2]};
  }
  static bool Finite(core::Vec3 p) {
    return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);
  }
  bool SameVoxelRoute(const core::Path& next) const {
    // Ignore the moving start connector. Republishing an identical voxel route
    // would reset MPPI's warm start and command conditioner every timer tick.
    const auto& a = next.points_enu_m; const auto& b = last_route_.points_enu_m;
    if (a.size()!=b.size() || a.size()<2) return false;
    for (std::size_t i=1; i<a.size(); ++i)
      if (std::abs(a[i].x-b[i].x)>1e-6 || std::abs(a[i].y-b[i].y)>1e-6 ||
          std::abs(a[i].z-b[i].z)>1e-6) return false;
    return true;
  }
  void Publish(const core::Path& path) {
    nav_msgs::msg::Path m; m.header.frame_id=frame_; m.header.stamp=now();
    for (const auto& p:path.points_enu_m) {
      geometry_msgs::msg::PoseStamped pose; pose.header=m.header;
      pose.pose.position.x=p.x; pose.pose.position.y=p.y; pose.pose.position.z=p.z;
      pose.pose.orientation.w=1; m.poses.push_back(pose);
    }
    pub_->publish(m);
  }
  void Replan() {
    if (!have_goal_ || !have_state_) return;
    const double age=(now()-stamp_).seconds();
    if (age<-.05 || age>max_age_s_) return;
    const auto start=std::chrono::steady_clock::now();
    const auto result=planner_->Plan(state_,goal_);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    ++attempts_;
    if (result.status!=core::StatusCode::kOk) {
      if (published_) {Publish({});published_=false;}
      RCLCPP_WARN(get_logger(),"Static A* attempt=%zu failed ms=%.2f: %s",attempts_,ms,result.detail.c_str());
      return;
    }
    const bool retain=published_ && core::CanRetainStaticRoute(
        *planner_,last_route_,state_,max_connector_m_);
    const bool changed=!published_ || (!retain && !SameVoxelRoute(result.path));
    if (changed) {Publish(result.path);last_route_=result.path;published_=true;}
    RCLCPP_INFO(get_logger(),"Static A* attempt=%zu ms=%.2f expanded=%zu points=%zu published=%d state_age_s=%.3f",
                attempts_,ms,result.expanded,result.path.points_enu_m.size(),changed,age);
  }
  std::unique_ptr<core::StaticOctomapPlanner> planner_;
  std::string frame_;
  octomap::OcTree map_{.5}; core::MapBounds3D bounds_;
  core::OctomapPlanConfig config_; core::Path last_route_;
  core::Vec3 state_{},goal_{}; rclcpp::Time stamp_{0,0,RCL_ROS_TIME};
  double max_connector_m_{.75};
  double max_age_s_{.5}; bool have_state_{false},have_goal_{false},published_{false};
  std::size_t attempts_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<StaticMapReplanner>());}
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("static_map_replanner"),"%s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
