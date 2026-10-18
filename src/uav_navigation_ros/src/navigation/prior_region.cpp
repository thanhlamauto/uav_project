#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "uav_navigation_core/static_octomap_planner.hpp"

namespace uav_navigation_ros {
namespace core = uav_navigation_core;
std::shared_ptr<const core::IFlightRegion> LoadPriorFlightRegion(rclcpp::Node& node) {
  if (!node.declare_parameter<bool>("safety.prior_map_enabled", true)) {
    RCLCPP_WARN(node.get_logger(), "Prior flight-region guard explicitly disabled");
    return nullptr;
  }
  const auto file=node.declare_parameter<std::string>("prior_map_file", "");
  octomap::OcTree map(.5);
  if (file.empty() || !map.readBinary(file))
    throw std::runtime_error("prior safety guard enabled but prior_map_file unreadable");
  auto parse=[](const std::vector<double>& p) {
    if(p.size()!=3) throw std::invalid_argument("prior bounds require xyz");
    return core::Vec3{p[0],p[1],p[2]};
  };
  core::MapBounds3D bounds;
  bounds.min=parse(node.declare_parameter<std::vector<double>>("map_min", {-4,-12,1}));
  bounds.max=parse(node.declare_parameter<std::vector<double>>("map_max", {26,12,9}));
  core::OctomapPlanConfig config;
  config.clearance_m=node.declare_parameter<double>("clearance_m",1.0);
  config.allow_unknown=false;
  const auto start=std::chrono::steady_clock::now();
  auto region=std::make_shared<core::StaticOctomapPlanner>(map,bounds,config);
  RCLCPP_INFO(node.get_logger(),
    "MPPI prior guard ready: %s, clearance %.2f m, %.2f ms; rollout + braking; camera cloud retained",
    file.c_str(),config.clearance_m,
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  return region;
}
}
