#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "uav_navigation_core/frame_math.hpp"
#include "uav_navigation_core/command_conditioner.hpp"
#include "uav_navigation_core/mppi/dynamics.hpp"
#include "uav_navigation_core/metric_depth.hpp"
#include "uav_navigation_core/octomap_planner.hpp"
#include "uav_navigation_core/trajectory_safety_checker.hpp"
namespace c = uav_navigation_core;
void Require(bool ok,const char *reason) {if(!ok) throw std::runtime_error(reason);}
int main() {
  try {
    c::VelocityCommandConditionerConfig limits;limits.max_speed_xy_m_s=.5;
    c::VelocityCommandConditioner conditioner(limits);c::State measured;
    const c::Control diagonal{{.5,.5,0},0};
    for(int i=0;i<30;++i) {
      const auto applied=conditioner.Apply(measured,diagonal);
      Require(std::hypot(applied.velocity_enu_m_s.x,applied.velocity_enu_m_s.y)<=.5+1e-12,
              "diagonal command must respect XY speed norm");
    }
    c::mppi::MppiDynamicsConfig dynamics;dynamics.vmax_m_s=.5;
    const c::mppi::MultirotorMotionModel model(dynamics);c::mppi::MppiState predicted;
    for(int i=0;i<30;++i) predicted=model.Step(predicted,diagonal,.1);
    Require(std::hypot(predicted.applied_control.velocity_enu_m_s.x,
                       predicted.applied_control.velocity_enu_m_s.y)<=.5+1e-12,
            "predicted actuator limit must match published limit");
    const double h=std::sqrt(.5);
    const auto v=c::RotateBodyToEnu({1,0,0},{0,0,h,h});
    Require(std::abs(v.x)<1e-9 && std::abs(v.y-1)<1e-9,"body velocity must rotate with yaw");
    bool rejected=false;
    try {c::NormalizeQuaternion({0,0,0,0});} catch(const std::invalid_argument&) {rejected=true;}
    Require(rejected,"invalid EKF orientation must be rejected");
    const float d[]={4,4,4,4};
    auto cloud=c::MetricDepthToPoints(d,2,2,1,{2,4,.5,.5});
    Require(cloud.size()==4 && cloud[0].x==-1 && cloud[0].y==-.5 && cloud[0].z==4,
            "backprojection must use calibrated fx/fy and optical axes");
    const float invalid[]={std::numeric_limits<float>::quiet_NaN(),0,40,4};
    Require(c::MetricDepthToPoints(invalid,2,2,1,{2,4,.5,.5}).size()==1,
            "invalid pixels must not create obstacle/free rays");
    octomap::OcTree map(.5);
    c::IntegrateOctomapCloud(map,{.25,.25,.25},{{3.25,.25,.25}});
    Require(map.search(3.25,.25,.25) && map.isNodeOccupied(map.search(3.25,.25,.25)),"ray endpoint occupied");
    Require(map.search(1.25,.25,.25) && !map.isNodeOccupied(map.search(1.25,.25,.25)),"ray interior free");
    c::MapBounds3D bounds{{0,0,0},{5,3,3}};
    c::OctomapPlanConfig config;config.clearance_m=0;
    Require(c::PlanOctomap3D(map,bounds,{.25,.25,.25},{4.25,1.25,.25},config).status!=c::StatusCode::kOk,
            "A* must reject an unknown goal");
    c::TrajectorySafetyConfig safety_config;
    safety_config.collision_radius_m=.5;safety_config.stopping_clearance_m=.5;
    const c::TrajectorySafetyChecker checker(safety_config);
    c::State start;start.position_enu_m={0,0,3};
    c::Trajectory t;t.points={{0,start,{}}};
    auto end=start;end.position_enu_m={2,0,3};t.points.push_back({1,end,{}});
    c::ObstacleMap obstacles;obstacles.observation_valid=true;obstacles.points_enu_m={{1,0,3}};
    Require(!checker.Evaluate(start,t,obstacles).safe,"swept collision between endpoints must fail");
    std::cout<<"Runtime geometry and safety checks passed\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
