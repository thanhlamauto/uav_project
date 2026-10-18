#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include "uav_navigation_core/static_octomap_planner.hpp"
#include "uav_navigation_core/static_route.hpp"
#include "uav_navigation_core/trajectory_safety_checker.hpp"
namespace c=uav_navigation_core;
void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
c::State State(c::Vec3 p,c::Vec3 v={}) { c::State s;s.position_enu_m=p;s.velocity_enu_m_s=v;return s; }
c::Trajectory Path(c::State a,c::State b) { c::Trajectory t;t.points={{0,a,{}},{.1,b,{}}};return t; }
int main(int argc,char** argv) {
 try {
  octomap::OcTree map(.5);
  c::MapBounds3D bounds{{-4,-4,0},{8,8,6}};
  for(double z=.25;z<6;z+=.5) for(double y=-3.75;y<8;y+=.5)
    for(double x=-3.75;x<8;x+=.5) map.updateNode(x,y,z,false);
  map.updateNode(.25,.25,2.25,true);
  c::OctomapPlanConfig cfg;cfg.clearance_m=1;
  auto region=std::make_shared<c::StaticOctomapPlanner>(map,bounds,cfg);
  c::TrajectorySafetyConfig sc;sc.collision_radius_m=.2;sc.stopping_clearance_m=.2;
  c::TrajectorySafetyChecker guarded(sc,nullptr,region),cloud_only(sc);
  c::ObstacleMap cloud;cloud.observation_valid=true;cloud.points_enu_m={{100,100,100}};
  auto a=State({-3,.25,2.25}),b=State({3,.25,2.25});
  Check(region->SegmentAllowed(a.position_enu_m,a.position_enu_m),"valid start");
  Check(region->SegmentAllowed(b.position_enu_m,b.position_enu_m),"valid end");
  auto t=Path(a,b);
  Check(cloud_only.Evaluate(a,t,cloud).safe,"reproduce cloud-only blind spot");
  Check(guarded.Evaluate(a,t,cloud).reason=="prior_region_collision","swept prior collision");
  b=State({-2.5,.25,2.25});t=Path(a,b);
  Check(guarded.Evaluate(a,t,cloud).safe,"known-free trajectory must pass");
  cloud.points_enu_m={{-2.75,.25,2.25}};
  Check(!guarded.Evaluate(a,t,cloud).safe,"new camera obstacle must remain active");
  cloud.points_enu_m={{100,100,100}};
  a=State({-1.25,.25,2.25},{1,0,0});b=a;t=Path(a,b);
  Check(region->SegmentAllowed(a.position_enu_m,b.position_enu_m),"rollout before stopping is free");
  Check(guarded.Evaluate(a,t,cloud).reason=="prior_region_stopping_clearance","braking tail hits buffer");
  // A trajectory cannot teleport away from an invalid measured initial state.
  a=State({.25,.25,2.25});b=State({-3,.25,2.25});t=Path(b,b);
  Check(!guarded.Evaluate(a,t,cloud).safe,"initial-to-first connector must be checked");
  Check(!region->SegmentAllowed({-4.01,0,2},{-3,0,2}),"bounds fail closed");
  Check(!region->SegmentAllowed({0,0,std::numeric_limits<double>::quiet_NaN()},{0,0,2}),"NaN fail closed");
  // A very short corner intersection can fall between regularly spaced samples.
  Check(!region->SegmentAllowed({-1.001,-.999,2.25},{-.999,-1.001,2.25}),"corner supercover");
  map.deleteNode(5.25,5.25,2.25);
  c::StaticOctomapPlanner unknown(map,bounds,cfg);
  Check(!unknown.SegmentAllowed({5.25,5.25,2.25},{5.25,5.25,2.25}),"unknown rejected");
  c::Path route;route.points_enu_m={{-2.25,-3.25,2.25},{-2.25,3.25,2.25}};
  Check(c::CanRetainStaticRoute(*region,route,{-2.5,0,2.25},.75),"safe nearby route retained");
  Check(!c::CanRetainStaticRoute(*region,route,{-3.5,0,2.25},.75),"far route rejected");
  Check(!c::CanRetainStaticRoute(*region,route,{2.25,0,2.25},6),"blocked connector rejected");
  Check(!c::CanRetainStaticRoute(*region,{},a.position_enu_m,.75),"empty route rejected");
  sc.braking_acceleration_m_s2=.4;
  c::TrajectorySafetyChecker conservative(sc,nullptr,region);
  a=State({-2.25,3.25,.25},{0,0,-.4});t=Path(a,a);
  Check(!conservative.Evaluate(a,t,cloud).safe,"weakest-axis brake must respect lower bound");
  if(argc==2) {
    octomap::OcTree forest(.5);Check(forest.readBinary(argv[1]),"forest prior readable");
    c::MapBounds3D fb{{-4,-12,1},{26,12,9}};
    c::StaticOctomapPlanner f(forest,fb,cfg);
    Check(!f.SegmentAllowed({4.022538395,4.362494550,2.049438168},
                           {4.022538395,4.362494550,2.049438168}),"recorded Astar failure must be rejected");
    std::mt19937 rng(73);std::uniform_real_distribution<double> x(-4,26),y(-12,12),z(1,9);
    const auto begin=std::chrono::steady_clock::now();
    for(int i=0;i<20000;++i) {
      const c::Vec3 p{x(rng),y(rng),z(rng)};
      Check(f.SegmentAllowed(p,p)==c::OctomapSegmentFree(forest,fb,p,p,1),"cache/exact point parity");
    }
    std::cout<<"forest_point_parity=20000 PASS exact_audit_ms="
      <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()<<'\n';
    std::size_t valid=0;
    const auto t0=std::chrono::steady_clock::now();
    for(int i=0;i<100000;++i) valid+=f.SegmentAllowed({-2.25,-9.25,4.25},{-1.75,-8.75,4.75});
    std::cout<<"segment_queries=100000 valid="<<valid<<" time_ms="
      <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count()<<'\n';
  }
  std::cout<<"prior swept/braking/initial/unknown/cloud checks PASS\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
