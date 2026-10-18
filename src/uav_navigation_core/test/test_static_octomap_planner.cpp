#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include "uav_navigation_core/static_octomap_planner.hpp"

namespace c=uav_navigation_core;
double Length(const c::Path& p) {
  double v=0;
  for(std::size_t i=1;i<p.points_enu_m.size();++i) {
    const auto a=p.points_enu_m[i-1],b=p.points_enu_m[i];
    v+=std::sqrt(std::pow(a.x-b.x,2)+std::pow(a.y-b.y,2)+std::pow(a.z-b.z,2));
  }
  return v;
}
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  octomap::OcTree map(.5);
  if(!map.readBinary(argv[1])) return 2;
  c::MapBounds3D bounds{{-4,-12,1},{26,12,9}};
  c::OctomapPlanConfig cfg;cfg.clearance_m=1;
  const auto begin=std::chrono::steady_clock::now();
  c::StaticOctomapPlanner cache(map,bounds,cfg);
  const double setup=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::mt19937 rng(73);std::uniform_real_distribution<double> x(-3,25),y(-11,11),z(1.1,8.9);
  std::size_t success=0;double worst=0;
  for(int i=0;i<40;++i) {
    c::Vec3 s{i==0 ? 0.0:x(rng),i==0 ? 0.0:y(rng),i==0 ? 3.2:z(rng)};
    const c::Vec3 g{20,4,4};
    const auto old=c::PlanOctomap3D(map,bounds,s,g,cfg);
    const auto t=std::chrono::steady_clock::now();
    const auto next=cache.Plan(s,g);
    worst=std::max(worst,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count());
    if(old.status!=next.status || std::abs(Length(old.path)-Length(next.path))>1e-6)
      throw std::runtime_error("cached and reference A* disagree at case "+std::to_string(i));
    for(std::size_t k=1;k<next.path.points_enu_m.size();++k)
      if(!c::OctomapSegmentFree(map,bounds,next.path.points_enu_m[k-1],next.path.points_enu_m[k],1))
        throw std::runtime_error("unsafe cached edge");
    success+=next.status==c::StatusCode::kOk;
  }
  // A mutation of the caller's tree must not change the immutable cache.
  const c::Vec3 s{0,0,3.2},g{20,4,4};
  auto before=cache.Plan(s,g);map.updateNode(0,0,3.2,true);
  const auto after=cache.Plan(s,g);
  if(before.status!=after.status || Length(before.path)!=Length(after.path))
    throw std::runtime_error("static map changed through caller alias");
  std::cout<<"cases=40 successful="<<success<<" parity=PASS setup_ms="<<setup
           <<" cached_plan_max_ms="<<worst<<" immutable_copy=PASS\n";
}
