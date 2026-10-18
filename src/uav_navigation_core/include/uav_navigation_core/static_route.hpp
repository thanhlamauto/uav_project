#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include "uav_navigation_core/flight_region.hpp"
namespace uav_navigation_core {
// A prior route does not expire simply because A* finds another equal-cost
// voxel route. Retain only when the current center can safely join it nearby.
inline bool CanRetainStaticRoute(const IFlightRegion& region, const Path& path,
                                const Vec3& p, double max_connector_m) {
  if (path.points_enu_m.size()<2 || !std::isfinite(max_connector_m) || max_connector_m<0)
    return false;
  double best=std::numeric_limits<double>::infinity(); Vec3 join{};
  for(std::size_t i=1;i<path.points_enu_m.size();++i) {
    const auto a=path.points_enu_m[i-1],b=path.points_enu_m[i];
    const Vec3 d{b.x-a.x,b.y-a.y,b.z-a.z};
    const double n=d.x*d.x+d.y*d.y+d.z*d.z;
    const double t=std::clamp(((p.x-a.x)*d.x+(p.y-a.y)*d.y+(p.z-a.z)*d.z)/std::max(n,1e-12),0.0,1.0);
    const Vec3 q{a.x+t*d.x,a.y+t*d.y,a.z+t*d.z};
    const double e=(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z);
    if(e<best){best=e;join=q;}
  }
  return best<=max_connector_m*max_connector_m && region.SegmentAllowed(p,join);
}
}
