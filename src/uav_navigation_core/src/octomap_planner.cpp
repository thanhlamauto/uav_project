#include "uav_navigation_core/octomap_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace uav_navigation_core {
namespace {
struct Cell { int x, y, z; };
struct CellHash {
  std::size_t operator()(Cell c) const {
    std::size_t h = std::hash<int>{}(c.x);
    h ^= std::hash<int>{}(c.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(c.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
bool Finite(Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
double Distance(Vec3 a, Vec3 b) {
  return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}
Cell ToCell(Vec3 p, const MapBounds3D& b, double r) {
  return {static_cast<int>(std::floor((p.x-b.min.x)/r)),
          static_cast<int>(std::floor((p.y-b.min.y)/r)),
          static_cast<int>(std::floor((p.z-b.min.z)/r))};
}
Vec3 Center(Cell c, const MapBounds3D& b, double r) {
  return {b.min.x+(c.x+0.5)*r, b.min.y+(c.y+0.5)*r, b.min.z+(c.z+0.5)*r};
}
bool CellFree(const octomap::OcTree& map, const MapBounds3D& bounds,
              Vec3 p, double clearance, bool allow_unknown) {
  if (!bounds.Contains(p)) return false;
  // Inflate the vehicle footprint, including half a voxel for rasterization.
  const double r = map.getResolution();
  const double radius = clearance + std::sqrt(3.0)*r/2.0;
  const int steps = static_cast<int>(std::ceil(radius/r));
  for (int dz=-steps; dz<=steps; ++dz)
    for (int dy=-steps; dy<=steps; ++dy)
      for (int dx=-steps; dx<=steps; ++dx) {
        const double d = r*std::sqrt(double(dx*dx+dy*dy+dz*dz));
        if (d > radius + 1e-9) continue;
        Vec3 q{p.x+dx*r,p.y+dy*r,p.z+dz*r};
        if (!bounds.Contains(q)) continue;  // outside ROI is not a flyable center
        const auto* node = map.search(q.x,q.y,q.z);
        if ((!node && !allow_unknown) || (node && map.isNodeOccupied(node))) return false;
      }
  return true;
}
struct Record { double g{std::numeric_limits<double>::infinity()}; Cell parent{}; bool parent_valid{false}; bool closed{false}; };
struct Item { double f, g; Cell cell; };
struct Greater { bool operator()(const Item& a, const Item& b) const { return a.f > b.f; } };
}  // namespace

bool MapBounds3D::Contains(const Vec3& p) const {
  return Finite(p) && p.x >= min.x && p.y >= min.y && p.z >= min.z &&
         p.x < max.x && p.y < max.y && p.z < max.z;
}

bool OctomapSegmentFree(const octomap::OcTree& map, const MapBounds3D& bounds,
                        const Vec3& from, const Vec3& to, double clearance,
                        bool allow_unknown) {
  if (!Finite(from) || !Finite(to) || clearance < 0) return false;
  const int steps = std::max(1, static_cast<int>(std::ceil(
      Distance(from,to)/(map.getResolution()/3.0))));
  for (int i=0; i<=steps; ++i) {
    const double t = double(i)/steps;
    if (!CellFree(map,bounds,{from.x+t*(to.x-from.x),from.y+t*(to.y-from.y),
                               from.z+t*(to.z-from.z)},clearance,allow_unknown)) return false;
  }
  return true;
}

OctomapPlan PlanOctomap3D(const octomap::OcTree& map, const MapBounds3D& bounds,
                          const Vec3& start, const Vec3& goal, OctomapPlanConfig config) {
  OctomapPlan result;
  const double r=map.getResolution();
  if (!bounds.Contains(start) || !bounds.Contains(goal) || r<=0 ||
      config.clearance_m<0 || config.max_expansions==0) {
    result.status=StatusCode::kInvalidInput;
    result.detail="invalid bounds, endpoints, or config";
    return result;
  }
  if (!CellFree(map,bounds,start,config.clearance_m,config.allow_unknown) ||
      !CellFree(map,bounds,goal,config.clearance_m,config.allow_unknown)) {
    result.detail="start or goal is occupied/unknown at required clearance";
    return result;
  }
  const Cell s=ToCell(start,bounds,r), g=ToCell(goal,bounds,r);
  std::unordered_map<Cell,Record,CellHash> records;
  std::unordered_map<Cell,bool,CellHash> safe_cache;
  auto safe=[&](Cell c) {
    auto it=safe_cache.find(c);
    if (it!=safe_cache.end()) return it->second;
    const auto p=Center(c,bounds,r);
    return safe_cache.emplace(c,CellFree(map,bounds,p,config.clearance_m,
                                          config.allow_unknown)).first->second;
  };
  std::priority_queue<Item,std::vector<Item>,Greater> open;
  records[s].g=0;
  open.push({Distance(Center(s,bounds,r),Center(g,bounds,r)),0,s});
  bool found=false;
  while (!open.empty()) {
    const auto current=open.top(); open.pop();
    auto& rec=records[current.cell];
    if (rec.closed || current.g>rec.g+1e-9) continue;
    rec.closed=true;
    if (++result.expanded>config.max_expansions) {
      result.detail="A* exceeded max_expansions"; return result;
    }
    if (current.cell==g) { found=true; break; }
    for (int dz=-1;dz<=1;++dz) for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
      if (dx==0 && dy==0 && dz==0) continue;
      Cell next{current.cell.x+dx,current.cell.y+dy,current.cell.z+dz};
      if (!safe(next)) continue;
      // A diagonal move may not squeeze between occupied/unknown face cells.
      bool corner=false;
      if (dx && !safe({current.cell.x+dx,current.cell.y,current.cell.z})) corner=true;
      if (dy && !safe({current.cell.x,current.cell.y+dy,current.cell.z})) corner=true;
      if (dz && !safe({current.cell.x,current.cell.y,current.cell.z+dz})) corner=true;
      if (corner) continue;
      if ((std::abs(dx)+std::abs(dy)+std::abs(dz))>1 &&
          !OctomapSegmentFree(map,bounds,Center(current.cell,bounds,r),
                              Center(next,bounds,r),config.clearance_m,
                              config.allow_unknown)) continue;
      const double ng=rec.g+r*std::sqrt(double(dx*dx+dy*dy+dz*dz));
      auto& nr=records[next];
      if (nr.closed || ng>=nr.g) continue;
      nr.g=ng; nr.parent=current.cell; nr.parent_valid=true;
      open.push({ng+Distance(Center(next,bounds,r),Center(g,bounds,r)),ng,next});
    }
  }
  if (!found) { result.detail="no observed-free 3D path"; return result; }
  std::vector<Cell> cells;
  for (Cell c=g;;) {
    cells.push_back(c);
    if (c==s) break;
    const auto it=records.find(c);
    if (it==records.end() || !it->second.parent_valid) {
      result.status=StatusCode::kInternalError; result.detail="broken predecessor chain"; return result;
    }
    c=it->second.parent;
  }
  std::reverse(cells.begin(),cells.end());
  result.path.points_enu_m.push_back(start);
  for (const Cell c:cells) {
    const auto p=Center(c,bounds,r);
    if (Distance(result.path.points_enu_m.back(),p)>1e-9)
      result.path.points_enu_m.push_back(p);
  }
  result.path.points_enu_m.push_back(goal);
  // Endpoint connectors also have to satisfy clearance; otherwise no path is published.
  if (!OctomapSegmentFree(map,bounds,result.path.points_enu_m[0],result.path.points_enu_m[1],
                          config.clearance_m,config.allow_unknown) ||
      !OctomapSegmentFree(map,bounds,result.path.points_enu_m[result.path.points_enu_m.size()-2],
                          goal,config.clearance_m,config.allow_unknown)) {
    result.path.points_enu_m.clear(); result.detail="unsafe endpoint connector"; return result;
  }
  result.status=StatusCode::kOk;
  result.detail="3D A* path through known free voxels";
  return result;
}

void IntegrateOctomapCloud(octomap::OcTree& map, const Vec3& sensor_origin,
                           const std::vector<Vec3>& endpoints, double max_range_m) {
  if (!Finite(sensor_origin) || max_range_m<=0) throw std::invalid_argument("invalid sensor origin/range");
  octomap::Pointcloud cloud;
  for (const auto& p:endpoints)
    if (Finite(p) && Distance(sensor_origin,p)>0.05)
      cloud.push_back(p.x,p.y,p.z);
  if (cloud.size()==0) return;
  map.insertPointCloud(cloud,octomap::point3d(sensor_origin.x,sensor_origin.y,sensor_origin.z),
                       max_range_m,false,true);
  map.updateInnerOccupancy();
}

}  // namespace uav_navigation_core
