#include "uav_navigation_core/static_octomap_planner.hpp"
#include <algorithm>
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
struct Record { double g{std::numeric_limits<double>::infinity()}; Cell parent{}; bool parent_valid{false}; bool closed{false}; };
struct Item { double f, g; Cell cell; };
struct Greater { bool operator()(const Item& a, const Item& b) const { return a.f > b.f; } };
}  // namespace

class StaticOctomapPlanner::Impl {
 public:
  const octomap::OcTree map;
  const MapBounds3D bounds;
  const OctomapPlanConfig config;
  int nx,ny,nz;
  std::vector<unsigned char> free;
  Impl(const octomap::OcTree& tree,MapBounds3D b,OctomapPlanConfig c)
      :map(tree),bounds(b),config(c) {
    const double r=map.getResolution();
    if (!Finite(b.min) || !Finite(b.max) || !std::isfinite(c.clearance_m) ||
        c.clearance_m<0 || c.max_expansions==0)
      throw std::invalid_argument("invalid static prior bounds/config");
    for (double v:{b.min.x,b.min.y,b.min.z,b.max.x,b.max.y,b.max.z})
      if (std::abs(v/r-std::round(v/r))>1e-8)
        throw std::invalid_argument("static cache requires voxel-aligned bounds");
    nx=std::lround((b.max.x-b.min.x)/r);
    ny=std::lround((b.max.y-b.min.y)/r);
    nz=std::lround((b.max.z-b.min.z)/r);
    if (nx<=0 || ny<=0 || nz<=0 || nx*double(ny)*nz>1000000)
      throw std::invalid_argument("invalid static cache dimensions");
    free.resize(nx*ny*nz);
    for (int z=0;z<nz;++z) for(int y=0;y<ny;++y) for(int x=0;x<nx;++x) {
      const Cell cell{x,y,z};const auto p=Center(cell,bounds,r);
      free[Index(cell)]=OctomapSegmentFree(map,bounds,p,p,c.clearance_m,c.allow_unknown);
    }
  }
  std::size_t Index(Cell c) const {return (c.z*ny+c.y)*nx+c.x;}
  bool Safe(Cell c) const {
    return c.x>=0 && c.y>=0 && c.z>=0 && c.x<nx && c.y<ny && c.z<nz && free[Index(c)];
  }
  // Supercover voxel traversal: visit every crossed cell, including the face
  // cells touched by a diagonal corner. Uniform point samples can miss a short
  // intersection. With aligned bounds, CellFree's integer-resolution offsets
  // address the same OctoMap keys everywhere inside each cached voxel.
  bool RegionSegment(Vec3 a,Vec3 b) const {
    if (!bounds.Contains(a) || !bounds.Contains(b)) return false;
    const double r=map.getResolution();
    Cell cell=ToCell(a,bounds,r), end=ToCell(b,bounds,r);
    if (!Safe(cell) || !Safe(end)) return false;
    const double origin[3]={a.x,a.y,a.z};
    const double delta[3]={b.x-a.x,b.y-a.y,b.z-a.z};
    const double lo[3]={bounds.min.x,bounds.min.y,bounds.min.z};
    int coord[3]={cell.x,cell.y,cell.z}, step[3];
    double next[3], increment[3];
    for (int axis=0;axis<3;++axis) {
      step[axis]=(delta[axis]>0)-(delta[axis]<0);
      if (!step[axis]) {
        next[axis]=increment[axis]=std::numeric_limits<double>::infinity();
      } else {
        const double boundary=lo[axis]+(coord[axis]+(step[axis]>0))*r;
        next[axis]=(boundary-origin[axis])/delta[axis];
        increment[axis]=r/std::abs(delta[axis]);
      }
    }
    for (int count=0;count<=nx+ny+nz+3;++count) {
      const double t=std::min({next[0],next[1],next[2]});
      if (t>1 || !std::isfinite(t)) return true;
      int crossed=0;
      for (int axis=0;axis<3;++axis)
        if (std::abs(next[axis]-t)<1e-12) crossed|=1<<axis;
      for (int mask=1;mask<8;++mask) {
        if ((mask&crossed)!=mask) continue;
        int c[3]={coord[0],coord[1],coord[2]};
        for (int axis=0;axis<3;++axis) if(mask&(1<<axis)) c[axis]+=step[axis];
        if (!Safe({c[0],c[1],c[2]})) return false;
      }
      for (int axis=0;axis<3;++axis) if(crossed&(1<<axis)) {
        coord[axis]+=step[axis]; next[axis]+=increment[axis];
      }
    }
    return false;
  }
  // Preserve the validated A* search/tie-breaking behavior. The MPPI guard
  // uses the conservative supercover traversal above for continuous motion.
  bool Segment(Vec3 a,Vec3 b) const {
    const double r=map.getResolution();
    const int n=std::max(1,int(std::ceil(Distance(a,b)/(r/3))));
    for(int i=0;i<=n;++i) {
      const double t=double(i)/n;
      Vec3 p{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};
      if (!bounds.Contains(p) || !Safe(ToCell(p,bounds,r))) return false;
    }
    return true;
  }
  OctomapPlan Plan(const Vec3& start,const Vec3& goal) {
  OctomapPlan result;
  const double r=map.getResolution();
  if (!bounds.Contains(start) || !bounds.Contains(goal) || r<=0 ||
      config.clearance_m<0 || config.max_expansions==0) {
    result.status=StatusCode::kInvalidInput;
    result.detail="invalid bounds, endpoints, or config";
    return result;
  }
  if (!OctomapSegmentFree(map,bounds,start,start,config.clearance_m,config.allow_unknown) ||
      !OctomapSegmentFree(map,bounds,goal,goal,config.clearance_m,config.allow_unknown)) {
    result.detail="start or goal is occupied/unknown at required clearance";
    return result;
  }
  const Cell s=ToCell(start,bounds,r), g=ToCell(goal,bounds,r);
  std::unordered_map<Cell,Record,CellHash> records;
  auto safe=[&](Cell c) { return Safe(c); };
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
          !Segment(Center(current.cell,bounds,r),Center(next,bounds,r))) continue;
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
  for (std::size_t i=1;i<result.path.points_enu_m.size();++i) {
    if (!OctomapSegmentFree(map,bounds,result.path.points_enu_m[i-1],
                            result.path.points_enu_m[i],config.clearance_m,config.allow_unknown)) {
      result.path.points_enu_m.clear();
      result.detail="cached route failed original OctoMap clearance check";
      return result;
    }
  }
  result.status=StatusCode::kOk;
  result.detail="3D A* path through known free voxels";
  return result;
}
};
StaticOctomapPlanner::StaticOctomapPlanner(const octomap::OcTree& m,MapBounds3D b,OctomapPlanConfig c)
    :impl_(std::make_unique<Impl>(m,b,c)) {}
StaticOctomapPlanner::~StaticOctomapPlanner()=default;
bool StaticOctomapPlanner::SegmentAllowed(const Vec3& a,const Vec3& b) const {
  return impl_->RegionSegment(a,b);
}
OctomapPlan StaticOctomapPlanner::Plan(const Vec3& s,const Vec3& g) {return impl_->Plan(s,g);}
} // namespace uav_navigation_core
