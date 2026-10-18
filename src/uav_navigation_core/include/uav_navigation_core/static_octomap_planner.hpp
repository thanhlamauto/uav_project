#pragma once
#include <memory>
#include "uav_navigation_core/flight_region.hpp"
#include "uav_navigation_core/octomap_planner.hpp"

namespace uav_navigation_core {
// Immutable, voxel-aligned prior. Inflation is computed once; search remains
// 26-connected A*. Every returned segment is checked against the original tree.
class StaticOctomapPlanner final : public IFlightRegion {
 public:
  StaticOctomapPlanner(const octomap::OcTree&, MapBounds3D, OctomapPlanConfig);
  ~StaticOctomapPlanner();
  bool SegmentAllowed(const Vec3& from, const Vec3& to) const override;
  OctomapPlan Plan(const Vec3& start, const Vec3& goal);
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
