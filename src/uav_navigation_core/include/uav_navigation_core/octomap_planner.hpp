#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include "uav_navigation_core/types.hpp"

namespace uav_navigation_core {

struct MapBounds3D {
  Vec3 min{};
  Vec3 max{};
  bool Contains(const Vec3& point) const;
};

struct OctomapPlanConfig {
  double clearance_m{0.75};
  std::size_t max_expansions{250000};
  // Unknown is never interpreted as observed free.
  bool allow_unknown{false};
};

struct OctomapPlan {
  StatusCode status{StatusCode::kNoPath};
  Path path{};
  std::size_t expanded{0};
  std::string detail;
};

// OctoMap is the occupancy source. The search itself is a 26-connected 3D A*.
OctomapPlan PlanOctomap3D(const octomap::OcTree& map,
                          const MapBounds3D& bounds,
                          const Vec3& start, const Vec3& goal,
                          OctomapPlanConfig config = {});

bool OctomapSegmentFree(const octomap::OcTree& map,
                        const MapBounds3D& bounds,
                        const Vec3& from, const Vec3& to,
                        double clearance_m, bool allow_unknown = false);

void IntegrateOctomapCloud(octomap::OcTree& map, const Vec3& sensor_origin,
                           const std::vector<Vec3>& endpoints,
                           double max_range_m = 30.0);

}  // namespace uav_navigation_core
