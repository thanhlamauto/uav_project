#pragma once
#include "uav_navigation_core/types.hpp"
namespace uav_navigation_core {
// Feasibility of the vehicle center after map inflation. This is a hard
// constraint, not a distance field; it never filters camera obstacles.
class IFlightRegion {
 public:
  virtual ~IFlightRegion() = default;
  virtual bool SegmentAllowed(const Vec3& from, const Vec3& to) const = 0;
};
}
