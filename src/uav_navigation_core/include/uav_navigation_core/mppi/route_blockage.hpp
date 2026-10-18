#pragma once

#include <vector>

#include "uav_navigation_core/mppi/path_reference.hpp"

namespace uav_navigation_core::mppi {

// True when a recent occupied point intersects the safety corridor of the
// reference path ahead of the robot. This is a route validity check, not a
// collision-free local replanner or a free-space inference.
bool RouteBlockedByCloud(const PathReference &path, const Vec3 &position,
                         const std::vector<Vec3> &occupied_points,
                         double lookahead_m, double corridor_radius_m,
                         double sample_spacing_m = 0.25);

} // namespace uav_navigation_core::mppi
