#pragma once

#include <string>
#include <vector>

namespace uav_map_tools {
struct Vec3 { double x{}, y{}, z{}; };
struct Obstacle {
  enum class Kind { kBox, kCylinder, kSphere };
  std::string name;
  Kind kind{Kind::kBox};
  Vec3 center{}, size{};
  double yaw_rad{}, radius_m{};
};

// Static inline SDF collisions, box/cylinder/sphere, yaw-only poses in radians.
// Included iris UAVs are ignored; other unresolved includes/geometry are errors.
std::vector<Obstacle> LoadSdfGeometry(const std::string& path);
bool IntersectsVoxel(const Obstacle& obstacle, Vec3 center, double resolution_m);
}  // namespace uav_map_tools
