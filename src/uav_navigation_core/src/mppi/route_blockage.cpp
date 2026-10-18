#include "uav_navigation_core/mppi/route_blockage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav_navigation_core::mppi {
bool RouteBlockedByCloud(const PathReference &path, const Vec3 &position,
                         const std::vector<Vec3> &occupied_points,
                         double lookahead_m, double corridor_radius_m,
                         double sample_spacing_m) {
  if (!std::isfinite(lookahead_m) || lookahead_m <= 0.0 ||
      !std::isfinite(corridor_radius_m) || corridor_radius_m <= 0.0 ||
      !std::isfinite(sample_spacing_m) || sample_spacing_m <= 0.0)
    throw std::invalid_argument("invalid route blockage configuration");
  if (occupied_points.empty())
    return false;
  const double start = path.Progress(position);
  const double end = std::min(path.length_m(), start + lookahead_m);
  const double radius2 = corridor_radius_m * corridor_radius_m;
  const auto count = std::max(1, static_cast<int>(
      std::ceil((end - start) / sample_spacing_m)));
  for (int i = 0; i <= count; ++i) {
    const Vec3 target = path.Sample(
        start + (end - start) * static_cast<double>(i) / count);
    for (const auto &point : occupied_points) {
      const double dx = point.x - target.x;
      const double dy = point.y - target.y;
      const double dz = point.z - target.z;
      if (dx * dx + dy * dy + dz * dz <= radius2)
        return true;
    }
  }
  return false;
}
} // namespace uav_navigation_core::mppi
