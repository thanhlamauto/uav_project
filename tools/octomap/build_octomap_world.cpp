// Offline preparation: SDF collision geometry -> full.bt and prior.bt.
// No ROS node, simulator process, or dependency on files outside this project.
#include "sdf_geometry.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <octomap/OcTree.h>

namespace map = uav_map_tools;
namespace fs = std::filesystem;
namespace {
struct Options {
  std::string world, output;
  map::Vec3 minimum{}, maximum{};
  bool have_min{false}, have_max{false};
  double resolution{.5}, hidden_fraction{0};
  std::uint32_t seed{7};
};
void Usage() {
  std::cout << "Usage: build_octomap_world --world WORLD.sdf --output DIRECTORY\n"
      "  --min X Y Z --max X Y Z [--resolution 0.5]\n"
      "  [--hidden-fraction 0.0] [--seed 7]\n"
      "Bounds must match runtime map_min/map_max and align to voxel resolution.\n"
      "Writes full.bt and prior.bt; hidden prior collisions are assumed FREE, not unknown.\n";
}
double Number(const std::string& text) {
  std::size_t used=0;
  const double value=std::stod(text,&used);
  if (used!=text.size() || !std::isfinite(value))
    throw std::invalid_argument("expected a finite number: " + text);
  return value;
}
Options Parse(int argc, char** argv) {
  Options o;
  for (int i=1;i<argc;++i) {
    const std::string arg=argv[i];
    const auto next=[&]() -> std::string {
      if (++i>=argc) throw std::invalid_argument("missing value after " + arg);
      return argv[i];
    };
    if (arg=="--world") o.world=next();
    else if (arg=="--output") o.output=next();
    else if (arg=="--min" || arg=="--max") {
      const double x=Number(next()), y=Number(next()), z=Number(next());
      if (arg=="--min") {o.minimum={x,y,z};o.have_min=true;}
      else {o.maximum={x,y,z};o.have_max=true;}
    } else if (arg=="--resolution") o.resolution=Number(next());
    else if (arg=="--hidden-fraction") o.hidden_fraction=Number(next());
    else if (arg=="--seed") {
      const auto text=next();
      std::size_t used=0;
      const auto seed=std::stoull(text,&used);
      if (used!=text.size() || text.front()=='-' || seed>std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("seed must be a uint32 integer");
      o.seed=static_cast<std::uint32_t>(seed);
    } else throw std::invalid_argument("unknown option: " + arg);
  }
  if (o.world.empty() || o.output.empty() || !o.have_min || !o.have_max)
    throw std::invalid_argument("--world, --output, --min and --max are required");
  if (o.resolution<=0 || o.hidden_fraction<0 || o.hidden_fraction>1)
    throw std::invalid_argument("resolution must be positive; hidden-fraction must be 0..1");
  return o;
}
std::array<int,3> GridSize(const Options& o) {
  const double lo[]{o.minimum.x,o.minimum.y,o.minimum.z};
  const double hi[]{o.maximum.x,o.maximum.y,o.maximum.z};
  std::array<int,3> count{};
  double cells=1;
  for (int i=0;i<3;++i) {
    const double n=(hi[i]-lo[i])/o.resolution;
    const double origin=lo[i]/o.resolution;
    if (!std::isfinite(n) || !std::isfinite(origin) || n<1 || n>5000000 ||
        std::abs(n-std::round(n))>1e-6 || std::abs(origin-std::round(origin))>1e-6)
      throw std::invalid_argument("min/max must increase and align to voxel resolution");
    count[i]=static_cast<int>(std::round(n));
    cells*=count[i];
  }
  if (cells>5000000) throw std::invalid_argument("ROI exceeds 5 million voxels; reduce bounds or increase resolution");
  return count;
}
bool Hidden(const std::string& name, const Options& o) {
  // Same stable FNV-based selection as the original preparation tool.
  std::uint32_t hash=2166136261u ^ o.seed;
  for (unsigned char c:name) {hash^=c;hash*=16777619u;}
  return double(hash%1000000u)/1000000.0<o.hidden_fraction;
}
template<class Callback>
void ForEachVoxel(const Options& o, const std::array<int,3>& count, Callback callback) {
  for (int z=0;z<count[2];++z)
    for (int y=0;y<count[1];++y)
      for (int x=0;x<count[0];++x)
        callback(map::Vec3{o.minimum.x+(x+.5)*o.resolution,
                           o.minimum.y+(y+.5)*o.resolution,
                           o.minimum.z+(z+.5)*o.resolution});
}
void Build(const Options& o) {
  const auto count=GridSize(o);
  const auto obstacles=map::LoadSdfGeometry(o.world);
  octomap::OcTree full(o.resolution), prior(o.resolution);
  octomap::OcTreeKey key;
  if (!full.coordToKeyChecked(o.minimum.x,o.minimum.y,o.minimum.z,key) ||
      !full.coordToKeyChecked(o.maximum.x-o.resolution/2,
                             o.maximum.y-o.resolution/2,o.maximum.z-o.resolution/2,key))
    throw std::invalid_argument("bounds exceed OctoMap coordinate range");
  // A simulation prior assumes known free space throughout this ROI.
  ForEachVoxel(o,count,[&](map::Vec3 p) {
    full.updateNode(octomap::point3d(p.x,p.y,p.z),false,true);
    prior.updateNode(octomap::point3d(p.x,p.y,p.z),false,true);
  });
  std::size_t hidden=0;
  for (const auto& obstacle:obstacles) {
    const bool omit=Hidden(obstacle.name,o);
    hidden+=omit;
    ForEachVoxel(o,count,[&](map::Vec3 p) {
      if (!map::IntersectsVoxel(obstacle,p,o.resolution)) return;
      const octomap::point3d point(p.x,p.y,p.z);
      for (int vote=0;vote<4;++vote) {
        full.updateNode(point,true,true);
        if (!omit) prior.updateNode(point,true,true);
      }
    });
  }
  full.updateInnerOccupancy();
  prior.updateInnerOccupancy();
  fs::create_directories(o.output);
  const auto full_path=fs::path(o.output)/"full.bt", prior_path=fs::path(o.output)/"prior.bt";
  if (!full.writeBinary(full_path.string()) || !prior.writeBinary(prior_path.string()))
    throw std::runtime_error("cannot write output OctoMap files");
  std::cout << "full_map=" << fs::absolute(full_path) << "\nprior_map=" << fs::absolute(prior_path)
            << "\nresolution_m=" << o.resolution
            << "\nbounds_min=[" << o.minimum.x << ',' << o.minimum.y << ',' << o.minimum.z << ']'
            << "\nbounds_max=[" << o.maximum.x << ',' << o.maximum.y << ',' << o.maximum.z << ']'
            << "\nstatic_collisions=" << obstacles.size() << " hidden_collisions=" << hidden
            << "\nhidden_fraction=" << o.hidden_fraction << " seed=" << o.seed << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  if (argc==2 && std::string(argv[1])=="--help") {Usage();return 0;}
  try {Build(Parse(argc,argv));return 0;}
  catch (const std::exception& error) {std::cerr << "OctoMap preparation failed: " << error.what() << '\n';return 1;}
}
