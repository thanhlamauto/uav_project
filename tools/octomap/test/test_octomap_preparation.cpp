#include "sdf_geometry.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <octomap/OcTree.h>

namespace fs = std::filesystem;
namespace {
void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
bool Occupied(const octomap::OcTree& tree, double x, double y, double z) {
  const auto* node=tree.search(x,y,z);
  Check(node!=nullptr,"expected observed voxel");
  return tree.isNodeOccupied(node);
}
void RejectGeometry(const fs::path& file, const std::string& body) {
  {
    std::ofstream out(file);
    out << "<sdf version='1.9'><world name='test'>" << body << "</world></sdf>";
  }
  bool rejected=false;
  try {uav_map_tools::LoadSdfGeometry(file.string());}
  catch (const std::runtime_error&) {rejected=true;}
  Check(rejected,"unsupported geometry/frame must fail instead of becoming free");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    Check(argc==3,"fixture and output directory required");
    const auto obstacles=uav_map_tools::LoadSdfGeometry(argv[1]);
    Check(obstacles.size()==3,"three static collisions; dynamic UAV/model ignored");
    Check(std::abs(obstacles[0].center.x)<1e-9 &&
          std::abs(obstacles[0].center.y-1)<1e-9,"parent yaw rotates link translation");
    Check(uav_map_tools::IntersectsVoxel(obstacles[0],{0,2,2.5},.5),"rotated box rasterized");
    Check(!uav_map_tools::IntersectsVoxel(obstacles[0],{1.5,1,2.5},.5),"box yaw changes footprint");
    const fs::path output=argv[2];
    octomap::OcTree full(.1), prior(.1), partial_full(.1), partial_prior(.1);
    Check(full.readBinary((output/"full/full.bt").string()),"read full map");
    Check(prior.readBinary((output/"full/prior.bt").string()),"read complete prior");
    Check(partial_full.readBinary((output/"partial/full.bt").string()),"read partial-run GT");
    Check(partial_prior.readBinary((output/"partial/prior.bt").string()),"read incomplete prior");
    Check(std::abs(full.getResolution()-.5)<1e-9,"resolution survives .bt serialization");
    for (const auto& o:obstacles) {
      Check(Occupied(full,o.center.x,o.center.y,o.center.z),"full map contains each obstacle");
      Check(Occupied(prior,o.center.x,o.center.y,o.center.z),"default prior contains each obstacle");
      Check(Occupied(partial_full,o.center.x,o.center.y,o.center.z),"hiding never changes GT");
      Check(!Occupied(partial_prior,o.center.x,o.center.y,o.center.z),"hidden obstacle is assumed free");
    }
    Check(!Occupied(full,-3.75,-3.75,.25),"empty interior is known free");
    Check(full.search(8.25,0,2)==nullptr,"outside ROI stays unknown");
    RejectGeometry(output/"bad_include.sdf","<include><uri>model://unknown_building</uri></include>");
    RejectGeometry(output/"bad_mesh.sdf","<model name='m'><static>true</static><link name='l'>"
      "<collision name='c'><geometry><mesh/></geometry></collision></link></model>");
    RejectGeometry(output/"bad_pose.sdf","<model name='m'><static>true</static>"
      "<pose relative_to='other'>0 0 0 0 0 0</pose></model>");
    RejectGeometry(output/"bad_roll.sdf","<model name='m'><static>true</static>"
      "<pose>0 0 0 0.3 0 0</pose></model>");
    std::cout << "PASS: geometry, ROI, full/partial maps and unsupported-input rejection\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';return 1;
  }
}
