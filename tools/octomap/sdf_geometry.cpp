#include "sdf_geometry.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>

namespace uav_map_tools {
namespace {
using Element = tinyxml2::XMLElement;
struct Pose { Vec3 position{}; double yaw{}; };

std::string Text(const Element* e) {
  return e && e->GetText() ? e->GetText() : "";
}
std::string Name(const Element* e) {
  const char* name = e->Attribute("name");
  if (!name || !*name) throw std::runtime_error("SDF model/link/collision needs a name");
  return name;
}
template<std::size_t N>
std::array<double,N> Numbers(const Element* e) {
  std::array<double,N> values{};
  std::istringstream input(Text(e));
  for (auto& value : values)
    if (!(input >> value) || !std::isfinite(value))
      throw std::runtime_error("SDF geometry/pose needs finite numeric values");
  std::string extra;
  if (input >> extra) throw std::runtime_error("extra values in SDF geometry/pose");
  return values;
}
double Positive(const Element* e) {
  const double value = Numbers<1>(e)[0];
  if (value <= 0) throw std::runtime_error("SDF dimensions must be positive");
  return value;
}
Pose ReadPose(const Element* parent) {
  const auto* e = parent->FirstChildElement("pose");
  if (!e) return {};
  const char* relative = e->Attribute("relative_to");
  const char* format = e->Attribute("rotation_format");
  if ((relative && *relative) || (format && std::string(format) != "euler_rpy") ||
      (e->Attribute("degrees") && std::string(e->Attribute("degrees")) != "false"))
    throw std::runtime_error("only parent-relative Euler poses in radians are supported");
  const auto p = Numbers<6>(e);
  if (std::abs(p[3]) > 1e-9 || std::abs(p[4]) > 1e-9)
    throw std::runtime_error("SDF collision roll/pitch is unsupported; use yaw-only geometry");
  return {{p[0],p[1],p[2]},p[5]};
}
Pose Compose(const Pose& a, const Pose& b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {{a.position.x+c*b.position.x-s*b.position.y,
           a.position.y+s*b.position.x+c*b.position.y,
           a.position.z+b.position.z},a.yaw+b.yaw};
}
void CheckHierarchy(const Element* e) {
  if (e->FirstChildElement("include") || e->FirstChildElement("model") ||
      e->FirstChildElement("frame"))
    throw std::runtime_error("nested models/includes/named frames are unsupported");
}
}  // namespace

std::vector<Obstacle> LoadSdfGeometry(const std::string& path) {
  tinyxml2::XMLDocument document;
  if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("cannot read SDF: " + path);
  const auto* sdf = document.FirstChildElement("sdf");
  const auto* world = sdf ? sdf->FirstChildElement("world") : nullptr;
  if (!world) throw std::runtime_error("SDF must contain a <world>");
  if (world->FirstChildElement("frame"))
    throw std::runtime_error("named world frames are unsupported");
  for (auto* e = world->FirstChildElement("include"); e; e = e->NextSiblingElement("include")) {
    const auto uri = Text(e->FirstChildElement("uri"));
    if (uri.rfind("model://iris_with_",0) != 0)
      throw std::runtime_error("unresolved SDF include: " + uri + "; inline its static collisions first");
  }
  std::vector<Obstacle> result;
  for (auto* model = world->FirstChildElement("model"); model; model = model->NextSiblingElement("model")) {
    const auto is_static = Text(model->FirstChildElement("static"));
    if (is_static != "true" && is_static != "1") continue;
    CheckHierarchy(model);
    const auto model_pose = ReadPose(model);
    for (auto* link = model->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
      CheckHierarchy(link);
      const auto link_pose = Compose(model_pose,ReadPose(link));
      for (auto* collision = link->FirstChildElement("collision"); collision;
           collision = collision->NextSiblingElement("collision")) {
        const auto pose = Compose(link_pose,ReadPose(collision));
        const auto* geometry = collision->FirstChildElement("geometry");
        if (!geometry) throw std::runtime_error("SDF collision has no geometry");
        Obstacle obstacle;
        obstacle.name = Name(model)+"/"+Name(link)+"/"+Name(collision);
        obstacle.center = pose.position;
        obstacle.yaw_rad = pose.yaw;
        if (const auto* box = geometry->FirstChildElement("box")) {
          const auto v = Numbers<3>(box->FirstChildElement("size"));
          if (v[0]<=0 || v[1]<=0 || v[2]<=0) throw std::runtime_error("box dimensions must be positive");
          obstacle.size = {v[0],v[1],v[2]};
        } else if (const auto* cylinder = geometry->FirstChildElement("cylinder")) {
          obstacle.kind = Obstacle::Kind::kCylinder;
          obstacle.radius_m = Positive(cylinder->FirstChildElement("radius"));
          obstacle.size.z = Positive(cylinder->FirstChildElement("length"));
        } else if (const auto* sphere = geometry->FirstChildElement("sphere")) {
          obstacle.kind = Obstacle::Kind::kSphere;
          obstacle.radius_m = Positive(sphere->FirstChildElement("radius"));
        } else {
          throw std::runtime_error("unsupported collision geometry: " + obstacle.name);
        }
        result.push_back(obstacle);
      }
    }
  }
  return result;
}

bool IntersectsVoxel(const Obstacle& o, Vec3 p, double resolution_m) {
  const double half = resolution_m/2;
  const double dx=p.x-o.center.x, dy=p.y-o.center.y, dz=p.z-o.center.z;
  if (o.kind == Obstacle::Kind::kSphere) {
    const double radius=o.radius_m+std::sqrt(3.0)*half;
    return dx*dx+dy*dy+dz*dz <= radius*radius;
  }
  if (std::abs(dz)>o.size.z/2+half) return false;
  if (o.kind == Obstacle::Kind::kCylinder) {
    const double radius=o.radius_m+std::sqrt(2.0)*half;
    return dx*dx+dy*dy <= radius*radius;
  }
  const double c=std::cos(o.yaw_rad), s=std::sin(o.yaw_rad);
  const double padding=half*(std::abs(c)+std::abs(s));
  return std::abs(c*dx+s*dy)<=o.size.x/2+padding &&
         std::abs(-s*dx+c*dy)<=o.size.y/2+padding;
}
}  // namespace uav_map_tools
