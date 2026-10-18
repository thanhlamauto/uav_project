#pragma once
// Simulation-only TCP v1: little-endian IEEE754 peers (Apple Silicon / Jetson).
// State pose: world ENU; twist: body FLU, exactly as Gazebo OdometryPublisher.
// Commands/FCU health do NOT travel through this socket: real MAVROS uses SITL TCP.
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <sys/socket.h>
namespace sim {
constexpr uint32_t magic = 0x55535031;
struct Frame {
  uint32_t tag = magic, bytes = 0;
  uint64_t id = 0;
  double clock_s = 0, state_s = 0, image_s = 0, capture_age_s = 0;
  double p[3]{}, q[4]{0,0,0,1}, v[3]{}, w[3]{};
};
struct Ack { uint32_t tag = magic, reserved = 0; uint64_t id = 0; };
static_assert(sizeof(Frame) == 152 && sizeof(Ack) == 16, "wire ABI changed");
inline bool Transfer(int fd, void* data, size_t count, bool sending) {
  auto* p = static_cast<char*>(data);
  while (count) {
    auto n = sending ? ::send(fd,p,count,0) : ::recv(fd,p,count,0);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;
    p += n; count -= static_cast<size_t>(n);
  }
  return true;
}
inline bool Valid(const Frame& f) {
  if (f.tag != magic || f.bytes > 1024*1024 || !std::isfinite(f.clock_s) ||
      !std::isfinite(f.state_s) || !std::isfinite(f.image_s) ||
      !std::isfinite(f.capture_age_s) || f.clock_s < 0 || f.state_s < 0 ||
      f.capture_age_s < 0 || f.capture_age_s > .25 ||
      f.state_s > f.clock_s+.05 || f.clock_s-f.state_s > .25 ||
      (f.bytes && (f.image_s < 0 || f.image_s > f.clock_s+.05 ||
                   f.clock_s-f.image_s > .25))) return false;
  double norm = 0;
  for (double x:f.q) { if (!std::isfinite(x)) return false; norm+=x*x; }
  for (const auto* vector : {f.p,f.v,f.w})
    for (int i=0;i<3;++i) if (!std::isfinite(vector[i])) return false;
  return std::abs(norm-1) < .01;
}
inline bool Receive(int fd, Frame& f) {
  if (!Transfer(fd,&f,8,false) || f.tag != magic) return false;
  return Transfer(fd,reinterpret_cast<char*>(&f)+8,sizeof(f)-8,false) && Valid(f);
}
} // namespace sim
