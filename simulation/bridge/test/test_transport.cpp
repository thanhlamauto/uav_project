#include "sim_wire.hpp"
#include "jpeg_rgb.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>
static void Require(bool x,const char* why) {if (!x) throw std::runtime_error(why);}
int main() {
  sim::Frame f;f.clock_s=10.1;f.state_s=10;f.image_s=10;f.bytes=100;f.id=77;
  Require(sim::Valid(f),"valid frame rejected");
  auto invalid=f;invalid.q[3]=0;Require(!sim::Valid(invalid),"zero quaternion accepted");
  invalid=f;invalid.w[1]=std::numeric_limits<double>::quiet_NaN();
  Require(!sim::Valid(invalid),"nonfinite twist accepted");
  invalid=f;invalid.state_s=9;Require(!sim::Valid(invalid),"stale state accepted");
  int sockets[2];Require(socketpair(AF_UNIX,SOCK_STREAM,0,sockets)==0,"socketpair");
  std::thread writer([&] {
    auto* p=reinterpret_cast<char*>(&f);
    for (size_t i=0;i<sizeof(f);++i) Require(sim::Transfer(sockets[0],p+i,1,true),"fragment send");
    close(sockets[0]);
  });
  sim::Frame received;
  Require(sim::Receive(sockets[1],received) && received.id==77,"fragmented receive failed");
  Require(!sim::Receive(sockets[1],received),"EOF accepted");
  close(sockets[1]);writer.join();
  Require(socketpair(AF_UNIX,SOCK_STREAM,0,sockets)==0,"socketpair");
  uint32_t old[2]{0x55415633,100};
  sim::Transfer(sockets[0],old,sizeof(old),true);
  Require(!sim::Receive(sockets[1],received),"legacy protocol accepted");
  close(sockets[0]);close(sockets[1]);
  std::string rgb(640*360*3,'\x40');
  auto jpeg=remote::EncodeRgb(rgb.data());std::vector<unsigned char> decoded;
  Require(remote::DecodeRgb(jpeg,decoded) && decoded.size()==rgb.size(),"JPEG roundtrip");
  Require(!remote::DecodeRgb({1,2,3},decoded),"invalid JPEG accepted");
  std::cout << "PASS wire fragmentation, EOF, legacy rejection, timestamps, angular twist, JPEG\n";
}
