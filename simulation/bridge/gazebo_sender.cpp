// Gazebo -> Orin only. This process cannot arm or send FCU commands.
#include "sim_wire.hpp"
#include "jpeg_rgb.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <functional>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <mutex>
#include <netinet/tcp.h>
#include <thread>
#include <unistd.h>
using Clock = std::chrono::steady_clock;
static volatile sig_atomic_t running = 1;
static void Stop(int) { running = 0; }
static double Age(Clock::time_point t) {
  return std::chrono::duration<double>(Clock::now()-t).count();
}
template<class T> double Seconds(const T& t) { return t.sec()+t.nsec()*1e-9; }
int main(int argc, char** argv) {
  if (argc != 3) { std::cerr << "gazebo_sender ORIN_IPV4 PORT\n"; return 2; }
  signal(SIGINT,Stop); signal(SIGTERM,Stop); signal(SIGPIPE,SIG_IGN);
  sim::Frame shared;
  Clock::time_point state_received{}, image_received{}, clock_received{};
  std::string rgb;
  uint64_t image_id=0;
  std::mutex mutex;
  gz::transport::Node node;
  node.Subscribe("/clock",std::function<void(const gz::msgs::Clock&)>([&](const auto& c) {
    std::lock_guard<std::mutex> lock(mutex);
    shared.clock_s=Seconds(c.sim()); clock_received=Clock::now();
  }));
  node.Subscribe("/iris/odometry",std::function<void(const gz::msgs::Odometry&)>([&](const auto& o) {
    std::lock_guard<std::mutex> lock(mutex);
    shared.state_s=Seconds(o.header().stamp());
    const auto& p=o.pose().position(); const auto& q=o.pose().orientation();
    const auto& v=o.twist().linear(); const auto& w=o.twist().angular();
    shared.p[0]=p.x();shared.p[1]=p.y();shared.p[2]=p.z();
    shared.q[0]=q.x();shared.q[1]=q.y();shared.q[2]=q.z();shared.q[3]=q.w();
    shared.v[0]=v.x();shared.v[1]=v.y();shared.v[2]=v.z();
    shared.w[0]=w.x();shared.w[1]=w.y();shared.w[2]=w.z();
    state_received=Clock::now();
  }));
  node.Subscribe("/sensor_suite/rgb",std::function<void(const gz::msgs::Image&)>([&](const auto& i) {
    if (i.width()!=640 || i.height()!=360 || i.step()!=1920 ||
        i.pixel_format_type()!=gz::msgs::PixelFormatType::RGB_INT8 ||
        i.data().size()!=640*360*3) return;
    std::lock_guard<std::mutex> lock(mutex);
    rgb=i.data(); shared.image_s=Seconds(i.header().stamp());
    image_received=Clock::now(); ++image_id;
  }));
  int fd=socket(AF_INET,SOCK_STREAM,0);
  sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(std::stoi(argv[2]));
  if (fd<0 || inet_pton(AF_INET,argv[1],&address.sin_addr)!=1 ||
      connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0) {
    perror("connect Orin; start receiver first");return 1;
  }
  int yes=1;setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes));
  timeval timeout{1,0};
  setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  uint64_t previous_image=0, packets=0;
  bool healthy=true;
  while (running) {
    const auto next=Clock::now()+std::chrono::milliseconds(33);
    sim::Frame frame;std::string image;bool fresh=false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      frame=shared;frame.id=image_id;
      fresh=Age(state_received)<=.25 && Age(clock_received)<=.25;
      if (image_id!=previous_image && Age(image_received)<.15) {
        image=rgb;frame.capture_age_s=Age(image_received);
      }
    }
    if (!fresh) {std::this_thread::sleep_until(next);continue;}
    std::vector<unsigned char> encoded;
    if (!image.empty()) encoded=remote::EncodeRgb(image.data());
    frame.bytes=encoded.size();
    if (!sim::Valid(frame)) {std::this_thread::sleep_until(next);continue;}
    sim::Ack ack;
    if (!sim::Transfer(fd,&frame,sizeof(frame),true) ||
        (!encoded.empty() && !sim::Transfer(fd,encoded.data(),encoded.size(),true)) ||
        !sim::Transfer(fd,&ack,sizeof(ack),false) || ack.tag!=sim::magic || ack.id!=frame.id) {
      std::cerr << "Orin disconnected; no reconnect/queued frames\n";healthy=false;break;
    }
    if (!encoded.empty()) previous_image=frame.id;
    if (++packets%30==0)
      std::cout << "clock=" << frame.clock_s << " image=" << previous_image
                << " position=" << frame.p[0] << ',' << frame.p[1] << ',' << frame.p[2] << std::endl;
    std::this_thread::sleep_until(next);
  }
  close(fd);return healthy?0:1;
}
