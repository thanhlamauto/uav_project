// No localization estimation, planning, command relay or fake FCU state here.
#include "sim_wire.hpp"
#include "jpeg_rgb.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <netinet/tcp.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <unistd.h>
static builtin_interfaces::msg::Time Stamp(double s) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(s*1e9)),RCL_ROS_TIME);
}
int main(int argc,char** argv) {
  signal(SIGPIPE,SIG_IGN);
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("gazebo_receiver");
  const auto ip=node->declare_parameter<std::string>("bind_address","192.168.55.1");
  const int port=node->declare_parameter<int>("port",7447);
  auto clocks=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::ClockQoS());
  auto states=node->create_publisher<nav_msgs::msg::Odometry>("/iris/odometry",rclcpp::SensorDataQoS());
  auto images=node->create_publisher<sensor_msgs::msg::Image>("/camera/image_rect_color",rclcpp::SensorDataQoS());
  auto cameras=node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info",rclcpp::SensorDataQoS());
  int server=socket(AF_INET,SOCK_STREAM,0),yes=1;
  setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
  sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
  if (server<0 || inet_pton(AF_INET,ip.c_str(),&address.sin_addr)!=1 ||
      bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 || listen(server,1)<0) {
    perror("listen USB");return 1;
  }
  std::cout << "Gazebo ingress listening " << ip << ':' << port << std::endl;
  int fd=-1;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    pollfd pending{server,POLLIN,0};
    if (poll(&pending,1,50)>0) {fd=accept(server,nullptr,nullptr);break;}
  }
  if (fd<0) {close(server);return 1;}
  timeval timeout{1,0};
  setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes));
  uint64_t packets=0,images_received=0;
  double last_clock=-1,last_state=-1,last_image=-1;
  bool healthy=true;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    pollfd pending{fd,POLLIN,0};
    if (poll(&pending,1,50)<=0) continue;
    sim::Frame f;
    if (!sim::Receive(fd,f) || f.clock_s<last_clock || f.state_s<last_state) {
      healthy=false;break;
    }
    std::vector<unsigned char> encoded(f.bytes),rgb;
    if (f.bytes && (!sim::Transfer(fd,encoded.data(),encoded.size(),false) ||
                    !remote::DecodeRgb(encoded,rgb) || f.image_s<=last_image)) {
      healthy=false;break;
    }
    rosgraph_msgs::msg::Clock clock;clock.clock=Stamp(f.clock_s);clocks->publish(clock);
    nav_msgs::msg::Odometry state;
    state.header.frame_id="odom";state.child_frame_id="base_link";
    state.header.stamp=Stamp(f.state_s);
    auto& p=state.pose.pose.position;p.x=f.p[0];p.y=f.p[1];p.z=f.p[2];
    auto& q=state.pose.pose.orientation;q.x=f.q[0];q.y=f.q[1];q.z=f.q[2];q.w=f.q[3];
    auto& v=state.twist.twist.linear;v.x=f.v[0];v.y=f.v[1];v.z=f.v[2];
    auto& w=state.twist.twist.angular;w.x=f.w[0];w.y=f.w[1];w.z=f.w[2];
    if (f.state_s>last_state) states->publish(state);
    if (f.bytes) {
      sensor_msgs::msg::Image image;
      image.header.frame_id="camera_optical_frame";image.header.stamp=Stamp(f.image_s);
      image.width=640;image.height=360;image.step=1920;image.encoding="rgb8";
      image.data=std::move(rgb);
      sensor_msgs::msg::CameraInfo camera;camera.header=image.header;
      camera.width=640;camera.height=360;camera.distortion_model="plumb_bob";
      camera.d={0,0,0,0,0};
      const double focal=640/(2*std::tan(1.3962634/2));
      camera.k={focal,0,319.5,0,focal,179.5,0,0,1};
      camera.r={1,0,0,0,1,0,0,0,1};
      camera.p={focal,0,319.5,0,0,focal,179.5,0,0,0,1,0};
      cameras->publish(camera);images->publish(image);
      last_image=f.image_s;++images_received;
    }
    last_clock=f.clock_s;last_state=f.state_s;
    sim::Ack ack;ack.id=f.id;
    if (!sim::Transfer(fd,&ack,sizeof(ack),true)) {healthy=false;break;}
    if (++packets%30==0)
      std::cout << "packets=" << packets << " rgb=" << images_received
                << " sim_s=" << f.clock_s << " z=" << f.p[2] << std::endl;
  }
  close(fd);close(server);rclcpp::shutdown();
  if (!healthy) std::cerr << "Simulator disconnected or invalid packet; stop launch before restarting\n";
  return healthy?0:1;
}
