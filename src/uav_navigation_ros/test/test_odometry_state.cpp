#include <cmath>
#include <iostream>
#include <stdexcept>
#include "uav_navigation_ros/odometry_state.hpp"
void Require(bool value,const char* reason) {if(!value) throw std::runtime_error(reason);}
int main() {
  try {
    nav_msgs::msg::Odometry message;
    message.header.frame_id="map";message.child_frame_id="base_link";
    message.header.stamp.sec=10;message.header.stamp.nanosec=125;
    // roll=60 degrees: yaw derivative is .5*r for pitch=0 and body q=0.
    message.pose.pose.orientation.x=.5;
    message.pose.pose.orientation.w=std::sqrt(.75);
    message.pose.pose.position.x=7;message.pose.pose.position.y=-6;
    message.twist.twist.angular.z=.4;
    const auto body=uav_navigation_ros::StateFromOdometry(message,"map","base_link");
    Require(std::abs(body.yaw_rate_enu_rad_s-.2)<1e-12,"body angular rate uses full orientation");
    Require(body.stamp_ns==10000000125LL && body.position_enu_m.y==-6,
            "state timestamp/world position changed");
    message.child_frame_id="map";
    message.twist.twist.angular.y=-std::sqrt(.75)*.4;
    message.twist.twist.angular.z=.2;
    const auto world=uav_navigation_ros::StateFromOdometry(message,"map","base_link");
    Require(std::abs(world.yaw_rate_enu_rad_s-body.yaw_rate_enu_rad_s)<1e-12,
            "body and world twist contracts disagree");
    for(int bad=0;bad<4;++bad) {
      auto invalid=message;
      if(bad==0) invalid.twist.twist.angular.x=NAN;
      if(bad==1) invalid.child_frame_id="other";
      if(bad==2) invalid.header.frame_id="unregistered";
      if(bad==3) invalid.pose.pose.orientation.w=invalid.pose.pose.orientation.x=0;
      bool rejected=false;
      try {uav_navigation_ros::StateFromOdometry(invalid,"map","base_link");}
      catch(const std::invalid_argument&) {rejected=true;}
      Require(rejected,"invalid angular odometry was accepted");
    }
    std::cout<<"PASS angular odometry frame conversion and validation\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
