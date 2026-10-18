#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "uav_navigation_core/frame_math.hpp"
#include "uav_navigation_core/command_conditioner.hpp"
#include "uav_navigation_core/mppi/dynamics.hpp"
#include "uav_navigation_core/mppi/optimizer.hpp"
namespace c=uav_navigation_core;
namespace m=c::mppi;
void Require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
template<class F> void Reject(F function) {
  bool rejected=false;
  try {function();} catch(const std::invalid_argument&) {rejected=true;}
  Require(rejected,"invalid angular state/config accepted");
}
int main() {
  try {
    const double roll=.7,pitch=.4,yaw=.8;
    const double cr=std::cos(roll/2),sr=std::sin(roll/2);
    const double cp=std::cos(pitch/2),sp=std::sin(pitch/2);
    const double cy=std::cos(yaw/2),sy=std::sin(yaw/2);
    const std::array<double,4> q{sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
                                cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};
    const c::Vec3 body{.2,-.3,.4};
    const auto omega=c::RotateBodyToEnu(body,q);
    const double actual=c::YawRateFromAngularVelocityEnu(omega,q);
    const double expected=(body.y*std::sin(roll)+body.z*std::cos(roll))/std::cos(pitch);
    Require(std::abs(actual-expected)<1e-12,"tilted body angular velocity -> Euler yaw rate");
    Require(std::abs(actual-body.z)>.1,"test must distinguish yaw rate from body omega_z");
    const double h=std::sqrt(.5);
    Reject([&]{c::YawRateFromAngularVelocityEnu(omega,{0,h,0,h});});
    Reject([&]{c::YawRateFromAngularVelocityEnu({NAN,0,0},q);});

    m::MppiDynamicsConfig config;config.yaw_response_model=true;
    m::MultirotorMotionModel model(config);
    m::MppiState state;state.yaw_rate_enu_rad_s=.4;
    const auto stopped_command=model.Step(state,{},.1);
    Require(stopped_command.yaw_rate_enu_rad_s>0 && stopped_command.yaw_rate_enu_rad_s<.4,
            "zero command must decelerate a rotating UAV, not instantly stop");
    Require(stopped_command.yaw_enu_rad>0,"residual rotation must change predicted heading");
    state.yaw_rate_enu_rad_s=-.4;
    Require(model.Step(state,{},.1).yaw_enu_rad<0,"opposite measured rate changes rollout");
    state.yaw_rate_enu_rad_s=1.2;
    const auto overspeed=model.Step(state,{},.1);
    Require(overspeed.yaw_rate_enu_rad_s>config.yaw_rate_max_rad_s,
            "physical measured overspeed must not be clipped to command limit");
    Require(std::abs(overspeed.yaw_rate_enu_rad_s-1.2)<=config.max_yaw_accel_rad_s2*.1+1e-12,
            "predicted angular deceleration exceeds configured bound");
    state.yaw_rate_enu_rad_s=NAN;
    Reject([&]{model.Step(state,{},.1);});
    config.yaw_tau_s=0;Reject([&]{m::MultirotorMotionModel invalid(config);});
    config.yaw_tau_s=.5;

    // Same yaw, path, candidate controls, cost and deterministic noise. Only
    // measured yaw rate changes. Optimizer should counter the current rotation.
    m::MppiCostConfig weights;
    weights.w_goal=weights.w_terminal=weights.w_obstacle=weights.w_collision=0;
    weights.w_effort=weights.w_smoothness=0;weights.w_yaw=1000;
    m::CostEvaluator evaluator(weights);
    m::MppiOptimizerConfig optimizer_config;optimizer_config.lambda=.01;
    m::MppiOptimizer optimizer(optimizer_config);
    const size_t horizon=10;
    m::ControlSequence nominal(horizon,c::Control{{.5,0,0},0});
    m::ControlBatch noise(3,m::ControlSequence(horizon));
    for(size_t t=0;t<horizon;++t) {
      noise[0][t].yaw_rate_enu_rad_s=-.6;
      noise[2][t].yaw_rate_enu_rad_s=.6;
    }
    double selected[2]{};
    for(int i=0;i<2;++i) {
      state={};state.yaw_rate_enu_rad_s=i==0?.4:-.4;
      m::CostContext context;context.initial_state=state;
      const auto result=optimizer.OptimizeInjected(model,evaluator,context,state,nominal,
                                                    nominal.back(),noise,.1);
      selected[i]=result.updated_nominal.front().yaw_rate_enu_rad_s;
    }
    Require(selected[0]<-.01 && selected[1]>.01,
            "optimizer must react to measured rotation even at identical yaw");
    c::VelocityCommandConditioner conditioner;
    c::State measured;measured.yaw_rate_enu_rad_s=.4;
    Require(conditioner.Apply(measured,{}).yaw_rate_enu_rad_s>0,
            "conditioner restart must initialize from measured yaw rate");
    std::cout<<"PASS yaw-state dynamics and optimizer: positive-rate command="
             <<selected[0]<<", negative-rate command="<<selected[1]<<'\n';
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
