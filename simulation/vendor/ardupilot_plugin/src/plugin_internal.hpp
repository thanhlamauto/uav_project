#pragma once
/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ArduPilotPlugin.hh"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>

#include <gz/common/SignalHandler.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Filter.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/PID.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "SocketUDP.hh"
#include "Util.hh"

#define DEBUG_JSON_IO 0
#define MAX_MOTORS 255

/// \brief class Control is responsible for controlling a joint
class Control
{
  /// \brief Constructor
  public: Control()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  public: ~Control() {}

  /// \brief The PWM channel used to command this control
  public: int channel = 0;

  /// \brief Next command to be applied to the joint
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: gz::math::PID pid;

  /// \brief The controller type
  ///
  /// Valid controller types are:
  ///   VELOCITY control velocity of joint
  ///   POSITION control position of joint
  ///   EFFORT control effort of joint
  ///   COMMAND control sends command to topic
  public: std::string type;

  /// \brief Use force controller
  public: bool useForce = true;

  /// \brief The name of the joint being controlled
  public: std::string jointName;

  /// \brief The name of the topic to forward this command
  public: std::string cmdTopic;

  /// \brief The joint being controlled
  public: gz::sim::Entity joint;

  /// \brief A multiplier to scale the raw input command
  public: double multiplier = 1.0;

  /// \brief An offset to shift the zero-point of the raw input command
  public: double offset = 0.0;

  /// \brief Lower bound of PWM input, has default (1000).
  ///
  /// The lower bound of PWM input should match SERVOX_MIN for this channel.
  public: double servo_min = 1000.0;

  /// \brief Upper limit of PWM input, has default (2000).
  ///
  /// The upper limit of PWM input should match SERVOX_MAX for this channel.
  public: double servo_max = 2000.0;

  /// Flag set to true when servos are ready to output (pwm != 0).
  public: bool outputReady = false;

  /// \brief Publisher for sending commands
  public: gz::transport::Node::Publisher pub;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: gz::math::OnePole<double> filter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

/////////////////////////////////////////////////
// Wrapper class to store callback functions
template <typename M>
class OnMessageWrapper
{
  /// \brief Callback function type definition
  public: typedef std::function<void(const M &)> callback_t;

  /// \brief Callback function
  public: callback_t callback;

  /// \brief Constructor
  public: OnMessageWrapper(const callback_t &_callback)
    : callback(_callback)
  {
  }

  /// \brief Callback function
  public: void OnMessage(const M &_msg)
  {
    if (callback)
    {
      callback(_msg);
    }
  }
};

typedef std::shared_ptr<OnMessageWrapper<
    gz::msgs::LaserScan>> RangeOnMessageWrapperPtr;

/////////////////////////////////////////////////
// Private data class
class gz::sim::systems::ArduPilotPluginPrivate
{
  /// \brief The model
  public: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief The entity representing the link containing the imu sensor.
  public: gz::sim::Entity imuLink{gz::sim::kNullEntity};

  /// \brief The model name;
  public: std::string modelName;

  /// \brief The world
  public: gz::sim::World world{gz::sim::kNullEntity};

  /// \brief The world name;
  public: std::string worldName;

  /// \brief Array of controllers
  public: std::vector<Control> controls;

  /// \brief keep track of controller update sim-time.
  public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Keep track of the time the last servo packet was received.
  public: std::chrono::steady_clock::duration lastServoPacketRecvTime{0};

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Socket manager
  public: SocketUDP sock = SocketUDP(true, true);

  /// \brief The address for the flight dynamics model (i.e. this plugin)
  public: std::string fdm_address{"127.0.0.1"};

  /// \brief The address for the SITL flight controller - auto detected
  public: const char* fcu_address{nullptr};

  /// \brief The port for the flight dynamics model
  public: uint16_t fdm_port_in{9002};

  /// \brief The port for the SITL flight controller - auto detected
  public: uint16_t fcu_port_out;

  /// \brief The name of the IMU sensor
  public: std::string imuName;

  /// \brief Set true to enforce lock-step simulation
  public: bool isLockStep{false};

  /// \brief Set true to prevent SITL from trying to sync with wall-time
  public: bool isNoTimeSync{true};

  /// \brief Set true if have 32 servo channels
  public: bool have32Channels{false};

  /// \brief Have we initialized subscription to the IMU data yet?
  public: bool imuInitialized{false};

  /// \brief We need an gz-transport Node to subscribe to IMU data
  public: gz::transport::Node node;

  /// \brief A copy of the most recently received IMU data message
  public: gz::msgs::IMU imuMsg;

  /// \brief Have we received at least one IMU data message?
  public: bool imuMsgValid{false};

  /// \brief This mutex should be used when accessing imuMsg or imuMsgValid
  public: std::mutex imuMsgMutex;

  /// \brief This subscriber callback latches the most recently received
  ///        IMU data message for later use.
  public: void ImuCb(const gz::msgs::IMU &_msg)
  {
    std::lock_guard<std::mutex> lock(this->imuMsgMutex);
    imuMsg = _msg;
    imuMsgValid = true;
  }

  // Range sensors

  /// \brief This mutex must be used when accessing ranges
  public: std::mutex rangeMsgMutex;

  /// \brief A copy of the most recently received range data
  public: std::vector<double> ranges;

  /// \brief Callbacks for each range sensor
  public: std::vector<RangeOnMessageWrapperPtr> rangeCbs;

  /// \brief This subscriber callback latches the most recently received
  /// data message for later use.
  ///
  /// \todo(anyone) using msgs::LaserScan as a proxy for msgs::SonarStamped
  public: void RangeCb(const gz::msgs::LaserScan &_msg, int _sensorIndex)
  {
    // Extract data
    double range_max = _msg.range_max();
    auto&& ranges = _msg.ranges();
    auto&& intensities = _msg.intensities();

    // If there is no return, the range should be greater than range_max
    double sample_min = 2.0 * range_max;
    for (auto&& range : ranges)
    {
      sample_min = std::min(
          sample_min, std::isinf(range) ? 2.0 * range_max : range);
    }

    // Aquire lock and update the range data
    std::lock_guard<std::mutex> lock(this->rangeMsgMutex);
    this->ranges[_sensorIndex] = sample_min;
  }

  // Anemometer

  /// \brief The entity representing the anemometer.
  public: gz::sim::Entity anemometerEntity{gz::sim::kNullEntity};

  /// \brief The name of the anemometer.
  public: std::string anemometerName;

  /// \brief This mutex must be used when accessing the anemometer.
  public: std::mutex anemometerMsgMutex;

  /// \brief Have we initialized subscription to the anemometer data yet?
  public: bool anemometerInitialized{false};

  /// \brief A copy of the most recently received apparent wind message.
  public: gz::msgs::Vector3d anemometerMsg;

  /// \brief Callback for the anemometer.
  public: void AnemometerCb(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->anemometerMsgMutex);
    anemometerMsg = _msg;
  }

  /// \brief Pointer to an GPS sensor [optional]
  //  public: sensors::GpsSensorPtr gpsSensor;

  /// \brief Pointer to an Rangefinder sensor [optional]
  //  public: sensors::RaySensorPtr rangefinderSensor;

  /// \brief Set to true when the ArduPilot flight controller is online
  ///
  /// Set to false when Gazebo starts to prevent blocking, true when
  /// the ArduPilot controller is detected and online, and false if the
  /// connection to the ArduPilot controller times out.
  public: bool arduPilotOnline{false};

  /// \brief Number of consecutive missed ArduPilot controller messages
  public: int connectionTimeoutCount{0};

  /// \brief Max number of consecutive missed ArduPilot controller
  ///        messages before timeout
  public: int connectionTimeoutMaxCount;

  /// \brief Transform from model orientation to x-forward and z-up
  public: gz::math::Pose3d modelXYZToAirplaneXForwardZDown;

  /// \brief Transform from world frame to NED frame
  public: gz::math::Pose3d gazeboXYZToNED;

  /// \brief Last received frame rate from the ArduPilot controller
  public: uint16_t fcu_frame_rate;

  /// \brief Last received frame count from the ArduPilot controller
  public: uint32_t fcu_frame_count = -1;

  // M7.4b FDM stream diagnostics. Exposed once per simulated second through a
  // machine-readable log line so the harness can distinguish a lost FDM stream
  // from controller behaviour.
  public: uint64_t servoPacketsReceived{0};
  public: uint64_t missedFrames{0};
  public: uint64_t timeoutEvents{0};
  public: std::chrono::steady_clock::time_point lastServoPacketWall{};
  public: double lastFdmHealthLogSimTime{0.0};
  public: uint64_t lastFdmHealthCount{0};

  /// \brief Last sent JSON string, so we can resend if needed.
  public: std::string json_str;

  /// \brief A copy of the most recently received signal.
  public: int signal{0};

  /// \brief Signal handler.
  public: gz::common::SignalHandler sigHandler;

  /// \brief Signal handler callback.
  public: void OnSignal(int _sig)
  {
      gzdbg << "Plugin received signal[" << _sig  << "]\n";
      this->signal = _sig;
  }
};
