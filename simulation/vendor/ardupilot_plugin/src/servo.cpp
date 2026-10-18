#include "plugin_internal.hpp"
namespace
{
/// \brief Get a servo packet. Templated for 16 or 32 channel packets.
template<typename TServoPacket>
ssize_t getServoPacket(
  SocketUDP &_sock,
  const char *&_fcu_address,
  uint16_t &_fcu_port_out,
  uint32_t _waitMs,
  const std::string &_modelName,
  TServoPacket &_pkt
)
{
    ssize_t recvSize = _sock.recv(&_pkt, sizeof(TServoPacket), _waitMs);

    _sock.get_client_address(_fcu_address, _fcu_port_out);

    // drain the socket in the case we're backed up
    int counter = 0;
    while (true)
    {
        TServoPacket last_pkt;
        auto recvSize_last = _sock.recv(&last_pkt, sizeof(TServoPacket), 0ul);
        if (recvSize_last == -1)
        {
            break;
        }
        counter++;
        _pkt = last_pkt;
        recvSize = recvSize_last;
    }
    if (counter > 0)
    {
        gzwarn << "[" << _modelName << "] "
               << "Drained n packets: " << counter << "\n";
    }
    return recvSize;
}
}  // namespace

/////////////////////////////////////////////////
bool gz::sim::systems::ArduPilotPlugin::ReceiveServoPacket()
{
    // Added detection for whether ArduPilot is online or not.
    // If ArduPilot is detected (receive of fdm packet from someone),
    // then socket receive wait time is increased from 1ms to 1 sec
    // to accomodate network jitter.
    // If ArduPilot is not detected, receive call blocks for 1ms
    // on each call.
    // Once ArduPilot presence is detected, it takes this many
    // missed receives before declaring the FCS offline.

    uint32_t waitMs;
    if (this->dataPtr->arduPilotOnline)
    {
        // Increase timeout for recv once we detect a packet from ArduPilot FCS.
        // If this value is too high then it will block the main Gazebo
        // update loop and adversely affect the RTF.
        waitMs = 10;
    }
    else
    {
        // Otherwise skip quickly and do not set control force.
        waitMs = 1;
    }

    // 16 / 32 channel compatibility
    uint16_t pkt_magic{0};
    uint16_t pkt_frame_rate{0};
    uint16_t pkt_frame_count{0};
    std::array<uint16_t, 32> pkt_pwm;
    ssize_t recvSize{-1};
    if (this->dataPtr->have32Channels)
    {
      servo_packet_32 pkt;
      recvSize = getServoPacket(
          this->dataPtr->sock,
          this->dataPtr->fcu_address,
          this->dataPtr->fcu_port_out,
          waitMs,
          this->dataPtr->modelName,
          pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }
    else
    {
      servo_packet_16 pkt;
      recvSize = getServoPacket(
          this->dataPtr->sock,
          this->dataPtr->fcu_address,
          this->dataPtr->fcu_port_out,
          waitMs,
          this->dataPtr->modelName,
          pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }

    // didn't receive a packet, increment timeout count if online, then return
    if (recvSize == -1)
    {
        if (this->dataPtr->arduPilotOnline)
        {
            ++this->dataPtr->timeoutEvents;
            if (++this->dataPtr->connectionTimeoutCount >
            this->dataPtr->connectionTimeoutMaxCount)
            {
                this->dataPtr->connectionTimeoutCount = 0;

                // for lock-step resend last state rather than time out
                if (this->dataPtr->isLockStep)
                {
                    this->SendState();
                }
                else
                {
                    this->dataPtr->arduPilotOnline = false;
                    gzwarn << "[" << this->dataPtr->modelName << "] "
                        << "Broken ArduPilot connection,"
                        << " resetting motor control.\n";
                    this->ResetPIDs();
                }
            }
        }
        return false;
    }

#if DEBUG_JSON_IO
    int max_servo_channels = this->dataPtr->have32Channels ? 32 : 16;

    // debug: inspect sitl packet
    std::ostringstream oss;
    oss << "recv " << recvSize << " bytes from "
        << this->dataPtr->fcu_address << ":"
        << this->dataPtr->fcu_port_out << "\n";
    // oss << "magic: " << pkt_magic << "\n";
    // oss << "frame_rate: " << pkt_frame_rate << "\n";
    oss << "frame_count: " << pkt_frame_count << "\n";
    // oss << "pwm: [";
    // for (auto i=0; i<max_servo_channels - 1; ++i) {
    //     oss << pkt_pwm[i] << ", ";
    // }
    // oss << pkt_pwm[max_servo_channels - 1] << "]\n";
    gzdbg << "\n" << oss.str();
#endif

    // check magic, return if invalid
    constexpr uint16_t magic_16 = 18458;
    constexpr uint16_t magic_32 = 29569;
    uint16_t magic = this->dataPtr->have32Channels ? magic_32 : magic_16;
    if (magic != pkt_magic)
    {
        gzwarn << "Incorrect protocol magic "
            << pkt_magic << " should be "
            << magic << "\n";
        return false;
    }

    // the controller is online
    if (!this->dataPtr->arduPilotOnline)
    {
        this->dataPtr->arduPilotOnline = true;

        gzlog << "[" << this->dataPtr->modelName << "] "
            << "Connected to ArduPilot controller @ "
            << this->dataPtr->fcu_address << ":" << this->dataPtr->fcu_port_out
            << "\n";
    }

    // update frame rate
    this->dataPtr->fcu_frame_rate = pkt_frame_rate;

    // check for controller reset
    if (pkt_frame_count < this->dataPtr->fcu_frame_count)
    {
        /// \todo(anyone) implement re-initialisation
        gzwarn << "ArduPilot controller has reset\n";
    }

    // check for duplicate frame
    else if (pkt_frame_count == this->dataPtr->fcu_frame_count)
    {
        gzwarn << "Duplicate input frame\n";

        // for lock-step resend last state rather than ignore
        if (this->dataPtr->isLockStep)
        {
            this->SendState();
        }

        return false;
    }

    // check for skipped frames
    else if (pkt_frame_count != this->dataPtr->fcu_frame_count + 1
        && this->dataPtr->arduPilotOnline)
    {
        this->dataPtr->missedFrames +=
            pkt_frame_count - this->dataPtr->fcu_frame_count;
        gzwarn << "Missed "
            << pkt_frame_count - this->dataPtr->fcu_frame_count
            << " input frames\n";
    }

    // update frame count
    this->dataPtr->fcu_frame_count = pkt_frame_count;

    // reset the connection timeout so we don't accumulate
    this->dataPtr->connectionTimeoutCount = 0;
    ++this->dataPtr->servoPacketsReceived;
    this->dataPtr->lastServoPacketWall = std::chrono::steady_clock::now();

    this->UpdateMotorCommands(pkt_pwm);

    return true;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::UpdateMotorCommands(
    const std::array<uint16_t, 32> &_pwm)
{
    int max_servo_channels = this->dataPtr->have32Channels ? 32 : 16;

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->dataPtr->controls.size(); ++i)
    {
        // enforce limit on the number of <control> elements
        if (i < MAX_MOTORS)
        {
            if (this->dataPtr->controls[i].channel < max_servo_channels)
            {
                // convert pwm to raw cmd: [servo_min, servo_max] => [0, 1],
                // default is: [1000, 2000] => [0, 1]
                const double pwm = _pwm[this->dataPtr->controls[i].channel];
                const double pwm_min = this->dataPtr->controls[i].servo_min;
                const double pwm_max = this->dataPtr->controls[i].servo_max;
                const double multiplier = this->dataPtr->controls[i].multiplier;
                const double offset = this->dataPtr->controls[i].offset;

                // pwm = 0 => no servo output.
                if (_pwm[this->dataPtr->controls[i].channel] == 0)
                {
                  this->dataPtr->controls[i].outputReady = false;
                  this->dataPtr->controls[i].cmd = 0.0;
#if 0
                  gzdbg << "apply input chan["
                      << this->dataPtr->controls[i].channel
                      << "] to control chan[" << i
                      << "] with joint name ["
                      << this->dataPtr->controls[i].jointName
                      << "] vehicle not ready.\n";
#endif
                }
                else
                {
                  this->dataPtr->controls[i].outputReady = true;

                  // bound incoming cmd between 0 and 1
                  double raw_cmd = (pwm - pwm_min)/(pwm_max - pwm_min);
                  raw_cmd = gz::math::clamp(raw_cmd, 0.0, 1.0);
                  this->dataPtr->controls[i].cmd =
                      multiplier * (raw_cmd + offset);
#if 0
                  gzdbg << "apply input chan["
                      << this->dataPtr->controls[i].channel
                      << "] to control chan[" << i
                      << "] with joint name ["
                      << this->dataPtr->controls[i].jointName
                      << "] ready [" << this->dataPtr->controls[i].outputReady
                      << "] pwm [" << pwm
                      << "] raw cmd [" << raw_cmd
                      << "] adjusted cmd [" << this->dataPtr->controls[i].cmd
                      << "].\n";
#endif
                }
            }
            else
            {
                gzerr << "[" << this->dataPtr->modelName << "] "
                    << "control[" << i << "] channel ["
                    << this->dataPtr->controls[i].channel
                    << "] is greater than the number of servo channels ["
                    << max_servo_channels
                    << "], control not applied.\n";
            }
        }
        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                << "too many motors, skipping [" << i
                << " > " << MAX_MOTORS << "].\n";
        }
    }
}
