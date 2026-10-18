#include "plugin_internal.hpp"
void gz::sim::systems::ArduPilotPlugin::CreateStateJSON(
    double _simTime,
    const gz::sim::EntityComponentManager &_ecm) const
{
    // Make a local copy of the latest IMU data (it's filled in
    // on receipt by ImuCb()).
    gz::msgs::IMU imuMsg;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
        // Wait until we've received a valid message.
        if (!this->dataPtr->imuMsgValid)
        {
            return;
        }
        imuMsg = this->dataPtr->imuMsg;
    }

    // it is assumed that the imu orientation conforms to the
    // aircraft convention:
    //   x-forward
    //   y-right
    //   z-down

    // get linear acceleration
    gz::math::Vector3d linearAccel{
        imuMsg.linear_acceleration().x(),
        imuMsg.linear_acceleration().y(),
        imuMsg.linear_acceleration().z()
    };

    // get angular velocity
    gz::math::Vector3d angularVel{
        imuMsg.angular_velocity().x(),
        imuMsg.angular_velocity().y(),
        imuMsg.angular_velocity().z(),
    };

    /*
      Gazebo versus ArduPilot frame conventions
      =========================================

      1. ArduPilot assumes an aircraft convention for body and world frames:

        ArduPilot world frame is: x-north, y-east, z-down (NED)
        ArduPilot body frame is:  x-forward, y-right, z-down

      2. The Gazebo frame convention is:

        Gazebo world frame is:    x-east, y-north, z-up (ENU)
        Gazebo body frame is:     x-forward, y-left, z-up

        Reference:
          https://gazebosim.org/api/gazebo/7.0/spherical_coordinates.html

        In some cases the Gazebo body frame may use a non-standard convention,
        for example the Zephyr delta wing model has x-left, y-back, z-up.

      3. The position and orientation provided to ArduPilot must be given as the
      transform from the Aircraft world frame to the Aircraft body frame.
      We denote this as:

        wldAToBdyA = ^{w_A}\xi_{b_A}

      By which we mean that a vector expressed in Aircraft body frame
      coordinates may be represented in Aircraft world frame coordinates by
      the transform:

        ^{wldA}v = ^{wldA}\xi_{bdyA} ^{bdyA}v
                 = ^{wldA}rot_{bdyA} ^{bdyA}v + ^{wldA}pos_{bdyA}

      i.e. a rotation from the world to body frame followed by a translation.

      Combining transforms
      ====================

      1. Gazebo supplies us with the transform from the Gazebo world frame
      to the Gazebo body frame (which we recall may be non-standard)

        wldGToBdyG = ^{wldG}\xi_{bdyG}

      2. The transform from the Aircraft world frame to the Gazebo world frame
      is fixed (provided Gazebo does not modify its convention)

        wldAToWldG = ^{wldA}\xi_{wldG} = Pose3d(0, 0, 0, -GZ_PI, 0, 0)

      Note that this is the inverse of the plugin parameter <gazeboXYZToNED>
      which tells us how to rotate a Gazebo world frame into an Aircraft
      world frame.

      3. The transform from the Aircraft body frame to the Gazebo body frame
      is denoted:

        bdyAToBdyG = ^{bdyA}\xi_{bdyG}

      This will typically be Pose3d(0, 0, 0, -GZ_PI, 0, 0) but in some cases
      will vary. For instance the Zephyr uses the transform
      Pose3d(0, 0, 0, -GZ_PI, 0, GZ_PI/2)

      Note this is also the inverse of the plugin parameter
      <modelXYZToAirplaneXForwardZDown> which tells us how to rotate a Gazebo
      model frame into an Aircraft body frame

      4. Finally we compose the transforms to get the one required for the
      ArduPilot JSON interface:

        ^{wldA}\xi_{bdyA} = ^{wldA}\xi_{wldG} * ^{wldG}\xi_{bdyG}
                          * (^{bdyA}\xi_{bdyG})^{-1}

      where we use:

        ^{bdyG}\xi_{bdyA} = (^{bdyA}\xi_{bdyG})^{-1}

      In C++ variables names this is:

        wldAToBdyA = wldAToWldG * wldGToBdyG * bdyAToBdyG.Inverse()
    */

    // get pose and velocity in Gazebo world frame
    const gz::sim::components::WorldPose* worldPose =
        _ecm.Component<gz::sim::components::WorldPose>(
            this->dataPtr->imuLink);

    const gz::sim::components::WorldLinearVelocity* worldLinearVel =
        _ecm.Component<gz::sim::components::WorldLinearVelocity>(
            this->dataPtr->imuLink);

    // position and orientation transform (Aircraft world to Aircraft body)
    gz::math::Pose3d bdyAToBdyG =
        this->dataPtr->modelXYZToAirplaneXForwardZDown.Inverse();

    /// \todo(srmainwaring) check for error.
    /// The inverse may be incorrect. The error is not evident when using
    /// the transform from the original plugin:
    ///   <gazeboXYZToNED>0 0 0 GZ_PI 0 0</gazeboXYZToNED>
    /// but is when using the correct transform which is
    ///   <gazeboXYZToNED>0 0 0 GZ_PI 0 GZ_PI/2</gazeboXYZToNED>
    ///
    gz::math::Pose3d wldAToWldG = this->dataPtr->gazeboXYZToNED.Inverse();
    // gz::math::Pose3d wldAToWldG = this->dataPtr->gazeboXYZToNED;

    gz::math::Pose3d wldGToBdyG = worldPose->Data();
    gz::math::Pose3d wldAToBdyA =
        wldAToWldG * wldGToBdyG * bdyAToBdyG.Inverse();

    // velocity transformation
    gz::math::Vector3d velWldG = worldLinearVel->Data();
    gz::math::Vector3d velWldA = wldAToWldG.Rot() * velWldG + wldAToWldG.Pos();

    // require the duration since sim start in seconds
    double timestamp = _simTime;

    // Anemometer
    // AP_WindVane_SITL use apparent wind speed and dir in body frame.
    // WNDVN_SPEED_TYPE 11
    // WNDVN_TYPE       11
    double windSpdBdyA{0.0};
    double windDirBdyA{0.0};

    if (this->dataPtr->anemometerInitialized)
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->anemometerMsgMutex);

        // Anemometer sensors reports apparent wind velocity in sensor frame.
        auto windVelSnsG = gz::msgs::Convert(this->dataPtr->anemometerMsg);

        // sensor pose relative to the world frame
        auto wldGToSnsG = gz::sim::worldPose(
            this->dataPtr->anemometerEntity, _ecm);

        auto bdyAToWldA = wldAToBdyA.Inverse();
        auto bdyAToSnsG = bdyAToWldA * wldAToWldG * wldGToSnsG;

        // rotate to AP body (FRD) frame
        auto windVelBdyA = bdyAToSnsG.Rot().RotateVector(windVelSnsG);

        // speed and direction - consider only xy-components and switch sign
        // of direction (Gazebo specifies where wind is going to,
        // AruPilot expects where wind is from).
        double windXBdyA = windVelBdyA.X() * -1.0;
        double windYBdyA = windVelBdyA.Y() * -1.0;
        windSpdBdyA = std::sqrt(windXBdyA * windXBdyA + windYBdyA * windYBdyA);
        windDirBdyA = atan2(windYBdyA, windXBdyA);

        double windXSnsG = windVelSnsG.X();
        double windYSnsG = windVelSnsG.Y();
        auto windSpdSnsG = std::sqrt(
            windXSnsG * windXSnsG + windYSnsG * windYSnsG);
        auto windDirSnsG = atan2(windYSnsG, windXSnsG);

        // gzdbg << "\nEuler angles:\n"
        //       << "bdyAToBdyG:  " << bdyAToBdyG.Rot().Euler() << "\n"
        //       << "Wind velocity:\n"
        //       << "windVelSnsG: " << windVelSnsG << "\n"
        //       << "windVelBdyA: " << windVelBdyA << "\n"
        //       << "Wind speed and direction:\n"
        //       << "windSpdSnsG: " << windSpdSnsG << "\n"
        //       << "windDirSnsG: " << windDirSnsG * 180 / GZ_PI <<  "\n"
        //       << "windSpdBdyA: " << windSpdBdyA << "\n"
        //       << "windDirBdyA: " << windDirBdyA * 180 / GZ_PI <<  "\n"
        //       << "\n";
    }

    // build JSON document
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();

    writer.Key("timestamp");
    writer.Double(timestamp);

    writer.Key("imu");
    writer.StartObject();
    writer.Key("gyro");
    writer.StartArray();
    writer.Double(angularVel.X());
    writer.Double(angularVel.Y());
    writer.Double(angularVel.Z());
    writer.EndArray();
    writer.Key("accel_body");
    writer.StartArray();
    writer.Double(linearAccel.X());
    writer.Double(linearAccel.Y());
    writer.Double(linearAccel.Z());
    writer.EndArray();
    writer.EndObject();

    writer.Key("position");
    writer.StartArray();
    writer.Double(wldAToBdyA.Pos().X());
    writer.Double(wldAToBdyA.Pos().Y());
    writer.Double(wldAToBdyA.Pos().Z());
    writer.EndArray();

    // ArduPilot quaternion convention: q[0] = 1 for identity.
    writer.Key("quaternion");
    writer.StartArray();
    writer.Double(wldAToBdyA.Rot().W());
    writer.Double(wldAToBdyA.Rot().X());
    writer.Double(wldAToBdyA.Rot().Y());
    writer.Double(wldAToBdyA.Rot().Z());
    writer.EndArray();

    writer.Key("velocity");
    writer.StartArray();
    writer.Double(velWldA.X());
    writer.Double(velWldA.Y());
    writer.Double(velWldA.Z());
    writer.EndArray();

    // Range sensor
    {
      // Aquire lock on this->dataPtr->ranges
      std::lock_guard<std::mutex> lock(this->dataPtr->rangeMsgMutex);

      // Assume that all range sensors with index less than
      // ranges.size() provide data.
      // Use switch-case fall-through to set each range sensor
      switch (std::min<size_t>(6, this->dataPtr->ranges.size()))
      {
      case 6:
          writer.Key("rng_6");
          writer.Double(this->dataPtr->ranges[5]);
      case 5:
          writer.Key("rng_5");
          writer.Double(this->dataPtr->ranges[4]);
      case 4:
          writer.Key("rng_4");
          writer.Double(this->dataPtr->ranges[3]);
      case 3:
          writer.Key("rng_3");
          writer.Double(this->dataPtr->ranges[2]);
      case 2:
          writer.Key("rng_2");
          writer.Double(this->dataPtr->ranges[1]);
      case 1:
          writer.Key("rng_1");
          writer.Double(this->dataPtr->ranges[0]);
      default:
          break;
      }
    }

    // Wind sensor
    if (this->dataPtr->anemometerInitialized)
    {
      writer.Key("windvane");
      writer.StartObject();
      writer.Key("direction");
      writer.Double(windDirBdyA);
      writer.Key("speed");
      writer.Double(windSpdBdyA);
      writer.EndObject();
    }

    // Set no time sync
    writer.Key("no_time_sync");
    writer.Bool(this->dataPtr->isNoTimeSync);

    // Set no lockstep
    writer.Key("no_lockstep");
    writer.Bool(!this->dataPtr->isLockStep);

    writer.EndObject();

    // get JSON
    this->dataPtr->json_str = "\n" + std::string(s.GetString()) + "\n";
    // gzdbg << this->dataPtr->json_str << "\n";
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::SendState() const
{
#if DEBUG_JSON_IO
    auto bytes_sent =
#endif
    this->dataPtr->sock.sendto(
        this->dataPtr->json_str.c_str(),
        this->dataPtr->json_str.size(),
        this->dataPtr->fcu_address,
        this->dataPtr->fcu_port_out);

#if DEBUG_JSON_IO
    gzdbg << "sent " << bytes_sent <<  " bytes to "
        << this->dataPtr->fcu_address << ":"
        << this->dataPtr->fcu_port_out << "\n"
        << "frame_count: " << this->dataPtr->fcu_frame_count << "\n";
#endif
}
