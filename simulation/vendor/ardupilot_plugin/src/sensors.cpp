#include "plugin_internal.hpp"
void gz::sim::systems::ArduPilotPlugin::LoadImuSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
    this->dataPtr->imuName =
        _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadGpsSensors(
    sdf::ElementPtr /*_sdf*/,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
  /*
  NOT MERGED IN MASTER YET
  // Get GPS
  std::string gpsName = _sdf->Get("imuName",
      static_cast<std::string>("gps_sensor")).first;
  std::vector<std::string> gpsScopedName =
      SensorScopedName(this->dataPtr->model, gpsName);
  if (gpsScopedName.size() > 1)
  {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "multiple names match [" << gpsName << "] using first found"
           << " name.\n";
    for (unsigned k = 0; k < gpsScopedName.size(); ++k)
    {
      gzwarn << "  sensor " << k << " [" << gpsScopedName[k] << "].\n";
    }
  }

  if (gpsScopedName.size() > 0)
  {
    this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
      (sensors::SensorManager::Instance()->GetSensor(gpsScopedName[0]));
  }

  if (!this->dataPtr->gpsSensor)
  {
    if (gpsScopedName.size() > 1)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "first gps_sensor scoped name [" << gpsScopedName[0]
             << "] not found, trying the rest of the sensor names.\n";
      for (unsigned k = 1; k < gpsScopedName.size(); ++k)
      {
        this->dataPtr->gpsSensor =
            std::dynamic_pointer_cast<sensors::GpsSensor>
          (sensors::SensorManager::Instance()->GetSensor(gpsScopedName[k]));
        if (this->dataPtr->gpsSensor)
        {
          gzwarn << "found [" << gpsScopedName[k] << "]\n";
          break;
        }
      }
    }

    if (!this->dataPtr->gpsSensor)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "gps_sensor scoped name [" << gpsName
             << "] not found, trying unscoped name.\n" << "\n";
      this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
        (sensors::SensorManager::Instance()->GetSensor(gpsName));
    }

    if (!this->dataPtr->gpsSensor)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "gps [" << gpsName
             << "] not found, skipping gps support.\n" << "\n";
    }
    else
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "  found "  << " [" << gpsName << "].\n";
    }
  }
  */
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadRangeSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
    struct SensorIdentifier
    {
        std::string type;
        int index;
        std::string topic;
    };
    std::vector<SensorIdentifier> sensorIds;

    // read sensor elements
    sdf::ElementPtr sensorSdf;
    if (_sdf->HasElement("sensor"))
    {
        sensorSdf = _sdf->GetElement("sensor");
    }

    while (sensorSdf)
    {
        SensorIdentifier sensorId;

        // <type> is required
        if (sensorSdf->HasElement("type"))
        {
            sensorId.type = sensorSdf->Get<std::string>("type");
        }
        else
        {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                << "sensor element 'type' not specified, skipping.\n";
        }

        // <index> is required
        if (sensorSdf->HasElement("index"))
        {
            sensorId.index = sensorSdf->Get<int>("index");
        }
        else
        {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                << "sensor element 'index' not specified, skipping.\n";
        }

        // <topic> is required
        if (sensorSdf->HasElement("topic"))
        {
            sensorId.topic = sensorSdf->Get<std::string>("topic");
        }
        else
        {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                << "sensor element 'topic' not specified, skipping.\n";
        }

        sensorIds.push_back(sensorId);

        sensorSdf = sensorSdf->GetNextElement("sensor");

        gzmsg << "[" << this->dataPtr->modelName << "] range "
            << "type: " << sensorId.type
            << ", index: " << sensorId.index
            << ", topic: " << sensorId.topic
            << "\n";
    }

    /// \todo(anyone) gazebo classic has different rules for generating
    /// topic names, gazebo sim would benefit from similar rules when providing
    /// topics names in sdf sensors elements.

    // get the topic prefix
    // std::string topicPrefix = "~/";
    // topicPrefix += this->dataPtr->modelName;
    // boost::replace_all(topicPrefix, "::", "/");

    // subscriptions
    for (auto &&sensorId : sensorIds)
    {
        /// \todo(anyone) see comment above re. topics
        /// fully qualified topic name
        /// std::string topicName = topicPrefix;
        /// topicName.append("/").append(sensorId.topic);
        std::string topicName = sensorId.topic;

        // Bind the sensor index to the callback function
        // (adjust from unit to zero offset)
        OnMessageWrapper<gz::msgs::LaserScan>::callback_t fn =
            std::bind(
                &gz::sim::systems::ArduPilotPluginPrivate::RangeCb,
                this->dataPtr.get(),
                std::placeholders::_1,
                sensorId.index - 1);

        // Wrap the std::function so we can register the callback
        auto callbackWrapper = RangeOnMessageWrapperPtr(
            new OnMessageWrapper<gz::msgs::LaserScan>(fn));

        auto callback = &OnMessageWrapper<gz::msgs::LaserScan>::OnMessage;

        // Subscribe to range sensor topic
        this->dataPtr->node.Subscribe(
            topicName, callback, callbackWrapper.get());

        this->dataPtr->rangeCbs.push_back(callbackWrapper);

        /// \todo(anyone) initalise ranges properly
        /// (AP convention for ignored value?)
        this->dataPtr->ranges.push_back(-1.0);

        gzmsg << "[" << this->dataPtr->modelName << "] subscribing to "
              << topicName << "\n";
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadWindSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
    this->dataPtr->anemometerName =
        _sdf->Get("anemometer", static_cast<std::string>("")).first;
}
