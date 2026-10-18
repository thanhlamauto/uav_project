#include "plugin_internal.hpp"
void gz::sim::systems::ArduPilotPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    static bool calledInitAnemometerOnce{false};
    if (!this->dataPtr->anemometerName.empty() &&
        !this->dataPtr->anemometerInitialized &&
        !calledInitAnemometerOnce)
    {
        calledInitAnemometerOnce = true;
        std::string anemometerTopicName;

        // try scoped names first
        auto entities = entitiesFromScopedName(
            this->dataPtr->anemometerName, _ecm, this->dataPtr->model.Entity());

        // fall-back to unscoped name
        if (entities.empty())
        {
          entities = EntitiesFromUnscopedName(
            this->dataPtr->anemometerName, _ecm, this->dataPtr->model.Entity());
        }

        if (!entities.empty())
        {
          if (entities.size() > 1)
          {
            gzwarn << "Multiple anemometers with name ["
                   << this->dataPtr->anemometerName << "] found. "
                   << "Using the first one.\n";
          }

          // select first entity
          this->dataPtr->anemometerEntity = *entities.begin();

          // validate
          if (!_ecm.EntityHasComponentType(this->dataPtr->anemometerEntity,
              gz::sim::components::CustomSensor::typeId))
          {
            gzerr << "Entity with name ["
                  << this->dataPtr->anemometerName
                  << "] is not an anemometer.\n";
          }
          else
          {
            gzmsg << "Found anemometer with name ["
                  << this->dataPtr->anemometerName
                  << "].\n";

            // verify the parent of the anemometer is a link.
            gz::sim::Entity parent = _ecm.ParentEntity(
                this->dataPtr->anemometerEntity);
            if (_ecm.EntityHasComponentType(parent,
                gz::sim::components::Link::typeId))
            {
                anemometerTopicName = gz::sim::scopedName(
                    this->dataPtr->anemometerEntity, _ecm) + "/anemometer";

                gzdbg << "Computed anemometers topic to be: "
                    << anemometerTopicName << ".\n";
            }
            else
            {
              gzerr << "Parent of anemometer ["
                    << this->dataPtr->anemometerName
                    << "] is not a link.\n";
            }
          }
        }
        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "anemometer [" << this->dataPtr->anemometerName
                  << "] not found, abort ArduPilot plugin." << "\n";
            return;
        }

        this->dataPtr->node.Subscribe(anemometerTopicName,
            &gz::sim::systems::ArduPilotPluginPrivate::AnemometerCb,
            this->dataPtr.get());

        // Make sure that the anemometer entity has WorldPose
        // and WorldLinearVelocity components, which we'll need later.
        enableComponent<components::WorldPose>(
            _ecm, this->dataPtr->anemometerEntity, true);
        enableComponent<components::WorldLinearVelocity>(
            _ecm, this->dataPtr->anemometerEntity, true);

        this->dataPtr->anemometerInitialized = true;
    }

    // This lookup is done in PreUpdate() because in Configure()
    // it's not possible to get the fully qualified topic name we want
    if (!this->dataPtr->imuInitialized)
    {
        // Set unconditionally because we're only going to try this once.
        this->dataPtr->imuInitialized = true;
        std::string imuTopicName;

        // The model must contain an imu sensor element:
        //  <sensor name="..." type="imu">
        //
        // Extract the following:
        //  - Sensor topic name: to subscribe to the imu data
        //  - Link containing the sensor: to get the pose to transform to
        //    the correct frame for ArduPilot

        // try scoped names first
        auto entities = entitiesFromScopedName(
            this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());

        // fall-back to unscoped name
        if (entities.empty())
        {
          entities = EntitiesFromUnscopedName(
            this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());
        }

        if (!entities.empty())
        {
          if (entities.size() > 1)
          {
            gzwarn << "Multiple IMU sensors with name ["
                   << this->dataPtr->imuName << "] found. "
                   << "Using the first one.\n";
          }

          // select first entity
          gz::sim::Entity imuEntity = *entities.begin();

          // validate
          if (!_ecm.EntityHasComponentType(imuEntity,
              gz::sim::components::Imu::typeId))
          {
            gzerr << "Entity with name ["
                  << this->dataPtr->imuName
                  << "] is not an IMU sensor\n";
          }
          else
          {
            gzmsg << "Found IMU sensor with name ["
                  << this->dataPtr->imuName
                  << "]\n";

            // verify the parent of the imu sensor is a link.
            gz::sim::Entity parent = _ecm.ParentEntity(imuEntity);
            if (_ecm.EntityHasComponentType(parent,
                gz::sim::components::Link::typeId))
            {
                this->dataPtr->imuLink = parent;

                imuTopicName = gz::sim::scopedName(
                    imuEntity, _ecm) + "/imu";

                gzdbg << "Computed IMU topic to be: "
                    << imuTopicName << std::endl;
            }
            else
            {
              gzerr << "Parent of IMU sensor ["
                    << this->dataPtr->imuName
                    << "] is not a link\n";
            }
          }
        }
        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "imu_sensor [" << this->dataPtr->imuName
                  << "] not found, abort ArduPilot plugin." << "\n";
            return;
        }

        this->dataPtr->node.Subscribe(imuTopicName,
            &gz::sim::systems::ArduPilotPluginPrivate::ImuCb,
            this->dataPtr.get());

        // Make sure that the 'imuLink' entity has WorldPose
        // and WorldLinearVelocity components, which we'll need later.
        enableComponent<components::WorldPose>(
            _ecm, this->dataPtr->imuLink, true);
        enableComponent<components::WorldLinearVelocity>(
            _ecm, this->dataPtr->imuLink, true);
    }
    else
    {
        // Update the control surfaces.
        if (!_info.paused && _info.simTime >
            this->dataPtr->lastControllerUpdateTime)
        {
            if (this->dataPtr->isLockStep)
            {
                while (!this->ReceiveServoPacket() &&
                    this->dataPtr->arduPilotOnline)
                {
                    // SIGNINT should interrupt this loop.
                    if (this->dataPtr->signal != 0)
                    {
                        break;
                    }
                }
                this->dataPtr->lastServoPacketRecvTime = _info.simTime;
            }
            else if (this->ReceiveServoPacket())
            {
                this->dataPtr->lastServoPacketRecvTime = _info.simTime;
            }

            if (this->dataPtr->arduPilotOnline)
            {
                double dt =
                    std::chrono::duration_cast<std::chrono::duration<double> >(
                        _info.simTime - this->dataPtr->
                            lastControllerUpdateTime).count();
                this->ApplyMotorForces(dt, _ecm);
            }
        }
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    // Publish the new state.
    if (!_info.paused && _info.simTime > this->dataPtr->lastControllerUpdateTime
        && this->dataPtr->arduPilotOnline)
    {
        double t =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                _info.simTime).count();
        this->CreateStateJSON(t, _ecm);
        this->SendState();
        this->dataPtr->lastControllerUpdateTime = _info.simTime;
    }

    // M7.4b: one machine-readable FDM stream health line per simulated second.
    const double sim_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            _info.simTime).count();
    if (sim_s - this->dataPtr->lastFdmHealthLogSimTime >= 1.0)
    {
        const double window =
            std::max(sim_s - this->dataPtr->lastFdmHealthLogSimTime, 1e-6);
        const uint64_t count = this->dataPtr->servoPacketsReceived;
        const double rx_hz =
            static_cast<double>(count - this->dataPtr->lastFdmHealthCount) /
            window;
        double rx_age_s = -1.0;
        if (count > 0)
        {
            rx_age_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                this->dataPtr->lastServoPacketWall).count();
        }
        gzwarn << "[fdm_health] model=" << this->dataPtr->modelName
              << " online=" << this->dataPtr->arduPilotOnline
              << " rx_hz=" << rx_hz
              << " rx_total=" << count
              << " rx_age_s=" << rx_age_s
              << " missed=" << this->dataPtr->missedFrames
              << " timeouts=" << this->dataPtr->timeoutEvents
              << " sim_s=" << sim_s << std::endl;
        this->dataPtr->lastFdmHealthLogSimTime = sim_s;
        this->dataPtr->lastFdmHealthCount = count;
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::ResetPIDs()
{
  // Reset velocity PID for controls
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    this->dataPtr->controls[i].cmd = 0;
    // this->dataPtr->controls[i].pid.Reset();
  }
}
