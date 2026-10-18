#include "plugin_internal.hpp"
bool gz::sim::systems::ArduPilotPlugin::InitSockets(sdf::ElementPtr _sdf) const
{
    // get the fdm address if provided, otherwise default to localhost
    this->dataPtr->fdm_address =
        _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;

    this->dataPtr->fdm_port_in =
        _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;

    // output port configuration is automatic
    if (_sdf->HasElement("listen_addr")) {
        gzwarn << "Param <listen_addr> is deprecated,"
            << " connection is auto detected\n";
    }
    if (_sdf->HasElement("fdm_port_out")) {
        gzwarn << "Param <fdm_port_out> is deprecated,"
            << " connection is auto detected\n";
    }

    // bind the socket
    if (!this->dataPtr->sock.bind(this->dataPtr->fdm_address.c_str(),
        this->dataPtr->fdm_port_in))
    {
        gzerr << "[" << this->dataPtr->modelName << "] "
            << "failed to bind with "
            << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in
            << " aborting plugin.\n";
        return false;
    }
    gzlog << "[" << this->dataPtr->modelName << "] "
        << "flight dynamics model @ "
        << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in
        << "\n";
    return true;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::ApplyMotorForces(
    const double _dt,
    gz::sim::EntityComponentManager &_ecm)
{
  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    // skip servo output while not ready
    if (!this->dataPtr->controls[i].outputReady)
    {
      continue;
    }

    // Publish commands to be relayed to other plugins
    if (this->dataPtr->controls[i].type == "COMMAND")
    {
      msgs::Double cmd;
      cmd.set_data(this->dataPtr->controls[i].cmd);
      this->dataPtr->controls[i].pub.Publish(cmd);
      continue;
    }

    gz::sim::components::JointForceCmd* jfcComp = nullptr;
    gz::sim::components::JointVelocityCmd* jvcComp = nullptr;
    if (this->dataPtr->controls[i].useForce ||
        this->dataPtr->controls[i].type == "EFFORT")
    {
      jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(
          this->dataPtr->controls[i].joint);
      if (jfcComp == nullptr)
      {
        jfcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
            gz::sim::components::JointForceCmd({0}));
      }
    }
    else if (this->dataPtr->controls[i].type == "VELOCITY")
    {
      jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(
          this->dataPtr->controls[i].joint);
      if (jvcComp == nullptr)
      {
        jvcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
            gz::sim::components::JointVelocityCmd({0}));
      }
    }

    if (this->dataPtr->controls[i].useForce)
    {
      if (this->dataPtr->controls[i].type == "VELOCITY")
      {
        const double velTarget = this->dataPtr->controls[i].cmd /
          this->dataPtr->controls[i].rotorVelocitySlowdownSim;
        gz::sim::components::JointVelocity* vComp =
          _ecm.Component<gz::sim::components::JointVelocity>(
              this->dataPtr->controls[i].joint);
        if (vComp && !vComp->Data().empty())
        {
            const double vel = vComp->Data()[0];
            const double error = vel - velTarget;
            const double force = this->dataPtr->controls[i].pid.Update(
                error, std::chrono::duration<double>(_dt));
            jfcComp->Data()[0] = force;
        }
      }
      else if (this->dataPtr->controls[i].type == "POSITION")
      {
        const double posTarget = this->dataPtr->controls[i].cmd;
        gz::sim::components::JointPosition* pComp =
          _ecm.Component<gz::sim::components::JointPosition>(
              this->dataPtr->controls[i].joint);
        if (pComp && !pComp->Data().empty())
        {
            const double pos = pComp->Data()[0];
            const double error = pos - posTarget;
            const double force = this->dataPtr->controls[i].pid.Update(
                error, std::chrono::duration<double>(_dt));
            jfcComp->Data()[0] = force;
        }
      }
      else if (this->dataPtr->controls[i].type == "EFFORT")
      {
        const double force = this->dataPtr->controls[i].cmd;
        jfcComp->Data()[0] = force;
      }
      else
      {
        // do nothing
      }
    }
    else
    {
      if (this->dataPtr->controls[i].type == "VELOCITY")
      {
        jvcComp->Data()[0] = this->dataPtr->controls[i].cmd;
      }
      else if (this->dataPtr->controls[i].type == "POSITION")
      {
        /// \todo(anyone) figure out whether position control matters,
        /// and if so, how to use it.
        gzwarn << "Failed to do position control on joint " << i <<
            " because there's no JointPositionCmd component (yet?)" << "/n";
      }
      else if (this->dataPtr->controls[i].type == "EFFORT")
      {
        const double force = this->dataPtr->controls[i].cmd;
        jvcComp->Data()[0] = force;
      }
      else
      {
        // do nothing
      }
    }
  }
}
