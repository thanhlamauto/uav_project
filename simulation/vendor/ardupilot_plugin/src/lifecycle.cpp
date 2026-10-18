#include "plugin_internal.hpp"
#include <gz/plugin/Register.hh>
// MAX_MOTORS limits the maximum number of <control> elements that
// can be defined in the <plugin>.
#define MAX_MOTORS 255

// Register plugin
GZ_ADD_PLUGIN(gz::sim::systems::ArduPilotPlugin,
              gz::sim::System,
              gz::sim::systems::ArduPilotPlugin::ISystemConfigure,
              gz::sim::systems::ArduPilotPlugin::ISystemPostUpdate,
              gz::sim::systems::ArduPilotPlugin::ISystemReset,
              gz::sim::systems::ArduPilotPlugin::ISystemPreUpdate)
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ArduPilotPlugin, "ArduPilotPlugin")

double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
double Control::kDefaultFrequencyCutoff = 5.0;
double Control::kDefaultSamplingRate = 0.2;

/////////////////////////////////////////////////
gz::sim::systems::ArduPilotPlugin::ArduPilotPlugin()
  : dataPtr(new ArduPilotPluginPrivate())
{
}

/////////////////////////////////////////////////
gz::sim::systems::ArduPilotPlugin::~ArduPilotPlugin()
{
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::Reset(const UpdateInfo &_info,
                                              EntityComponentManager &_ecm)
{
  if (!_ecm.EntityHasComponentType(this->dataPtr->imuLink,
      components::WorldPose::typeId))
  {
      _ecm.CreateComponent(this->dataPtr->imuLink,
          gz::sim::components::WorldPose());
  }
  if (!_ecm.EntityHasComponentType(this->dataPtr->imuLink,
      components::WorldLinearVelocity::typeId))
  {
      _ecm.CreateComponent(this->dataPtr->imuLink,
      gz::sim::components::WorldLinearVelocity());
  }

  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
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
  }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*&_eventMgr*/)
{
  // Make a clone so that we can call non-const methods
  sdf::ElementPtr sdfClone = _sdf->Clone();

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ArduPilotPlugin should be attached to a model "
      << "entity. Failed to initialize." << "\n";
    return;
  }
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(
      _ecm.EntityByComponents(components::World()));
  if (!this->dataPtr->world.Valid(_ecm))
  {
    gzerr << "World entity not found" <<std::endl;
    return;
  }
  if (this->dataPtr->world.Name(_ecm).has_value())
  {
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();
  }

  // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
  // x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->dataPtr->modelXYZToAirplaneXForwardZDown =
    gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("modelXYZToAirplaneXForwardZDown"))
  {
    this->dataPtr->modelXYZToAirplaneXForwardZDown =
        sdfClone->Get<gz::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
  }

  // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->dataPtr->gazeboXYZToNED = gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("gazeboXYZToNED"))
  {
    this->dataPtr->gazeboXYZToNED =
        sdfClone->Get<gz::math::Pose3d>("gazeboXYZToNED");
  }

  // Load control channel params
  this->LoadControlChannels(sdfClone, _ecm);

  // Load sensor params
  this->LoadImuSensors(sdfClone, _ecm);
  this->LoadGpsSensors(sdfClone, _ecm);
  this->LoadRangeSensors(sdfClone, _ecm);
  this->LoadWindSensors(sdfClone, _ecm);

  // Initialise sockets
  if (!InitSockets(sdfClone))
  {
    return;
  }

  // Missed update count before we declare arduPilotOnline status false
  this->dataPtr->connectionTimeoutMaxCount =
    sdfClone->Get("connectionTimeoutMaxCount", 10).first;

  // Enforce lock-step simulation (has default: false)
  this->dataPtr->isLockStep =
    sdfClone->Get("lock_step", this->dataPtr->isLockStep).first;

  // Prevent SITL attempting time-sync (has default: true)
  this->dataPtr->isNoTimeSync =
    sdfClone->Get("no_time_sync", this->dataPtr->isNoTimeSync).first;

    this->dataPtr->have32Channels =
    sdfClone->Get("have_32_channels", false).first;

  // Add the signal handler
  this->dataPtr->sigHandler.AddCallback(
      std::bind(
        &gz::sim::systems::ArduPilotPluginPrivate::OnSignal,
        this->dataPtr.get(),
        std::placeholders::_1));

  gzlog << "[" << this->dataPtr->modelName << "] "
        << "ArduPilot ready to fly. The force will be with you" << "\n";
}
