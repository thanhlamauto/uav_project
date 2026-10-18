# Python launch files

Start with the minimal [integration guide](../../docs/INTEGRATION.md).
`app.launch.py` composes the components below. Each include has its own scope,
so one component's `node_package` / `node_executable` cannot affect another.

| Component | Standalone launch | Enable in app | Replacement launch argument |
|---|---|---|---|
| State + TF | `odometry_tf.launch.py` | `start_localization` | `localization_launch_file` |
| Depth | `monocular_depth.launch.py` | `start_depth` | `depth_launch_file` |
| OctoMap + A* | `octomap_route_replanner.launch.py` | `start_global_planner` | `global_planner_launch_file` |
| MPPI + safety | `local_navigation.launch.py` | `start_local_planner` | `local_planner_launch_file` |
| FCU adapter | `autopilot_adapter.launch.py` | `start_adapter` | `adapter_launch_file` |
| MAVROS | `mavros.launch.py` | `start_mavros` | `mavros_launch_file` |

All components are enabled by default. `params_file` defaults to the installed
`config/runtime.yaml`. Explicit launch parameters override matching YAML values.
Defaults target simulation (`localization_source=gazebo`, `use_sim_time=true`).

## Run or replace a component

After building and sourcing your workspace:

```bash
ros2 launch uav_navigation_bringup monocular_depth.launch.py \
  params_file:=/data/uav/runtime.yaml model_file:=/data/models/depth_fp16.plan \
  use_sim_time:=false
ros2 launch uav_navigation_bringup monocular_depth.launch.py --show-args
```

For the whole app, select a custom Python launch or disable a component:

```bash
ros2 launch uav_navigation_bringup app.launch.py \
  depth_launch_file:=/absolute/path/custom_depth.launch.py
# Alternatively, start your replacement separately:
ros2 launch uav_navigation_bringup app.launch.py start_depth:=false
```

The replacement launch receives the following arguments:

| Component | Forwarded arguments |
|---|---|
| State + TF | `params_file`, `use_sim_time`, `localization_source` |
| Depth | `params_file`, `use_sim_time`, `model_file`, `depth_backend` |
| Global planner | `params_file`, `use_sim_time`, `prior_map` |
| Local planner | `params_file`, `use_sim_time`, `prior_map` |
| Adapter | `params_file`, `use_sim_time` |
| MAVROS | `fcu_url` |

Declare these arguments in your replacement and map them to your node's parameters.
Preserve the [message/frame/QoS contract](../../docs/INTEGRATION.md#data-contract).
For an executable replacement with the same parameters, the five runtime leaf
launches also accept `node_package` and `node_executable`. ROS node names stay
fixed to match the YAML sections. Run only one publisher for each component output.

To change only the model, set `model_file` and `depth_backend`:
`auto` selects OpenCV CPU for `.onnx`, TensorRT for `.plan` / `.engine`.
The model must match the current metric-depth preprocessing/output contract.

`mavros.launch.py` wraps upstream `mavros/launch/apm.launch` (XML) and forwards
`fcu_url`. Keep `launch_xml` installed; upstream XML is not copied into this project.

## Simulation and rebuild

`uav_simulation/forest.launch.py` forwards all app arguments. Receiver controls:
`start_receiver`, `receiver_launch_file`, `receiver_bind_address`, `receiver_port`.
Its standalone `gazebo_receiver.launch.py` accepts `bind_address`, `port`,
`node_package` and `node_executable`. See [simulation setup](../../simulation/README.md).

After changing launch/config files, rebuild and source the package:

```bash
colcon build --packages-select uav_navigation_bringup
colcon test --packages-select uav_navigation_bringup
colcon test-result --verbose
source install/setup.bash
```

CMake installs `launch/` and `config/`. Rebuild the Docker image to update container
launch files. A standalone launch does not create its sensor/state inputs or arm/take off.
