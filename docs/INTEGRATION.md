# Integration — ROS 2 Humble

Copy the three `src/uav_navigation_*` packages into your workspace's `src/`.
`simulation/` is optional. Runtime algorithms are C++17; Python only starts nodes.

## Build

On Ubuntu 22.04 with ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to uav_navigation_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv5
source install/setup.bash
```

The OpenCV path assumes the installation used by [Dockerfile.cpu](../deploy/Dockerfile.cpu).
The current ONNX model needs a compatible DNN importer; the recipe builds OpenCV 5.
For Orin/TensorRT, use [the GPU build](../deploy/build_orin.sh).
Model weights and maps are supplied separately. A TensorRT `.plan` must match
the target GPU and TensorRT runtime; use a compatible `.onnx` for CPU inference.

## Include in your launch

Copy `src/uav_navigation_bringup/config/runtime.yaml` to your own config directory.
Update topics, camera extrinsics, map bounds and motion limits. Keep its node names.
Use absolute paths accessible to the running process/container.
Add `<exec_depend>uav_navigation_bringup</exec_depend>` to your bringup package's `package.xml`.

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('uav_navigation_bringup'), 'launch', 'app.launch.py',
            ])),
            launch_arguments={
                'params_file': '/data/uav/runtime.yaml',
                'model_file': '/data/models/depth_fp16.plan',
                'prior_map': '/data/maps/prior.bt',
                'localization_source': 'mavros',
                'use_sim_time': 'false',
                'start_mavros': 'false',  # Your launch already starts MAVROS.
            }.items(),
        ),
    ])
```

To reuse only one component, include its launch instead of `app.launch.py`.
Each component can also run directly:

```bash
ros2 launch uav_navigation_bringup local_navigation.launch.py \
  params_file:=/data/uav/runtime.yaml prior_map:=/data/maps/prior.bt use_sim_time:=false
```

MPPI still needs state, a global path, obstacle cloud and TF.
Disable an existing component with `start_depth:=false`, or replace its launch with
`depth_launch_file:=/absolute/path/custom_depth.launch.py`.
See [component launches and arguments](../src/uav_navigation_bringup/README.md).

## Data contract

Default topics below; configure alternatives in your YAML or use ROS remapping.

| Data | Topic / type | Convention |
|---|---|---|
| RGB input | `/camera/image_rect_color` · `sensor_msgs/msg/Image` | Rectified RGB/BGR uint8, 640×360 |
| Calibration | `/camera/camera_info` · `sensor_msgs/msg/CameraInfo` | Same size and optical frame as RGB |
| State | `/localization/odometry` · `nav_msgs/msg/Odometry` | Pose in `map` ENU; linear/angular velocity in `base_link` FLU |
| Goal | `/goal_pose` · `geometry_msgs/msg/PoseStamped` | XYZ in metres, `map` |
| A* → MPPI | `/planning/global_path` · `nav_msgs/msg/Path` | `map`; reliable, transient local, depth 1 |
| Depth → MPPI | `/perception/obstacles_camera` · `sensor_msgs/msg/PointCloud2` | XYZ float32 in camera optical frame |
| MPPI → adapter | `/control/safe_velocity_command` · `geometry_msgs/msg/TwistStamped` | Linear ENU m/s; `angular.z` yaw rate rad/s |
| Adapter → MAVROS | `/mavros/setpoint_velocity/cmd_vel` · `geometry_msgs/msg/TwistStamped` | Same ENU/yaw-rate convention |
| FCU → adapter | `/mavros/state` · `mavros_msgs/msg/State` | Connected, armed, flight mode; reliable QoS |

- Provide TF `map → base_link → camera_optical_frame` at the image timestamp.
  Optical axes: x right, y down, z forward. Sensor inputs use SensorDataQoS.
- `localization_source:=mavros` reads `/mavros/local_position/odom` and publishes
  normalized state + TF. For your own state/TF publisher, use `start_localization:=false`.
- The prior `.bt` must share the localization origin and axes. A* and MPPI must
  use the same `prior_map`, `map_min`, `map_max` and `clearance_m`.
- Depth feeds local obstacle avoidance; it does **not** update the fixed A* map.
- Simulation: `use_sim_time:=true`, publish `/clock`, and use Gazebo state via
  `localization_source:=gazebo`. All input timestamps must use the same clock.
- Start the camera and MAVROS separately when disabled here. The adapter forwards
  commands only when FCU is connected, armed and in `GUIDED`; launch does not arm/take off.

All project launch files are `.launch.py`. The MAVROS wrapper still loads MAVROS's
upstream XML `apm.launch`, so `launch_xml` remains a dependency.
