"""Simulator ingress plus the same runtime composition used by the UAV."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('uav_navigation_bringup')
    simulation = FindPackageShare('uav_simulation')
    defaults = {
        'params_file': '/simulation/runtime.yaml',
        'fcu_url': 'tcp://192.168.55.100:5762',
        'model_file': '/models/depth_fp16.plan',
        'depth_backend': 'auto',
        'prior_map': '/maps/prior.bt',
        'localization_source': 'gazebo',
        'use_sim_time': 'true',
        'start_receiver': 'true',
        'receiver_bind_address': '192.168.55.1',
        'receiver_port': '7447',
        'receiver_launch_file': PathJoinSubstitution(
            [simulation, 'launch', 'gazebo_receiver.launch.py']),
    }
    components = {
        'mavros': 'mavros.launch.py',
        'localization': 'odometry_tf.launch.py',
        'depth': 'monocular_depth.launch.py',
        'global_planner': 'octomap_route_replanner.launch.py',
        'local_planner': 'local_navigation.launch.py',
        'adapter': 'autopilot_adapter.launch.py',
    }
    for component, filename in components.items():
        defaults['start_' + component] = 'true'
        defaults[component + '_launch_file'] = PathJoinSubstitution(
            [bringup, 'launch', filename])
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    runtime_arguments = {name: LaunchConfiguration(name) for name in defaults
                         if not name.startswith('receiver_')
                         and name != 'start_receiver'}
    return LaunchDescription([
        *arguments,
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LaunchConfiguration('receiver_launch_file')),
            condition=IfCondition(LaunchConfiguration('start_receiver')),
            launch_arguments={
                'bind_address': LaunchConfiguration('receiver_bind_address'),
                'port': LaunchConfiguration('receiver_port'),
            }.items(),
        )]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [bringup, 'launch', 'app.launch.py'])),
            launch_arguments=runtime_arguments.items(),
        )]),
    ])
