"""Compose the runtime; each include can be disabled or replaced independently."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('uav_navigation_bringup')
    arguments = [
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'runtime.yaml'])),
        DeclareLaunchArgument('model_file', default_value='/models/depth_fp16.plan'),
        DeclareLaunchArgument('depth_backend', default_value='auto'),
        DeclareLaunchArgument('prior_map', default_value='/maps/prior.bt'),
        DeclareLaunchArgument('localization_source', default_value='gazebo'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600'),
    ]
    components = [
        ('mavros', 'mavros.launch.py', ('fcu_url',)),
        ('localization', 'odometry_tf.launch.py',
         ('params_file', 'use_sim_time', 'localization_source')),
        ('depth', 'monocular_depth.launch.py',
         ('params_file', 'use_sim_time', 'model_file', 'depth_backend')),
        ('global_planner', 'octomap_route_replanner.launch.py',
         ('params_file', 'use_sim_time', 'prior_map')),
        ('local_planner', 'local_navigation.launch.py',
         ('params_file', 'use_sim_time', 'prior_map')),
        ('adapter', 'autopilot_adapter.launch.py', ('params_file', 'use_sim_time')),
    ]
    includes = []
    for component, filename, forwarded in components:
        start = 'start_' + component
        replacement = component + '_launch_file'
        arguments.extend([
            DeclareLaunchArgument(start, default_value='true'),
            DeclareLaunchArgument(replacement, default_value=PathJoinSubstitution(
                [share, 'launch', filename]), description='Python launch to include'),
        ])
        # Keep leaf node_package/node_executable defaults local to this include.
        includes.append(GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(LaunchConfiguration(replacement)),
                condition=IfCondition(LaunchConfiguration(start)),
                launch_arguments={name: LaunchConfiguration(name)
                                  for name in forwarded}.items(),
            ),
        ]))
    return LaunchDescription([*arguments, *includes])
