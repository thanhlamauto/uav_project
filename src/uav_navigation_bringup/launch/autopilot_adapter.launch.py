"""Validate velocity commands and forward setpoints to MAVROS."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('uav_navigation_bringup'), 'config', 'runtime.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('node_package', default_value='uav_navigation_ros'),
        DeclareLaunchArgument('node_executable', default_value='autopilot_adapter_node'),
    ]
    node = Node(
        package=LaunchConfiguration('node_package'),
        executable=LaunchConfiguration('node_executable'),
        name='autopilot_adapter',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )
    return LaunchDescription([*arguments, node])
