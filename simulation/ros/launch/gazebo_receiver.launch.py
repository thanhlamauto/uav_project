"""Receive simulator RGB, state and clock over the TCP bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('bind_address', default_value='192.168.55.1'),
        DeclareLaunchArgument('port', default_value='7447'),
        DeclareLaunchArgument('node_package', default_value='uav_simulation'),
        DeclareLaunchArgument('node_executable', default_value='gazebo_receiver'),
    ]
    node = Node(
        package=LaunchConfiguration('node_package'),
        executable=LaunchConfiguration('node_executable'),
        name='gazebo_receiver',
        output='screen',
        parameters=[
            {
                'bind_address': ParameterValue(
                    LaunchConfiguration('bind_address'), value_type=str),
                'port': ParameterValue(
                    LaunchConfiguration('port'), value_type=int),
            },
        ],
    )
    return LaunchDescription([*arguments, node])
