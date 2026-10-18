"""Python wrapper around MAVROS's installed upstream ArduPilot launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600'),
        IncludeLaunchDescription(
            # Upstream apm.launch is XML, even though our wrapper is Python.
            AnyLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('mavros'), 'launch', 'apm.launch'])),
            # Upstream node.launch already names its UAS plugins /mavros.
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'namespace': '',
            }.items(),
        ),
    ])
