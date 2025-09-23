from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    connection_url = LaunchConfiguration('connection_url', default='udp://:14540')
    takeoff_altitude_m = LaunchConfiguration('takeoff_altitude_m', default='5.0')

    return LaunchDescription([
        DeclareLaunchArgument('connection_url', default_value='udp://:14540'),
        DeclareLaunchArgument('takeoff_altitude_m', default_value='5.0'),
        Node(
            package='drone_takeoff',
            executable='takeoff_node',
            name='takeoff_node',
            output='screen',
            parameters=[{
                'connection_url': connection_url,
                'takeoff_altitude_m': takeoff_altitude_m,
            }],
        ),
    ])


