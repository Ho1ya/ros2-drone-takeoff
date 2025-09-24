from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    range_topic = LaunchConfiguration('range_topic', default='')

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('range_topic', default_value=''),
        Node(
            package='drone_takeoff',
            executable='lidar_listener_node',
            name='lidar_listener_node',
            output='screen',
            parameters=[{'scan_topic': scan_topic}],
        ),
        Node(
            package='drone_takeoff',
            executable='ultrasonic_listener_node',
            name='ultrasonic_listener_node',
            output='screen',
            parameters=[{'range_topic': range_topic}],
        ),
    ])


