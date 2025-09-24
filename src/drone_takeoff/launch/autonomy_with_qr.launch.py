from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    connection_url = LaunchConfiguration('connection_url', default='udp://:14540')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    image_topic = LaunchConfiguration('image_topic', default='/camera/image_raw')
    publish_text_topic = LaunchConfiguration('publish_text_topic', default='/qr_detector/text')
    show_debug_window = LaunchConfiguration('show_debug_window', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('connection_url', default_value='udp://:14540'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('publish_text_topic', default_value='/qr_detector/text'),
        DeclareLaunchArgument('show_debug_window', default_value='false'),
        Node(
            package='drone_takeoff',
            executable='qr_detector_node',
            name='qr_detector_node',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'publish_text_topic': publish_text_topic,
                'show_debug_window': show_debug_window,
            }],
        ),
        Node(
            package='drone_takeoff',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
            parameters=[{
                'connection_url': connection_url,
                'scan_topic': scan_topic,
                'qr_text_topic': publish_text_topic,
                # keep depth/imu disabled by default, can be overridden
                'depth_topic': '',
                'imu_topic': '',
            }],
        ),
    ])



