from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    connection_url = LaunchConfiguration('connection_url', default='udp://:14540')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    range_topic = LaunchConfiguration('range_topic', default='')
    qr_text_topic = LaunchConfiguration('qr_text_topic', default='/qr_detector/text')
    takeoff_altitude_m = LaunchConfiguration('takeoff_altitude_m', default='3.0')
    cruise_speed_m_s = LaunchConfiguration('cruise_speed_m_s', default='1.0')
    wall_distance_min_m = LaunchConfiguration('wall_distance_min_m', default='1.0')
    doorway_min_width_rad = LaunchConfiguration('doorway_min_width_rad', default='0.35')
    control_rate_hz = LaunchConfiguration('control_rate_hz', default='10.0')
    max_yaw_rate_deg_s = LaunchConfiguration('max_yaw_rate_deg_s', default='45.0')
    depth_topic = LaunchConfiguration('depth_topic', default='')
    depth_obstacle_distance_m = LaunchConfiguration('depth_obstacle_distance_m', default='1.0')
    depth_center_fraction = LaunchConfiguration('depth_center_fraction', default='0.25')
    imu_topic = LaunchConfiguration('imu_topic', default='')

    return LaunchDescription([
        DeclareLaunchArgument('connection_url', default_value='udp://:14550'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('range_topic', default_value=''),
        DeclareLaunchArgument('qr_text_topic', default_value='/qr_detector/text'),
        DeclareLaunchArgument('takeoff_altitude_m', default_value='3.0'),
        DeclareLaunchArgument('cruise_speed_m_s', default_value='1.0'),
        DeclareLaunchArgument('wall_distance_min_m', default_value='1.0'),
        DeclareLaunchArgument('doorway_min_width_rad', default_value='0.35'),
        DeclareLaunchArgument('control_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('max_yaw_rate_deg_s', default_value='45.0'),
        DeclareLaunchArgument('depth_topic', default_value=''),
        DeclareLaunchArgument('depth_obstacle_distance_m', default_value='1.0'),
        DeclareLaunchArgument('depth_center_fraction', default_value='0.25'),
        DeclareLaunchArgument('imu_topic', default_value=''),
        Node(
            package='drone_takeoff',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
            parameters=[{
                'connection_url': connection_url,
                'scan_topic': scan_topic,
                'qr_text_topic': qr_text_topic,
                'range_topic': range_topic,
                'takeoff_altitude_m': takeoff_altitude_m,
                'cruise_speed_m_s': cruise_speed_m_s,
                'wall_distance_min_m': wall_distance_min_m,
                'doorway_min_width_rad': doorway_min_width_rad,
                'control_rate_hz': control_rate_hz,
                'max_yaw_rate_deg_s': max_yaw_rate_deg_s,
                'depth_topic': depth_topic,
                'depth_obstacle_distance_m': depth_obstacle_distance_m,
                'depth_center_fraction': depth_center_fraction,
                'imu_topic': imu_topic,
            }],
        ),
    ])


