from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ROS-side topics (what our nodes use)
    ros_scan = LaunchConfiguration('ros_scan', default='/scan')
    ros_image = LaunchConfiguration('ros_image', default='/camera/image_raw')
    ros_depth = LaunchConfiguration('ros_depth', default='/camera/depth/image_raw')
    ros_imu = LaunchConfiguration('ros_imu', default='/imu')

    # Gazebo Garden (gz) topics to bridge from sim
    gz_scan = LaunchConfiguration('gz_scan', default='/scan')
    gz_image = LaunchConfiguration('gz_image', default='/camera')
    gz_depth = LaunchConfiguration('gz_depth', default='/depth')
    gz_imu = LaunchConfiguration('gz_imu', default='/imu')

    # parameter_bridge argument format: <gz_topic>@<ros_type>@<gz_type> [<direction>]
    # If ROS and GZ topics differ, add remaps using -r
    args = [
        # LaserScan
        [
            [gz_scan, '@', 'sensor_msgs/msg/LaserScan', '@', 'gz.msgs.LaserScan'],
            ['-r', [gz_scan, ':=', ros_scan]],
        ],
        # RGB image
        [
            [gz_image, '@', 'sensor_msgs/msg/Image', '@', 'gz.msgs.Image'],
            ['-r', [gz_image, ':=', ros_image]],
        ],
        # Depth image
        [
            [gz_depth, '@', 'sensor_msgs/msg/Image', '@', 'gz.msgs.Image'],
            ['-r', [gz_depth, ':=', ros_depth]],
        ],
        # IMU
        [
            [gz_imu, '@', 'sensor_msgs/msg/Imu', '@', 'gz.msgs.IMU'],
            ['-r', [gz_imu, ':=', ros_imu]],
        ],
    ]

    flat_args = []
    for spec in args:
        flat_args.append(''.join([str(x) for x in spec[0]]))
        flat_args.extend(['-r', ''.join([str(x) for x in spec[1][1:]])])

    return LaunchDescription([
        DeclareLaunchArgument('ros_scan', default_value='/scan'),
        DeclareLaunchArgument('ros_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('ros_depth', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('ros_imu', default_value='/imu'),
        DeclareLaunchArgument('gz_scan', default_value='/scan'),
        DeclareLaunchArgument('gz_image', default_value='/camera'),
        DeclareLaunchArgument('gz_depth', default_value='/depth'),
        DeclareLaunchArgument('gz_imu', default_value='/imu'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_parameter_bridge',
            output='screen',
            arguments=flat_args,
        ),
    ])



