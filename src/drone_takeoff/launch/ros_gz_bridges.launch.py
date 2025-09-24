from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Разворачиваем значения LaunchConfiguration в строки
    ros_scan = LaunchConfiguration('ros_scan').perform(context)
    ros_image = LaunchConfiguration('ros_image').perform(context)
    ros_depth = LaunchConfiguration('ros_depth').perform(context)
    ros_imu = LaunchConfiguration('ros_imu').perform(context)

    gz_scan = LaunchConfiguration('gz_scan').perform(context)
    gz_image = LaunchConfiguration('gz_image').perform(context)
    gz_depth = LaunchConfiguration('gz_depth').perform(context)
    gz_imu = LaunchConfiguration('gz_imu').perform(context)

    flat_args = [
        f"{gz_scan}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan", f"-r", f"{gz_scan}:={ros_scan}",
        f"{gz_image}@sensor_msgs/msg/Image@gz.msgs.Image", f"-r", f"{gz_image}:={ros_image}",
        f"{gz_depth}@sensor_msgs/msg/Image@gz.msgs.Image", f"-r", f"{gz_depth}:={ros_depth}",
        f"{gz_imu}@sensor_msgs/msg/Imu@gz.msgs.IMU", f"-r", f"{gz_imu}:={ros_imu}"
    ]

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_parameter_bridge',
            output='screen',
            arguments=flat_args,
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ros_scan', default_value='/scan'),
        DeclareLaunchArgument('ros_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('ros_depth', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('ros_imu', default_value='/imu'),
        DeclareLaunchArgument('gz_scan', default_value='/scan'),
        DeclareLaunchArgument('gz_image', default_value='/camera'),
        DeclareLaunchArgument('gz_depth', default_value='/depth'),
        DeclareLaunchArgument('gz_imu', default_value='/imu'),
        OpaqueFunction(function=launch_setup)
    ])
