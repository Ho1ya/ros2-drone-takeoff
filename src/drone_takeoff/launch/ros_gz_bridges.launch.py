from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_SCAN = "/scan"
ROS_IMAGE = "/camera/image_raw"
ROS_DEPTH = "/camera/depth/image_raw"
ROS_IMU = "/imu"

GZ_SCAN = "/scan"
GZ_IMAGE = "/camera"
GZ_DEPTH = "/depth"
GZ_IMU = "/imu"


def generate_launch_description():
    flat_args = [
        f"{GZ_SCAN}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        f"{GZ_IMAGE}@sensor_msgs/msg/Image@gz.msgs.Image",
        f"{GZ_DEPTH}@sensor_msgs/msg/Image@gz.msgs.Image",
        f"{GZ_IMU}@sensor_msgs/msg/Imu@gz.msgs.IMU"
    ]

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_parameter_bridge',
            output='screen',
            arguments=flat_args,
        )
    ])