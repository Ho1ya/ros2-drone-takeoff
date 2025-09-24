from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarListenerNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_listener_node')
        self.declare_parameter('scan_topic', '/scan')
        self.scan_topic: str = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'Listening LiDAR scans on {self.scan_topic}')

        self._msg_counter: int = 0

    def _on_scan(self, msg: LaserScan) -> None:
        self._msg_counter += 1
        if self._msg_counter % 10 == 0:
            valid_ranges = [r for r in msg.ranges if r != float('inf') and r == r]
            if valid_ranges:
                min_range = min(valid_ranges)
                max_range = max(valid_ranges)
                self.get_logger().info(
                    f'Lidar: {len(msg.ranges)} rays, min={min_range:.2f} m, max={max_range:.2f} m'
                )


def main() -> None:
    rclpy.init()
    node = LidarListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


