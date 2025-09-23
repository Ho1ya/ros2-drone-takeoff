import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicListenerNode(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_listener_node')
        self.declare_parameter('range_topic', '/ultrasonic')
        self.range_topic: str = self.get_parameter('range_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(Range, self.range_topic, self._on_range, 10)
        self.get_logger().info(f'Listening ultrasonic range on {self.range_topic}')

        self._msg_counter: int = 0

    def _on_range(self, msg: Range) -> None:
        self._msg_counter += 1
        if self._msg_counter % 5 == 0:
            self.get_logger().info(
                f'Ultrasonic: {msg.range:.2f} m (min {msg.min_range:.2f}, max {msg.max_range:.2f})'
            )


def main() -> None:
    rclpy.init()
    node = UltrasonicListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


