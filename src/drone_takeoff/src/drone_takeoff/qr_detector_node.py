from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError


class QrDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('qr_detector_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_text_topic', '/qr_detector/text')
        self.declare_parameter('show_debug_window', False)

        self.image_topic: str = self.get_parameter('image_topic').get_parameter_value().string_value
        self.publish_text_topic: str = (
            self.get_parameter('publish_text_topic').get_parameter_value().string_value
        )
        self.show_debug_window: bool = bool(self.get_parameter('show_debug_window').value)

        self.bridge = CvBridge()
        self.qr = cv2.QRCodeDetector()

        self.text_pub = self.create_publisher(String, self.publish_text_topic, 10)
        self.subscription = self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(f'Subscribed to {self.image_topic}. Debug window: {self.show_debug_window}')

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:  # noqa: F841
            self.get_logger().warn('Failed to convert image')
            return

        # Detect and decode
        data, points, _ = self.qr.detectAndDecode(frame)

        if points is not None and len(points) > 0:
            pts = points.reshape(-1, 2).astype(int)
            for i in range(len(pts)):
                pt1 = tuple(pts[i])
                pt2 = tuple(pts[(i + 1) % len(pts)])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        if data:
            self.get_logger().info(f'QR: {data}')
            self.text_pub.publish(String(data=data))

        if self.show_debug_window:
            cv2.imshow('qr_detector', frame)
            cv2.waitKey(1)


def main() -> None:
    rclpy.init()
    node = QrDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_debug_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


