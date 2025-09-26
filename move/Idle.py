import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import AttitudeTarget, Vector3
import math

G = 9.81  # гравитация для аппроксимации наклонов

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = AttitudeTarget().orientation
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class IdleNode(Node):
    """
    Нода, которая держит дрон на текущей высоте.
    Включение/выключение через топик /idle_enable (std_msgs/Bool)
    """

    def __init__(self, publish_hz=20.0, kp=0.5, feedforward=0.5):
        super().__init__('idle_node')
        self.enabled = False
        self.current_alt = None
        self.target_alt = None
        self.kp = kp
        self.feedforward = feedforward

        # Publisher для attitude
        self.pub = self.create_publisher(AttitudeTarget,
                                         '/uav1/setpoint_raw/attitude',
                                         10)
        # Подписка на высоту
        self.sub_alt = self.create_subscription(
            PointStamped,
            '/point/raw/altitude',
            self.alt_callback,
            10
        )
        # Подписка на включение/выключение IDLE
        self.create_subscription(
            Bool,
            '/idle_enable',
            self.cmd_callback,
            10
        )
        # Таймер публикации
        self.create_timer(1.0 / publish_hz, self.timer_callback)
        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg.orientation = euler_to_quaternion(0, 0, 0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.0

        self.get_logger().info("IdleNode initialized")

    def alt_callback(self, msg: PointStamped):
        self.current_alt = msg.point.z
        if self.target_alt is None:
            self.target_alt = self.current_alt  # первый раз — берем текущую высоту

    def cmd_callback(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f"Idle {'enabled' if self.enabled else 'disabled'}")

    def timer_callback(self):
        if not self.enabled or self.current_alt is None:
            return

        # P-контроллер по высоте
        error = self.target_alt - self.current_alt
        thrust = self.feedforward + self.kp * error
        thrust = max(0.0, min(1.0, thrust))

        # Нейтральные roll/pitch/yaw
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(0.0, 0.0, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = thrust
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IdleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
