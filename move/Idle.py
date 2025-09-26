import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
import math
import numpy as np

G = 9.81  # гравитация


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Idle(Node):
    def __init__(self):
        super().__init__('uav_idle')

        # Publisher attitude
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)

        # Subscriptions
        self.sub_pos = self.create_subscription(
            PointStamped,
            '/uav1/global_position/local',
            self.pos_callback,
            10
        )
        self.sub_alt = self.create_subscription(
            PointStamped,
            '/uav1/global_position/rel_alt',
            self.alt_callback,
            10
        )

        # PID gains
        self.kp_xy = 0.5
        self.kp_z = 0.5
        self.feedforward_z = 0.5

        # State
        self.current_pos = None
        self.current_alt = None
        self.target_pos = None
        self.target_alt = None
        self.enabled = True

        # Last AttitudeTarget
        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (
                AttitudeTarget.IGNORE_ROLL_RATE |
                AttitudeTarget.IGNORE_PITCH_RATE |
                AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg.orientation = euler_to_quaternion(0, 0, 0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.0

        # Timer loop
        self.create_timer(0.05, self.idle_loop)
        self.get_logger().info("Idle node initialized")

    # ----- Callbacks -----
    def pos_callback(self, msg: PointStamped):
        self.current_pos = np.array([msg.point.x, msg.point.y])
        if self.target_pos is None:
            self.target_pos = self.current_pos.copy()

    def alt_callback(self, msg: PointStamped):
        self.current_alt = msg.point.z
        if self.target_alt is None:
            self.target_alt = self.current_alt

    # ----- Idle loop -----
    def idle_loop(self):
        if not self.enabled or self.current_pos is None or self.current_alt is None:
            return

        # Compute X/Y control
        error_xy = self.target_pos - self.current_pos
        roll = -self.kp_xy * error_xy[1]
        pitch = self.kp_xy * error_xy[0]

        # Compute Z control
        error_z = self.target_alt - self.current_alt
        thrust = self.feedforward_z + self.kp_z * error_z
        thrust = max(0.0, min(1.0, thrust))

        # Publish AttitudeTarget
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll, pitch, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = thrust
        msg.type_mask = (
                AttitudeTarget.IGNORE_ROLL_RATE |
                AttitudeTarget.IGNORE_PITCH_RATE |
                AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg = msg
        self.pub.publish(msg)

    # ----- Enable / Disable -----
    def enable(self):
        self.enabled = True
        if self.current_pos is not None:
            self.target_pos = self.current_pos.copy()
        if self.current_alt is not None:
            self.target_alt = self.current_alt
        self.get_logger().info("Idle ENABLED")

    def disable(self):
        self.enabled = False
        self.get_logger().info("Idle DISABLED")


# ----- Main -----
def main(args=None):
    rclpy.init(args=args)
    node = Idle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
