import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Altitude
import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q


class Idle(Node):
    def __init__(self):
        super().__init__('idle_controller')

        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)
        self.sub_alt = self.create_subscription(
            Altitude,
            '/uav1/global_position/rel_alt',
            self.alt_callback,
            10
        )

        self.current_alt = None
        self.target_alt = None
        self.kp = 0.4
        self.feedforward = 0.5  # тяга для висения
        self.idle_enabled = False

        # 20 Гц цикл публикации
        self.create_timer(0.05, self.idle_loop)
        self.get_logger().info("Idle controller ready")

    def alt_callback(self, msg: Altitude):
        self.current_alt = msg.relative
        if self.target_alt is None:
            self.target_alt = self.current_alt

    def enable_idle(self, enable: bool):
        self.idle_enabled = enable
        if enable and self.current_alt is not None:
            self.target_alt = self.current_alt
            self.get_logger().info(f"IDLE enabled at {self.target_alt:.2f} m")
        else:
            self.get_logger().info("IDLE disabled")

    def idle_loop(self):
        if not self.idle_enabled or self.current_alt is None:
            return

        error = self.target_alt - self.current_alt
        thrust = self.feedforward + self.kp * error
        thrust = max(0.0, min(1.0, thrust))

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
        self.get_logger().info(f"[IDLE] alt={self.current_alt:.2f}, target={self.target_alt:.2f}, thrust={thrust:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = Idle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
