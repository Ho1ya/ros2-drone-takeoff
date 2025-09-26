#!/usr/bin/env python3
# idle_node.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Imu, FluidPressure
from mavros_msgs.msg import AttitudeTarget, Vector3
import math

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
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

class IdleNode(Node):
    """
    Idle controller node.
    - Service /idle/set_enabled (std_srvs/SetBool) to enable/disable IDLE.
    - When enabled, subscribes to IMU and barometer, publishes AttitudeTarget at pub_hz.
    - When disabled, unsubscribes and stops publishing.
    """
    def __init__(self):
        super().__init__('idle_node')

        # Publisher uses same topic as planner so only one active publisher at a time is expected
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)

        # Service to enable/disable idle
        self.srv = self.create_service(SetBool, '/idle/set_enabled', self.handle_set_enabled)

        # internal state
        self.enabled = False
        self._imu_sub = None
        self._baro_sub = None
        self._timer = None

        self.current_alt = None
        self.target_alt = None
        self.vx = 0.0
        self.vy = 0.0

        # parameters (can be tuned)
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('kp_z', 0.5)
        self.declare_parameter('feedforward', 0.5)
        self.declare_parameter('pub_hz', 20.0)

        self.get_logger().info("IdleNode initialized (disabled). Call /idle/set_enabled to enable.")

    def handle_set_enabled(self, req, resp):
        enable = bool(req.data)
        if enable == self.enabled:
            resp.success = True
            resp.message = f"Idle already {'enabled' if enable else 'disabled'}"
            return resp

        if enable:
            self._enable_idle()
            resp.success = True
            resp.message = "Idle enabled"
        else:
            self._disable_idle()
            resp.success = True
            resp.message = "Idle disabled"
        return resp

    def _enable_idle(self):
        # create subscriptions
        if self._imu_sub is None:
            self._imu_sub = self.create_subscription(Imu, '/uav1/imu/data', self._imu_cb, 20)
        if self._baro_sub is None:
            self._baro_sub = self.create_subscription(FluidPressure, '/uav1/global_position/rel_alt', self._baro_cb, 10)
        # start timer
        if self._timer is None:
            hz = float(self.get_parameter('pub_hz').value)
            self._timer = self.create_timer(1.0 / hz, self._idle_timer_cb)
        self.enabled = True
        # set target to current if known
        if self.current_alt is not None:
            self.target_alt = self.current_alt
        self.get_logger().info("Idle enabled: subscribed to IMU and barometer, publishing setpoints.")

    def _disable_idle(self):
        # destroy subscriptions and timer
        if self._imu_sub is not None:
            self.destroy_subscription(self._imu_sub)
            self._imu_sub = None
        if self._baro_sub is not None:
            self.destroy_subscription(self._baro_sub)
            self._baro_sub = None
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        self.enabled = False
        self.get_logger().info("Idle disabled: unsubscribed and stopped publishing.")

    def _imu_cb(self, msg: Imu):
        # Using linear_acceleration as approximation for fast compensation.
        # For proper vx/vy estimation use velocity from EKF or integrate with a filter.
        self.vx = msg.linear_acceleration.x
        self.vy = msg.linear_acceleration.y

    def _baro_cb(self, msg: FluidPressure):
        # Convert pressure to approximate altitude (meters)
        P0 = 101325.0
        P = msg.fluid_pressure
        try:
            self.current_alt = (1.0 - (P / P0) ** 0.1903) * 44330.0
        except Exception:
            self.current_alt = None
        if self.target_alt is None and self.current_alt is not None:
            self.target_alt = self.current_alt

    def _idle_timer_cb(self):
        # publish attitude target to keep UAV hovering
        if self.current_alt is None:
            return

        kp_xy = float(self.get_parameter('kp_xy').value)
        kp_z = float(self.get_parameter('kp_z').value)
        feedforward = float(self.get_parameter('feedforward').value)

        # roll/pitch compensation from IMU (simple proportional to measured accelerations)
        roll = -kp_xy * self.vy
        pitch = kp_xy * self.vx

        # altitude control (P)
        target_alt = getattr(self, 'target_alt', self.current_alt)
        err_z = target_alt - self.current_alt
        thrust = feedforward + kp_z * err_z
        thrust = max(0.0, min(1.0, thrust))

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll, pitch, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = float(thrust)
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self.pub.publish(msg)

    # utility to update target altitude externally
    def set_target_alt(self, alt: float):
        self.target_alt = float(alt)
        self.get_logger().info(f"Idle target altitude set to {self.target_alt:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = IdleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
