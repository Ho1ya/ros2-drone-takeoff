import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure
from mavros_msgs.msg import AttitudeTarget
from std_msgs.msg import Bool
import math
import time
import numpy as np

G = 9.81  # гравитация для аппроксимации наклонов


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
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


class Planner(Node):
    def __init__(self):
        super().__init__('planner_idle')

        # Publisher AttitudeTarget
        self.pub = self.create_publisher(AttitudeTarget,
                                         '/mavros/setpoint_raw/attitude',
                                         10)

        # ----------------- Sensors -----------------
        self.vx = 0.0
        self.vy = 0.0
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.current_alt = None
        self.target_alt = None
        self.sub_barometer = self.create_subscription(FluidPressure, '/barometer', self.barometer_callback, 10)

        # ----------------- IDLE -----------------
        self.idle_enabled = True
        self.kp_xy = 0.2
        self.kp_z = 0.5
        self.feedforward = 0.5
        self.idle_publish_hz = 20.0
        self.create_timer(1.0 / self.idle_publish_hz, self.idle_callback)

        # Внешнее управление IDLE
        self.create_subscription(Bool, '/idle_enable', self.idle_cmd_callback, 10)

        # Последнее сообщение
        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg.orientation = euler_to_quaternion(0, 0, 0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.0

        # ----------------- Planner Trajectory -----------------
        self.trajectory = []  # список (t, vx, vy, vz)
        self.traj_index = 0
        self.traj_start_time = None
        self.traj_timer = self.create_timer(0.01, self.trajectory_callback)  # 100 Hz для исполнения

        self.get_logger().info("Planner with IMU + Barometer IDLE initialized")

    # ----------------- CALLBACKS -----------------
    def imu_callback(self, msg: Imu):
        self.vx = msg.linear_acceleration.x
        self.vy = msg.linear_acceleration.y

    def barometer_callback(self, msg: FluidPressure):
        P0 = 101325.0
        P = msg.fluid_pressure
        self.current_alt = (1.0 - (P / P0) ** 0.1903) * 44330.0
        if self.target_alt is None:
            self.target_alt = self.current_alt

    def idle_cmd_callback(self, msg: Bool):
        self.idle_enabled = msg.data
        self.get_logger().info(f"IDLE {'enabled' if self.idle_enabled else 'disabled'}")

    # ----------------- IDLE -----------------
    def idle_callback(self):
        if not self.idle_enabled or self.current_alt is None or self.trajectory:
            return

        roll = -self.kp_xy * self.vy
        pitch = self.kp_xy * self.vx
        error_z = self.target_alt - self.current_alt
        thrust = self.feedforward + self.kp_z * error_z
        thrust = max(0.0, min(1.0, thrust))

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

    # ----------------- Trajectory -----------------
    def plan(self, p0, p1, v_max=1.0, a_max=0.5, dt=0.1):
        p0, p1 = np.array(p0), np.array(p1)
        direction = p1 - p0
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0, 0.0, 0.0, 0.0)]
        unit_dir = direction / distance

        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc ** 2

        if 2 * d_acc > distance:
            v_peak = np.sqrt(a_max * distance)
            t_acc = v_peak / a_max
            t_total = 2 * t_acc
        else:
            d_cruise = distance - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise

        t = 0.0
        trajectory = []
        while t <= t_total + dt:
            if t < t_acc:
                v = a_max * t
            elif t < t_total - t_acc:
                v = v_max
            else:
                v = a_max * (t_total - t)
            v_vec = unit_dir * v
            trajectory.append((t, v_vec[0], v_vec[1], v_vec[2]))
            t += dt
        return trajectory

    def attitude_from_velocity(self, vx, vy, yaw=0.0, thrust=0.5):
        pitch = math.atan2(vx, G)
        roll = math.atan2(-vy, G)
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll, pitch, yaw)
        msg.body_rate = Vector3()
        msg.thrust = thrust
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg = msg
        self.pub.publish(msg)

    # ----------------- Trajectory Execution -----------------
    def execute_trajectory(self, start, goal):
        self.idle_enabled = False
        self.trajectory = self.plan(start, goal)
        self.traj_index = 0
        self.traj_start_time = time.time()
        self.target_alt = goal[2]

    def trajectory_callback(self):
        if not self.trajectory:
            return

        t_elapsed = time.time() - self.traj_start_time
        if self.traj_index >= len(self.trajectory):
            # Завершили траекторию
            self.trajectory = []
            self.idle_enabled = True
            self._last_msg.thrust = 0.5  # держим hover
            return

        t, vx, vy, vz = self.trajectory[self.traj_index]
        if t_elapsed >= t:
            self.attitude_from_velocity(vx, vy, thrust=0.5)
            self.traj_index += 1

    # ----------------- Action Callback -----------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("Goal received, executing trajectory")
        start = [0.0, 0.0, self.current_alt or 0.0]
        goal = [goal_handle.request.pose.pose.position.x,
                goal_handle.request.pose.pose.position.y,
                goal_handle.request.pose.pose.position.z]

        self.execute_trajectory(start, goal)
        # Сразу помечаем goal как успешный, потому что выполнение идёт в таймере
        goal_handle.succeed()
        return goal_handle.result


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
