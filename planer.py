#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool
from nav2_msgs.action import NavigateToPose
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


class PID:
    def __init__(self, kp, ki, kd, max_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_output = max_output

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.max_output is not None:
            output = max(-self.max_output, min(self.max_output, output))
        return output


class Planner(Node):
    def __init__(self, pid_params=None, v_max=1.0):
        super().__init__('planner')

        # PID параметры (можно настраивать)
        pid_params = pid_params or {'kp': 1.0, 'ki': 0.0, 'kd': 0.5}
        self.pid_x = PID(**pid_params, max_output=v_max)
        self.pid_y = PID(**pid_params, max_output=v_max)
        self.pid_z = PID(**pid_params, max_output=v_max)

        self.v_max = v_max
        self.dt = 0.05  # частота управления 20 Гц

        # Публикация команд
        self.pub = self.create_publisher(
            AttitudeTarget, '/uav1/setpoint_raw/attitude', 10
        )
        self.arm_client = self.create_client(CommandBool, '/uav1/cmd/arming')

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        # Подписка на позицию
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.pos_sub = self.create_subscription(
            PoseStamped, '/uav1/global_position/local', self.pos_callback, 10
        )

        # Текущая цель
        self.target_pos = None

        # Плавный старт
        self.current_speed_factor = 0.0
        self.acceleration = 0.5  # за 2 секунды выйдет на максимум

        # Таймер управления
        self.create_timer(self.dt, self._control_step)

        self.get_logger().info("Planner started and ready")

    def pos_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def arm_uav(self):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return False

        req = CommandBool.Request()
        req.value = True

        try:
            result = self.arm_client.call(req)  # СИНХРОННЫЙ вызов
        except Exception as e:
            self.get_logger().error(f"Arming call failed: {e}")
            return False

        if result and result.success:
            self.get_logger().info("UAV armed successfully")
            return True
        else:
            self.get_logger().error("Failed to arm UAV")
            return False

    def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received")

        if not self.arm_uav():
            goal_handle.abort()
            return NavigateToPose.Result()

        self.target_pos = np.array([
            goal_handle.request.pose.pose.position.x,
            goal_handle.request.pose.pose.position.y,
            goal_handle.request.pose.pose.position.z
        ])
        self.current_speed_factor = 0.0  # сброс плавного старта

        # Цикл ожидания достижения цели
        while rclpy.ok():
            error = np.linalg.norm(self.target_pos - self.current_pos)

            # Feedback
            fb = NavigateToPose.Feedback()
            fb.current_pose = PoseStamped()
            fb.current_pose.pose.position = Point(
                x=float(self.current_pos[0]),
                y=float(self.current_pos[1]),
                z=float(self.current_pos[2])
            )
            goal_handle.publish_feedback(fb)

            if error < 0.05:  # достигли цели
                break

            rclpy.spin_once(self, timeout_sec=self.dt)

        self.get_logger().info("Target reached, hovering forever")
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _control_step(self):
        if self.target_pos is None:
            return

        # Плавный набор скорости
        self.current_speed_factor = min(
            1.0, self.current_speed_factor + self.acceleration * self.dt
        )

        # Ошибка по осям
        error = self.target_pos - self.current_pos
        vx = self.pid_x.update(error[0], self.dt) * self.current_speed_factor
        vy = self.pid_y.update(error[1], self.dt) * self.current_speed_factor
        vz = self.pid_z.update(error[2], self.dt) * self.current_speed_factor

        # Roll/Pitch для коррекции положения
        pitch = math.atan2(vx, G)
        roll = math.atan2(-vy, G)

        # Коррекция thrust с учётом наклона
        cos_term = math.cos(roll) * math.cos(pitch)
        if abs(cos_term) < 1e-3:
            cos_term = 1e-3  # защита от деления на ноль

        desired_force = G + vz
        thrust = desired_force / (G * cos_term)
        thrust = max(0.0, min(1.0, thrust))

        # Публикация команды
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
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    try:
        rclpy.spin(planner)
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
