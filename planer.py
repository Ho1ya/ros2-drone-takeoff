import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, PointStamped, Vector3, Quaternion
from mavros_msgs.msg import AttitudeTarget
from nav2_msgs.action import NavigateToPose
import math
import time
import asyncio
import numpy as np

G = 9.81  # гравитация для аппроксимации углов

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
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
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        # Publisher для AttitudeTarget
        self.pub = self.create_publisher(AttitudeTarget,
                                         '/mavros/setpoint_raw/attitude',
                                         10)

        # Подписка на высоту
        self.current_alt = None
        self.target_alt = None
        self.sub_alt = self.create_subscription(PointStamped,
                                               '/point/raw/altitude',
                                               self.alt_callback, 10)

        # IDLE параметры
        self.idle_enabled = True
        self.kp_idle = 0.5
        self.feedforward_idle = 0.5
        self.idle_publish_hz = 20.0
        self.create_timer(1.0 / self.idle_publish_hz, self.idle_callback)

        # Хранение последнего сообщения для публикации
        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg.orientation = euler_to_quaternion(0,0,0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.0

        self.get_logger().info("Planner with IDLE initialized")

    def alt_callback(self, msg: PointStamped):
        self.current_alt = msg.point.z
        if self.target_alt is None:
            self.target_alt = self.current_alt

    def idle_callback(self):
        if not self.idle_enabled or self.current_alt is None:
            return

        # Простое удержание высоты через P-контроллер
        error = self.target_alt - self.current_alt
        thrust = self.feedforward_idle + self.kp_idle * error
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

        self._last_msg = msg
        self.pub.publish(msg)

    def plan(self, p0, p1, v_max=1.0, a_max=0.5, dt=0.1):
        p0, p1 = np.array(p0), np.array(p1)
        direction = p1 - p0
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0, 0.0, 0.0, 0.0)]
        unit_dir = direction / distance

        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc**2

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

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Goal received, exiting IDLE")
        self.idle_enabled = False  # выход из IDLE

        start = [0.0, 0.0, 0.0]  # упрощение
        goal = [goal_handle.request.pose.pose.position.x,
                goal_handle.request.pose.pose.position.y,
                goal_handle.request.pose.pose.position.z]

        trajectory = self.plan(start, goal, v_max=1.0, a_max=0.5, dt=0.1)

        t_start = time.time()
        for (t, vx, vy, vz) in trajectory:
            while time.time() - t_start < t:
                await asyncio.sleep(0.001)
            self.attitude_from_velocity(vx, vy, yaw=0.0, thrust=0.5)

            fb = NavigateToPose.Feedback()
            fb.current_pose.pose.position = Point(x=vx, y=vy, z=vz)
            goal_handle.publish_feedback(fb)

        self.get_logger().info("Trajectory finished, entering IDLE")
        self.idle_enabled = True  # снова включаем IDLE

        result = NavigateToPose.Result()
        goal_handle.succeed()
        return result


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
