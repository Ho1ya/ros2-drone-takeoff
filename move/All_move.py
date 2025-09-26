import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from nav2_msgs.action import NavigateToPose
import numpy as np
import time
import asyncio
import math

G = 9.81  # гравитация для аппроксимации углов из скоростей


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


class Move(Node):
    def __init__(self, publish_hz: float = 20.0):
        super().__init__('move_attitude')

        self.pub = self.create_publisher(AttitudeTarget,
                                         '/mavros/setpoint_raw/attitude',
                                         10)

        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg.orientation = euler_to_quaternion(0.0, 0.0, 0.0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.0

        self._pub_timer = self.create_timer(1.0 / publish_hz, self._publish_current)
        self.get_logger().info(f"Move initialized, publishing {publish_hz} Hz")

    def _publish_current(self):
        self._last_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._last_msg)

    def set_attitude(self, roll=0.0, pitch=0.0, yaw=0.0, thrust=0.0):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll, pitch, yaw)
        msg.body_rate = Vector3()
        msg.thrust = max(0.0, min(1.0, thrust))
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg = msg
        self.get_logger().debug(f"Set attitude roll={roll:.2f}, pitch={pitch:.2f}, thrust={thrust:.2f}")

    def attitude_from_velocity(self, vx, vy, yaw=0.0, thrust=0.5):
        pitch = math.atan2(vx, G)
        roll = math.atan2(-vy, G)
        self.set_attitude(roll=roll, pitch=pitch, yaw=yaw, thrust=thrust)


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self.move = Move()
        self.get_logger().info("Planner ready (NavigateToPose)")

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

    async def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received")

        start = np.array([0.0, 0.0, 0.0])  # упрощение: считаем старт из (0,0,0)
        goal = np.array([goal_handle.request.pose.pose.position.x,
                         goal_handle.request.pose.pose.position.y,
                         goal_handle.request.pose.pose.position.z])

        v_max, a_max, dt = 1.0, 0.5, 0.1
        trajectory = self.plan(start, goal, v_max, a_max, dt)

        t_start = time.time()
        for (t, vx, vy, vz) in trajectory:
            while time.time() - t_start < t:
                await asyncio.sleep(0.001)

            self.move.attitude_from_velocity(vx, vy, yaw=0.0, thrust=0.5)

            fb = NavigateToPose.Feedback()
            fb.current_pose = PoseStamped()
            fb.current_pose.pose.position = Point(x=vx, y=vy, z=vz)
            goal_handle.publish_feedback(fb)

        self.move.set_attitude(0.0, 0.0, 0.0, 0.0)

        result = NavigateToPose.Result()
        self.get_logger().info("Trajectory execution finished")
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    move_node = Move()
    planner_node = Planner()

    # Используем мультипоточный executor, чтобы обе ноды могли работать параллельно
    executor = MultiThreadedExecutor()
    executor.add_node(move_node)
    executor.add_node(planner_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Корректное завершение
        executor.shutdown()
        move_node.destroy_node()
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()