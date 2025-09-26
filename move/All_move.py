import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose
import math
import numpy as np
from collections import deque
import time

G = 9.81  # гравитация

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q

class Idle(Node):
    """Держит дрон в позиции x,y,z и компенсирует наклон через IMU"""
    def __init__(self):
        super().__init__('idle')
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)
        self.sub_imu = self.create_subscription(Imu, '/uav1/imu/data', self.imu_callback, 10)
        self.sub_pos = self.create_subscription(PoseStamped, '/uav1/global_position/local', self.pos_callback, 10)
        self.target_pos = None
        self.current_pos = None
        self.current_imu = None
        self.kp_xy = 0.5
        self.kp_z = 0.5
        self.publish_hz = 20.0
        self.timer = self.create_timer(1.0/self.publish_hz, self._hold_position)
        self.active = True

    def imu_callback(self, msg):
        self.current_imu = msg

    def pos_callback(self, msg):
        self.current_pos = msg.pose.position
        if self.target_pos is None:
            self.target_pos = Point(
                x=self.current_pos.x,
                y=self.current_pos.y,
                z=self.current_pos.z
            )

    def _hold_position(self):
        if not self.active or self.current_pos is None or self.current_imu is None or self.target_pos is None:
            return

        # Ошибка по позиции
        ex = self.target_pos.x - self.current_pos.x
        ey = self.target_pos.y - self.current_pos.y
        ez = self.target_pos.z - self.current_pos.z

        # Простая компенсация крена/тангажа по ошибке
        roll_cmd = np.clip(-ey * self.kp_xy, -0.2, 0.2)
        pitch_cmd = np.clip(ex * self.kp_xy, -0.2, 0.2)
        thrust_cmd = np.clip(0.5 + ez*self.kp_z, 0.0, 1.0)

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = thrust_cmd
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self.pub.publish(msg)

    def enable(self):
        self.active = True
        if self.current_pos is not None:
            self.target_pos = Point(
                x=self.current_pos.x,
                y=self.current_pos.y,
                z=self.current_pos.z
            )

    def disable(self):
        self.active = False

class Planner(Node):
    """Принимает NavigateToPose и летит к цели с отключением Idle"""
    def __init__(self, idle_node):
        super().__init__('planner')
        self.idle = idle_node
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self._trajectory = deque()
        self.timer = self.create_timer(0.05, self._step_trajectory)
        self.kp_xy = 0.5
        self.kp_z = 0.5
        self.current_pos = None
        self.sub_pos = self.create_subscription(PoseStamped, '/uav1/global_position/local', self.pos_callback, 10)

    def pos_callback(self, msg):
        self.current_pos = msg.pose.position

    def plan_trajectory(self, start, goal, v_max=1.0, a_max=0.5, dt=0.1):
        start, goal = np.array(start), np.array(goal)
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0,0,0,0)]
        unit_dir = direction / distance
        t_acc = v_max / a_max
        d_acc = 0.5*a_max*t_acc**2
        if 2*d_acc > distance:
            v_peak = np.sqrt(a_max*distance)
            t_acc = v_peak / a_max
            t_total = 2*t_acc
        else:
            d_cruise = distance - 2*d_acc
            t_cruise = d_cruise / v_max
            t_total = 2*t_acc + t_cruise
        trajectory = []
        t = 0.0
        while t <= t_total+dt:
            if t < t_acc:
                v = a_max*t
            elif t < t_total-t_acc:
                v = v_max
            else:
                v = a_max*(t_total-t)
            v_vec = unit_dir*v
            trajectory.append((t,v_vec[0],v_vec[1],v_vec[2]))
            t += dt
        return trajectory

    def _step_trajectory(self):
        if not self._trajectory or self.current_pos is None:
            return
        t, vx, vy, vz = self._trajectory.popleft()
        # Ошибка по позиции
        ex = (self._goal_pos.x - self.current_pos.x)
        ey = (self._goal_pos.y - self.current_pos.y)
        ez = (self._goal_pos.z - self.current_pos.z)
        roll_cmd = np.clip(-ey * self.kp_xy, -0.2, 0.2)
        pitch_cmd = np.clip(ex * self.kp_xy, -0.2, 0.2)
        thrust_cmd = np.clip(0.5 + ez*self.kp_z, 0.0,1.0)
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = thrust_cmd
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self.pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received, disabling Idle")
        self.idle.disable()
        self._goal_pos = goal_handle.request.pose.pose.position
        start = [self.current_pos.x, self.current_pos.y, self.current_pos.z] if self.current_pos else [0,0,0]
        goal = [self._goal_pos.x, self._goal_pos.y, self._goal_pos.z]
        self._trajectory = deque(self.plan_trajectory(start, goal))
        goal_handle.succeed()
        self.get_logger().info("Trajectory loaded, Idle will resume after completion")
        # по завершению траектории включаем Idle
        self.idle.enable()
        return NavigateToPose.Result()

def main(args=None):
    rclpy.init(args=args)
    idle_node = Idle()
    planner_node = Planner(idle_node)
    try:
        rclpy.spin(planner_node)
    finally:
        planner_node.destroy_node()
        idle_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
