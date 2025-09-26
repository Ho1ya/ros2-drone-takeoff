import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from nav2_msgs.action import NavigateToPose
import math
import numpy as np
from collections import deque

DEBUG = 1  # 0 - логи минимальны, 1 - детальные

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

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self._last_msg.orientation = euler_to_quaternion(0,0,0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.5

        self._trajectory = deque()
        self._target_altitude = 0.5

        # Таймер для выполнения траектории
        self.create_timer(0.05, self._step_trajectory)
        self.get_logger().info("Planner ready and publishing /mavros/setpoint_raw/attitude")

    def _step_trajectory(self):
        if not self._trajectory:
            return
        t, vx, vy, vz = self._trajectory.popleft()

        pitch = math.atan2(vx, G)
        roll = math.atan2(-vy, G)
        thrust = max(0.0, min(1.0, 0.5 + 0.4*(self._target_altitude - vz)))

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

        if DEBUG:
            self.get_logger().info(
                f"[DEBUG] Step t={t:.2f}s | vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} | "
                f"roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, thrust={thrust:.2f}"
            )

    def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received")
        start = [0.0,0.0,0.0]
        goal = [
            goal_handle.request.pose.pose.position.x,
            goal_handle.request.pose.pose.position.y,
            goal_handle.request.pose.pose.position.z
        ]
        trajectory = self.plan(start, goal)
        self._trajectory = deque(trajectory)
        self._target_altitude = goal[2]

        if DEBUG:
            self.get_logger().info(f"[DEBUG] Goal: x={goal[0]:.2f}, y={goal[1]:.2f}, z={goal[2]:.2f}")
            self.get_logger().info(f"[DEBUG] Trajectory steps: {len(trajectory)}")

        # feedback
        for t,vx,vy,vz in trajectory:
            fb = NavigateToPose.Feedback()
            fb.current_pose = PoseStamped()
            fb.current_pose.pose.position = Point(x=vx, y=vy, z=vz)
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        self.get_logger().info("Trajectory finished")
        return NavigateToPose.Result()

    def plan(self, start, goal, v_max=1.0, a_max=0.5, dt=0.1):
        start, goal = np.array(start), np.array(goal)
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0,0.0,0.0,0.0)]
        unit_dir = direction / distance
        t_acc = v_max / a_max
        d_acc = 0.5*a_max*t_acc**2
        if 2*d_acc > distance:
            v_peak = math.sqrt(a_max*distance)
            t_acc = v_peak / a_max
            t_total = 2*t_acc
        else:
            d_cruise = distance-2*d_acc
            t_cruise = d_cruise/v_max
            t_total = 2*t_acc + t_cruise
        trajectory=[]
        t=0.0
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
        if DEBUG:
            self.get_logger().info(f"[DEBUG] Planned trajectory length: {len(trajectory)} steps")
        return trajectory

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
