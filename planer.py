import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from nav2_msgs.action import NavigateToPose
import math
import time
import asyncio
import numpy as np

G = 9.81

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
        self._last_msg.orientation = euler_to_quaternion(0.0, 0.0, 0.0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.5

        # публикуем сетпоинт постоянно
        self.create_timer(0.05, self._publish_current)

        self.get_logger().info("Planner ready (no Move import)")

    def _publish_current(self):
        self._last_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._last_msg)

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

    async def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received")
        start = [0.0,0.0,0.0]
        goal = [
            goal_handle.request.pose.pose.position.x,
            goal_handle.request.pose.pose.position.y,
            goal_handle.request.pose.pose.position.z
        ]
        trajectory = self.plan(start, goal)

        t_start = time.time()
        for t, vx, vy, vz in trajectory:
            while time.time() - t_start < t:
                await asyncio.sleep(0.001)
            self.attitude_from_velocity(vx, vy, yaw=0.0, thrust=0.5)
            fb = NavigateToPose.Feedback()
            fb.current_pose = PoseStamped()
            fb.current_pose.pose.position = Point(x=vx, y=vy, z=vz)
            goal_handle.publish_feedback(fb)

        # стоп
        self._last_msg.thrust = 0.0
        self._last_msg.orientation = euler_to_quaternion(0,0,0)
        self.get_logger().info("Trajectory finished")
        goal_handle.succeed()
        return NavigateToPose.Result()

    def plan(self, start, goal, v_max=1.0, a_max=0.5, dt=0.1):
        start, goal = np.array(start), np.array(goal)
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0,0.0,0.0,0.0)]
        unit_dir = direction / distance
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc**2
        if 2*d_acc>distance:
            v_peak = math.sqrt(a_max*distance)
            t_acc = v_peak/a_max
            t_total = 2*t_acc
        else:
            d_cruise = distance-2*d_acc
            t_cruise = d_cruise/v_max
            t_total = 2*t_acc + t_cruise
        trajectory=[]
        t=0.0
        while t<=t_total+dt:
            if t<t_acc:
                v = a_max*t
            elif t<t_total-t_acc:
                v = v_max
            else:
                v = a_max*(t_total-t)
            v_vec = unit_dir*v
            trajectory.append((t,v_vec[0],v_vec[1],v_vec[2]))
            t+=dt
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
