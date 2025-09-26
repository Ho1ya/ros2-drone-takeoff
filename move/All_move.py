import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, PointStamped, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
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
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Planner(Node):
    def __init__(self):
        super().__init__('planner_idle')

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

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        # Idle PID gains
        self.kp_xy = 0.5
        self.kp_z = 0.5
        self.feedforward_z = 0.5

        # State
        self.target_pos = None
        self.target_alt = None
        self.current_pos = None
        self.current_alt = None
        self.idle_enabled = True

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

        # Timer to run Idle loop
        self.create_timer(0.05, self.idle_loop)  # 20Hz

        # Trajectory
        self._trajectory = deque()

        self.get_logger().info("Planner with IDLE initialized")

    # ----- Callbacks -----
    def pos_callback(self, msg: PointStamped):
        self.current_pos = np.array([msg.point.x, msg.point.y])

    def alt_callback(self, msg: PointStamped):
        self.current_alt = msg.point.z
        if self.target_alt is None:
            self.target_alt = self.current_alt

    # ----- Idle control -----
    def idle_loop(self):
        if not self.idle_enabled or self.current_pos is None or self.current_alt is None:
            return

        # Error in X/Y
        error_xy = self.target_pos - self.current_pos if self.target_pos is not None else np.array([0.0, 0.0])
        roll = -self.kp_xy * error_xy[1]  # roll ~ vy
        pitch = self.kp_xy * error_xy[0]  # pitch ~ vx

        # Error in Z
        error_z = self.target_alt - self.current_alt
        thrust = self.feedforward_z + self.kp_z * error_z
        thrust = max(0.0, min(1.0, thrust))

        # Publish
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

    # ----- Planner -----
    def plan(self, start, goal, v_max=1.0, a_max=0.5, dt=0.05):
        start, goal = np.array(start), np.array(goal)
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0, 0.0, 0.0, 0.0)]
        unit_dir = direction / distance
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc ** 2
        if 2 * d_acc > distance:
            v_peak = math.sqrt(a_max * distance)
            t_acc = v_peak / a_max
            t_total = 2 * t_acc
        else:
            d_cruise = distance - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise
        trajectory = []
        t = 0.0
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

    # ----- Action callback -----
    def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received")
        self.idle_enabled = False

        goal = goal_handle.request.pose.pose.position
        self.target_pos = np.array([goal.x, goal.y])
        self.target_alt = goal.z

        # Build trajectory
        start = [0.0, 0.0, 0.0]
        goal_vec = [goal.x, goal.y, goal.z]
        traj = self.plan(start, goal_vec)
        self._trajectory = deque(traj)

        # Execute trajectory
        t_start = time.time()
        while self._trajectory:
            t, vx, vy, vz = self._trajectory.popleft()
            self.attitude_from_velocity(vx, vy, thrust=0.5)
            time.sleep(0.05)

        self.get_logger().info("Goal reached, entering IDLE")
        self.idle_enabled = True
        goal_handle.succeed()
        return NavigateToPose.Result()


# ----- Main -----
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
