import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3, Quaternion
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from collections import deque
import numpy as np
import math

G = 9.81  # гравитация

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)
    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q

def quaternion_to_euler(q: Quaternion):
    # Возвращает roll, pitch, yaw
    w,x,y,z = q.w,q.x,q.y,q.z
    t0 = +2.0*(w*x + y*z)
    t1 = +1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(t0,t1)
    t2 = +2.0*(w*y - z*x)
    t2 = +1.0 if t2>+1.0 else t2
    t2 = -1.0 if t2<-1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0*(w*z + x*y)
    t4 = +1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(t3,t4)
    return roll, pitch, yaw

class Idle(Node):
    def __init__(self):
        super().__init__('idle')
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)
        self.sub_local = self.create_subscription(PointStamped, '/uav1/global_position/local', self.local_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/uav1/imu/data', self.imu_callback, 10)

        self.current_pos = np.array([0.0,0.0,0.0])
        self.current_orientation = Quaternion()
        self.enabled = True
        self.target_pos = np.array([0.0,0.0,0.0])

        # ПИД коэффициенты
        self.kp_xy = 0.6
        self.kp_z = 0.5

        self.timer = self.create_timer(0.05, self.update)

        self._last_msg = AttitudeTarget()
        self._last_msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE |
                                    AttitudeTarget.IGNORE_PITCH_RATE |
                                    AttitudeTarget.IGNORE_YAW_RATE)
        self._last_msg.orientation = euler_to_quaternion(0,0,0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.5

    def enable(self):
        self.enabled = True
        self.target_pos[:] = self.current_pos

    def disable(self):
        self.enabled = False

    def local_callback(self, msg: PointStamped):
        self.current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.enabled and np.all(self.target_pos==0.0):
            self.target_pos[:] = self.current_pos

    def imu_callback(self, msg: Imu):
        self.current_orientation = msg.orientation

    def update(self):
        if not self.enabled:
            return
        error = self.target_pos - self.current_pos
        thrust = 0.5 + self.kp_z*error[2]
        thrust = np.clip(thrust, 0.0, 1.0)

        roll_desired = self.kp_xy*error[1]  # Y -> roll
        pitch_desired = self.kp_xy*error[0]  # X -> pitch

        roll_actual, pitch_actual, _ = quaternion_to_euler(self.current_orientation)
        roll_cmd = roll_desired - roll_actual
        pitch_cmd = pitch_desired - pitch_actual

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = thrust
        msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE |
                         AttitudeTarget.IGNORE_PITCH_RATE |
                         AttitudeTarget.IGNORE_YAW_RATE)
        self._last_msg = msg
        self.pub.publish(msg)

class Planner(Node):
    def __init__(self, idle_node):
        super().__init__('planner')
        self.idle = idle_node
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)
        self._action_server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute_callback)
        self._trajectory = deque()
        self._target_pos = np.array([0.0,0.0,0.0])
        self.sub_imu = self.create_subscription(Imu, '/uav1/imu/data', self.imu_callback, 10)
        self.current_orientation = Quaternion()
        self.create_timer(0.05, self._step_trajectory)

    def imu_callback(self, msg: Imu):
        self.current_orientation = msg.orientation

    def plan(self, start, goal, v_max=1.0, a_max=0.5, dt=0.05):
        start, goal = np.array(start), np.array(goal)
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance==0:
            return [(0.0,0.0,0.0,0.0)]
        unit_dir = direction / distance
        t_acc = v_max / a_max
        d_acc = 0.5*a_max*t_acc**2
        if 2*d_acc>distance:
            v_peak = math.sqrt(a_max*distance)
            t_acc = v_peak/a_max
            t_total = 2*t_acc
        else:
            d_cruise = distance-2*d_acc
            t_cruise = d_cruise/v_max
            t_total = 2*t_acc + t_cruise
        traj=[]
        t=0.0
        while t<=t_total+dt:
            if t<t_acc:
                v=a_max*t
            elif t<t_total-t_acc:
                v=v_max
            else:
                v=a_max*(t_total-t)
            v_vec = unit_dir*v
            traj.append((t,v_vec[0],v_vec[1],v_vec[2]))
            t+=dt
        return traj

    def _step_trajectory(self):
        if not self._trajectory:
            return
        t,vx,vy,vz = self._trajectory.popleft()
        pitch_des = math.atan2(vx, G)
        roll_des = math.atan2(-vy, G)

        roll_actual, pitch_actual, _ = quaternion_to_euler(self.current_orientation)
        roll_cmd = roll_des - roll_actual
        pitch_cmd = pitch_des - pitch_actual

        thrust = np.clip(0.5 + 0.5*(self._target_pos[2]-vz),0.0,1.0)

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = thrust
        msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE |
                         AttitudeTarget.IGNORE_PITCH_RATE |
                         AttitudeTarget.IGNORE_YAW_RATE)
        self.pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.idle.disable()
        start = self.idle.current_pos
        goal = np.array([goal_handle.request.pose.pose.position.x,
                         goal_handle.request.pose.pose.position.y,
                         goal_handle.request.pose.pose.position.z])
        self._trajectory = deque(self.plan(start, goal))
        self._target_pos = goal

        for t,vx,vy,vz in self._trajectory:
            fb = NavigateToPose.Feedback()
            fb.current_pose.pose.position.x = vx
            fb.current_pose.pose.position.y = vy
            fb.current_pose.pose.position.z = vz
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
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
