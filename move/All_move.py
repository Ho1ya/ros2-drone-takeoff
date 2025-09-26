#!/usr/bin/env python3
# planner.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool
import math
import numpy as np
from collections import deque
import time
import collections

DEBUG = True
G = 9.81

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

# --- small LPF and simple velocity estimator (from IMU linear_acceleration) ---
class LowPass:
    def __init__(self, tau, dt, init=0.0):
        self.tau = float(tau)
        self.dt = float(dt)
        self.alpha = (self.dt) / (self.tau + self.dt) if (self.tau + self.dt) != 0 else 1.0
        self.y = float(init)
    def update(self, x):
        self.y += self.alpha * (x - self.y)
        return self.y

class VelocityEstimator:
    """
    Very simple estimator:
    - keep moving window to estimate accel bias
    - integrate accel-bias to get velocity and low-pass it
    """
    def __init__(self, dt=0.02, bias_window=200, lpf_tau=0.5):
        self.dt = dt
        self.vx = 0.0
        self.vy = 0.0
        self.buffer_x = collections.deque(maxlen=bias_window)
        self.buffer_y = collections.deque(maxlen=bias_window)
        self.lpf_vx = LowPass(lpf_tau, dt)
        self.lpf_vy = LowPass(lpf_tau, dt)

    def update(self, ax, ay, dt=None):
        if dt is None:
            dt = self.dt
        else:
            self.dt = dt
            # update LPF alpha for new dt
            self.lpf_vx.dt = dt
            self.lpf_vy.dt = dt
            self.lpf_vx.alpha = (self.lpf_vx.dt) / (self.lpf_vx.tau + self.lpf_vx.dt)
            self.lpf_vy.alpha = (self.lpf_vy.dt) / (self.lpf_vy.tau + self.lpf_vy.dt)

        self.buffer_x.append(ax)
        self.buffer_y.append(ay)
        bias_x = sum(self.buffer_x) / len(self.buffer_x)
        bias_y = sum(self.buffer_y) / len(self.buffer_y)
        ax_unbiased = ax - bias_x
        ay_unbiased = ay - bias_y
        self.vx += ax_unbiased * dt
        self.vy += ay_unbiased * dt
        self.vx = self.lpf_vx.update(self.vx)
        self.vy = self.lpf_vy.update(self.vy)
        return self.vx, self.vy

    def reset(self):
        self.vx = 0.0
        self.vy = 0.0
        self.buffer_x.clear()
        self.buffer_y.clear()

class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        # publisher for attitude commands
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)

        # arm service client (optional)
        self.arm_client = self.create_client(CommandBool, '/uav1/cmd/arming')

        # idle service client (to disable/enable the IdleNode)
        self.idle_client = self.create_client(SetBool, '/idle/set_enabled')

        # action server: NavigateToPose for compatibility with nav2-ish callers
        self._action_server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute_callback)

        # trajectory queue and timers
        self._trajectory = deque()
        self._target_altitude = 0.5
        self.create_timer(0.05, self._step_trajectory)  # 20 Hz execution step

        # zero-attitude timer placeholders
        self._zero_timer = None
        self._zero_count = 0

        # velocity estimator using IMU
        self.vel_est = VelocityEstimator(dt=0.02, bias_window=200, lpf_tau=0.3)
        self._last_imu_time = None
        self.sub_imu = self.create_subscription(
            # read linear_acceleration from IMU to estimate vx/vy
            __import__('sensor_msgs.msg', fromlist=['Imu']).Imu,
            '/uav1/imu/data',
            self.imu_callback,
            50
        )

        # optional baro subscription if we want to read altitude locally
        self.current_alt = None
        self.sub_baro = self.create_subscription(
            __import__('sensor_msgs.msg', fromlist=['FluidPressure']).FluidPressure,
            '/uav1/global_position/rel_alt',
            self.baro_callback,
            10
        )

        self.get_logger().info("Planner ready (navigate_to_pose). Uses Idle service to coordinate control.")

    # -------------------------
    # utilities: idle service call
    # -------------------------
    def _set_idle(self, enable: bool, timeout=2.0) -> bool:
        if not self.idle_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning("Idle service not available")
            return False
        req = SetBool.Request()
        req.data = bool(enable)
        fut = self.idle_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            self.get_logger().info(f"Idle service replied: success={res.success}, msg='{res.message}'")
            return res.success
        else:
            self.get_logger().warning("Idle service call failed or timed out")
            return False

    def arm_uav(self):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("Arming service not available")
            return False
        req = CommandBool.Request()
        req.value = True
        fut = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        try:
            return fut.result().success
        except Exception:
            return False

    # -------------------------
    # IMU / Barometer callbacks
    # -------------------------
    def imu_callback(self, msg):
        # estimate dt from clock (best-effort)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_imu_time is None:
            dt = self.vel_est.dt
        else:
            dt = now - self._last_imu_time
            if dt <= 0 or dt > 0.5:
                dt = self.vel_est.dt
        self._last_imu_time = now
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        vx, vy = self.vel_est.update(ax, ay, dt)
        # store estimated velocities for use in compensation
        self.vx = vx
        self.vy = vy

    def baro_callback(self, msg):
        try:
            P0 = 101325.0
            P = msg.fluid_pressure
            self.current_alt = (1.0 - (P / P0) ** 0.1903) * 44330.0
        except Exception:
            self.current_alt = None

    # -------------------------
    # zero attitude (non-blocking) - publishes N zero setpoints via timer
    # -------------------------
    def send_zero_attitude(self, count=10, hz=20.0):
        self._zero_count = int(count)
        period = 1.0 / float(hz)

        def timer_cb():
            if self._zero_count <= 0:
                # cancel timer
                try:
                    self._zero_timer.cancel()
                except Exception:
                    pass
                self._zero_timer = None
                return
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.orientation = euler_to_quaternion(0.0, 0.0, 0.0)
            msg.body_rate = Vector3()
            # keep a small positive thrust so we don't drop; actual thrust controlled by planner
            msg.thrust = 0.5
            msg.type_mask = (
                AttitudeTarget.IGNORE_ROLL_RATE |
                AttitudeTarget.IGNORE_PITCH_RATE |
                AttitudeTarget.IGNORE_YAW_RATE
            )
            self.pub.publish(msg)
            if DEBUG:
                self.get_logger().info("[DEBUG] zero attitude pub")
            self._zero_count -= 1

        # create timer
        self._zero_timer = self.create_timer(period, timer_cb)

    # -------------------------
    # Trajectory planning and execution
    # -------------------------
    def plan(self, start, goal, v_max=1.0, a_max=0.5, dt=0.1):
        start = np.array(start); goal = np.array(goal)
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0,0.0,0.0,0.0)]
        unit = direction / distance
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc**2
        if 2*d_acc > distance:
            v_peak = math.sqrt(a_max * distance)
            t_acc = v_peak / a_max
            t_total = 2*t_acc
        else:
            d_cruise = distance - 2*d_acc
            t_cruise = d_cruise / v_max
            t_total = 2*t_acc + t_cruise
        traj=[]
        t=0.0
        while t <= t_total + dt:
            if t < t_acc:
                v = a_max * t
            elif t < t_total - t_acc:
                v = v_max
            else:
                v = a_max * (t_total - t)
            v_vec = unit * v
            traj.append((t, float(v_vec[0]), float(v_vec[1]), float(v_vec[2])))
            t += dt
        if DEBUG:
            self.get_logger().info(f"[DEBUG] Planned trajectory: {len(traj)} steps")
        return traj

    def _step_trajectory(self):
        if not self._trajectory:
            return
        # get next step
        t, vx_cmd, vy_cmd, vz_cmd = self._trajectory.popleft()

        # tracking controller: accelerate towards desired speed (compensate inertia)
        K_track = 1.2  # tune this
        a_des_x = K_track * (vx_cmd - getattr(self, 'vx', 0.0))
        a_des_y = K_track * (vy_cmd - getattr(self, 'vy', 0.0))

        # translate desired accel to small angles
        pitch = math.atan2(a_des_x, G)
        roll  = math.atan2(-a_des_y, G)

        # altitude: use stored _target_altitude; simple P (planner maintains _target_altitude)
        thrust = max(0.0, min(1.0, 0.5 + 0.4 * (self._target_altitude - vz_cmd)))

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
        if DEBUG:
            self.get_logger().info(f"[STEP] vx_cmd={vx_cmd:.2f}, vy_cmd={vy_cmd:.2f}, est_vx={getattr(self,'vx',0.0):.2f}, est_vy={getattr(self,'vy',0.0):.2f}, roll={math.degrees(roll):.1f}, pitch={math.degrees(pitch):.1f}, thrust={thrust:.2f}")

    # -------------------------
    # Action callback (non-blocking)
    # -------------------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("NavigateToPose goal received")

        # 1) disable idle so planner has exclusive control (best-effort)
        idle_disabled = self._set_idle(False)
        if not idle_disabled:
            self.get_logger().warning("Unable to disable idle; proceeding anyway")

        # 2) send a few zeroed attitude setpoints (non-blocking)
        self.send_zero_attitude(count=10, hz=20.0)

        # 3) optionally arm UAV (best-effort)
        if not self.arm_uav():
            self.get_logger().warning("Arming failed or service missing; proceeding anyway")

        # 4) plan and start trajectory (execution is done in _step_trajectory timer)
        start = [0.0, 0.0, 0.0]
        goal = [
            goal_handle.request.pose.pose.position.x,
            goal_handle.request.pose.pose.position.y,
            goal_handle.request.pose.pose.position.z
        ]
        trajectory = self.plan(start, goal)
        self._trajectory = deque(trajectory)
        # set target altitude for thrust control (planner uses goal z)
        self._target_altitude = goal[2]

        # publish initial debug / feedback once
        if DEBUG:
            self.get_logger().info(f"[DEBUG] Goal: x={goal[0]:.2f}, y={goal[1]:.2f}, z={goal[2]:.2f}")
            self.get_logger().info(f"[DEBUG] Trajectory steps: {len(trajectory)}")

        # publish feedback a single time (clients may expect streaming feedback; we keep it simple)
        for t,vx,vy,vz in trajectory:
            fb = NavigateToPose.Feedback()
            fb.current_pose = PoseStamped()
            fb.current_pose.pose.position = Point(x=vx, y=vy, z=vz)
            goal_handle.publish_feedback(fb)

        # 5) return success immediately — execution continues in background timers
        # Optionally, you can keep goal_handle and call succeed() from a completion callback when the traj is finished.
        # For simplicity we return now (non-blocking); IDLE must be re-enabled after trajectory ends
        # We'll spin a small waiter to ensure the trajectory timer started (non-blocking)
        goal_handle.succeed()

        # re-enable idle when trajectory completes (we can't block here; so spawn a tiny waiter on background via timer)
        def completion_checker():
            if not self._trajectory:
                # trajectory finished -> re-enable idle
                self._set_idle(True)
                # cancel this checker timer
                try:
                    checker.cancel()
                except Exception:
                    pass

        # create a short-living timer to monitor trajectory completion
        checker = self.create_timer(0.1, completion_checker)

        self.get_logger().info("Trajectory started (planner returned).")
        return NavigateToPose.Result()

def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
