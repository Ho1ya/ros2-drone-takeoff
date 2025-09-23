import asyncio
import math
import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Range, Image, Imu
from cv_bridge import CvBridge
from std_msgs.msg import String

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw,
)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


class AutonomyNode(Node):
    def __init__(self) -> None:
        super().__init__('autonomy_node')

        # Parameters
        self.declare_parameter('connection_url', 'udp://:14540')
        self.declare_parameter('scan_topic', '')
        self.declare_parameter('range_topic', '')
        self.declare_parameter('qr_text_topic', '/qr_detector/text')
        self.declare_parameter('takeoff_altitude_m', 3.0)
        self.declare_parameter('cruise_speed_m_s', 1.0)
        self.declare_parameter('wall_distance_min_m', 1.0)
        self.declare_parameter('doorway_min_width_rad', 0.35)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('max_yaw_rate_deg_s', 45.0)
        # Optional depth camera + IMU
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('depth_obstacle_distance_m', 1.0)
        self.declare_parameter('depth_center_fraction', 0.25)
        self.declare_parameter('imu_topic', '')

        self.connection_url: str = self.get_parameter('connection_url').get_parameter_value().string_value
        self.scan_topic: str = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.range_topic: str = self.get_parameter('range_topic').get_parameter_value().string_value
        self.qr_text_topic: str = self.get_parameter('qr_text_topic').get_parameter_value().string_value
        self.takeoff_altitude_m: float = float(self.get_parameter('takeoff_altitude_m').value)
        self.cruise_speed_m_s: float = float(self.get_parameter('cruise_speed_m_s').value)
        self.wall_distance_min_m: float = float(self.get_parameter('wall_distance_min_m').value)
        self.doorway_min_width_rad: float = float(self.get_parameter('doorway_min_width_rad').value)
        self.control_period_s: float = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.max_yaw_rate_deg_s: float = float(self.get_parameter('max_yaw_rate_deg_s').value)
        self.depth_topic: str = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.depth_obstacle_distance_m: float = float(self.get_parameter('depth_obstacle_distance_m').value)
        self.depth_center_fraction: float = float(self.get_parameter('depth_center_fraction').value)
        self.imu_topic: str = self.get_parameter('imu_topic').get_parameter_value().string_value

        # State
        self._scan_msg: Optional[LaserScan] = None
        self._range_msg: Optional[Range] = None
        self._depth_center_dist_m: Optional[float] = None
        self._imu_angular_speed: Optional[float] = None
        self._cv_bridge: Optional[CvBridge] = None
        self._qr_text: Optional[str] = None
        self._state_lock = threading.Lock()
        self._running = True

        # Subscriptions
        if self.scan_topic:
            self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 5)
            self.get_logger().info(f'Autonomy: subscribing LaserScan on {self.scan_topic}')
        else:
            self.get_logger().info('Autonomy: LaserScan disabled (scan_topic empty)')

        if self.range_topic:
            self.create_subscription(Range, self.range_topic, self._on_range, 5)
            self.get_logger().info(f'Autonomy: subscribing Range on {self.range_topic}')
        else:
            self.get_logger().info('Autonomy: Range disabled (range_topic empty)')

        if self.depth_topic:
            self._cv_bridge = CvBridge()
            self.create_subscription(Image, self.depth_topic, self._on_depth, 5)
            self.get_logger().info(f'Autonomy: subscribing Depth image on {self.depth_topic}')
        else:
            self.get_logger().info('Autonomy: Depth disabled (depth_topic empty)')

        if self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)
            self.get_logger().info(f'Autonomy: subscribing IMU on {self.imu_topic}')
        else:
            self.get_logger().info('Autonomy: IMU disabled (imu_topic empty)')
        self.create_subscription(String, self.qr_text_topic, self._on_qr_text, 5)

        # Start async main
        asyncio.get_event_loop().create_task(self._run())

    def _on_scan(self, msg: LaserScan) -> None:
        with self._state_lock:
            self._scan_msg = msg

    def _on_qr_text(self, msg: String) -> None:
        with self._state_lock:
            self._qr_text = msg.data.strip() if msg.data else ''

    def _on_range(self, msg: Range) -> None:
        with self._state_lock:
            self._range_msg = msg

    def _on_depth(self, msg: Image) -> None:
        if self._cv_bridge is None:
            return
        try:
            # Depth encoding could be 32FC1 or 16UC1; convert to meters
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        h, w = cv_img.shape[:2]
        cx0 = int(w * (0.5 - self.depth_center_fraction * 0.5))
        cy0 = int(h * (0.5 - self.depth_center_fraction * 0.5))
        cx1 = int(w * (0.5 + self.depth_center_fraction * 0.5))
        cy1 = int(h * (0.5 + self.depth_center_fraction * 0.5))
        roi = cv_img[cy0:cy1, cx0:cx1]
        # Convert to float meters
        roi_f = roi.astype('float32')
        # If 16U assume millimeters
        if msg.encoding.lower() in ('16uc1', 'mono16'):
            roi_f = roi_f / 1000.0
        # Filter invalids
        finite = roi_f[~(roi_f <= 0.0) & ~(roi_f != roi_f) & ~(roi_f == float('inf'))]
        if finite.size > 0:
            center_dist = float(float(finite.mean()))
        else:
            center_dist = None
        with self._state_lock:
            self._depth_center_dist_m = center_dist

    def _on_imu(self, msg: Imu) -> None:
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        ang = math.sqrt(wx * wx + wy * wy + wz * wz)
        with self._state_lock:
            self._imu_angular_speed = float(ang)

    async def _run(self) -> None:
        drone = System()
        try:
            await drone.connect(system_address=self.connection_url)

            self.get_logger().info('Waiting for connection...')
            async for state in drone.core.connection_state():
                if state.is_connected:
                    break

            self.get_logger().info('Waiting for global position...')
            async for h in drone.telemetry.health():
                if h.is_global_position_ok and h.is_home_position_ok:
                    break

            await drone.action.set_takeoff_altitude(self.takeoff_altitude_m)
            await drone.action.arm()
            await drone.action.takeoff()
            self.get_logger().info('Takeoff commanded; waiting to reach altitude...')
            await asyncio.sleep(6.0)

            # Start offboard control in NED frame with yaw heading
            try:
                await drone.offboard.start(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            except OffboardError:
                # Need to set an initial setpoint before start on some firmwares
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await drone.offboard.start()

            self.get_logger().info('Offboard started. Exploring until QR detected...')

            yaw_deg = 0.0
            while rclpy.ok() and self._running:
                qr_text = self._get_qr_text()
                if qr_text:
                    self.get_logger().info(f'QR found: {qr_text}. Initiating safe landing...')
                    break

                scan = self._get_scan()
                rng = self._get_range()
                depth_center = self._get_depth_center()
                imu_ang = self._get_imu_angular_speed()
                vx, vy, vz, yaw_deg = self._compute_cmd(scan, rng, depth_center, imu_ang, yaw_deg)
                try:
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, yaw_deg))
                except OffboardError as exc:  # noqa: F841
                    self.get_logger().warn('Offboard setpoint failed; retrying')

                await asyncio.sleep(self.control_period_s)

            # Hover then land
            try:
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg))
                await asyncio.sleep(0.5)
                await drone.offboard.stop()
            except OffboardError:
                pass

            await self._safe_land(drone)
            self.get_logger().info('Landed. Disarming...')
            await drone.action.disarm()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Autonomy error: {exc}')
        finally:
            rclpy.shutdown()

    def _get_scan(self) -> Optional[LaserScan]:
        with self._state_lock:
            return self._scan_msg

    def _get_qr_text(self) -> Optional[str]:
        with self._state_lock:
            return self._qr_text

    def _get_range(self) -> Optional[Range]:
        with self._state_lock:
            return self._range_msg

    def _get_depth_center(self) -> Optional[float]:
        with self._state_lock:
            return self._depth_center_dist_m

    def _get_imu_angular_speed(self) -> Optional[float]:
        with self._state_lock:
            return self._imu_angular_speed

    def _compute_cmd(self, scan: Optional[LaserScan], rng: Optional[Range], depth_center_m: Optional[float], imu_ang_speed: Optional[float], current_yaw_deg: float) -> Tuple[float, float, float, float]:
        # Default: move forward slowly
        vx = self.cruise_speed_m_s
        vy = 0.0
        vz = 0.0
        yaw_deg = current_yaw_deg

        # If we have depth center, treat as forward obstacle distance
        forward_dist = None
        if depth_center_m is not None:
            forward_dist = depth_center_m
        elif rng is not None and rng.range == rng.range and rng.range != float('inf'):
            forward_dist = rng.range

        if (scan is None or not scan.ranges) and forward_dist is not None:
            if forward_dist < min(self.wall_distance_min_m, self.depth_obstacle_distance_m):
                vx = 0.0
                yaw_deg = current_yaw_deg + self.max_yaw_rate_deg_s * self.control_period_s
            return vx, vy, vz, yaw_deg

        if scan is None or not scan.ranges:
            # no sensors: slowly rotate and crawl
            yaw_deg = current_yaw_deg + 0.5 * self.max_yaw_rate_deg_s * self.control_period_s
            vx = 0.5 * self.cruise_speed_m_s
            return vx, vy, vz, yaw_deg

        # Convert scan to polar sectors; find widest gap with distances > wall_distance_min
        angles: List[float] = []
        for i in range(len(scan.ranges)):
            angles.append(scan.angle_min + i * scan.angle_increment)

        safe_mask = [
            (r if (r == r and r != float('inf')) else 1000.0) > self.wall_distance_min_m for r in scan.ranges
        ]

        # Find contiguous True segments (gaps)
        gaps: List[Tuple[int, int]] = []
        start = None
        for i, ok in enumerate(safe_mask):
            if ok and start is None:
                start = i
            if (not ok or i == len(safe_mask) - 1) and start is not None:
                end = i if not ok else i
                gaps.append((start, end))
                start = None

        # Choose best gap: prefer one centered near 0 angle and wide enough (doorway_min_width)
        best = None
        best_score = -1.0
        for s, e in gaps:
            if e <= s:
                continue
            width_rad = abs(angles[e] - angles[s])
            center_rad = 0.5 * (angles[e] + angles[s])
            score = width_rad - 0.5 * abs(center_rad)  # wide and near forward
            if width_rad >= self.doorway_min_width_rad and score > best_score:
                best = (s, e, center_rad, width_rad)
                best_score = score

        # If a good gap found, steer heading toward its center
        if best is not None:
            _, _, center_rad, width_rad = best
            # Convert desired heading to yaw rate command by adjusting yaw_deg
            desired_yaw_deg = math.degrees(center_rad)
            delta = clamp(desired_yaw_deg - current_yaw_deg, -self.max_yaw_rate_deg_s * self.control_period_s, self.max_yaw_rate_deg_s * self.control_period_s)
            yaw_deg = current_yaw_deg + delta
            # Speed reduce if narrow
            narrow_factor = clamp(width_rad / (self.doorway_min_width_rad * 2.0), 0.4, 1.0)
            vx = self.cruise_speed_m_s * narrow_factor
        else:
            # No gap; rotate in place to search
            vx = 0.0
            # Respect IMU angular speed; if already rotating fast, reduce command
            yaw_step = self.max_yaw_rate_deg_s * self.control_period_s
            if imu_ang_speed is not None and imu_ang_speed > math.radians(60.0):
                yaw_step *= 0.5
            yaw_deg = current_yaw_deg + yaw_step

        return vx, vy, vz, yaw_deg

    async def _safe_land(self, drone: System) -> None:
        try:
            await drone.action.land()
        except Exception:
            self.get_logger().warn('Land action failed, attempting disarm later')
        await asyncio.sleep(8.0)


def main() -> None:
    rclpy.init()
    node = AutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


