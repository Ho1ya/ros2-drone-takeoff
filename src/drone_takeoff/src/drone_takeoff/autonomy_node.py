import asyncio
import math
import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
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
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('qr_text_topic', '/qr_detector/text')
        self.declare_parameter('takeoff_altitude_m', 3.0)
        self.declare_parameter('cruise_speed_m_s', 1.0)
        self.declare_parameter('wall_distance_min_m', 1.0)
        self.declare_parameter('doorway_min_width_rad', 0.35)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('max_yaw_rate_deg_s', 45.0)

        self.connection_url: str = self.get_parameter('connection_url').get_parameter_value().string_value
        self.scan_topic: str = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.qr_text_topic: str = self.get_parameter('qr_text_topic').get_parameter_value().string_value
        self.takeoff_altitude_m: float = float(self.get_parameter('takeoff_altitude_m').value)
        self.cruise_speed_m_s: float = float(self.get_parameter('cruise_speed_m_s').value)
        self.wall_distance_min_m: float = float(self.get_parameter('wall_distance_min_m').value)
        self.doorway_min_width_rad: float = float(self.get_parameter('doorway_min_width_rad').value)
        self.control_period_s: float = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.max_yaw_rate_deg_s: float = float(self.get_parameter('max_yaw_rate_deg_s').value)

        # State
        self._scan_msg: Optional[LaserScan] = None
        self._qr_text: Optional[str] = None
        self._state_lock = threading.Lock()
        self._running = True

        # Subscriptions
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 5)
        self.create_subscription(String, self.qr_text_topic, self._on_qr_text, 5)

        # Start async main
        asyncio.get_event_loop().create_task(self._run())

    def _on_scan(self, msg: LaserScan) -> None:
        with self._state_lock:
            self._scan_msg = msg

    def _on_qr_text(self, msg: String) -> None:
        with self._state_lock:
            self._qr_text = msg.data.strip() if msg.data else ''

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
                vx, vy, vz, yaw_deg = self._compute_cmd_from_scan(scan, yaw_deg)
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

    def _compute_cmd_from_scan(self, scan: Optional[LaserScan], current_yaw_deg: float) -> Tuple[float, float, float, float]:
        # Default: move forward slowly
        vx = self.cruise_speed_m_s
        vy = 0.0
        vz = 0.0
        yaw_deg = current_yaw_deg

        if scan is None or not scan.ranges:
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
            yaw_deg = current_yaw_deg + self.max_yaw_rate_deg_s * self.control_period_s

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


