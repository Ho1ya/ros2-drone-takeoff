import asyncio
import rclpy
from rclpy.node import Node

from mavsdk import System


class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        self.declare_parameter('connection_url', 'udp://:14540')
        self.declare_parameter('takeoff_altitude_m', 5.0)
        self.declare_parameter('arm_timeout_s', 10.0)
        self.declare_parameter('connect_timeout_s', 20.0)

        self.connection_url: str = self.get_parameter('connection_url').get_parameter_value().string_value
        self.takeoff_altitude_m: float = self.get_parameter('takeoff_altitude_m').get_parameter_value().double_value
        self.arm_timeout_s: float = float(self.get_parameter('arm_timeout_s').value)
        self.connect_timeout_s: float = float(self.get_parameter('connect_timeout_s').value)

        self.get_logger().info(
            f"Starting MAVSDK connection to {self.connection_url}, takeoff_altitude={self.takeoff_altitude_m} m"
        )

        # Run the async workflow in background
        asyncio.get_event_loop().create_task(self._run())

    async def _run(self) -> None:
        try:
            drone = System()
            await drone.connect(system_address=self.connection_url)

            self.get_logger().info('Waiting for system to connect...')
            try:
                await asyncio.wait_for(self._wait_is_connected(drone), timeout=self.connect_timeout_s)
            except asyncio.TimeoutError:
                self.get_logger().error('Timeout waiting for vehicle connection')
                rclpy.shutdown()
                return

            self.get_logger().info('Connected. Waiting for global position...')
            await self._wait_has_global_position(drone)
            self.get_logger().info('Global position OK. Arming...')

            await drone.action.set_takeoff_altitude(self.takeoff_altitude_m)
            await drone.action.arm()

            self.get_logger().info('Taking off...')
            await drone.action.takeoff()

            await asyncio.sleep(8.0)
            self.get_logger().info('Takeoff command sent. Holding position.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error during takeoff sequence: {exc}')
        finally:
            # Keep node alive; user can Ctrl+C or kill launch
            pass

    async def _wait_is_connected(self, drone: System) -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    async def _wait_has_global_position(self, drone: System) -> None:
        async for hpos in drone.telemetry.health():
            if hpos.is_global_position_ok and hpos.is_home_position_ok:
                return


def main() -> None:
    rclpy.init()
    node = TakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


