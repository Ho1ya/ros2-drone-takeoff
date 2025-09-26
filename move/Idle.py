# idle_node.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import AttitudeTarget, Vector3
import math

G = 9.81

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = AttitudeTarget().orientation
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q

class IdleNode(Node):
    """
    Независимая нода IDLE.
    - Сервис /idle/set_enabled (std_srvs/SetBool) для включения/выключения IDLE.
    - Публикует AttitudeTarget в /uav1/setpoint_raw/attitude, когда включена.
    - Подписывается на /uav1/imu/data и /uav1/global_position/rel_alt только когда включена.
    """
    def __init__(self):
        super().__init__('idle_node')

        # publisher для сетпоинтов (тот же топик что и у Planner)
        self.pub = self.create_publisher(AttitudeTarget, '/uav1/setpoint_raw/attitude', 10)

        # сервис управления IDLE
        self.srv = self.create_service(SetBool, '/idle/set_enabled', self.handle_set_enabled)

        # внутреннее состояние
        self.enabled = False
        self._imu_sub = None
        self._baro_sub = None
        self._timer = None
        self.current_alt = None
        self.vx = 0.0
        self.vy = 0.0

        # параметры регуляции (можно настраивать через параметры)
        self.declare_parameter('kp_xy', 0.2)
        self.declare_parameter('kp_z', 0.5)
        self.declare_parameter('feedforward', 0.5)
        self.declare_parameter('pub_hz', 20.0)

        self.get_logger().info("IdleNode initialized (disabled). Call /idle/set_enabled to enable.")

    # ----------------- сервис включения/выключения -----------------
    def handle_set_enabled(self, request, response):
        enable = bool(request.data)
        if enable == self.enabled:
            response.success = True
            response.message = f"Idle already {'enabled' if enable else 'disabled'}"
            return response

        if enable:
            self._enable_idle()
            response.success = True
            response.message = "Idle enabled"
        else:
            self._disable_idle()
            response.success = True
            response.message = "Idle disabled"
        return response

    def _enable_idle(self):
        # создаём подписки
        if self._imu_sub is None:
            self._imu_sub = self.create_subscription(Imu, '/uav1/imu/data', self._imu_cb, 10)
        if self._baro_sub is None:
            self._baro_sub = self.create_subscription(FluidPressure, '/uav1/global_position/rel_alt', self._baro_cb, 10)

        # запускаем таймер публикации
        if self._timer is None:
            hz = float(self.get_parameter('pub_hz').value)
            self._timer = self.create_timer(1.0 / hz, self._idle_timer_cb)

        self.enabled = True
        self.get_logger().info("Idle enabled: subscribed to IMU and barometer, publishing setpoints.")

    def _disable_idle(self):
        # уничтожаем подписки и таймер — node перестает реагировать на сенсоры
        if self._imu_sub is not None:
            self.destroy_subscription(self._imu_sub)
            self._imu_sub = None
        if self._baro_sub is not None:
            self.destroy_subscription(self._baro_sub)
            self._baro_sub = None
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        self.enabled = False
        self.get_logger().info("Idle disabled: unsubscribed and stopped publishing.")

    # ----------------- callbacks сенсоров -----------------
    def _imu_cb(self, msg: Imu):
        # тут используем ускорения как приближение скоростей (проводите калибровку)
        # если у тебя есть готовая vx/vy — используй их
        self.vx = msg.linear_acceleration.x
        self.vy = msg.linear_acceleration.y

    def _baro_cb(self, msg: FluidPressure):
        # простая аппроксимация давления -> высота
        P0 = 101325.0
        P = msg.fluid_pressure
        self.current_alt = (1.0 - (P / P0) ** 0.1903) * 44330.0

    # ----------------- таймер публикации IDLE -----------------
    def _idle_timer_cb(self):
        # если вдруг нет данных — не публикуем
        if self.current_alt is None:
            return

        kp_xy = float(self.get_parameter('kp_xy').value)
        kp_z = float(self.get_parameter('kp_z').value)
        feedforward = float(self.get_parameter('feedforward').value)

        roll = -kp_xy * self.vy
        pitch = kp_xy * self.vx
        # target altitude хранится локально — если не задан, используем current_alt
        # Можно добавить сервис/топик, чтобы задать желаемую цель извне
        target_alt = getattr(self, 'target_alt', self.current_alt)
        err_z = target_alt - self.current_alt
        thrust = feedforward + kp_z * err_z
        thrust = max(0.0, min(1.0, thrust))

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = euler_to_quaternion(roll, pitch, 0.0)
        msg.body_rate = Vector3()
        msg.thrust = float(thrust)
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        self.pub.publish(msg)

    # ----------------- утилиты -----------------
    def set_target_alt(self, alt):
        self.target_alt = float(alt)

def main(args=None):
    rclpy.init(args=args)
    node = IdleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
