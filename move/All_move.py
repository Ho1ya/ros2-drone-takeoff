import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3, PointStamped,Point
from mavros_msgs.msg import AttitudeTarget
import math
from rclpy.action import ActionServer
from my_msgs.action import PlanPath
import numpy as np
import time
import asyncio
from rclpy.executors import MultiThreadedExecutor

G = 9.81  # гравитация, используется для аппроксимации углов из скоростей


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Преобразование углов Эйлера (рад) -> Quaternion (x,y,z,w)
    """
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


class Move(Node):
    def __init__(self, publish_hz: float = 20.0):
        """
        Move node that publishes AttitudeTarget to /mavros/setpoint_raw/attitude
        and subscribes to /point/raw/altitude (PointStamped) to read current altitude.
        publish_hz: частота непрерывной посылки сетпоинтов (обычно >= 10..20 Hz).
        """
        super().__init__('move_attitude')

        # Publisher attitude setpoints
        self.pub = self.create_publisher(AttitudeTarget,
                                         '/mavros/setpoint_raw/attitude',
                                         10)

        # Subscription to raw altitude (PointStamped expected)
        self.sub_alt = self.create_subscription(
            PointStamped,
            '/point/raw/altitude',
            self._alt_callback,
            10
        )

        # State
        self.current_altitude = None     # float (z) from PointStamped
        self.target_altitude = None      # optional target for hold_altitude
        self._last_msg = AttitudeTarget()
        # default: ignore body rates (we'll control orientation + thrust by default)
        self._last_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )
        # default orientation = identity
        self._last_msg.orientation = euler_to_quaternion(0.0, 0.0, 0.0)
        self._last_msg.body_rate = Vector3()
        self._last_msg.thrust = 0.0

        # Timer to continuously publish last setpoint
        self._pub_timer = self.create_timer(1.0 / publish_hz, self._publish_current)
        self.get_logger().info(f"Move (attitude) initialized, publishing at {publish_hz} Hz")

    def _alt_callback(self, msg: PointStamped):
        self.current_altitude = float(msg.point.z)
        # debug
        self.get_logger().debug(f"Altitude update: {self.current_altitude:.3f}")

    def _publish_current(self):
        # update header stamp and publish
        self._last_msg.header.stamp = self.get_clock().now().to_msg()
        # optional: set a frame_id if needed, e.g. 'base_link'
        # self._last_msg.header.frame_id = 'base_link'
        self.pub.publish(self._last_msg)

    def set_attitude(self,
                     roll: float = None,
                     pitch: float = None,
                     yaw: float = None,
                     body_rate: tuple = None,
                     thrust: float = None):
        """
        Установить желаемую ориентацию/скорости/тягу.
        - roll, pitch, yaw : если указаны — задаём ориентацию (в радианах)
        - body_rate : (wx, wy, wz) — если указан, будет использована поле body_rate
        - thrust : float [0.0..1.0] (интерпретация зависит от прошивки)
        Тип маски рассчитывается автоматически:
          - если orientation не указан -> IGNORE_ATTITUDE
          - если body_rate не указан -> IGNORE_ROLL_RATE|IGNORE_PITCH_RATE|IGNORE_YAW_RATE
          - если thrust is None -> IGNORE_THRUST
        """
        msg = AttitudeTarget()
        mask = 0

        # Orientation
        if roll is None or pitch is None or yaw is None:
            mask |= AttitudeTarget.IGNORE_ATTITUDE
            # keep previous orientation value if any
            msg.orientation = self._last_msg.orientation
        else:
            msg.orientation = euler_to_quaternion(roll, pitch, yaw)

        # Body rates
        if body_rate is None:
            mask |= (AttitudeTarget.IGNORE_ROLL_RATE |
                     AttitudeTarget.IGNORE_PITCH_RATE |
                     AttitudeTarget.IGNORE_YAW_RATE)
            msg.body_rate = Vector3()
        else:
            wx, wy, wz = body_rate
            msg.body_rate = Vector3(x=wx, y=wy, z=wz)

        # Thrust
        if thrust is None:
            mask |= AttitudeTarget.IGNORE_THRUST
            msg.thrust = self._last_msg.thrust
        else:
            # clamp thrust to sensible range [0.0, 1.0]
            t = float(thrust)
            t = max(0.0, min(1.0, t))
            msg.thrust = t

        msg.type_mask = mask
        msg.header.stamp = self.get_clock().now().to_msg()
        self._last_msg = msg  # store as last to be repeatedly published by timer

        self.get_logger().info(
            f"Set attitude: roll={roll}, pitch={pitch}, yaw={yaw}, body_rate={body_rate}, thrust={msg.thrust:.3f}, mask={mask}"
        )

    def hold_altitude(self, target_z: float, kp: float = 0.4, feedforward: float = 0.5,
                      min_thrust: float = 0.0, max_thrust: float = 1.0):
        """
        Простой P-контроллер для удержания высоты: thrust = feedforward + kp * (z_err)
        feedforward = приближённая тяга на 0-уровне (обычно ~0.5), kp нужно подобрать.
        После расчёта thrust вызывает set_attitude(...) с сохранённой ориентацией.
        """
        if self.current_altitude is None:
            self.get_logger().warning("No altitude measurement yet; cannot hold altitude")
            return

        err = float(target_z) - float(self.current_altitude)
        thrust = feedforward + kp * err
        thrust = max(min_thrust, min(max_thrust, thrust))

        # сохранить текущ quaternion (если есть), иначе (0,0,0)
        if self._last_msg.type_mask & AttitudeTarget.IGNORE_ATTITUDE:
            # если ранее игнорировали ориентацию — задаём нейтральную
            q = euler_to_quaternion(0.0, 0.0, 0.0)
            # set attitude with only thrust (ignore body rates)
            self.set_attitude(roll=0.0, pitch=0.0, yaw=0.0, thrust=thrust)
        else:
            # взять текущ quaternion и переотправить с новым thrust
            # вычислим текущ rpy лишь для ясности — тут проще взять last orientation and keep it
            # but set_attitude expects rpy — we cannot easily extract rpy; instead build msg directly:
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.orientation = self._last_msg.orientation
            msg.body_rate = Vector3()  # ignore body rates
            msg.thrust = thrust
            msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE |
                             AttitudeTarget.IGNORE_PITCH_RATE |
                             AttitudeTarget.IGNORE_YAW_RATE)
            self._last_msg = msg
            self.get_logger().info(f"Holding altitude {target_z:.2f} -> thrust {thrust:.3f}")

    def attitude_from_velocity(self, vx: float, vy: float, yaw: float = 0.0, thrust: float = None):
        """
        Простейшая аппроксимация: желаемые малые углы наклона для получения vx, vy:
        pitch ≈ atan2(vx, g), roll ≈ atan2(-vy, g)
        (знаю, что это грубая линейная модель — нужно tune в реальном роботе)
        """
        pitch = math.atan2(vx, G)
        roll = math.atan2(-vy, G)
        self.set_attitude(roll=roll, pitch=pitch, yaw=yaw, thrust=thrust)


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self._action_server = ActionServer(
            self,
            PlanPath,
            'plan_path',
            self.execute_callback
        )
        self.move = Move()
        self.get_logger().info("Planner init")

    def plan(self, p0, p1, v_max, a_max, dt):
        """
        Вернуть список (t, vx, vy, vz).
        """
        p0, p1 = np.array(p0), np.array(p1)
        direction = p1 - p0
        distance = np.linalg.norm(direction)
        if distance == 0:
            return [(0.0, 0.0, 0.0, 0.0)]

        unit_dir = direction / distance

        # Время разгона и торможения
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc**2

        if 2 * d_acc > distance:
            # Треугольный профиль (не успеваем достичь v_max)
            v_peak = np.sqrt(a_max * distance)
            t_acc = v_peak / a_max
            t_total = 2 * t_acc
        else:
            d_cruise = distance - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise

        # Считаем список скоростей с абсолютным временем
        t = 0.0
        trajectory = []
        while t <= t_total + dt:
            if t < t_acc:
                v = a_max * t
            elif t < t_total - t_acc:
                v = v_max
            else:
                t_dec = t_total - t
                v = a_max * t_dec

            v_vec = unit_dir * v
            trajectory.append((t, v_vec[0], v_vec[1], v_vec[2]))
            t += dt

        return trajectory

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Получен запрос на новую траекторию")

        start = np.array([goal_handle.request.start.x,
                          goal_handle.request.start.y,
                          goal_handle.request.start.z])
        goal = np.array([goal_handle.request.goal.x,
                         goal_handle.request.goal.y,
                         goal_handle.request.goal.z])

        v_max = goal_handle.request.v_max
        a_max = goal_handle.request.a_max
        dt = goal_handle.request.dt

        # 1. Построим траекторию
        trajectory = self.plan(start, goal, v_max, a_max, dt)
        self.get_logger().info(f"Trajectory has {len(trajectory)} steps")

        # Целевая высота (берём из goal)
        target_alt = float(goal[2])

        # 2. Исполняем траекторию строго по времени
        t_start = time.time()
        for (t, vx, vy, vz) in trajectory:
            # Ждём пока не наступит момент t
            while time.time() - t_start < t:
                await asyncio.sleep(0.001)

            # Горизонтальные скорости -> attitude
            # (yaw = 0.0, thrust пока не задаём)
            self.move.attitude_from_velocity(vx, vy, yaw=0.0, thrust=None)

            # Высота -> thrust через P-контроллер
            self.move.hold_altitude(target_alt)

            # Feedback для клиента (передаём текущую желаемую скорость)
            fb = PlanPath.Feedback()
            fb.current_point = Point(x=vx, y=vy, z=vz)
            goal_handle.publish_feedback(fb)

        # 3. Останов: нейтральные углы + нулевая тяга
        self.move.set_attitude(roll=0.0, pitch=0.0, yaw=0.0, thrust=0.0)

        # 4. Отправляем результат
        result = PlanPath.Result()
        result.success = True
        self.get_logger().info("Trajectory execution finished")
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    move_node = Move()
    planner_node = Planner()

    # Используем мультипоточный executor, чтобы обе ноды могли работать параллельно
    executor = MultiThreadedExecutor()
    executor.add_node(move_node)
    executor.add_node(planner_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Корректное завершение
        executor.shutdown()
        move_node.destroy_node()
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()