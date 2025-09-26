from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
import numpy as np
import time
import asyncio

from my_msgs.action import PlanPath
from move import Move


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
