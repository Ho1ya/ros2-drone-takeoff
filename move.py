import common
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Move(Node):
    def __init__(self):
        super().__init__('move')
        self.publisher = self.create_publisher(Twist,
                                               '/mavros/setpoint_velocity/cmd_vel_unstamped',
                                               10)
        self.get_logger().info("Move initialized")

    def send_velocity(self,vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.publisher.publish(msg)
        self.get_logger().info(f"Скорости X={vx},Y={vy},Z={vz},YAW={yaw_rate}")