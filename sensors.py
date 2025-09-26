import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher = self.create_publisher(LaserScan, '/lidar', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

    def publish_scan(self):
        """
        На реальном роботе здесь будет чтение с лидара, преобразованное в LaserScan.
        Пока пример с пустыми диапазонами.
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        msg.angle_min = -3.14
        msg.angle_max = 3.14
        msg.angle_increment = 0.01
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [msg.range_max for _ in range(int((msg.angle_max - msg.angle_min)/msg.angle_increment))]
        self.publisher.publish(msg)



class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.05, self.publish_imu)  # 20 Hz

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_frame'
        # На реальном роботе здесь данные с IMU
        self.publisher.publish(msg)
