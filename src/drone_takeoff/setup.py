from setuptools import setup

package_name = 'drone_takeoff'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/takeoff.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'mavsdk', 'opencv-python', 'numpy', 'cv_bridge'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS2 node to command PX4 takeoff via MAVSDK in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'takeoff_node = drone_takeoff.takeoff_node:main',
            'qr_detector_node = drone_takeoff.qr_detector_node:main',
            'lidar_listener_node = drone_takeoff.lidar_listener_node:main',
            'ultrasonic_listener_node = drone_takeoff.ultrasonic_listener_node:main',
        ],
    },
)

