from setuptools import setup, find_packages

package_name = 'drone_takeoff'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/takeoff.launch.py',
            'launch/qr_detector.launch.py',
            'launch/autonomy.launch.py',
            'launch/autonomy_with_qr.launch.py',
            'launch/range_sensors.launch.py',
            'launch/ros_gz_bridges.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'mavsdk', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS 2 package with takeoff, QR detector, and sensor listeners',
    license='MIT',
    entry_points={
        'console_scripts': [
            'takeoff_node = drone_takeoff.takeoff_node:main',
            'qr_detector_node = drone_takeoff.qr_detector_node:main',
            'lidar_listener_node = drone_takeoff.lidar_listener_node:main',
            'ultrasonic_listener_node = drone_takeoff.ultrasonic_listener_node:main',
            'autonomy_node = drone_takeoff.autonomy_node:main',
        ],
    },
)

