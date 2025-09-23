# drone_takeoff (ROS2 + MAVSDK)

Minimal ROS2 node to connect to a PX4 SITL drone in Gazebo and command takeoff using MAVSDK.

## Prerequisites
- ROS2 Humble or newer installed and sourced
- Python 3.10+
- Gazebo and PX4 SITL (gz-sim or Gazebo Classic)
- MAVSDK-Python (`pip install mavsdk`)

## Setup
```bash
# In workspace root
python -m venv .venv
# Windows PowerShell:
. .venv/Scripts/Activate.ps1
# Linux/macOS:
source .venv/bin/activate
pip install -U pip
pip install -r src/drone_takeoff/requirements.txt

# Build ROS2 package
colcon build --packages-select drone_takeoff
# Source workspace
# Windows PowerShell:
./install/setup.ps1
# Linux/macOS:
. install/setup.bash
```

## Run PX4 + Gazebo (example)
- Start PX4 SITL with UDP endpoint 14540 (PX4 default).
- Ensure the vehicle is reachable at `udp://:14540` or change `connection_url`.

PX4 gz sim example:
```bash
# In PX4 repo (example)
make px4_sitl gz_x500
```

## Launch takeoff
```bash
ros2 launch drone_takeoff takeoff.launch.py takeoff_altitude_m:=5.0 connection_url:=udp://:14540
```

## QR detector
Subscribe to a camera `sensor_msgs/Image` topic and decode QR codes with OpenCV.

```bash
# Defaults use bottom camera topic
ros2 launch drone_takeoff qr_detector.launch.py \
  image_topic:=/uav1/camera_down \
  publish_text_topic:=/qr_detector/text \
  show_debug_window:=false

# Echo decoded strings
ros2 topic echo /qr_detector/text
```

## Range sensors (LiDAR + Ultrasonic)
Listen to LiDAR (`sensor_msgs/LaserScan`) and ultrasonic (`sensor_msgs/Range`).

```bash
# Launch both listeners (ultrasonic optional)
ros2 launch drone_takeoff range_sensors.launch.py \
  scan_topic:=/uav1/scan \
  range_topic:='' 

# Inspect topics
ros2 topic echo /uav1/scan
```

## Autonomy: explore, avoid walls/doorways, stop on QR, safe land
The autonomy node uses MAVSDK offboard velocity control. Sensors are optional and defaulted to your topics:
- LiDAR: `/uav1/scan` — gap detection for wall avoidance and doorways.
- Forward depth camera: `/uav1/depth_camera` — center ROI average as forward distance.
- IMU: `/uav1/mavros/imu/data` — throttles yaw if angular speed is high to improve stability.
- QR text: `/qr_detector/text` (from bottom camera `/uav1/camera_down`).

```bash
ros2 launch drone_takeoff autonomy.launch.py \
  connection_url:=udp://:14540 \
  scan_topic:=/uav1/scan \
  depth_topic:=/uav1/depth_camera \
  imu_topic:=/uav1/mavros/imu/data \
  qr_text_topic:=/qr_detector/text \
  takeoff_altitude_m:=3.0 \
  cruise_speed_m_s:=1.0 \
  wall_distance_min_m:=1.0 \
  doorway_min_width_rad:=0.35 \
  control_rate_hz:=10.0 \
  max_yaw_rate_deg_s:=45.0
```

## Parameters
- `connection_url` (string): MAVSDK connection URL (default `udp://:14540`).
- `takeoff_altitude_m` (double): Takeoff altitude meters (default 5.0).
- `arm_timeout_s` (double): Timeout for arming (default 10.0).
- `connect_timeout_s` (double): Timeout for connection (default 20.0).
- `image_topic` (string): Camera topic for QR detector (default `/uav1/camera_down`).
- `publish_text_topic` (string): Output decoded text topic (default `/qr_detector/text`).
- `show_debug_window` (bool): Show OpenCV window (default `false`).
- `scan_topic` (string): LiDAR scan topic (default `/uav1/scan`).
- `range_topic` (string): Ultrasonic range topic (empty disables ultrasonic).
- `depth_topic` (string): Forward depth image topic (default `/uav1/depth_camera`).
- `depth_obstacle_distance_m` (double): Threshold to treat as obstacle (meters).
- `depth_center_fraction` (double): Fraction of image width/height for center ROI (0-1).
- `imu_topic` (string): IMU topic (default `/uav1/mavros/imu/data`).
- Autonomy specific: `cruise_speed_m_s`, `wall_distance_min_m`, `doorway_min_width_rad`, `control_rate_hz`, `max_yaw_rate_deg_s`.

## Notes
- For ArduPilot SITL, adapt URLs (e.g. `udp://:14550`).
- Wait for GPS/health OK in sim before takeoff.
