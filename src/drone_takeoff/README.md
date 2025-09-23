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
. .venv/Scripts/activate  # PowerShell: .venv\Scripts\Activate.ps1
pip install -U pip
pip install mavsdk

# Build ROS2 package
colcon build --packages-select drone_takeoff
# Source workspace (PowerShell on Windows):
install\setup.ps1
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

## Parameters
- `connection_url` (string): MAVSDK connection URL (default `udp://:14540`).
- `takeoff_altitude_m` (double): Takeoff altitude meters (default 5.0).
- `arm_timeout_s` (double): Timeout for arming (default 10.0).
- `connect_timeout_s` (double): Timeout for connection (default 20.0).

## Notes
- For ArduPilot SITL, adapt URLs (e.g. `udp://:14550`).
- Wait for GPS/health OK in sim before takeoff.
