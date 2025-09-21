# Four_Wheels_Steering_bot (Isaac Sim + ROS2)

## Overview
High-fidelity 4-wheel steering rover simulation in Isaac Sim with ROS2 integration.  
Features include:

- Encoder support for steering
- LiDAR, IMU, GPS, and RGBD stereo cameras
- ROS2 communication bridge
- Bare-metal simulation ready to run out-of-the-box
- ROS2 workspace with autonomous, control, GPS, and utility controllers for rover navigation and motion control

This repository contains the **bare-metal simulation** (`sim.usd`) and required assets. Optional enhanced environment shaders and textures are available separately.

---

## Repository Structure

```
Four_Wheels_Steering_bot/
├─ sim_assets/       # Meshes and assets required for the rover
│  └─ meshes.zip
├─ sim.usd           # Bare-metal USD file of the rover
├─ LICENSE
├─ README.md
└─ Example_ros2_ws/  # ROS2 workspace for controlling and simulating the rover
```

**Inside `Example_ros2_ws/` (ROS2 workspace):**

```
Example_ros2_ws/
├─ src/
│  ├─ autonomous/   # Go-to-goal and path planning nodes
│  ├─ control2/     # Master control node (steering, throttle, etc.)
│  ├─ controller/   # Utility controllers for motor/PID
│  └─ gps/          # GPS processing and odometry conversion
├─ build/           # Colcon build artifacts
├─ install/         # Colcon install workspace
└─ log/             # Build logs
```

---

## Setup Instructions

### 1. Clone the repository

```bash
git clone https://github.com/Avg2006/Four_Wheels_Steering_bot.git
cd Four_Wheels_Steering_bot
```

### 2. Prepare assets

```bash
unzip sim_assets/meshes.zip -d sim_assets/
```

Optional: Add environment shaders/textures to `sim_assets/` for better visuals.

---

### 3. ROS2 Workspace Setup

1. Source ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

2. Navigate to your ROS2 workspace:

```bash
cd Example_ros2_ws
```

3. Build the workspace using `colcon`:

```bash
colcon build
```

4. Source the local workspace:

```bash
source install/setup.bash
```

---

### 4. ROS2 Bridge Setup

Configure the ROS2 bridge environment before running Isaac Sim:

```bash
export ROS_DISTRO=humble && \
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/avg/isaacsim/exts/isaacsim.ros2.bridge/humble/lib && \
export ROS_DOMAIN_ID=5
```

> **Note:** Adjust the `LD_LIBRARY_PATH` if Isaac Sim is installed in a different location.  
> Example above assumes installation in `~/isaacsim`.

---

### 5. Launch Isaac Sim

```bash
cd ~/isaacsim && ./isaac-sim.sh
```

Check ROS2 topics in a separate terminal:

```bash
ros2 topic list
ros2 topic echo <topic_name>
```

---

### 6. ROS2 Packages & Nodes

| Package      | Node / Launch               | Description |
|-------------|----------------------------|-------------|
| `autonomous` | `go_to_goal.py`             | Autonomous navigation, waypoint following |
| `control2`   | `master_code.py`            | Master control node (steering, throttle, joystick) |
| `controller` | Utility controllers         | PID control, motor mixing, steering |
| `gps`        | `gps_fix.py`, `gps_to_xy.py` | GPS data processing, latitude/longitude → odometry |

---

### 7. ROS2 Topics

| Topic                          | Direction | Description | Message Type |
|--------------------------------|-----------|-------------|--------------|
| `/enc_auto`                     | Output    | Encoder data for steering | `std_msgs/msg/Float32MultiArray` |
| `/imu`                          | Output    | IMU sensor data | `sensor_msgs/msg/Imu` |
| `/motor_pwm`                    | Input     | Control wheel RPM and steer | `std_msgs/msg/Int32MultiArray` |
| `/odom`                         | Output    | Rover odometry | `nav_msgs/msg/Odometry` |
| `/rgbd_camera/left_image_info`  | Output    | Left camera info | `sensor_msgs/msg/CameraInfo` |
| `/rgbd_camera/left_image_raw`   | Output    | Left camera feed | `sensor_msgs/msg/Image` |
| `/rgbd_camera/right_image_info` | Output    | Right camera info | `sensor_msgs/msg/CameraInfo` |
| `/rgbd_camera/right_image_raw`  | Output    | Right camera feed | `sensor_msgs/msg/Image` |
| `/sim/gps`                      | Output    | GPS data | `geometry_msgs/msg/Point` |
| `/sim/laser_scan`               | Output    | LiDAR scan data | `sensor_msgs/msg/LaserScan` |

---

## License

- Non-commercial use only.
- Access granted to Team Anveshak (IIT Madras).
- Any other use requires written permission from the author.

---

## Notes

- Bare-metal simulation runs immediately once meshes are unzipped.
- Optional shader assets enhance visuals but are not required.
- Keep all paths relative within `sim_assets/` for portability.
- ROS2 workspace provides autonomous control, master control, GPS processing, and utility controllers.
