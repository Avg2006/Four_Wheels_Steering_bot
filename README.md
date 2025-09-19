# Four_Wheels_Steering_bot (Isaac Sim)

## Overview
High-fidelity 4-wheel steering rover simulation in Isaac Sim with ROS2 integration.
Features include:

- Encoder support for steering
- LiDAR, IMU, GPS, and RGBD stereo cameras
- ROS2 communication bridge
- Bare-metal simulation ready to run out-of-the-box

This repository contains the **bare-metal simulation** (`sim.usd`) and small assets required to run it.
Optional enhanced environment shaders and textures are available separately.

---

## Repository Structure

```
Four_Wheels_Steering_bot/
├─ sim_assets/       # Meshes and assets required for the rover
│  └─ meshes.zip
├─ sim.usd           # Bare-metal USD file of the rover
├─ LICENSE
└─ README.md
```

---

## Setup Instructions

### 1. Clone the repository

```bash
git clone https://github.com/Avg2006/Four_Wheels_Steering_bot.git
cd Four_Wheels_Steering_bot
```

### 2. Prepare assets

1. Unzip the mandatory meshes:

```bash
unzip sim_assets/meshes.zip -d sim_assets/
```

2. (Optional) Download additional environment shaders and ground assets:

[Google Drive Link](https://drive.google.com/file/d/1K6RV3u_LgPo8qWWvxvkTqgR6eYvJ_b-i/view?usp=sharing)

Unzip this into `sim_assets/` as well:

```bash
unzip <downloaded_file>.zip -d sim_assets/
```

---

### 3. ROS2 Integration

1. Start the **ROS2 bridge** in Isaac Sim and set the **Domain ID** to `5`.
2. In every other terminal where you want to access ROS2 topics:

```bash
export ROS_DOMAIN_ID=5
```

3. You can now inspect topics using:

```bash
ros2 topic echo <topic_name>
```

---

## ROS2 Topics

| Topic                          | Direction | Description | Message Type |
|--------------------------------|-----------|-------------|--------------|
| `/enc_auto`                     | Output    | Encoder data for steer | `std_msgs/msg/Float32MultiArray` |
| `/imu`                          | Output    | IMU sensor data | `sensor_msgs/msg/Imu` |
| `/motor_pwm`                    | Input     | Control wheel RPM and steer (max 128) | `std_msgs/msg/Int32MultiArray` |
| `/odom`                         | Output    | Rover odometry | `nav_msgs/msg/Odometry` |
| `/rgbd_camera/left_image_info`  | Output    | Left camera info | `sensor_msgs/msg/CameraInfo` |
| `/rgbd_camera/left_image_raw`   | Output    | Left camera feed | `sensor_msgs/msg/Image` |
| `/rgbd_camera/right_image_info` | Output    | Right camera info | `sensor_msgs/msg/CameraInfo` |
| `/rgbd_camera/right_image_raw`  | Output    | Right camera feed | `sensor_msgs/msg/Image` |
| `/sim/gps`                      | Output    | GPS data (x → latitude, y → longitude, accuracy 2.5–5 m) | `geometry_msgs/msg/Point` |
| `/sim/laser_scan`               | Output    | LiDAR scan data | `sensor_msgs/msg/LaserScan` |

---

## License

- Non-commercial use only.
- Access granted to Team Anveshak (IIT Madras).
- Any other use requires written permission from the author.

---

## Notes

- The bare-metal simulation is ready to run once meshes are unzipped.
- Optional shader assets enhance the ground and environment visuals but are not required.
- Make sure all paths remain relative within `sim_assets/` for portability.
