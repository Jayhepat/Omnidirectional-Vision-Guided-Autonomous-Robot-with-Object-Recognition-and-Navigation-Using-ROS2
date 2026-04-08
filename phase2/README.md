# Phase 2 — Gazebo Simulation (ROS2 Humble)

## Overview

This folder contains the **Gazebo Ignition (Fortress)** simulation for the Omnisight
mecanum-wheel autonomous robot, running on **ROS2 Humble** with Ubuntu 22.04.

The simulation includes:
- Full 3D URDF model with mecanum wheels, RPLiDAR S2, Intel RealSense D435, and IMU
- 8×6 m indoor office world with furniture and obstacles
- ROS↔Gazebo topic bridges for all sensors
- RViz2 visualization with preconfigured layout

---

## Prerequisites

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# Ignition Gazebo Fortress
sudo apt install ros-humble-ros-gz

# Robot model tools
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui

# Optional: Nav2 stack for navigation
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox
```

---

## Workspace Structure

```
phase2/
└── omnisight_simulation/
    └── src/
        └── omnisight_sim/
            ├── package.xml
            ├── CMakeLists.txt
            ├── launch/
            │   ├── simulation.launch.py   ← Full Gazebo + RViz2
            │   └── display.launch.py      ← RViz2 only (no Gazebo)
            ├── worlds/
            │   └── omnisight_indoor.world ← 8x6m indoor office
            ├── config/
            │   └── ros_gz_bridge.yaml     ← Topic bridge config
            └── rviz/
                ├── omnisight_sim.rviz     ← Simulation view
                └── omnisight_display.rviz ← URDF display view
```

> **Note:** The `omnisight_sim` package depends on `mec_omnisight_description`
> from the `ros2_ws` folder. Build `ros2_ws` first.

---

## Build & Run

### Step 1 — Build the robot description (from ros2_ws)

```bash
cd ~/ros2_ws
colcon build --packages-select mec_omnisight_description
source install/setup.bash
```

### Step 2 — Build the simulation package

```bash
cd ~/phase2/omnisight_simulation
colcon build --packages-select omnisight_sim
source install/setup.bash
```

### Step 3 — Launch simulation

```bash
# Full simulation with Gazebo + RViz2
ros2 launch omnisight_sim simulation.launch.py

# Gazebo without RViz2
ros2 launch omnisight_sim simulation.launch.py rviz:=false

# RViz2 only (URDF viewer, no Gazebo)
ros2 launch omnisight_sim display.launch.py
```

---

## Topics Available After Launch

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | RPLiDAR S2 scan |
| `/cam_1/image` | `sensor_msgs/Image` | RealSense RGB |
| `/cam_1/depth_image` | `sensor_msgs/Image` | RealSense depth |
| `/imu/data` | `sensor_msgs/Imu` | IMU data |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command input |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/robot_description` | `std_msgs/String` | URDF string |

---

## Teleoperate the Robot in Simulation

```bash
# In a new terminal (after sourcing):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Known Issues / Notes

- This simulation uses **Ignition Gazebo Fortress** (default for ROS2 Humble).
  If you are on ROS2 Foxy/Galactic, use **Ignition Edifice** instead and adjust
  the `ros_gz_bridge` arguments accordingly.
- Mecanum wheel physics in Gazebo is approximated using friction parameters
  (mu=1.0, mu2=0.0 per roller axis). True mecanum behaviour requires a
  custom plugin; this setup provides a close approximation.
- The control plugin section in `robot_3d.urdf.xacro` is commented out.
  To enable full ros2_control integration, uncomment those lines and install
  `ros-humble-gz-ros2-control`.
