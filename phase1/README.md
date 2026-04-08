# Phase 1 — Omnisight Autonomous Patrol System

## Overview

Phase 1 implements the **complete autonomous indoor patrol system** running directly
on the **Raspberry Pi 4B** using **ROS2 Humble**. No simulation required — this is
the real hardware system.

The robot autonomously patrols a predefined indoor environment, detects strangers
using face recognition, monitors for scene changes (doors/windows/objects),
avoids obstacles, and sends alerts wirelessly to a monitoring PC.

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│               OMNISIGHT PATROL SYSTEM                │
│                                                     │
│  patrol_master ←→ motion_control                    │
│       ↑↓               ↑↓                           │
│  obstacle_avoidance   pan_tilt_scanner               │
│       ↑↓               ↑↓                           │
│  face_recognition ←→ scene_monitor                  │
│       ↓                 ↓                           │
│         alert_manager (→ PC via WiFi TCP)            │
└─────────────────────────────────────────────────────┘
```

---

## Package Structure

```
phase1/
└── omnisight_ws/
    └── src/
        └── omnisight_patrol/
            ├── package.xml
            ├── setup.py
            ├── QUICKSTART.txt            ← START HERE
            │
            ├── launch/
            │   └── patrol_system.launch.py  ← THE ONE COMMAND
            │
            ├── config/
            │   ├── patrol_waypoints.yaml    ← All settings
            │   ├── omnisight_room.xml       ← Room map + waypoints
            │   └── known_faces/             ← Add face photos here
            │       ├── jay.jpg
            │       ├── shiv.jpg
            │       └── vivek.jpg
            │
            ├── omnisight_patrol/            ← All 7 ROS2 nodes
            │   ├── patrol_master.py         ← State machine brain
            │   ├── motion_control.py        ← Mecanum kinematics
            │   ├── obstacle_avoidance.py    ← HC-SR04 ultrasonic
            │   ├── pan_tilt_scanner.py      ← 2-DOF servo head
            │   ├── scene_monitor.py         ← SSIM change detection
            │   ├── face_recognition_node.py ← MediaPipe stranger detection
            │   └── alert_manager.py         ← WiFi TCP alerts + buzzer
            │
            └── monitoring_client/
                └── pc_receiver.py           ← Run this on your PC
```

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Main controller | Raspberry Pi 4B (4GB RAM) |
| Motor driver | PCA9685 PWM board (I2C, 0x40) |
| Drive | 4× DC gear motors + Mecanum wheels |
| Camera | Raspberry Pi Camera Module v2 |
| Pan-tilt servos | 2× SG90/MG90S (GPIO 23, 24) |
| Ultrasonic sensor | HC-SR04 (TRIG: GPIO18, ECHO: GPIO24) |
| Buzzer | Passive buzzer (GPIO17) |
| Power | 12V LiPo battery pack |
| Network | WiFi (same LAN as monitoring PC) |

---

## Install Dependencies

```bash
# On Raspberry Pi
sudo apt install ros-humble-desktop python3-pip

pip3 install mediapipe face-recognition scikit-image \
             PyYAML adafruit-circuitpython-pca9685 \
             RPi.GPIO opencv-python numpy
```

---

## Build & Run

### Step 1 — On your PC (start the receiver first)

```bash
python3 monitoring_client/pc_receiver.py
# Note your PC IP shown on screen
```

### Step 2 — Configure on Raspberry Pi

Edit `config/patrol_waypoints.yaml`:
```yaml
monitoring_device_ip: "192.168.x.x"   # your PC IP
simulation_mode: false                  # true = test without hardware
```

### Step 3 — Build and launch

```bash
cd ~/phase1/omnisight_ws
colcon build --packages-select omnisight_patrol
source install/setup.bash
ros2 launch omnisight_patrol patrol_system.launch.py
```

The robot starts patrolling **automatically in 5 seconds**.

---

## ROS2 Topics

```bash
ros2 topic echo /omnisight/patrol_state       # IDLE/PATROLLING/RETURNING/WAITING
ros2 topic echo /omnisight/ultrasonic_distance
ros2 topic echo /omnisight/alert/stranger
ros2 topic echo /omnisight/alert/scene_change
```

---

## Simulation Mode (no hardware needed)

Set `simulation_mode: true` in `patrol_waypoints.yaml`, then run the same
launch command. All nodes start, hardware I/O is simulated in software.
