# ros2_ws — Robot Description Package

## Overview

This workspace contains the **URDF robot description** for the Omnisight mecanum-wheel
robot. It defines the robot's 3D model, joints, sensors, and Gazebo simulation
properties.

This package is a **shared dependency** used by both:
- `phase2/omnisight_simulation` — Gazebo simulation
- RViz2 visualisation

---

## Package Structure

```
ros2_ws/
└── src/
    └── mec_omnisight/
        ├── mec_omnisight/               ← Top-level metapackage
        │   ├── package.xml
        │   └── CMakeLists.txt
        │
        └── mec_omnisight_description/   ← Robot URDF + meshes
            ├── package.xml
            ├── CMakeLists.txt
            │
            ├── urdf/
            │   ├── robots/
            │   │   └── robot_3d.urdf.xacro      ← Main robot file
            │   ├── mech/
            │   │   ├── robot_3d_base.urdf.xacro  ← Chassis
            │   │   └── mecanum_wheel.urdf.xacro  ← 4 mecanum wheels
            │   └── sensors/
            │       ├── rgbd_camera.urdf.xacro    ← Intel RealSense D435
            │       ├── lidar.urdf.xacro          ← RPLiDAR S2
            │       └── imu.urdf.xacro            ← IMU
            │
            └── meshes/
                ├── robot_3d/visual/             ← Chassis + wheel STLs
                ├── rplidar/                     ← RPLiDAR S2 STL
                └── intel_realsense/visual/      ← RealSense D435 STL
```

---

## Robot Specifications (from URDF)

| Parameter | Value |
|-----------|-------|
| Chassis size | 300mm × 199mm × 262mm |
| Wheel radius | 32.5 mm |
| Wheel separation | 169 mm |
| Base mass | 4.6 kg |
| Wheel mass | 9.1 kg (each) |
| LiDAR | RPLiDAR S2 — 360°, 10Hz, 30m range |
| Camera | RealSense D435 — 424×240px, 2Hz in sim |
| IMU | 15Hz update rate |

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select mec_omnisight_description mec_omnisight
source install/setup.bash
```

---

## Quick URDF Check

```bash
# Check URDF is valid
xacro src/mec_omnisight/mec_omnisight_description/urdf/robots/robot_3d.urdf.xacro \
  use_gazebo:=false > /tmp/omnisight.urdf
check_urdf /tmp/omnisight.urdf
```
