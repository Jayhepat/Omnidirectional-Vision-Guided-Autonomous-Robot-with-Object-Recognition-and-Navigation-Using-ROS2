#!/usr/bin/env python3
"""
patrol_system.launch.py — THE ONE COMMAND TO RUN OMNISIGHT
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Usage (on Raspberry Pi):
  cd ~/omnisight_ws
  colcon build --packages-select omnisight_patrol
  source install/setup.bash
  ros2 launch omnisight_patrol patrol_system.launch.py

This starts ALL 7 nodes simultaneously:
  1. patrol_master        — state machine brain
  2. motion_control       — mecanum wheel driver
  3. pan_tilt_scanner     — 2-DOF camera head
  4. scene_monitor        — door/window/curtain change detection
  5. face_recognition_node— stranger detection + MediaPipe
  6. alert_manager        — WiFi TCP send + buzzer
  7. obstacle_avoidance   — ultrasonic watchdog
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory("omnisight_patrol")
    config  = os.path.join(pkg_dir, "config", "patrol_waypoints.yaml")

    # Common parameters passed to all nodes
    common_params = [config]

    return LaunchDescription([

        # ── Node 1: Patrol Master (brain — start first) ──────────
        Node(
            package    = "omnisight_patrol",
            executable = "patrol_master",
            name       = "patrol_master",
            output     = "screen",
            parameters = common_params,
        ),

        # ── Node 2: Motion Control ───────────────────────────────
        # Small delay so patrol_master is ready first
        TimerAction(period=1.0, actions=[
            Node(
                package    = "omnisight_patrol",
                executable = "motion_control",
                name       = "motion_control",
                output     = "screen",
                parameters = common_params,
            ),
        ]),

        # ── Node 3: Pan-Tilt Scanner ─────────────────────────────
        TimerAction(period=1.5, actions=[
            Node(
                package    = "omnisight_patrol",
                executable = "pan_tilt_scanner",
                name       = "pan_tilt_scanner",
                output     = "screen",
                parameters = common_params,
            ),
        ]),

        # ── Node 4: Obstacle Avoidance (safety — start early) ────
        TimerAction(period=1.5, actions=[
            Node(
                package    = "omnisight_patrol",
                executable = "obstacle_avoidance",
                name       = "obstacle_avoidance",
                output     = "screen",
                parameters = common_params,
            ),
        ]),

        # ── Node 5: Scene Monitor ────────────────────────────────
        TimerAction(period=2.0, actions=[
            Node(
                package    = "omnisight_patrol",
                executable = "scene_monitor",
                name       = "scene_monitor",
                output     = "screen",
                parameters = common_params,
            ),
        ]),

        # ── Node 6: Face Recognition ─────────────────────────────
        TimerAction(period=2.5, actions=[
            Node(
                package    = "omnisight_patrol",
                executable = "face_recognition_node",
                name       = "face_recognition_node",
                output     = "screen",
                parameters = common_params,
            ),
        ]),

        # ── Node 7: Alert Manager (last — needs others ready) ────
        TimerAction(period=3.0, actions=[
            Node(
                package    = "omnisight_patrol",
                executable = "alert_manager",
                name       = "alert_manager",
                output     = "screen",
                parameters = common_params,
            ),
        ]),

    ])
