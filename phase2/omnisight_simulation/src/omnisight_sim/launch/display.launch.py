#!/usr/bin/env python3
"""
display.launch.py — Visualise Omnisight robot in RViz2 (no Gazebo)
ROS2 Humble
VIT Bhopal SEEE Capstone 2022-2026

Useful for quickly inspecting the URDF / TF tree without running Gazebo.

Usage:
  ros2 launch omnisight_sim display.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory("mec_omnisight_description")
    pkg_sim  = get_package_share_directory("omnisight_sim")

    urdf_file = os.path.join(pkg_desc, "urdf", "robots", "robot_3d.urdf.xacro")
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", urdf_file, " use_gazebo:=false"]
    )

    rviz_config = os.path.join(pkg_sim, "rviz", "omnisight_display.rviz")

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    jsp_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([rsp_node, jsp_node, rviz_node])
