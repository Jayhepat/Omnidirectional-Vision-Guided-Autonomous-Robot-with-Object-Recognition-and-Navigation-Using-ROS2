#!/usr/bin/env python3
"""
simulation.launch.py — Omnisight Full Gazebo Simulation
ROS2 Humble + Ignition Gazebo (Fortress)
VIT Bhopal SEEE Capstone 2022-2026

Launches:
  1. Ignition Gazebo with omnisight_indoor.world
  2. robot_state_publisher (URDF → /tf)
  3. ros_gz_bridge (Gazebo ↔ ROS2 topic bridging)
  4. RViz2 (optional, default: true)

Usage:
  cd ~/phase2/omnisight_simulation
  colcon build
  source install/setup.bash
  ros2 launch omnisight_sim simulation.launch.py

Optional arguments:
  ros2 launch omnisight_sim simulation.launch.py rviz:=false
  ros2 launch omnisight_sim simulation.launch.py world:=empty
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package directories ────────────────────────────────────────
    pkg_sim  = get_package_share_directory("omnisight_sim")
    pkg_desc = get_package_share_directory("mec_omnisight_description")

    # ── Launch arguments ───────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_sim, "worlds", "omnisight_indoor.world"),
        description="Full path to Gazebo world file",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 for visualization",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    world      = LaunchConfiguration("world")
    rviz       = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── URDF from xacro ───────────────────────────────────────────
    urdf_file = os.path.join(
        pkg_desc, "urdf", "robots", "robot_3d.urdf.xacro"
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", urdf_file, " use_gazebo:=true"]
    )

    # ── Set GZ_SIM_RESOURCE_PATH so Gazebo finds meshes ─────────
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_desc, "meshes"),
    )

    # ── 1. Ignition Gazebo ────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── 2. Robot State Publisher ──────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ── 3. Spawn robot into Gazebo ─────────────────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_omnisight",
                output="screen",
                arguments=[
                    "-name", "omnisight",
                    "-topic", "robot_description",
                    "-x", "0.5",
                    "-y", "0.5",
                    "-z", "0.05",
                    "-Y", "0.0",
                ],
            )
        ],
    )

    # ── 4. ROS-Gazebo Bridge ──────────────────────────────────────
    #   Maps Ignition topics → ROS2 topics
    bridge_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_ros_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    # Clock
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    # LiDAR scan
                    "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    # RGBD camera
                    "/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                    "/cam_1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                    "/cam_1/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                    # IMU
                    "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                    # Odometry (from diff drive plugin)
                    "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    # TF
                    "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                    "/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                    # Cmd vel
                    "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                    # Joint states
                    "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                ],
            )
        ],
    )

    # ── 5. RViz2 ─────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_sim, "rviz", "omnisight_sim.rviz")
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        gz_resource_path,
        world_arg,
        rviz_arg,
        use_sim_time_arg,
        gz_sim,
        rsp_node,
        spawn_robot,
        bridge_node,
        rviz_node,
    ])
