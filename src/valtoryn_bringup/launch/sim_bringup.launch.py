#!/usr/bin/env python3
"""Simulation bringup launch for Valtoryn Reference AMR in Gazebo Harmonic.

Launches:
  1. Gazebo Harmonic with worlds/warehouse.sdf (headless or GUI)
  2. Robot model spawn via ros_gz_sim create
  3. robot_state_publisher
  4. ros_gz_bridge (/clock, /scan)
  5. Controller spawners (joint_state_broadcaster, diff_drive_controller)
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_file(relative_path: str) -> str:
    # Look in package share first, then source workspace
    try:
        share = get_package_share_directory("valtoryn_bringup")
        p = Path(share) / relative_path
        if p.exists():
            return str(p)
    except Exception:
        pass
    ws_p = Path(__file__).resolve().parent.parent / relative_path
    if ws_p.exists():
        return str(ws_p)
    root_p = Path(__file__).resolve().parent.parent.parent.parent / relative_path
    return str(root_p)


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    x_pose = LaunchConfiguration("x_pose").perform(context)
    y_pose = LaunchConfiguration("y_pose").perform(context)
    z_pose = LaunchConfiguration("z_pose").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    world_path = LaunchConfiguration("world").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)

    # Resolve world path if not absolute
    if not os.path.isabs(world_path):
        world_path = find_file(world_path)

    # URDF generation or load
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    if not urdf_path or not os.path.exists(urdf_path):
        # Fall back to generated or default location
        candidate = find_file("generated/ref-amr-diff-v1_gazebo.urdf")
        if os.path.exists(candidate):
            urdf_path = candidate
        else:
            urdf_path = find_file("generated/spike_robot.urdf")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    # 1. Gazebo Harmonic Launch
    gz_args = f"-r {world_path}"
    if headless:
        gz_args = f"-s -r {world_path}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # 2. Spawn AMR model in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "valtoryn_ref_amr",
            "-string", robot_description,
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-Y", yaw,
        ],
        output="screen",
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    # 4. ROS-Gazebo Parameter Bridge (/clock, /scan)
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 5. Controller Spawners
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    spawner_ddc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Launch diff drive controller after joint state broadcaster
    delayed_ddc = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_ddc],
        )
    )

    return [
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        gz_bridge,
        spawner_jsb,
        delayed_ddc,
    ]


def generate_launch_description() -> LaunchDescription:
    default_world = find_file("worlds/warehouse.sdf")
    default_controllers = find_file("config/controllers.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="True", description="Run Gazebo headlessly (-s)"),
        DeclareLaunchArgument("world", default_value=default_world, description="Path to warehouse SDF world"),
        DeclareLaunchArgument("x_pose", default_value="-7.0", description="Spawn X coordinate (meters)"),
        DeclareLaunchArgument("y_pose", default_value="4.0", description="Spawn Y coordinate (meters)"),
        DeclareLaunchArgument("z_pose", default_value="0.10", description="Spawn Z coordinate (meters)"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn Yaw orientation (radians)"),
        DeclareLaunchArgument("urdf_path", default_value="", description="Path to generated URDF description"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers, description="Path to controllers YAML"),
        OpaqueFunction(function=launch_setup),
    ])

