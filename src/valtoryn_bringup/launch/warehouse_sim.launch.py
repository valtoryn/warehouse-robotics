#!/usr/bin/env python3
"""Top-level warehouse simulation launch for Valtoryn reference AMR.

Composes:
  1. Gazebo Harmonic simulation bringup (sim_bringup.launch.py)
  2. Autonomous navigation stack (nav2.launch.py) or SLAM mapping (slam.launch.py)
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def find_file(relative_path: str) -> str:
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
    return str(Path(__file__).resolve().parent.parent.parent.parent / relative_path)


def generate_launch_description() -> LaunchDescription:
    slam = LaunchConfiguration("slam")
    headless = LaunchConfiguration("headless")
    urdf_path = LaunchConfiguration("urdf_path")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")

    default_world = find_file("worlds/warehouse.sdf")

    declare_slam = DeclareLaunchArgument("slam", default_value="False", description="Run SLAM Toolbox mapping instead of AMCL navigation")
    declare_headless = DeclareLaunchArgument("headless", default_value="True", description="Run Gazebo headlessly")
    declare_urdf_path = DeclareLaunchArgument("urdf_path", default_value="", description="Path to generated URDF description")
    declare_world = DeclareLaunchArgument("world", default_value=default_world, description="Path to warehouse SDF world")
    declare_x = DeclareLaunchArgument("x_pose", default_value="-7.0", description="Spawn X coordinate")
    declare_y = DeclareLaunchArgument("y_pose", default_value="4.0", description="Spawn Y coordinate")
    declare_z = DeclareLaunchArgument("z_pose", default_value="0.10", description="Spawn Z coordinate")
    declare_yaw = DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn Yaw coordinate")

    bringup_launch_path = os.path.join(
        get_package_share_directory("valtoryn_bringup"), "launch", "sim_bringup.launch.py"
    )
    nav2_launch_path = os.path.join(
        get_package_share_directory("valtoryn_bringup"), "launch", "nav2.launch.py"
    )
    slam_launch_path = os.path.join(
        get_package_share_directory("valtoryn_bringup"), "launch", "slam.launch.py"
    )

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            "headless": headless,
            "urdf_path": urdf_path,
            "world": world,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "z_pose": z_pose,
            "yaw": yaw,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        condition=UnlessCondition(slam),
    )

    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        condition=IfCondition(slam),
    )

    return LaunchDescription([
        declare_slam,
        declare_headless,
        declare_urdf_path,
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        sim_bringup,
        nav2,
        slam_nav,
    ])

