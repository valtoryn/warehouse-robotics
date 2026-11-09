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

    declare_slam = DeclareLaunchArgument("slam", default_value="false", description="Run SLAM Toolbox mapping instead of AMCL navigation")
    declare_headless = DeclareLaunchArgument("headless", default_value="true", description="Run Gazebo headlessly")

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
        launch_arguments={"headless": headless}.items(),
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
        sim_bringup,
        nav2,
        slam_nav,
    ])
