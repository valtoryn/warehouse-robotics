#!/usr/bin/env python3
"""SLAM Toolbox online asynchronous mapping launch for Valtoryn Reference AMR."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    default_params = find_file("config/slam_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to SLAM Toolbox parameters file",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        slam_node,
    ])
