#!/usr/bin/env python3
"""Autonomous Navigation (Nav2) launch for Valtoryn Reference AMR.

Launches:
  1. Map Server and AMCL (localization)
  2. Controller, Planner, Behavior Server, BT Navigator (navigation)
  3. Collision Monitor (optional, using /scan)
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    default_map = find_file("maps/warehouse_map.yaml")
    default_params = find_file("config/nav2_params.yaml")
    default_col_params = find_file("config/collision_monitor_params.yaml")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")

    declare_map = DeclareLaunchArgument("map", default_value=default_map, description="Full path to map yaml file")
    declare_params = DeclareLaunchArgument("params_file", default_value=default_params, description="Path to Nav2 params file")
    declare_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True", description="Use simulation clock")
    declare_autostart = DeclareLaunchArgument("autostart", default_value="True", description="Automatically startup Nav2 stack")
    declare_col_monitor = DeclareLaunchArgument(
        "use_collision_monitor", default_value="False",
        description="Launch Nav2 collision monitor using /scan"
    )

    # Standard Nav2 bringup launch
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    # Optional Collision Monitor node
    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[default_col_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_collision_monitor),
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_sim_time,
        declare_autostart,
        declare_col_monitor,
        nav2_bringup,
        collision_monitor_node,
    ])

