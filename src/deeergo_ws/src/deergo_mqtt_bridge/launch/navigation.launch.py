#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ============================================================
    # Package paths
    # ============================================================

    deergo_share = get_package_share_directory("deergo_mqtt_bridge")

    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # ============================================================
    # DeerGo Nav2 parameter YAML
    #
    # All Nav2 tuning is maintained here:
    #
    # deergo_mqtt_bridge/config/nav2_params.yaml
    # ============================================================

    params_file = os.path.join(
        deergo_share,
        "config",
        "nav2_params.yaml",
    )

    # ============================================================
    # Official Nav2 navigation launch
    # ============================================================

    nav2_launch = os.path.join(
        nav2_bringup_share,
        "launch",
        "navigation_launch.py",
    )

    # ============================================================
    # Nav2
    #
    # No DWB / costmap / planner / smoother parameter is modified
    # in this launch file.
    #
    # All parameter changes must be made in:
    #
    # config/nav2_params.yaml
    # ============================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params_file,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription(
        [
            nav2,
        ]
    )
