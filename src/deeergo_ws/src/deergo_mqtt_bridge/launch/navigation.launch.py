#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from deergo_mqtt_bridge.mppi_backend import select_profile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def cleanup_profile(context, path):
    if os.path.exists(path):
        os.unlink(path)
    return []


def launch_navigation(context, nav2_launch):
    requested = LaunchConfiguration("params_file").perform(context)
    selected, backend, reason = select_profile(
        requested, LaunchConfiguration("mppi_backend").perform(context)
    )
    actions = [LogInfo(msg=f"Nav2 backend: {backend}. {reason}")]
    if os.path.realpath(requested) != selected:
        actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(function=cleanup_profile, args=[selected])
                    ]
                )
            )
        )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": "false",
                "params_file": selected,
                "autostart": "true",
                "use_composition": "False",
                "use_respawn": "False",
                "log_level": "info",
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    # ============================================================
    # Package paths
    # ============================================================

    deergo_share = get_package_share_directory("deergo_mqtt_bridge")

    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # ============================================================
    # DeerGo Nav2 parameter YAML
    #
    # Preserve the existing profile as the default; MPPI is selectable separately.
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
    # MPPI profiles select a verified CUDA plugin or the official CPU fallback.
    # ============================================================

    nav2 = OpaqueFunction(function=launch_navigation, args=[nav2_launch])

    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=params_file),
            DeclareLaunchArgument(
                "mppi_backend", default_value="auto", choices=["auto", "cpu", "gpu"]
            ),
            nav2,
        ]
    )
