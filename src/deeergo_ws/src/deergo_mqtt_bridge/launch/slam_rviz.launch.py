#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ================================================================
# Check serialized map
# ================================================================


def validate_serialized_map(
    context,
    load_map,
    serialized_map,
):
    load_map_value = context.perform_substitution(load_map).strip().lower()

    if load_map_value not in (
        "true",
        "1",
        "yes",
        "on",
    ):
        return []

    data_file = serialized_map + ".data"
    posegraph_file = serialized_map + ".posegraph"

    missing_files = []

    if not os.path.isfile(data_file):
        missing_files.append(data_file)

    if not os.path.isfile(posegraph_file):
        missing_files.append(posegraph_file)

    if missing_files:
        raise RuntimeError(
            "\nSerialized map file missing:\n" + "\n".join(missing_files)
        )

    return [
        LogInfo(
            msg=(
                "\n"
                "============================================\n"
                "Loading serialized SLAM map\n"
                "============================================\n"
                f"DATA      : {data_file}\n"
                f"POSEGRAPH : {posegraph_file}\n"
                "Mode      : localization\n"
                "============================================"
            )
        )
    ]


# ================================================================
# Launch
# ================================================================


def generate_launch_description():
    # ============================================================
    # Package paths
    # ============================================================

    deergo_share = get_package_share_directory("deergo_mqtt_bridge")

    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    # ============================================================
    # DeerGo launch files
    # ============================================================

    deergo_nodes_launch = os.path.join(
        deergo_share,
        "launch",
        "deergo_node_subs.launch.py",
    )

    navigation_launch = os.path.join(
        deergo_share,
        "launch",
        "navigation.launch.py",
    )

    # ============================================================
    # SLAM Toolbox launch files
    # ============================================================

    mapping_launch = os.path.join(
        slam_toolbox_share,
        "launch",
        "online_async_launch.py",
    )

    localization_launch = os.path.join(
        slam_toolbox_share,
        "launch",
        "localization_launch.py",
    )

    # ============================================================
    # SLAM Toolbox YAML
    #
    # All SLAM tuning is maintained only in:
    #
    # config/slam_mapping.yaml
    # config/slam_localization.yaml
    # ============================================================

    mapping_params_file = os.path.join(
        deergo_share,
        "config",
        "slam_mapping.yaml",
    )

    localization_params_file = os.path.join(
        deergo_share,
        "config",
        "slam_localization.yaml",
    )

    # ============================================================
    # RViz
    # ============================================================

    rviz_config = os.path.join(
        deergo_share,
        "rviz",
        "deergo_slam.rviz",
    )

    # ============================================================
    # Serialized map
    #
    # Must match map_file_name in:
    # config/slam_localization.yaml
    # ============================================================

    serialized_map = os.path.expanduser("maps/deergo")

    # ============================================================
    # load_map
    #
    # false -> Mapping
    # true  -> Localization
    # ============================================================

    load_map = LaunchConfiguration("load_map")

    declare_load_map = DeclareLaunchArgument(
        "load_map",
        default_value="false",
        description=(
            "false = mapping, true = load serialized map and use localization mode"
        ),
    )

    validate_map = OpaqueFunction(
        function=validate_serialized_map,
        args=[
            load_map,
            serialized_map,
        ],
    )

    # ============================================================
    # DeerGo communication
    # ============================================================

    deergo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(deergo_nodes_launch)
    )

    # ============================================================
    # SLAM Mapping
    # ============================================================

    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": mapping_params_file,
        }.items(),
        condition=UnlessCondition(load_map),
    )

    # ============================================================
    # SLAM Localization
    # ============================================================

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": localization_params_file,
        }.items(),
        condition=IfCondition(load_map),
    )

    # ============================================================
    # Nav2
    # ============================================================

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    # ============================================================
    # RViz
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
    )

    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription(
        [
            declare_load_map,
            validate_map,
            deergo_nodes,
            slam_mapping,
            slam_localization,
            navigation,
            rviz,
        ]
    )
