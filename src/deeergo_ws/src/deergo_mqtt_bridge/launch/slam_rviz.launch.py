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
# Validate Cartographer PBStream
# ================================================================


def validate_pbstream(
    context,
    load_map,
    pbstream_file,
):
    load_map_value = context.perform_substitution(load_map).strip().lower()

    if load_map_value not in (
        "true",
        "1",
        "yes",
        "on",
    ):
        return []

    if not os.path.isfile(pbstream_file):
        raise RuntimeError(f"\nCartographer PBStream file missing:\n{pbstream_file}")

    return [
        LogInfo(
            msg=(
                "\n"
                "============================================\n"
                "Loading Cartographer serialized state\n"
                "============================================\n"
                f"PBSTREAM : {pbstream_file}\n"
                "Mode     : pure localization\n"
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
    # Cartographer configuration
    #
    # IMPORTANT:
    # Cartographer algorithm parameters are native LUA parameters.
    # They are intentionally kept outside this launch file.
    # ============================================================

    cartographer_config_dir = os.path.join(
        deergo_share,
        "config",
    )

    mapping_config = "cartographer_mapping.lua"

    localization_config = "cartographer_localization.lua"

    # ============================================================
    # RViz
    # ============================================================

    rviz_config = os.path.join(
        deergo_share,
        "rviz",
        "deergo_cartographer.rviz",
    )

    # ============================================================
    # Cartographer serialized state
    #
    # Must match map_manager.py:
    #   MAP_DIR  = "maps"
    #   MAP_NAME = "deergo_map"
    # ============================================================

    pbstream_file = os.path.abspath(os.path.expanduser("maps/deergo_map.pbstream"))

    # ============================================================
    # Launch arguments
    # ============================================================

    load_map = LaunchConfiguration("load_map")

    start_nav2 = LaunchConfiguration("start_nav2")

    start_mqtt = LaunchConfiguration("start_mqtt")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_load_map = DeclareLaunchArgument(
        "load_map",
        default_value="false",
        description=(
            "false = Cartographer mapping, "
            "true = load PBStream and use pure localization"
        ),
    )

    declare_start_nav2 = DeclareLaunchArgument(
        "start_nav2",
        default_value="false",
        description=(
            "Start Nav2 together with Cartographer. "
            "Keep false for the first LiDAR-only Cartographer test."
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    declare_start_mqtt = DeclareLaunchArgument(
        "start_mqtt",
        default_value="true",
        description="Start the MQTT velocity bridge",
    )

    validate_map = OpaqueFunction(
        function=validate_pbstream,
        args=[
            load_map,
            pbstream_file,
        ],
    )

    # ============================================================
    # DeerGo communication
    #
    # Expected to start:
    #   mqtt_bridge
    #   scan_bridge
    #   base_link -> laser frame static TF
    #   map_manager (if included in your current node launch)
    #
    # mqtt_bridge MUST NOT publish wheel /odom or odom->base_link.
    # ============================================================

    deergo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(deergo_nodes_launch),
        launch_arguments={"start_mqtt": start_mqtt}.items(),
    )

    # ============================================================
    # Cartographer Mapping
    #
    # Input:
    #   Cartographer default "scan" topic
    #       remapped to /scan_sync
    #
    # Output TF:
    #   map -> odom -> base_link
    # ============================================================

    cartographer_mapping = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            mapping_config,
        ],
        remappings=[
            ("scan", "/scan_sync"),
        ],
        condition=UnlessCondition(load_map),
    )

    # ============================================================
    # Cartographer Pure Localization
    #
    # Loads:
    #   maps/deergo_map.pbstream
    # ============================================================

    cartographer_localization = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            localization_config,
            "-load_state_filename",
            pbstream_file,
            "-load_frozen_state",
            "true",
        ],
        remappings=[
            ("scan", "/scan_sync"),
        ],
        condition=IfCondition(load_map),
    )

    # ============================================================
    # Occupancy Grid
    #
    # Cartographer internally stores submaps.
    # This node converts them to nav_msgs/OccupancyGrid on /map.
    # ============================================================

    occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=[
            "-resolution",
            "0.05",
            "-publish_period_sec",
            "1.0",
        ],
    )

    # ============================================================
    # Cartographer TF -> /odom (no TF broadcasting)
    # ============================================================

    cartographer_odom_bridge = Node(
        package="deergo_mqtt_bridge",
        executable="cartographer_odom_bridge",
        name="cartographer_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ============================================================
    # Nav2
    #
    # Disabled by default until Cartographer pose/TF is verified
    # and the Cartographer TF -> nav_msgs/Odometry bridge is verified.
    # ============================================================

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        condition=IfCondition(start_nav2),
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
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription(
        [
            declare_load_map,
            declare_start_nav2,
            declare_start_mqtt,
            declare_use_sim_time,
            validate_map,
            deergo_nodes,
            cartographer_mapping,
            cartographer_localization,
            occupancy_grid,
            cartographer_odom_bridge,
            navigation,
            rviz,
        ]
    )
