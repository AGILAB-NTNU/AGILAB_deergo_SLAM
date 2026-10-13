#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_mqtt = LaunchConfiguration("start_mqtt")

    declare_start_mqtt = DeclareLaunchArgument(
        "start_mqtt",
        default_value="true",
        description="Start the MQTT velocity bridge",
    )
    # ============================================================
    # MQTT Bridge
    # ============================================================

    mqtt_bridge = Node(
        package="deergo_mqtt_bridge",
        executable="mqtt_bridge",
        name="deergo_mqtt_bridge",
        output="screen",
        condition=IfCondition(start_mqtt),
    )

    # ============================================================
    # Scan Bridge
    # /scan -> /scan_sync
    # ============================================================

    scan_bridge = Node(
        package="deergo_mqtt_bridge",
        executable="scan_bridge",
        name="scan_bridge",
        output="screen",
    )

    # ============================================================
    # Static TF
    # base_link -> laser (matches Hokuyo LaserScan.header.frame_id)
    # Roll pi corrects the observed left/right mirror in the mounted LiDAR.
    # The laser X axis stays forward; its Y and Z axes are reversed.
    # ============================================================

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        output="screen",
        arguments=[
            "--x",
            "0.8",
            "--y",
            "0",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            # "3.141592653589793",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser",
        ],
    )

    return LaunchDescription(
        [
            declare_start_mqtt,
            mqtt_bridge,
            scan_bridge,
            static_tf,
        ]
    )
