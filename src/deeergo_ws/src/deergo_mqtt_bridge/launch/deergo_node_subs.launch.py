#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # MQTT Bridge
    # ============================================================

    mqtt_bridge = Node(
        package="deergo_mqtt_bridge",
        executable="mqtt_bridge",
        name="deergo_mqtt_bridge",
        output="screen",
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
    # base_link -> laser_link
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
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser_link",
        ],
    )

    return LaunchDescription(
        [
            mqtt_bridge,
            scan_bridge,
            static_tf,
        ]
    )
