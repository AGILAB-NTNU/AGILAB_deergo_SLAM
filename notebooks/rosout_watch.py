#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Log


TEST_DURATION = 20.0


class SlamScanMonitor(Node):

    def __init__(self):
        super().__init__("slam_scan_monitor")

        self.start_time = time.monotonic()

        self.scan_count = 0
        self.pose_count = 0
        self.queue_drop_count = 0

        self.scan_times = []
        self.pose_times = []

        # /scan_sync
        self.create_subscription(
            LaserScan,
            "/scan_sync",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # slam_toolbox scan-match pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/pose",
            self.pose_callback,
            10,
        )

        # /rosout
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Log,
            "/rosout",
            self.log_callback,
            rosout_qos,
        )

        self.timer = self.create_timer(
            0.1,
            self.check_time,
        )

        print()
        print("==============================================")
        print(" SLAM Toolbox Scan Monitor")
        print("==============================================")
        print(f"Duration : {TEST_DURATION:.0f} s")
        print("==============================================")
        print()

    def scan_callback(self, msg):
        self.scan_count += 1
        self.scan_times.append(time.monotonic())

    def pose_callback(self, msg):
        self.pose_count += 1
        self.pose_times.append(time.monotonic())

    def log_callback(self, msg):

        text = msg.msg
        node_name = msg.name

        if (
            "slam_toolbox" in node_name
            and "Message Filter dropping message" in text
            and "queue is full" in text
        ):
            self.queue_drop_count += 1

            print(
                f"[DROP] queue full | "
                f"total={self.queue_drop_count}"
            )

    def check_time(self):

        elapsed = time.monotonic() - self.start_time

        if elapsed >= TEST_DURATION:
            self.print_result()

            self.destroy_timer(self.timer)
            rclpy.shutdown()

    def print_result(self):

        print()
        print()
        print("==============================================================")
        print("                    20 SECOND RESULT")
        print("==============================================================")

        scan_hz = self.scan_count / TEST_DURATION
        pose_hz = self.pose_count / TEST_DURATION

        if self.scan_count > 0:
            drop_percent = (
                self.queue_drop_count
                / self.scan_count
                * 100.0
            )
        else:
            drop_percent = 0.0

        print()
        print("/scan_sync")
        print("----------------------------------------------")
        print(f"Received scans     : {self.scan_count}")
        print(f"Scan rate          : {scan_hz:.3f} Hz")

        print()
        print("slam_toolbox MessageFilter")
        print("----------------------------------------------")
        print(f"Queue-full drops   : {self.queue_drop_count}")
        print(f"Drop percentage    : {drop_percent:.2f} %")

        print()
        print("Scan match pose")
        print("----------------------------------------------")
        print(f"Pose messages      : {self.pose_count}")
        print(f"Pose rate          : {pose_hz:.3f} Hz")

        print()
        print("==============================================================")

        if self.pose_count == 0:
            print()
            print(
                "NOTE: /pose produced no messages."
            )
            print(
                "Check: ros2 topic list | grep pose"
            )

        print()


def main(args=None):

    rclpy.init(args=args)

    node = SlamScanMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.print_result()

    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()