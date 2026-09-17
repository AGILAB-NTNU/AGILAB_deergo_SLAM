#!/usr/bin/env python3

import csv
import os
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

TEST_DURATION = 10.0

OUTPUT_DIR = os.path.expanduser("~/ROS2_tb4/deergo_ws/timing_results")


class OdomStampDelayTest(Node):
    def __init__(self):
        super().__init__("odom_stamp_delay_test")

        os.makedirs(
            OUTPUT_DIR,
            exist_ok=True,
        )

        self.start_ns = time.monotonic_ns()

        self.rows = []

        self.sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.get_logger().info("Odom timestamp delay test started: 10 seconds")

    # ================================================================
    # Time
    # ================================================================

    def elapsed(self):
        return (time.monotonic_ns() - self.start_ns) / 1e9

    # ================================================================
    # Callback
    # ================================================================

    def odom_callback(self, msg):
        elapsed = self.elapsed()

        if elapsed > TEST_DURATION:
            return

        # ROS clock
        receive_ns = self.get_clock().now().nanoseconds

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        delay_ns = receive_ns - stamp_ns

        delay_ms = delay_ns / 1_000_000.0

        self.rows.append(
            {
                "sample": len(self.rows) + 1,
                "elapsed_s": elapsed,
                "odom_stamp_sec": stamp_ns / 1e9,
                "receive_time_sec": receive_ns / 1e9,
                "delay_ms": delay_ms,
            }
        )

    # ================================================================
    # Save CSV
    # ================================================================

    def save(self):
        path = os.path.join(
            OUTPUT_DIR,
            "odom_stamp_delay.csv",
        )

        with open(
            path,
            "w",
            newline="",
        ) as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "sample",
                    "elapsed_s",
                    "odom_stamp_sec",
                    "receive_time_sec",
                    "delay_ms",
                ],
            )

            writer.writeheader()
            writer.writerows(self.rows)

        delays = [row["delay_ms"] for row in self.rows]

        print()
        print("==============================================")
        print("        ODOM Timestamp Delay Result")
        print("==============================================")

        if delays:
            print(f"Samples : {len(delays)}")

            print(f"Mean    : {statistics.mean(delays):.3f} ms")

            print(f"STD     : {statistics.pstdev(delays):.3f} ms")

            print(f"Min     : {min(delays):.3f} ms")

            print(f"Max     : {max(delays):.3f} ms")

        print(f"CSV     : {path}")

        print("==============================================")


def main(args=None):
    rclpy.init(args=args)

    node = OdomStampDelayTest()

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

            if node.elapsed() >= TEST_DURATION:
                break

    except KeyboardInterrupt:
        pass

    node.save()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
