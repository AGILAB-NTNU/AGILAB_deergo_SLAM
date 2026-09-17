#!/usr/bin/env python3

import csv
import os
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TEST_DURATION = 10.0

OUTPUT_DIR = os.path.expanduser("~/ROS2_tb4/deergo_ws/timing_results")


class TopicTimingTest(Node):
    def __init__(self):
        super().__init__("topic_timing_test")

        os.makedirs(
            OUTPUT_DIR,
            exist_ok=True,
        )

        self.start_ns = time.monotonic_ns()

        # ------------------------------------------------------------
        # Data
        #
        # 每筆：
        # elapsed
        # arrival_monotonic
        # header_stamp
        # ------------------------------------------------------------

        self.odom_data = []
        self.scan_data = []

        # ============================================================
        # /odom
        # ============================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # ============================================================
        # /scan_sync
        # ============================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan_sync",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Timing test started: 10 seconds")

    # ================================================================
    # Helpers
    # ================================================================

    def get_elapsed(self):
        now_ns = time.monotonic_ns()

        return (now_ns - self.start_ns) / 1e9

    def stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec / 1e9

    # ================================================================
    # Odom
    # ================================================================

    def odom_callback(self, msg):
        elapsed = self.get_elapsed()

        if elapsed > TEST_DURATION:
            return

        arrival = time.monotonic_ns() / 1e9

        stamp = self.stamp_to_sec(msg.header.stamp)

        self.odom_data.append(
            (
                elapsed,
                arrival,
                stamp,
            )
        )

    # ================================================================
    # Scan
    # ================================================================

    def scan_callback(self, msg):
        elapsed = self.get_elapsed()

        if elapsed > TEST_DURATION:
            return

        arrival = time.monotonic_ns() / 1e9

        stamp = self.stamp_to_sec(msg.header.stamp)

        self.scan_data.append(
            (
                elapsed,
                arrival,
                stamp,
            )
        )

    # ================================================================
    # Statistics
    # ================================================================

    def calculate_statistics(
        self,
        data,
        topic_name,
        output_file,
    ):
        rows = []

        for second in range(10):
            start = float(second)
            end = float(second + 1)

            samples = [item for item in data if start <= item[0] < end]

            count = len(samples)

            # --------------------------------------------
            # Arrival interval
            # --------------------------------------------

            arrival_times = [item[1] for item in samples]

            arrival_intervals = [
                arrival_times[i] - arrival_times[i - 1]
                for i in range(
                    1,
                    len(arrival_times),
                )
            ]

            if arrival_intervals:
                arrival_mean = statistics.mean(arrival_intervals)

                arrival_std = statistics.pstdev(arrival_intervals)

                arrival_hz = 1.0 / arrival_mean if arrival_mean > 0 else 0.0

                arrival_min = min(arrival_intervals)

                arrival_max = max(arrival_intervals)

            else:
                arrival_mean = 0.0
                arrival_std = 0.0
                arrival_hz = 0.0
                arrival_min = 0.0
                arrival_max = 0.0

            # --------------------------------------------
            # Header timestamp interval
            # --------------------------------------------

            stamp_times = [item[2] for item in samples]

            stamp_intervals = [
                stamp_times[i] - stamp_times[i - 1]
                for i in range(
                    1,
                    len(stamp_times),
                )
            ]

            if stamp_intervals:
                stamp_mean = statistics.mean(stamp_intervals)

                stamp_std = statistics.pstdev(stamp_intervals)

                stamp_hz = 1.0 / stamp_mean if stamp_mean > 0 else 0.0

                stamp_min = min(stamp_intervals)

                stamp_max = max(stamp_intervals)

            else:
                stamp_mean = 0.0
                stamp_std = 0.0
                stamp_hz = 0.0
                stamp_min = 0.0
                stamp_max = 0.0

            rows.append(
                {
                    "second": second + 1,
                    "message_count": count,
                    "arrival_hz": arrival_hz,
                    "arrival_mean_period_ms": arrival_mean * 1000.0,
                    "arrival_std_ms": arrival_std * 1000.0,
                    "arrival_min_period_ms": arrival_min * 1000.0,
                    "arrival_max_period_ms": arrival_max * 1000.0,
                    "stamp_hz": stamp_hz,
                    "stamp_mean_period_ms": stamp_mean * 1000.0,
                    "stamp_std_ms": stamp_std * 1000.0,
                    "stamp_min_period_ms": stamp_min * 1000.0,
                    "stamp_max_period_ms": stamp_max * 1000.0,
                }
            )

        # ============================================================
        # CSV
        # ============================================================

        path = os.path.join(
            OUTPUT_DIR,
            output_file,
        )

        with open(
            path,
            "w",
            newline="",
        ) as f:
            writer = csv.DictWriter(
                f,
                fieldnames=rows[0].keys(),
            )

            writer.writeheader()
            writer.writerows(rows)

        # ============================================================
        # Terminal
        # ============================================================

        print()
        print(f"================ {topic_name} ================")

        for row in rows:
            print(
                f"sec {row['second']:02d} | "
                f"count={row['message_count']:2d} | "
                f"arrival={row['arrival_hz']:6.2f} Hz | "
                f"arrival std={row['arrival_std_ms']:6.3f} ms | "
                f"stamp={row['stamp_hz']:6.2f} Hz | "
                f"stamp std={row['stamp_std_ms']:6.3f} ms"
            )

        print()
        print(f"CSV: {path}")


def main(args=None):
    rclpy.init(args=args)

    node = TopicTimingTest()

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

            if node.get_elapsed() >= TEST_DURATION:
                break

    except KeyboardInterrupt:
        pass

    print()
    print("10 second measurement finished.")

    node.calculate_statistics(
        node.odom_data,
        "/odom",
        "odom_timing.csv",
    )

    node.calculate_statistics(
        node.scan_data,
        "/scan_sync",
        "scan_timing.csv",
    )

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
