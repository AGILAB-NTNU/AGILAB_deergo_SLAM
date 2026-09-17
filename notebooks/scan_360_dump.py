#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

OUTPUT_FILE = os.path.expanduser("~/ROS2_tb4/deergo_ws/scan_raw_2000.txt")


class ScanRawDump(Node):
    def __init__(self):
        super().__init__("scan_raw_dump")

        self.finished = False

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",  # 原始 LiDAR，不經過 filter
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Waiting for ONE raw /scan frame...")

    def scan_callback(self, msg):
        if self.finished:
            return

        self.finished = True

        close_points = []

        with open(
            OUTPUT_FILE,
            "w",
        ) as f:
            # ====================================================
            # Header
            # ====================================================

            f.write("Raw LiDAR Scan Dump\n")

            f.write("==============================\n")

            f.write(f"frame_id: {msg.header.frame_id}\n")

            f.write(f"stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}\n")

            f.write(f"angle_min_rad: {msg.angle_min}\n")

            f.write(f"angle_max_rad: {msg.angle_max}\n")

            f.write(f"angle_increment_rad: {msg.angle_increment}\n")

            f.write(f"range_min_m: {msg.range_min}\n")

            f.write(f"range_max_m: {msg.range_max}\n")

            f.write(f"number_of_ranges: {len(msg.ranges)}\n")

            f.write("==============================\n\n")

            f.write("index\tangle_deg\tdistance_m\n")

            # ====================================================
            # 所有原始點
            # ====================================================

            for i, distance in enumerate(msg.ranges):
                angle_rad = msg.angle_min + i * msg.angle_increment

                angle_deg = math.degrees(angle_rad)

                if math.isfinite(distance):
                    f.write(f"{i}\t{angle_deg:.4f}\t{distance:.4f}\n")

                    # 另外記下非常近的點
                    if distance < 0.30:
                        close_points.append(
                            (
                                i,
                                angle_deg,
                                distance,
                            )
                        )

                else:
                    f.write(f"{i}\t{angle_deg:.4f}\tinf\n")

            # ====================================================
            # < 0.30 m summary
            # ====================================================

            f.write("\n\n========================================\n")

            f.write("Points closer than 0.30 m\n")

            f.write("========================================\n")

            f.write("index\tangle_deg\tdistance_m\n")

            for (
                index,
                angle_deg,
                distance,
            ) in close_points:
                f.write(f"{index}\t{angle_deg:.4f}\t{distance:.4f}\n")

        # ========================================================
        # Terminal summary
        # ========================================================

        print()
        print("========================================")

        print(f"Raw scan points : {len(msg.ranges)}")

        print(f"Points < 0.30 m : {len(close_points)}")

        print("Saved to:")

        print(OUTPUT_FILE)

        print("========================================")

        if close_points:
            print()
            print("Close points:")

            for (
                index,
                angle_deg,
                distance,
            ) in close_points:
                print(
                    f"index={index:4d} | "
                    f"angle={angle_deg:8.3f} deg | "
                    f"distance={distance:.3f} m"
                )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = ScanRawDump()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
