#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanBridge(Node):

    def __init__(self):
        super().__init__("scan_bridge")

        # ========================================================
        # Counters
        # ========================================================

        self.frame_count = 0

        # ========================================================
        # Subscriber
        #
        # Raw Hokuyo LaserScan
        # ========================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # ========================================================
        # Publisher
        #
        # /scan_sync is now an exact pass-through of /scan.
        #
        # No:
        #   - timestamp correction
        #   - startup calibration
        #   - clock offset
        #   - range filtering
        #   - angle filtering
        #
        # header.stamp is preserved exactly.
        # ========================================================

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan_sync",
            qos_profile_sensor_data,
        )

        # ========================================================
        # Startup Information
        # ========================================================

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Scan Bridge started"
        )

        self.get_logger().info(
            "Input  : /scan"
        )

        self.get_logger().info(
            "Output : /scan_sync"
        )

        self.get_logger().info(
            "Mode   : direct pass-through"
        )

        self.get_logger().info(
            "Timestamp correction: DISABLED"
        )

        self.get_logger().info(
            "LiDAR filtering      : DISABLED"
        )

        self.get_logger().info(
            "/scan_sync = /scan"
        )

        self.get_logger().info(
            "========================================"
        )


    # ============================================================
    # Scan Callback
    # ============================================================

    def scan_callback(
        self,
        msg,
    ):

        self.frame_count += 1

        # --------------------------------------------------------
        # Publish the original LaserScan message directly.
        #
        # This preserves:
        #
        #   header.stamp
        #   header.frame_id
        #   angle_min
        #   angle_max
        #   angle_increment
        #   time_increment
        #   scan_time
        #   range_min
        #   range_max
        #   ranges
        #   intensities
        #
        # exactly as received from /scan.
        # --------------------------------------------------------

        self.scan_pub.publish(msg)

        # --------------------------------------------------------
        # Lightweight debug
        # --------------------------------------------------------

        if self.frame_count % 100 == 0:

            self.get_logger().info(
                f"forwarded={self.frame_count} | "
                f"points={len(msg.ranges)} | "
                f"frame_id={msg.header.frame_id}"
            )


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ScanBridge()

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