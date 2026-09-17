#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# ================================================================
# Startup Offset Calibration
#
# LiDAR 約 15 Hz
# 50 frames 約需要 3.3 秒
#
# 每次 scan_bridge 啟動：
#
# 1. 收集前 50 幀
# 2. 計算 PC receive time - LiDAR timestamp
# 3. 取 minimum
# 4. 固定成 constant offset
# 5. 後續完全不再修改
# ================================================================

CALIBRATION_SAMPLES = 50


class ScanBridge(Node):
    def __init__(self):
        super().__init__("scan_bridge")

        # ========================================================
        # Timestamp synchronization
        # ========================================================

        # 啟動校正時的 offset samples
        self.offset_samples = []

        # 校正完成後固定使用
        self.fixed_clock_offset_ns = None

        # 是否已完成 startup calibration
        self.offset_calibrated = False

        # ========================================================
        # Counters
        # ========================================================

        # /scan 收到幾幀
        self.frame_count = 0

        # /scan_sync 發布幾幀
        self.publish_count = 0

        # filter 累積過濾點數
        self.total_filtered_count = 0

        # ========================================================
        # Subscriber
        #
        # DeerGo 原始 LiDAR
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
        # 修正 timestamp
        # +
        # robot body filter
        # ========================================================

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan_sync",
            qos_profile_sensor_data,
        )

        # ========================================================
        # Startup information
        # ========================================================

        self.get_logger().info("========================================")

        self.get_logger().info("Scan Bridge started")

        self.get_logger().info("Input  : /scan")

        self.get_logger().info("Output : /scan_sync")

        self.get_logger().info(
            f"Startup offset calibration: " f"{CALIBRATION_SAMPLES} frames"
        )

        self.get_logger().info("During calibration, /scan_sync is paused")

        self.get_logger().info("Hard-coded LiDAR body filter enabled")

        self.get_logger().info("Rear filter: " "+169~+180 deg, -180~-176 deg")

        self.get_logger().info("Pillar filter: " "-167~-162 deg, +162~+167 deg")

        self.get_logger().info("========================================")

    # ============================================================
    # Robot Body Filter
    # ============================================================

    def filter_robot_body(
        self,
        scan_msg,
    ):
        filtered_this_scan = 0

        for i in range(len(scan_msg.ranges)):
            # ====================================================
            # 計算此 point 的實際角度
            # ====================================================

            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment

            angle_deg = math.degrees(angle_rad)

            # ====================================================
            # Filter 1
            #
            # 車尾固定遮擋
            #
            # 完全不檢查 distance
            # 只要角度進來就直接 inf
            # ====================================================

            rear_body_zone = angle_deg >= 169.0 or angle_deg <= -176.0

            # ====================================================
            # Filter 2
            #
            # 左右鋁柱
            #
            # 完全不檢查 distance
            # ====================================================

            aluminum_pillar_zone = (-167.0 <= angle_deg <= -162.0) or (
                162.0 <= angle_deg <= 167.0
            )

            # ====================================================
            # Hard Filter
            # ====================================================

            if rear_body_zone or aluminum_pillar_zone:
                scan_msg.ranges[i] = float("inf")

                filtered_this_scan += 1

        self.total_filtered_count += filtered_this_scan

        return filtered_this_scan

    # ============================================================
    # Startup Clock Offset Calibration
    # ============================================================

    def calibrate_clock_offset(
        self,
        lidar_ns,
        pc_now_ns,
    ):
        # ========================================================
        # 單幀 offset
        #
        # measured offset =
        #
        # PC receive time
        # -
        # LiDAR source timestamp
        #
        # 其中會包含：
        #
        # true clock offset
        # +
        # network / DDS delay
        #
        # 因此後面使用 minimum。
        # ========================================================

        measured_offset_ns = pc_now_ns - lidar_ns

        self.offset_samples.append(measured_offset_ns)

        sample_count = len(self.offset_samples)

        # ========================================================
        # Calibration progress
        # ========================================================

        if sample_count == 1 or sample_count % 10 == 0:
            self.get_logger().info(
                f"Offset calibration: " f"{sample_count}/" f"{CALIBRATION_SAMPLES}"
            )

        # ========================================================
        # 尚未收滿
        # ========================================================

        if sample_count < CALIBRATION_SAMPLES:
            return

        # ========================================================
        # 收滿 50 幀
        #
        # 只做一次！
        # ========================================================

        minimum_offset_ns = min(self.offset_samples)

        maximum_offset_ns = max(self.offset_samples)

        average_offset_ns = sum(self.offset_samples) / len(self.offset_samples)

        # ========================================================
        # 固定 offset
        #
        # 後面不再修改
        # ========================================================

        self.fixed_clock_offset_ns = minimum_offset_ns

        self.offset_calibrated = True

        # ========================================================
        # Statistics
        # ========================================================

        min_sec = minimum_offset_ns / 1_000_000_000.0

        avg_sec = average_offset_ns / 1_000_000_000.0

        max_sec = maximum_offset_ns / 1_000_000_000.0

        jitter_ms = (maximum_offset_ns - minimum_offset_ns) / 1_000_000.0

        self.get_logger().info("========================================")

        self.get_logger().info("LiDAR offset calibration FINISHED")

        self.get_logger().info(f"Samples       : " f"{CALIBRATION_SAMPLES}")

        self.get_logger().info(f"MIN offset    : " f"{min_sec:.9f} s")

        self.get_logger().info(f"AVG offset    : " f"{avg_sec:.9f} s")

        self.get_logger().info(f"MAX offset    : " f"{max_sec:.9f} s")

        self.get_logger().info(f"Offset jitter : " f"{jitter_ms:.3f} ms")

        self.get_logger().info(f"FIXED OFFSET  : " f"{min_sec:.9f} s")

        self.get_logger().info("Offset is now LOCKED")

        self.get_logger().info("/scan_sync publishing starts " "from next frame")

        self.get_logger().info("========================================")

    # ============================================================
    # Scan Callback
    # ============================================================

    def scan_callback(
        self,
        msg,
    ):
        self.frame_count += 1

        # ========================================================
        # 1. LiDAR 原始 timestamp
        # ========================================================

        lidar_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # ========================================================
        # 防止無效 source timestamp
        # ========================================================

        if lidar_ns <= 0:
            self.get_logger().warning("Invalid LiDAR timestamp. " "Scan ignored.")

            return

        # ========================================================
        # 2. PC 收到 scan 時的時間
        # ========================================================

        pc_now_ns = self.get_clock().now().nanoseconds

        # ========================================================
        # 3. Startup calibration
        #
        # 前 50 幀：
        #
        # 只量 offset
        # 不發布 /scan_sync
        #
        # 第 50 幀算完 fixed offset 後，
        # 仍然不發布。
        #
        # 從第 51 幀開始正式發布。
        # ========================================================

        if not self.offset_calibrated:
            self.calibrate_clock_offset(
                lidar_ns,
                pc_now_ns,
            )

            return

        # ========================================================
        # 4. 使用固定 constant offset
        #
        # 非常重要：
        #
        # 這裡不再：
        #
        # append offset
        # min(offset_samples)
        # update offset
        #
        # 後面永遠使用 startup 算好的同一個值。
        # ========================================================

        corrected_ns = lidar_ns + self.fixed_clock_offset_ns

        corrected_sec = corrected_ns // 1_000_000_000

        corrected_nanosec = corrected_ns % 1_000_000_000

        # ========================================================
        # 5. Copy scan
        # ========================================================

        scan_msg = copy.deepcopy(msg)

        # ========================================================
        # 6. Replace timestamp
        # ========================================================

        scan_msg.header.stamp.sec = int(corrected_sec)

        scan_msg.header.stamp.nanosec = int(corrected_nanosec)

        # ========================================================
        # 7. Hard-coded robot body filter
        # ========================================================

        filtered_this_scan = self.filter_robot_body(scan_msg)

        # ========================================================
        # 8. Publish /scan_sync
        # ========================================================

        self.scan_pub.publish(scan_msg)

        self.publish_count += 1

        # ========================================================
        # 9. Runtime Debug
        #
        # 這裡只監測現在的 receive delay。
        #
        # 絕對不會用它重新修改 fixed offset。
        # ========================================================

        if self.publish_count % 100 == 0:
            fixed_offset_sec = self.fixed_clock_offset_ns / 1_000_000_000.0

            receive_delay_ms = (pc_now_ns - corrected_ns) / 1_000_000.0

            self.get_logger().info(
                f"frame={self.frame_count} | "
                f"published={self.publish_count} | "
                f"points={len(scan_msg.ranges)} | "
                f"filtered={filtered_this_scan} | "
                f"fixed_offset="
                f"{fixed_offset_sec:.6f}s | "
                f"receive_delay="
                f"{receive_delay_ms:.3f}ms"
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
