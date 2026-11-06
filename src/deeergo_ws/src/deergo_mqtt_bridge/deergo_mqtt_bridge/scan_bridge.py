#!/usr/bin/env python3
"""Apply causal, per-beam EMA to /scan and publish /scan_sync with original timing."""

import math
from copy import deepcopy

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PARAMETER_DEFAULTS = {
    "ema_enabled": True,
    "ema_alpha": 0.8,
    "ema_jump_threshold_m": 0.03,
    "ema_reset_gap_s": 0.2,
}


class ScanEMAFilter:
    """Smooth valid ranges independently, bypassing discontinuities and bad returns."""

    def __init__(self, alpha=0.8, jump_threshold_m=0.03, reset_gap_s=0.2):
        for name, value in (
            ("ema_alpha", alpha),
            ("ema_jump_threshold_m", jump_threshold_m),
            ("ema_reset_gap_s", reset_gap_s),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a finite positive number")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")
        if alpha > 1:
            raise ValueError("ema_alpha must be in (0, 1]")
        self.alpha = alpha
        self.jump_threshold_m = jump_threshold_m
        self.reset_gap_ns = reset_gap_s * 1e9
        self._previous = None

    def apply(self, scan):
        raw = tuple(scan.ranges)
        filtered = list(raw)
        stamp_ns = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        geometry = (
            scan.header.frame_id,
            len(raw),
            scan.angle_min,
            scan.angle_max,
            scan.angle_increment,
            scan.time_increment,
            scan.scan_time,
            scan.range_min,
            scan.range_max,
        )
        if self._previous is not None:
            previous_geometry, previous_stamp, previous_raw, previous_filtered = (
                self._previous
            )
            if (
                geometry == previous_geometry
                and 0 < stamp_ns - previous_stamp <= self.reset_gap_ns
            ):
                for index, value in enumerate(raw):
                    previous = previous_raw[index]
                    valid = (
                        math.isfinite(value)
                        and scan.range_min <= value <= scan.range_max
                    )
                    previous_valid = (
                        math.isfinite(previous)
                        and scan.range_min <= previous <= scan.range_max
                    )
                    if (
                        valid
                        and previous_valid
                        and abs(value - previous) <= self.jump_threshold_m
                    ):
                        filtered[index] = (
                            self.alpha * value
                            + (1 - self.alpha) * previous_filtered[index]
                        )
        self._previous = (geometry, stamp_ns, raw, filtered)
        result = deepcopy(scan)
        result.ranges = filtered
        return result


def filter_from_settings(settings):
    if not isinstance(settings["ema_enabled"], bool):
        raise ValueError("ema_enabled must be a boolean")
    return ScanEMAFilter(
        settings["ema_alpha"],
        settings["ema_jump_threshold_m"],
        settings["ema_reset_gap_s"],
    )


class ScanBridge(Node):
    def __init__(self):
        super().__init__("scan_bridge")
        for name, default in PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self._settings = self.read_settings()
        self._ema = filter_from_settings(self._settings)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.frame_count = 0
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_sync", qos_profile_sensor_data
        )
        self.get_logger().info(
            "/scan -> /scan_sync; per-beam EMA; original timestamp, frame, "
            "angles, timing and intensities preserved"
        )
        self.log_settings()

    def read_settings(self):
        return {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}

    def validate_parameters(self, parameters):
        candidate = self.read_settings()
        for parameter in parameters:
            if parameter.name in candidate:
                candidate[parameter.name] = parameter.value
        try:
            filter_from_settings(candidate)
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        # Validation has no state changes: other callbacks may still reject it.
        return SetParametersResult(successful=True)

    def log_settings(self):
        self.get_logger().info(
            f"EMA enabled={self._settings['ema_enabled']}, "
            f"alpha={self._settings['ema_alpha']}, "
            f"jump={self._settings['ema_jump_threshold_m']} m, "
            f"reset_gap={self._settings['ema_reset_gap_s']} s"
        )

    def scan_callback(self, msg):
        settings = self.read_settings()
        if settings != self._settings:
            # Read committed parameters and discard history before using new settings.
            self._ema = filter_from_settings(settings)
            self._settings = settings
            self.log_settings()
        output = self._ema.apply(msg) if settings["ema_enabled"] else msg
        self.scan_pub.publish(output)
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"forwarded={self.frame_count} | points={len(msg.ranges)} | "
                f"frame_id={msg.header.frame_id} | EMA={settings['ema_enabled']}"
            )


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
