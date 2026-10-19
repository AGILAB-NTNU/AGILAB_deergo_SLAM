#!/usr/bin/env python3
"""Display the latest ZED RGB image without connecting to robot control."""

import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def select_rgb_topic(topics, namespace):
    """Discover only rectified color Image topics inside the camera namespace."""
    prefix = namespace.rstrip("/") + "/"
    candidates = [
        name
        for name, types in topics
        if name.startswith(prefix)
        and "sensor_msgs/msg/Image" in types
        and "/rgb/" in name
        and "rect" in name
        and not any(part in name for part in ("gray", "unrect", "raw"))
    ]
    return min(candidates, key=lambda name: (len(name), name), default=None)


def window_is_open(window):
    """Support GTK, where WND_PROP_VISIBLE returns -1 even for an open window."""
    try:
        visible = cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible > 0
        _, _, width, height = cv2.getWindowImageRect(window)
        return width > 0 and height > 0
    except cv2.error:
        return False


class ZedImageViewer(Node):
    """Keep one ROS message; conversion and all GUI work run in the main loop."""

    def __init__(self):
        super().__init__("zed_image_viewer")
        self.declare_parameter("image_topic", "")
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_subscription = None
        self.discovery_timer = self.create_timer(1.0, self.connect_image_topic)
        self.connect_image_topic()

    def connect_image_topic(self):
        if self.image_subscription is not None:
            return
        topic = self.get_parameter("image_topic").value
        prefix = self.get_namespace().rstrip("/") + "/"
        if topic:
            topic = self.resolve_topic_name(topic)
            if not topic.startswith(prefix):
                raise ValueError("image_topic must stay inside the viewer namespace")
        else:
            topic = select_rgb_topic(
                self.get_topic_names_and_types(), self.get_namespace()
            )
        if not topic:
            self.get_logger().info(
                "Waiting for a ZED RGB image topic...",
                throttle_duration_sec=5.0,
            )
            return
        self.image_subscription = self.create_subscription(
            Image,
            topic,
            self.receive_image,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        self.discovery_timer.cancel()
        self.get_logger().info(
            f"Displaying {topic}; close the window or press Q / Esc to exit"
        )

    def receive_image(self, message):
        self.latest_image = message

    def take_frame(self):
        message, self.latest_image = self.latest_image, None
        if message is None:
            return None
        try:
            return self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except (CvBridgeError, ValueError) as error:
            self.get_logger().warning(
                f"Cannot display image: {error}", throttle_duration_sec=5.0
            )
            return None


def main(args=None):
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print(
            "ZED viewer needs a desktop display; "
            "camera topics can run without the window."
        )
        return
    rclpy.init(args=args)
    node = None
    window = "DeerGo ZED Camera"
    try:
        node = ZedImageViewer()
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Waiting for ZED RGB image...",
            (30, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (230, 230, 230),
            2,
        )
        cv2.imshow(window, placeholder)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            frame = node.take_frame()
            if frame is not None:
                cv2.imshow(window, frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                break
            if not window_is_open(window):
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except cv2.error as error:
        if node is not None:
            node.get_logger().error(f"ZED window unavailable: {error}")
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
