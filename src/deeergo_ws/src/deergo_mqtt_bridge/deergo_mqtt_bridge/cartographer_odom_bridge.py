#!/usr/bin/env python3
"""Publish Cartographer's local TF pose and derived body velocity as odometry."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CartographerOdomBridge(Node):
    """Read odom -> base_link without broadcasting any transforms."""

    def __init__(self):
        super().__init__("cartographer_odom_bridge")
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.previous_pose = None
        self.timer = self.create_timer(0.02, self.publish_odometry)

    def publish_odometry(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", Time()
            )
        except TransformException:
            # Cartographer may not have received its first scan yet.
            return

        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        previous = self.previous_pose
        if previous is not None and stamp_ns <= previous[0]:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = translation.x
        odom.pose.pose.position.y = translation.y
        odom.pose.pose.position.z = translation.z
        odom.pose.pose.orientation = rotation

        if previous is not None:
            previous_stamp_ns, previous_x, previous_y, previous_yaw = previous
            dt = (stamp_ns - previous_stamp_ns) * 1e-9
            dx = translation.x - previous_x
            dy = translation.y - previous_y
            cos_yaw = math.cos(previous_yaw)
            sin_yaw = math.sin(previous_yaw)
            dtheta = yaw - previous_yaw
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
            odom.twist.twist.linear.x = (cos_yaw * dx + sin_yaw * dy) / dt
            odom.twist.twist.linear.y = (-sin_yaw * dx + cos_yaw * dy) / dt
            odom.twist.twist.angular.z = dtheta / dt

        self.previous_pose = (stamp_ns, translation.x, translation.y, yaw)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerOdomBridge()
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
