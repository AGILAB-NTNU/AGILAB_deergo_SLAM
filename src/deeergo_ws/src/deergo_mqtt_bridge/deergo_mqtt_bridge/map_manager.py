#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from slam_toolbox.srv import (
    SaveMap,
    SerializePoseGraph,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

MAP_DIR = "/home/helson/ROS2_tb4/deergo_ws/maps"
MAP_NAME = "deergo_map"


class MapManager(Node):
    def __init__(self):
        super().__init__("map_manager")

        # ============================================================
        # Map path
        # ============================================================

        os.makedirs(
            MAP_DIR,
            exist_ok=True,
        )

        self.map_path = os.path.join(
            MAP_DIR,
            MAP_NAME,
        )

        # ============================================================
        # Callback group
        # ============================================================

        self.callback_group = ReentrantCallbackGroup()

        # ============================================================
        # SLAM Toolbox clients
        # ============================================================

        self.save_map_client = self.create_client(
            SaveMap,
            "/slam_toolbox/save_map",
            callback_group=self.callback_group,
        )

        self.serialize_client = self.create_client(
            SerializePoseGraph,
            "/slam_toolbox/serialize_map",
            callback_group=self.callback_group,
        )

        # ============================================================
        # User service
        # ============================================================

        self.save_service = self.create_service(
            Trigger,
            "/save_deergo_map",
            self.save_map_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Map Manager ready")

        self.get_logger().info(f"Save path: {self.map_path}")

        self.get_logger().info("Service: /save_deergo_map")

    # ================================================================
    # Wait future
    # ================================================================

    def wait_future(self, future, timeout=10.0):
        start_time = time.time()

        while not future.done():
            if time.time() - start_time > timeout:
                return None

            time.sleep(0.05)

        return future.result()

    # ================================================================
    # /save_deergo_map
    # ================================================================

    def save_map_callback(self, request, response):
        self.get_logger().info("Saving DeerGo map...")

        # ============================================================
        # Check SLAM services
        # ============================================================

        if not self.save_map_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "/slam_toolbox/save_map not available"
            return response

        if not self.serialize_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "/slam_toolbox/serialize_map not available"
            return response

        # ============================================================
        # Save occupancy map
        #
        # .pgm
        # .yaml
        # ============================================================

        save_request = SaveMap.Request()

        save_request.name = String(data=self.map_path)

        self.get_logger().info(f"Saving occupancy map: {self.map_path}")

        save_future = self.save_map_client.call_async(save_request)

        save_result = self.wait_future(
            save_future,
            timeout=10.0,
        )

        if save_result is None:
            self.get_logger().error("SLAM map save timeout")

            response.success = False
            response.message = "SLAM map save timeout"

            return response

        if save_result.result != 0:
            self.get_logger().error(
                f"SLAM map save failed: " f"result={save_result.result}"
            )

            response.success = False
            response.message = f"SLAM map save failed: " f"result={save_result.result}"

            return response

        self.get_logger().info("Occupancy map saved")

        # ============================================================
        # Serialize SLAM pose graph
        #
        # .posegraph
        # .data
        # ============================================================

        serialize_request = SerializePoseGraph.Request()

        serialize_request.filename = self.map_path

        self.get_logger().info("Serializing SLAM pose graph...")

        serialize_future = self.serialize_client.call_async(serialize_request)

        serialize_result = self.wait_future(
            serialize_future,
            timeout=10.0,
        )

        if serialize_result is None:
            self.get_logger().error("Pose graph serialization timeout")

            response.success = False
            response.message = (
                "Map image saved, " "but pose graph serialization timed out"
            )

            return response

        if serialize_result.result != 0:
            self.get_logger().error(
                f"Pose graph serialization failed: " f"result={serialize_result.result}"
            )

            response.success = False
            response.message = (
                "Map image saved, "
                f"but pose graph failed: "
                f"result={serialize_result.result}"
            )

            return response

        self.get_logger().info("Pose graph saved")

        # ============================================================
        # Success
        # ============================================================

        response.success = True
        response.message = f"DeerGo map saved: {self.map_path}"

        return response


def main(args=None):
    rclpy.init(args=args)

    node = MapManager()

    executor = MultiThreadedExecutor(num_threads=4)

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
