#!/usr/bin/env python3

import os
import time

import rclpy
from cartographer_ros_msgs.srv import WriteState
from nav2_msgs.srv import SaveMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


MAP_DIR = "maps"
MAP_NAME = "deergo_map"


class MapManager(Node):

    def __init__(self):
        super().__init__("map_manager")

        # ============================================================
        # Map paths
        # ============================================================

        self.map_dir = os.path.abspath(MAP_DIR)

        os.makedirs(
            self.map_dir,
            exist_ok=True,
        )

        # Nav2 map_saver writes:
        #   deergo_map.yaml
        #   deergo_map.pgm
        self.map_path = os.path.join(
            self.map_dir,
            MAP_NAME,
        )

        # Cartographer serialized state
        self.pbstream_path = os.path.join(
            self.map_dir,
            MAP_NAME + ".pbstream",
        )

        # ============================================================
        # Callback group
        # ============================================================

        self.callback_group = ReentrantCallbackGroup()

        # ============================================================
        # Nav2 Map Saver client
        #
        # Saves Cartographer's /map OccupancyGrid to:
        #   .pgm
        #   .yaml
        #
        # Requires map_saver_server to be running.
        # ============================================================

        self.save_map_client = self.create_client(
            SaveMap,
            "/map_saver/save_map",
            callback_group=self.callback_group,
        )

        # ============================================================
        # Cartographer client
        #
        # Saves Cartographer SLAM state to:
        #   .pbstream
        # ============================================================

        self.write_state_client = self.create_client(
            WriteState,
            "/write_state",
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

        # ============================================================
        # Startup information
        # ============================================================

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Cartographer Map Manager ready"
        )

        self.get_logger().info(
            f"Occupancy map : {self.map_path}.yaml / .pgm"
        )

        self.get_logger().info(
            f"PBStream      : {self.pbstream_path}"
        )

        self.get_logger().info(
            "Service       : /save_deergo_map"
        )

        self.get_logger().info(
            "PBStream mode : include unfinished submaps"
        )

        self.get_logger().info(
            "========================================"
        )

    # ================================================================
    # Wait future
    # ================================================================

    def wait_future(
        self,
        future,
        timeout=10.0,
    ):
        start_time = time.monotonic()

        while not future.done():

            if time.monotonic() - start_time > timeout:
                return None

            time.sleep(0.05)

        return future.result()

    # ================================================================
    # /save_deergo_map
    # ================================================================

    def save_map_callback(
        self,
        request,
        response,
    ):
        self.get_logger().info(
            "Saving DeerGo Cartographer map..."
        )

        # ============================================================
        # Check required services
        # ============================================================

        if not self.save_map_client.wait_for_service(
            timeout_sec=2.0
        ):
            response.success = False
            response.message = (
                "/map_saver/save_map not available. "
                "Start Nav2 map_saver_server first."
            )

            return response

        if not self.write_state_client.wait_for_service(
            timeout_sec=2.0
        ):
            response.success = False
            response.message = (
                "/write_state not available. "
                "Cartographer node is not ready."
            )

            return response

        # ============================================================
        # 1. Save occupancy map
        #
        # Source:
        #   /map
        #
        # Output:
        #   maps/deergo_map.pgm
        #   maps/deergo_map.yaml
        # ============================================================

        save_request = SaveMap.Request()

        save_request.map_topic = "/map"
        save_request.map_url = self.map_path

        save_request.image_format = "pgm"
        save_request.map_mode = "trinary"

        save_request.free_thresh = 0.25
        save_request.occupied_thresh = 0.65

        self.get_logger().info(
            f"Saving occupancy map: {self.map_path}"
        )

        save_future = self.save_map_client.call_async(
            save_request
        )

        save_result = self.wait_future(
            save_future,
            timeout=15.0,
        )

        if save_result is None:

            self.get_logger().error(
                "Occupancy map save timeout"
            )

            response.success = False
            response.message = (
                "Occupancy map save timeout"
            )

            return response

        if not save_result.result:

            self.get_logger().error(
                "Occupancy map save failed"
            )

            response.success = False
            response.message = (
                "Occupancy map save failed"
            )

            return response

        self.get_logger().info(
            "Occupancy map saved"
        )

        # ============================================================
        # 2. Save Cartographer state
        #
        # Output:
        #   maps/deergo_map.pbstream
        #
        # include_unfinished_submaps=True:
        #   This behaves like a snapshot and does NOT require
        #   finishing the current trajectory first.
        #
        # This lets mapping continue after saving.
        # ============================================================

        write_request = WriteState.Request()

        write_request.filename = self.pbstream_path
        write_request.include_unfinished_submaps = True

        self.get_logger().info(
            f"Writing PBStream: {self.pbstream_path}"
        )

        write_future = self.write_state_client.call_async(
            write_request
        )

        write_result = self.wait_future(
            write_future,
            timeout=30.0,
        )

        if write_result is None:

            self.get_logger().error(
                "PBStream write timeout"
            )

            response.success = False
            response.message = (
                "Occupancy map saved, "
                "but PBStream write timed out"
            )

            return response

        # Cartographer services use StatusResponse.
        # gRPC status code 0 = OK.
        if write_result.status.code != 0:

            self.get_logger().error(
                "PBStream write failed: "
                f"code={write_result.status.code}, "
                f"message={write_result.status.message}"
            )

            response.success = False
            response.message = (
                "Occupancy map saved, "
                "but PBStream write failed: "
                f"{write_result.status.message}"
            )

            return response

        self.get_logger().info(
            "PBStream saved"
        )

        # ============================================================
        # Success
        # ============================================================

        response.success = True
        response.message = (
            "DeerGo map saved: "
            f"{self.map_path}.yaml/.pgm + "
            f"{self.pbstream_path}"
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = MapManager()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

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