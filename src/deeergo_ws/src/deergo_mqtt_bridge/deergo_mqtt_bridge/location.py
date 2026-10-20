#!/usr/bin/env python3
"""Save named map poses, or send a saved pose directly to Nav2 from a terminal."""

import argparse
import math
import os
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def default_points_file():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/deergo_mqtt_bridge/package.xml").is_file():
            return parent / "maps/deergo_locations.yaml"
    raise ValueError("Cannot locate the workspace maps folder. Specify --file.")


def load_points(path):
    if not path.exists():
        return {}
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if (
        not isinstance(document, dict)
        or document.get("frame_id") != "map"
        or not isinstance(document.get("locations"), dict)
    ):
        raise ValueError("Expected frame_id: map and a locations dictionary in YAML.")
    points = document["locations"]
    for name, pose in points.items():
        if (
            not isinstance(name, str)
            or not name.strip()
            or not isinstance(pose, dict)
            or set(pose) != {"x", "y", "yaw"}
            or any(
                type(v) not in (int, float) or not math.isfinite(v)
                for v in pose.values()
            )
        ):
            raise ValueError(f"Invalid map coordinates for {name!r}.")
    return points


def save_point(path, name, pose):
    points = load_points(path)
    points[name] = pose
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(
                {"frame_id": "map", "locations": points},
                stream,
                allow_unicode=True,
                sort_keys=True,
            )
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def current_pose(node):
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 5
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                tf = buffer.lookup_transform("map", "base_link", Time())
            except TransformException:
                continue
            stamp = tf.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if not stamp_ns or abs(node.get_clock().now().nanoseconds - stamp_ns) > 1e9:
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            values = (p.x, p.y, q.x, q.y, q.z, q.w)
            norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            if (
                not all(math.isfinite(v) for v in values)
                or not math.isfinite(norm)
                or norm < 1e-9
            ):
                raise ValueError("Invalid map pose.")
            x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            return {"x": p.x, "y": p.y, "yaw": yaw}
    finally:
        listener.unregister()
    raise RuntimeError(
        "No fresh map -> base_link TF. Start localization and check ROS_DOMAIN_ID."
    )


def send_goal(node, pose):
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    try:
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError(
                "Nav2 navigate_to_pose is unavailable. Start Nav2 first."
            )
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose["x"])
        goal.pose.pose.position.y = float(pose["y"])
        goal.pose.pose.orientation.z = math.sin(pose["yaw"] / 2)
        goal.pose.pose.orientation.w = math.cos(pose["yaw"] / 2)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError(
                "Goal reply timed out; navigation may be active. Check RViz."
            )
        if not future.result().accepted:
            raise RuntimeError("Nav2 rejected the goal.")
    finally:
        client.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("save", "list", "go"))
    parser.add_argument("name", nargs="?")
    parser.add_argument("--file", type=Path)
    options, ros_args = parser.parse_known_args(args)
    if options.command != "list" and not (options.name and options.name.strip()):
        parser.error("save/go requires a location name")
    if options.command == "list" and options.name:
        parser.error("list does not take a location name")
    node = None
    try:
        path = (options.file or default_points_file()).expanduser().resolve()
        points = load_points(path)
        if options.command == "list":
            print(yaml.safe_dump(points, allow_unicode=True, sort_keys=True))
            return 0
        if options.command == "go" and options.name not in points:
            raise ValueError(f"Unknown location '{options.name}'. Use location list.")
        rclpy.init(args=ros_args)
        node = rclpy.create_node("location_command")
        if options.command == "save":
            pose = current_pose(node)
            save_point(path, options.name, pose)
            print(f"Saved '{options.name}' in {path}: {pose}")
        else:
            send_goal(node, points[options.name])
            print(
                f"Nav2 accepted '{options.name}'. "
                "Navigation continues after this command exits."
            )
        return 0
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as exc:
        print(f"Error: {exc}")
        return 1
    except KeyboardInterrupt:
        print("Interrupted. If go was sent, check navigation in RViz.")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
