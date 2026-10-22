"""Check camera isolation and image selection without starting robot nodes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError
from cv_bridge import CvBridge
from deergo_mqtt_bridge import zed_camera_preflight, zed_image_viewer
from deergo_mqtt_bridge.zed_image_viewer import (
    ZedImageViewer,
    select_rgb_topic,
    window_is_open,
)
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, LogInfo
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "slam_launch", PACKAGE / "launch/slam_rviz.launch.py"
)
slam_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(slam_launch)


def test_discovery_ignores_other_cameras_depth_and_unrectified_images():
    rgb = "/zed/zed_node/rgb/color/rect/image"
    topics = [
        ("/another_camera/rgb/image_rect_color", ["sensor_msgs/msg/Image"]),
        ("/zed_other/rgb/image_rect_color", ["sensor_msgs/msg/Image"]),
        ("/zed/zed_node/depth/depth_registered", ["sensor_msgs/msg/Image"]),
        ("/zed/zed_node/rgb/image_rect_gray", ["sensor_msgs/msg/Image"]),
        ("/zed/zed_node/rgb/color/unrect/image", ["sensor_msgs/msg/Image"]),
        (
            "/zed/zed_node/rgb/image_rect_color/compressed",
            ["sensor_msgs/msg/CompressedImage"],
        ),
        (rgb, ["sensor_msgs/msg/Image"]),
    ]
    assert select_rgb_topic(topics, "/zed") == rgb
    assert select_rgb_topic(topics[:-1], "/zed") is None


def test_viewer_keeps_only_latest_message_and_converts_bgra():
    # Exercise the buffering/conversion code without DDS or a GUI.
    viewer = object.__new__(ZedImageViewer)
    viewer.bridge = CvBridge()
    first = viewer.bridge.cv2_to_imgmsg(np.zeros((2, 3, 4), dtype=np.uint8), "bgra8")
    pixels = np.full((2, 3, 4), 127, dtype=np.uint8)
    latest = viewer.bridge.cv2_to_imgmsg(pixels, "bgra8")
    viewer.receive_image(first)
    viewer.receive_image(latest)
    np.testing.assert_array_equal(viewer.take_frame(), pixels[:, :, :3])
    assert viewer.take_frame() is None


@pytest.mark.parametrize("size,expected", [((640, 360), True), ((-1, -1), False)])
def test_gtk_window_visibility_falls_back_to_image_rect(monkeypatch, size, expected):
    monkeypatch.setattr(zed_image_viewer.cv2, "getWindowProperty", lambda *_: -1.0)
    monkeypatch.setattr(
        zed_image_viewer.cv2, "getWindowImageRect", lambda _: (0, 0, *size)
    )
    assert window_is_open("ZED") is expected


def test_missing_wrapper_does_not_stop_slam(monkeypatch):
    def missing(_):
        raise PackageNotFoundError("zed_wrapper")

    monkeypatch.setattr(slam_launch, "get_package_share_directory", missing)
    actions = slam_launch.launch_zed_camera(LaunchContext(), str(PACKAGE))
    assert len(actions) == 1
    assert isinstance(actions[0], LogInfo)


@pytest.mark.parametrize("desktop", [True, False])
def test_camera_tf_remaps_are_scoped_and_viewer_needs_display(monkeypatch, desktop):
    monkeypatch.setattr(
        slam_launch, "get_package_share_directory", lambda _: "/fake/zed_wrapper"
    )
    context = LaunchContext()
    context.environment.pop("DISPLAY", None)
    context.environment.pop("WAYLAND_DISPLAY", None)
    if desktop:
        context.environment["DISPLAY"] = ":test"
    group = slam_launch.zed_camera_group(context, str(PACKAGE), "/fake/zed_wrapper")
    assert isinstance(group, GroupAction)
    actions = group.get_sub_entities()
    remaps = {
        perform_substitutions(context, action.src): perform_substitutions(
            context, action.dst
        )
        for action in actions
        if isinstance(action, SetRemap)
    }
    assert remaps["/tf"] == "/zed/tf"
    assert remaps["/tf_static"] == "/zed/tf_static"
    assert remaps["/diagnostics"] == "/zed/diagnostics"
    assert "ros_remaps" not in context.launch_configurations
    (camera,) = [
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    ]
    args = dict(camera.launch_arguments)
    assert args["container_name"] == ""
    assert args["namespace"] == ""
    assert args["node_name"] == "zed_node"
    for key in ("publish_tf", "publish_map_tf", "publish_imu_tf", "enable_gnss"):
        assert args[key] == "false"
    viewers = [action for action in actions if isinstance(action, Node)]
    assert len(viewers) == int(desktop)


def test_camera_configuration_disables_robot_state_outputs():
    document = yaml.safe_load((PACKAGE / "config/zed_camera.yaml").read_text())
    config = document["/**"]["ros__parameters"]
    tracking = config["pos_tracking"]
    assert not tracking["pos_tracking_enabled"]
    assert not tracking["publish_tf"]
    assert not tracking["publish_map_tf"]
    assert not tracking["publish_odom_pose"]
    assert config["depth"]["depth_stabilization"] == 0
    assert not config["mapping"]["mapping_enabled"]


def test_camera_keeps_full_resolution_and_numeric_depth_available():
    document = yaml.safe_load((PACKAGE / "config/zed_camera.yaml").read_text())
    config = document["/**"]["ros__parameters"]
    assert config["general"]["grab_resolution"] == "HD720"
    assert config["general"]["pub_resolution"] == "NATIVE"
    assert config["depth"]["publish_depth_map"]
    assert not config["depth"]["openni_depth_mode"]


def test_viewer_accepts_bgr_frames_without_changing_color_values():
    viewer = object.__new__(ZedImageViewer)
    viewer.bridge = CvBridge()
    pixels = np.array([[[10, 20, 30], [80, 90, 100]]], dtype=np.uint8)
    viewer.receive_image(viewer.bridge.cv2_to_imgmsg(pixels, "bgr8"))
    np.testing.assert_array_equal(viewer.take_frame(), pixels)


@pytest.mark.parametrize("busy", [True, False])
def test_preflight_resets_only_idle_zed(monkeypatch, tmp_path, busy):
    device = tmp_path / "2-1"
    video = device / "2-1:1.0/video4linux/video2"
    video.mkdir(parents=True)
    for name, value in [
        ("idVendor", "2b03"),
        ("idProduct", "f580"),
        ("busnum", "2"),
        ("devnum", "3"),
    ]:
        (device / name).write_text(value)
    opened, reset, closed = [], [], []

    def check_owner(command, **kwargs):
        assert command == ["fuser", "/test-usb/002/003", "/dev/video2"]
        return SimpleNamespace(returncode=0 if busy else 1)

    def open_usb(path, mode):
        opened.append(path)
        return 42

    monkeypatch.setattr(zed_camera_preflight.subprocess, "run", check_owner)
    monkeypatch.setattr(zed_camera_preflight.os, "open", open_usb)
    monkeypatch.setattr(zed_camera_preflight.os, "close", closed.append)
    monkeypatch.setattr(
        zed_camera_preflight.fcntl, "ioctl", lambda *args: reset.append(args)
    )
    assert zed_camera_preflight.reset_zed(tmp_path, Path("/test-usb")) is not busy
    assert opened == ([] if busy else [Path("/test-usb/002/003")])
    assert reset == ([] if busy else [(42, zed_camera_preflight.USBDEVFS_RESET)])
    assert closed == ([] if busy else [42])
