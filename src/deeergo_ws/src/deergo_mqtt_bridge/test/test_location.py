"""Check named point persistence and the exact Nav2 goal without moving a robot."""

import math
from types import SimpleNamespace

import pytest
from deergo_mqtt_bridge import location


def test_save_multiple_points_updates_only_the_named_point(tmp_path):
    path = tmp_path / "points.yaml"
    location.save_point(path, "home", {"x": 1.0, "y": 2.0, "yaw": 0.5})
    location.save_point(path, "站點_A", {"x": 3.0, "y": 4.0, "yaw": -0.5})
    location.save_point(path, "home", {"x": 5.0, "y": 6.0, "yaw": 0.0})
    assert location.load_points(path) == {
        "home": {"x": 5.0, "y": 6.0, "yaw": 0.0},
        "站點_A": {"x": 3.0, "y": 4.0, "yaw": -0.5},
    }


@pytest.mark.parametrize(
    "contents",
    [
        "frame_id: odom\nlocations: {}\n",
        "frame_id: map\nlocations: {home: {x: .nan, y: 0, yaw: 0}}\n",
        "frame_id: map\nlocations: {home: {x: true, y: 0, yaw: 0}}\n",
    ],
)
def test_bad_data_is_not_silently_overwritten(tmp_path, contents):
    path = tmp_path / "points.yaml"
    path.write_text(contents)
    with pytest.raises(ValueError):
        location.save_point(path, "home", {"x": 0, "y": 0, "yaw": 0})
    assert path.read_text() == contents


def test_failed_write_keeps_the_previous_points(tmp_path, monkeypatch):
    path = tmp_path / "points.yaml"
    location.save_point(path, "home", {"x": 1, "y": 2, "yaw": 0})
    before = path.read_bytes()

    def fail(*_):
        raise OSError("disk failure")

    monkeypatch.setattr(location.os, "replace", fail)
    with pytest.raises(OSError):
        location.save_point(path, "other", {"x": 3, "y": 4, "yaw": 0})
    assert path.read_bytes() == before
    assert list(tmp_path.iterdir()) == [path]


@pytest.mark.parametrize("accepted", [True, False])
def test_go_preserves_map_position_and_heading_and_checks_acceptance(
    monkeypatch, accepted
):
    sent = []
    destroyed = []
    future = SimpleNamespace(
        done=lambda: True, result=lambda: SimpleNamespace(accepted=accepted)
    )
    client = SimpleNamespace(
        wait_for_server=lambda **_: True,
        send_goal_async=lambda goal: sent.append(goal) or future,
        destroy=lambda: destroyed.append(True),
    )
    monkeypatch.setattr(location, "ActionClient", lambda *args: client)
    monkeypatch.setattr(
        location.rclpy, "spin_until_future_complete", lambda *a, **k: None
    )
    from builtin_interfaces.msg import Time

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))
        )
    )
    if accepted:
        location.send_goal(node, {"x": 1.2, "y": -3.4, "yaw": math.pi / 2})
    else:
        with pytest.raises(RuntimeError, match="rejected"):
            location.send_goal(node, {"x": 1.2, "y": -3.4, "yaw": math.pi / 2})
    pose = sent[0].pose
    assert pose.header.frame_id == "map" and pose.header.stamp.sec == 123
    assert pose.pose.position.x == 1.2 and pose.pose.position.y == -3.4
    assert pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert pose.pose.orientation.w == pytest.approx(math.cos(math.pi / 4))
    assert destroyed == [True]
