"""Exercise steering and marker-loss recovery without a simulator or ROS graph."""

from types import SimpleNamespace
from unittest.mock import Mock

import time
import numpy as np
import pytest
from rclpy.time import Time

from aruco_navigator.aruco_navigator_node import ArucoNavigatorNode, State
from aruco_navigator.marker_tracking import CornerTracker


@pytest.fixture
def controller():
    node = SimpleNamespace(
        p={
            "yaw_tolerance_px": 12.0,
            "kp_yaw": 0.003,
            "kp_dist": 0.0015,
            "max_align_angular_vel": 0.35,
            "min_angular_vel": 0.06,
            "max_linear_vel": 0.18,
            "min_linear_vel": 0.035,
            "target_marker_size_px": 166.0,
            "marker_size_m": 0.2,
            "dist_tolerance_px": 6.0,
            "observation_stale_sec": 0.15,
            "marker_timeout_sec": 4.0,
            "alignment_forward_speed": 0.08,
            "alignment_stop_error_fraction": 0.15,
            "alignment_near_target_ratio": 0.8,
            "min_detected_marker_side_px": 12.0,
            "marker_confirm_frames": 5,
            "sim_focal_length": 381.36,
            "recovery_far_target_ratio": 0.5,
            "recovery_speed": 0.06,
            "recovery_max_distance_m": 0.4,
            "recovery_timeout_sec": 8.0,
            "search_angular_vel": 0.12,
            "sensor_timeout_sec": 0.5,
            "image_max_latency_sec": 0.5,
        },
        state=State.ALIGNING,
        marker_id=3,
        used={1, 2},
        candidate_id=None,
        candidate_count=0,
        transit_forward=False,
        observation=(360.0, 20.0),
        last_seen=Time(seconds=10),
        latest_image=np.zeros((480, 640, 3), dtype=np.uint8),
        fx_ratio=1.0,
        yaw=1.5,
        target_yaw=1.5,
        cmd_pub=Mock(),
        reached=Mock(),
        detector=Mock(),
        fallback_detector=Mock(),
        tracker=CornerTracker(),
        calibrated=True,
        last_image_wall=time.monotonic(),
        last_odom_wall=time.monotonic(),
        observation_source="decoded",
        position=None,
        observation_bearing=None,
        get_clock=Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=10))),
        get_logger=Mock(return_value=Mock()),
        normalize=ArucoNavigatorNode.normalize,
    )
    node.fallback_detector.detectMarkers.return_value = ([], None, [])
    node.expected_marker_id = lambda: ArucoNavigatorNode.expected_marker_id(node)
    return node


def command(node):
    ArucoNavigatorNode.control(node)
    return node.cmd_pub.publish.call_args.args[0]


def frame(node, marker_id=3, side=12.0):
    corners = np.array(
        [[[350, 230], [350 + side, 230], [350 + side, 230 + side], [350, 230 + side]]],
        dtype=np.float32,
    )
    node.detector.detectMarkers.return_value = ([corners], np.array([[marker_id]]), [])
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        encoding="bgr8",
        height=480,
        width=640,
        step=1920,
        data=bytes(480 * 1920),
    )
    ArucoNavigatorNode.on_image(node, msg)


def test_moderate_error_corrected_while_travelling(controller):
    cmd = command(controller)
    assert 0 < cmd.linear.x <= 0.08
    assert cmd.angular.z < 0  # Marker to the right.
    controller.observation = (280.0, 20.0)
    cmd = command(controller)
    assert cmd.linear.x > 0
    assert cmd.angular.z > 0


@pytest.mark.parametrize("u,side", [(477.0, 20.0), (360.0, 140.0)])
def test_large_error_or_near_marker_aligns_in_place(controller, u, side):
    controller.observation = (u, side)
    cmd = command(controller)
    assert cmd.linear.x == 0
    assert cmd.angular.z < 0


def test_centered_approach_uses_size_error(controller):
    controller.observation = (320.0, 100.0)
    cmd = command(controller)
    assert controller.state == State.APPROACHING
    assert cmd.linear.x == pytest.approx(0.0015 * 66)


def test_arrival_zeros_entire_command(controller):
    controller.observation = (325.0, 166.0)
    cmd = command(controller)
    controller.reached.assert_called_once()
    assert cmd.linear.x == cmd.angular.z == 0


def test_arrival_requires_centering_despite_hysteresis(controller):
    controller.state = State.APPROACHING
    controller.observation = (335.0, 166.0)
    command(controller)
    controller.reached.assert_not_called()


def test_stale_observation_stops_motion(controller):
    controller.last_seen = Time(seconds=9)
    cmd = command(controller)
    assert cmd.linear.x == cmd.angular.z == 0


def test_distant_acquisition_needs_five_consecutive_frames(controller):
    controller.state = State.SEARCHING
    controller.marker_id = None
    for _ in range(4):
        frame(controller)
    assert controller.state == State.SEARCHING
    frame(controller, marker_id=1)  # Used ID interrupts confirmation.
    assert controller.candidate_count == 0
    for _ in range(5):
        frame(controller)
    assert controller.state == State.ALIGNING
    assert controller.marker_id == 3
    frame(controller, side=10.0)
    assert controller.observation[1] == pytest.approx(10.0)


def test_lost_target_returns_to_visual_servoing(controller):
    controller.last_seen = Time(seconds=5)
    cmd = command(controller)
    assert controller.state == State.SEARCHING
    assert cmd.linear.x == 0
    for _ in range(4):
        frame(controller)
    assert controller.state == State.SEARCHING
    frame(controller)
    assert controller.state == State.ALIGNING
    assert command(controller).linear.x > 0


@pytest.mark.parametrize("missing", ["calibrated", "last_image_wall", "last_odom_wall"])
def test_sensor_interlock(controller, missing):
    setattr(controller, missing, False if missing == "calibrated" else None)
    cmd = command(controller)
    assert cmd.linear.x == cmd.angular.z == 0


def test_tracking_alone_cannot_complete_marker(controller):
    controller.observation_source = "tracked"
    controller.observation = (320.0, 166.0)
    command(controller)
    controller.reached.assert_not_called()


def test_distant_loss_advances_with_bounds(controller):
    controller.position = (0.0, 0.0)
    controller.observation_bearing = controller.yaw
    controller.last_seen = Time(seconds=9)
    assert command(controller).linear.x == 0
    assert controller.state == State.RECOVERING
    assert command(controller).linear.x == 0.06
    controller.position = (0.41, 0.0)
    assert command(controller).linear.x == 0
    for _ in range(5):
        frame(controller)
    assert controller.state == State.ALIGNING


def test_recovery_does_not_advance_towards_close_marker(controller):
    controller.position = (0.0, 0.0)
    controller.observation_bearing = controller.yaw
    controller.observation = (330.0, 150.0)
    controller.last_seen = Time(seconds=9)
    assert command(controller).linear.x == 0
    assert controller.state != State.RECOVERING
