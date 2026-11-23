"""Short-lived visual tracking between independently decoded ArUco observations."""

from collections import deque

import cv2
import numpy as np


class FrameDelay:
    """
    Frame delay that ignores a constant offset between two machines' clocks.

    The camera stamps frames with the robot's clock and the controller compares
    them with its own clock, so the raw difference is the clock offset plus the
    transport delay. The smallest difference seen recently is used as the
    reference; a frame is late only when it arrives later than that.
    """

    def __init__(self, window_sec=5.0, clock_jump_sec=2.0):
        self.window_sec = window_sec
        self.clock_jump_sec = clock_jump_sec
        self.samples = deque()
        self.offset = None

    def update(self, receive_sec, stamp_sec):
        raw = receive_sec - stamp_sec
        while self.samples and receive_sec - self.samples[0][0] > self.window_sec:
            self.samples.popleft()
        if self.samples and raw - self.samples[0][1] > self.clock_jump_sec:
            # One of the clocks was stepped (for example by NTP): start again.
            self.samples.clear()
        while self.samples and self.samples[-1][1] >= raw:
            self.samples.pop()
        self.samples.append((receive_sec, raw))
        self.offset = self.samples[0][1]
        return raw - self.offset


def make_detector(**parameters):
    """Build a DICT_ARUCO_ORIGINAL detector on OpenCV before and after 4.7."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    modern = hasattr(cv2.aruco, "ArucoDetector")
    options = (
        cv2.aruco.DetectorParameters() if modern else cv2.aruco.DetectorParameters_create()
    )
    for name, value in parameters.items():
        setattr(options, name, value)
    if modern:
        return cv2.aruco.ArucoDetector(dictionary, options)
    return LegacyArucoDetector(dictionary, options)


class LegacyArucoDetector:
    """Same detectMarkers() call as cv2.aruco.ArucoDetector, for OpenCV 4.5 (ROS Humble)."""

    def __init__(self, dictionary, parameters):
        self.dictionary = dictionary
        self.parameters = parameters

    def detectMarkers(self, image):
        return cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)


class CornerTracker:
    """Track only a previously decoded ID, with bidirectional and geometry checks."""

    def __init__(self, max_duration=1.0, max_frame_gap=0.2):
        self.max_duration = max_duration
        self.max_frame_gap = max_frame_gap
        self.clear()

    def clear(self):
        self.gray = None
        self.points = None
        self.marker_id = None
        self.decoded_at = None
        self.frame_at = None

    def seed(self, gray, points, marker_id, stamp):
        self.gray = gray.copy()
        self.points = np.asarray(points, np.float32).reshape(4, 1, 2)
        self.marker_id = marker_id
        self.decoded_at = self.frame_at = stamp

    def update(self, gray, marker_id, stamp):
        if (
            self.gray is None
            or marker_id != self.marker_id
            or gray.shape != self.gray.shape
            or not 0 < stamp - self.frame_at <= self.max_frame_gap
            or not 0 <= stamp - self.decoded_at <= self.max_duration
        ):
            self.clear()
            return None
        options = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        forward, status, error = cv2.calcOpticalFlowPyrLK(
            self.gray, gray, self.points, None, **options
        )
        if forward is None or not status.all() or not np.isfinite(forward).all():
            self.clear()
            return None
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(
            gray, self.gray, forward, None, **options
        )
        if (
            backward is None
            or not back_status.all()
            or not np.isfinite(backward).all()
            or not np.isfinite(error).all()
            or np.max(np.linalg.norm(backward - self.points, axis=2)) > 1.0
            or float(error.max()) > 25.0
        ):
            self.clear()
            return None
        points = forward.reshape(4, 2)
        old_area = abs(cv2.contourArea(self.points))
        area = abs(cv2.contourArea(points))
        h, w = gray.shape
        if (
            not cv2.isContourConvex(points)
            or area < 25
            or old_area <= 0
            or not 0.6 < area / old_area < 1.6
            or np.max(np.linalg.norm(forward - self.points, axis=2)) > 30
            or np.any(points < 2)
            or np.any(points[:, 0] > w - 3)
            or np.any(points[:, 1] > h - 3)
        ):
            self.clear()
            return None
        self.gray, self.points, self.frame_at = gray.copy(), forward, stamp
        return points


def detect_with_fallback(gray, detector, fallback_detector, expected_id, scale=3):
    """Retry decoding an enlarged image; always return original-image coordinates."""
    corners, ids, _ = detector.detectMarkers(gray)
    if expected_id is None or (ids is not None and expected_id in ids):
        return corners, ids
    enlarged = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    extra_corners, extra_ids, _ = fallback_detector.detectMarkers(enlarged)
    if extra_ids is not None and expected_id in extra_ids:
        index = list(extra_ids.flatten()).index(expected_id)
        return [extra_corners[index] / scale], np.array([[expected_id]])
    return corners, ids
