"""Check tracking with moving image content, loss, and decoding time limits."""

from unittest.mock import Mock
import cv2
import numpy as np
from aruco_navigator.marker_tracking import CornerTracker, detect_with_fallback


def scene():
    image = np.full((160, 200), 220, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    image[60:80, 70:90] = cv2.aruco.generateImageMarker(dictionary, 3, 20)
    points = np.array([[70, 60], [89, 60], [89, 79], [70, 79]], np.float32)
    return image, points


def test_tracks_translation_and_expires_without_id_confirmation():
    original, points = scene()
    tracker = CornerTracker(max_duration=0.5)
    tracker.seed(original, points, 3, 0.0)
    for i in range(1, 6):
        shifted = cv2.warpAffine(
            original, np.float32([[1, 0, i], [0, 1, 0]]), (200, 160)
        )
        actual = tracker.update(shifted, 3, i * 0.1)
        assert actual is not None
        np.testing.assert_allclose(actual, points + [i, 0], atol=0.6)
    assert tracker.update(shifted, 3, 0.6) is None


def test_occlusion_is_rejected():
    original, points = scene()
    tracker = CornerTracker()
    tracker.seed(original, points, 3, 0.0)
    assert tracker.update(np.full_like(original, 220), 3, 0.05) is None


def test_wrong_id_and_long_frame_gap_are_rejected():
    original, points = scene()
    tracker = CornerTracker()
    tracker.seed(original, points, 3, 0.0)
    assert tracker.update(original, 1, 0.05) is None
    tracker.seed(original, points, 3, 0.0)
    assert tracker.update(original, 3, 0.3) is None


def test_fallback_returns_original_pixel_units():
    original, points = scene()
    native, fallback = Mock(), Mock()
    native.detectMarkers.return_value = ([], None, [])
    fallback.detectMarkers.return_value = (
        [points.reshape(1, 4, 2) * 3],
        np.array([[3]]),
        [],
    )
    corners, ids = detect_with_fallback(original, native, fallback, 3)
    np.testing.assert_allclose(corners[0].reshape(4, 2), points)
    assert ids[0, 0] == 3
