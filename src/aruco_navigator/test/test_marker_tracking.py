"""Check tracking with moving image content, loss, and decoding time limits."""

from unittest.mock import Mock
import cv2
import numpy as np
import pytest
from aruco_navigator.marker_tracking import (
    CornerTracker,
    FrameDelay,
    detect_with_fallback,
    make_detector,
)


def marker(marker_id, size):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    return cv2.aruco.drawMarker(dictionary, marker_id, size)


def scene():
    image = np.full((160, 200), 220, np.uint8)
    image[60:80, 70:90] = marker(3, 20)
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


def test_frame_delay_ignores_constant_clock_offset():
    delay = FrameDelay()
    # Sender clock 1889.8 s behind; transport delay 30-60 ms.
    assert delay.update(2000.00, 110.17) == pytest.approx(0.0, abs=1e-6)
    assert delay.update(2000.10, 110.27) == pytest.approx(0.0, abs=1e-6)
    assert delay.update(2000.20, 110.36) == pytest.approx(0.01, abs=1e-6)
    assert delay.offset == pytest.approx(1889.83, abs=1e-6)
    assert delay.update(2001.00, 110.30) == pytest.approx(0.87, abs=1e-6)


def test_frame_delay_forgets_old_samples():
    delay = FrameDelay(window_sec=5.0)
    delay.update(0.0, -0.01)
    assert delay.update(4.0, 3.95) == pytest.approx(0.04, abs=1e-6)
    assert delay.update(6.0, 5.95) == pytest.approx(0.0, abs=1e-6)


def test_frame_delay_restarts_after_clock_step():
    delay = FrameDelay(clock_jump_sec=2.0)
    delay.update(100.0, 99.95)
    # Robot clock stepped back by 30 s.
    assert delay.update(100.1, 70.0) == pytest.approx(0.0, abs=1e-6)
    assert delay.offset == pytest.approx(30.1, abs=1e-6)


@pytest.mark.parametrize("marker_id", [1, 2, 3, 4])
def test_detector_reads_marker_ids_on_installed_opencv(marker_id):
    image = np.full((240, 320), 255, np.uint8)
    image[80:160, 120:200] = marker(marker_id, 80)
    for detector in (make_detector(), make_detector(adaptiveThreshWinSizeMin=5)):
        corners, ids, _ = detector.detectMarkers(image)
        assert ids is not None and ids.flatten().tolist() == [marker_id]
