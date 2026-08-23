import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CompressedImage

from traffic_light.traffic_light_node import GoSignDecision, make_debug_image


def test_gosign_requires_stable_green_and_stops_immediately():
    decision = GoSignDecision(required_green_frames=3)

    assert decision.update([2]) is False
    assert decision.update([2]) is False
    assert decision.update([2]) is True
    assert decision.update([1]) is False
    assert decision.update([0, 2]) is False
    assert decision.update([2]) is False
    assert decision.update([]) is False


def test_gosign_rejects_invalid_confirmation_count():
    with pytest.raises(ValueError):
        GoSignDecision(required_green_frames=0)


def test_make_debug_image_preserves_header_and_encodes_jpeg():
    source_image = CompressedImage()
    source_image.header.frame_id = "camera"
    frame = np.zeros((8, 8, 3), dtype=np.uint8)

    debug_image = make_debug_image(frame, source_image)

    assert debug_image is not None
    assert debug_image.header.frame_id == "camera"
    assert debug_image.format == "jpeg"
    decoded = cv2.imdecode(
        np.frombuffer(debug_image.data, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert decoded.shape == frame.shape
