import pytest

from traffic_light.traffic_light_node import GoSignDecision


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
