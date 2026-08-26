import pytest

from config import settings
from mission.position import Position
from mission.waypoint import Waypoint


def make_position():
    return Position(latitude_deg=47.0, longitude_deg=8.0, altitude_m=20.0)


def test_defaults_come_from_settings():
    wp = Waypoint(position=make_position())
    assert wp.acceptance_radius_m == settings.WAYPOINT_ACCEPTANCE_RADIUS_M
    assert wp.altitude_tolerance_m == settings.WAYPOINT_ALTITUDE_TOLERANCE_M
    assert wp.timeout_s == settings.WAYPOINT_TIMEOUT
    assert wp.hold_time_s == 0.0


def test_explicit_overrides_are_respected():
    wp = Waypoint(
        position=make_position(),
        acceptance_radius_m=5.0,
        altitude_tolerance_m=2.0,
        timeout_s=30.0,
        hold_time_s=900,
    )
    assert wp.acceptance_radius_m == 5.0
    assert wp.altitude_tolerance_m == 2.0
    assert wp.timeout_s == 30.0
    assert wp.hold_time_s == 900


def test_negative_hold_time_rejected():
    with pytest.raises(ValueError):
        Waypoint(position=make_position(), hold_time_s=-1.0)


@pytest.mark.parametrize(
    "field", ["acceptance_radius_m", "altitude_tolerance_m", "timeout_s"]
)
def test_non_positive_tolerances_rejected(field):
    with pytest.raises(ValueError):
        Waypoint(position=make_position(), **{field: 0.0})


def test_is_immutable():
    wp = Waypoint(position=make_position())
    with pytest.raises(Exception):
        wp.hold_time_s = 100
