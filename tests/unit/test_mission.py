import pytest

from config import settings
from mission.mission import Mission, MissionStatus
from mission.position import Position
from mission.waypoint import Waypoint


def make_home():
    return Position(latitude_deg=47.0, longitude_deg=8.0, altitude_m=0.0)


def make_waypoint(altitude_m=None, hold_time_s=0.0, lat_offset=0.001):
    altitude_m = (
        altitude_m if altitude_m is not None else settings.OBSERVATION_ALTITUDE_M
    )
    return Waypoint(
        position=Position(
            latitude_deg=47.0 + lat_offset, longitude_deg=8.0, altitude_m=altitude_m
        ),
        hold_time_s=hold_time_s,
    )


def test_status_enum_has_no_paused_state():
    # No pause/resume for this phase -- see the project plan. Adding a
    # PAUSED status would invite an unreviewed resume path.
    assert not any("PAUS" in member.name for member in MissionStatus)


def test_validate_rejects_empty_mission():
    mission = Mission(waypoints=[], home=make_home())
    with pytest.raises(ValueError):
        mission.validate()


def test_validate_rejects_altitude_below_minimum():
    mission = Mission(
        waypoints=[make_waypoint(altitude_m=settings.MIN_FLIGHT_ALTITUDE_M - 1)],
        home=make_home(),
    )
    with pytest.raises(ValueError):
        mission.validate()


def test_validate_rejects_altitude_above_maximum():
    mission = Mission(
        waypoints=[make_waypoint(altitude_m=settings.MAX_FLIGHT_ALTITUDE_M + 1)],
        home=make_home(),
    )
    with pytest.raises(ValueError):
        mission.validate()


def test_validate_rejects_waypoint_too_far_from_home():
    far_waypoint = Waypoint(
        position=Position(
            latitude_deg=47.0 + 5.0,  # ~550 km away
            longitude_deg=8.0,
            altitude_m=settings.OBSERVATION_ALTITUDE_M,
        )
    )
    mission = Mission(waypoints=[far_waypoint], home=make_home())
    with pytest.raises(ValueError):
        mission.validate()


def test_validate_rejects_total_hold_time_exceeding_mission_timeout():
    mission = Mission(
        waypoints=[make_waypoint(hold_time_s=settings.MISSION_TIMEOUT + 1)],
        home=make_home(),
    )
    with pytest.raises(ValueError):
        mission.validate()


def test_validate_skips_distance_check_without_home():
    # Home is not known until after connecting to PX4; validate() must
    # not require it to be set (only the distance check depends on it).
    mission = Mission(waypoints=[make_waypoint()], home=None)
    mission.validate()


def test_validate_passes_for_a_reasonable_mission():
    mission = Mission(waypoints=[make_waypoint(hold_time_s=60)], home=make_home())
    mission.validate()


def test_status_transitions_happy_path():
    mission = Mission(waypoints=[make_waypoint()], home=make_home())
    assert mission.status == MissionStatus.PENDING

    mission.start()
    assert mission.status == MissionStatus.RUNNING
    assert mission.started_at is not None

    mission.complete()
    assert mission.status == MissionStatus.COMPLETED
    assert mission.ended_at is not None


def test_cannot_start_twice():
    mission = Mission(waypoints=[make_waypoint()], home=make_home())
    mission.start()
    with pytest.raises(RuntimeError):
        mission.start()


def test_cannot_complete_before_starting():
    mission = Mission(waypoints=[make_waypoint()], home=make_home())
    with pytest.raises(RuntimeError):
        mission.complete()


def test_abort_from_pending_records_reason():
    mission = Mission(waypoints=[make_waypoint()], home=make_home())
    mission.abort("preflight check failed")
    assert mission.status == MissionStatus.ABORTED
    assert mission.failure_reason == "preflight check failed"


def test_fail_from_running_records_reason():
    mission = Mission(waypoints=[make_waypoint()], home=make_home())
    mission.start()
    mission.fail("monitor raised MissionError")
    assert mission.status == MissionStatus.FAILED
    assert mission.failure_reason == "monitor raised MissionError"


def test_cannot_abort_a_completed_mission():
    mission = Mission(waypoints=[make_waypoint()], home=make_home())
    mission.start()
    mission.complete()
    with pytest.raises(RuntimeError):
        mission.abort("too late")


def test_mission_id_is_generated_and_unique():
    a = Mission(waypoints=[make_waypoint()], home=make_home())
    b = Mission(waypoints=[make_waypoint()], home=make_home())
    assert a.mission_id
    assert a.mission_id != b.mission_id
