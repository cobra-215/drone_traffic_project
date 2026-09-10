import pytest

from config import settings


def test_default_settings_are_valid():
    settings.validate_settings()


@pytest.fixture
def restore_settings():
    """Snapshot every module attribute validate_settings() depends on and
    restore it after the test, so tests can mutate the module in place."""

    tracked = [
        "BATTERY_PREFLIGHT_MIN_PERCENT",
        "BATTERY_RTL_THRESHOLD",
        "BATTERY_CRITICAL_THRESHOLD",
        "MIN_FLIGHT_ALTITUDE_M",
        "TAKEOFF_ALTITUDE_M",
        "OBSERVATION_ALTITUDE_M",
        "MAX_FLIGHT_ALTITUDE_M",
        "MAX_DISTANCE_FROM_HOME_M",
        "WAYPOINT_ACCEPTANCE_RADIUS_M",
        "WAYPOINT_ALTITUDE_TOLERANCE_M",
        "MISSION_SITE",
        "CAMERA_BACKEND",
        "CAMERA_RESOLUTION",
        "CAMERA_FRAMERATE",
        "CAMERA_BITRATE",
        "RECORDING_OUTPUT_DIR",
        "VIDEO_DURATION",
    ]
    original = {name: getattr(settings, name) for name in tracked}
    yield
    for name, value in original.items():
        setattr(settings, name, value)


def test_rejects_preflight_threshold_not_above_rtl_threshold(restore_settings):
    settings.BATTERY_PREFLIGHT_MIN_PERCENT = settings.BATTERY_RTL_THRESHOLD
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_rtl_threshold_not_above_critical_threshold(restore_settings):
    settings.BATTERY_RTL_THRESHOLD = settings.BATTERY_CRITICAL_THRESHOLD
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_takeoff_altitude_at_or_below_minimum(restore_settings):
    settings.TAKEOFF_ALTITUDE_M = settings.MIN_FLIGHT_ALTITUDE_M
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_observation_altitude_below_takeoff_altitude(restore_settings):
    settings.OBSERVATION_ALTITUDE_M = settings.TAKEOFF_ALTITUDE_M - 0.1
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_observation_altitude_at_or_above_maximum(restore_settings):
    settings.OBSERVATION_ALTITUDE_M = settings.MAX_FLIGHT_ALTITUDE_M
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_non_positive_distance_limit(restore_settings):
    settings.MAX_DISTANCE_FROM_HOME_M = 0
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_unknown_mission_site(restore_settings):
    settings.MISSION_SITE = "not_a_real_site"
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_unknown_camera_backend(restore_settings):
    settings.CAMERA_BACKEND = "webcam"
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_default_camera_backend_is_simulation():
    # This repository is checked out on machines with no camera (the
    # development machine, CI), so the committed default must stay
    # "simulation". Selecting "picamera2" is a deliberate, Pi-local
    # change -- see docs/raspberry_pi_setup.md.
    assert settings.CAMERA_BACKEND == "simulation"


def test_rejects_bad_camera_resolution(restore_settings):
    for bad in ((0, 1080), (1920,), (1920, 0), 1920, None):
        settings.CAMERA_RESOLUTION = bad
        with pytest.raises(ValueError):
            settings.validate_settings()


def test_accepts_camera_resolution_as_a_list(restore_settings):
    settings.CAMERA_RESOLUTION = [1280, 720]
    settings.validate_settings()


def test_rejects_non_positive_framerate(restore_settings):
    settings.CAMERA_FRAMERATE = 0
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_non_positive_bitrate(restore_settings):
    settings.CAMERA_BITRATE = -1
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_non_positive_video_duration(restore_settings):
    settings.VIDEO_DURATION = 0
    with pytest.raises(ValueError):
        settings.validate_settings()


def test_rejects_empty_recording_output_dir(restore_settings):
    settings.RECORDING_OUTPUT_DIR = ""
    with pytest.raises(ValueError):
        settings.validate_settings()
