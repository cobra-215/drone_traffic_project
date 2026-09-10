"""
Tests for SafetyManager.check_camera() -- the ground-side camera check
and the REQUIRE_CAMERA_PREFLIGHT policy around it.
"""

import pytest

from config import settings
from flight import exceptions as ex
from flight.safety import SafetyManager


class FakeCamera:
    def __init__(self, error=None):
        self._error = error
        self.checks = 0

    async def preflight_check(self):
        self.checks += 1
        if self._error is not None:
            raise self._error


def _safety(camera):
    # check_camera() touches neither the drone nor telemetry.
    return SafetyManager(drone=None, telemetry=None, camera=camera)


@pytest.fixture
def require_camera_preflight():
    original = settings.REQUIRE_CAMERA_PREFLIGHT

    def set_to(value):
        settings.REQUIRE_CAMERA_PREFLIGHT = value

    yield set_to
    settings.REQUIRE_CAMERA_PREFLIGHT = original


async def test_passing_camera_is_checked_once():
    camera = FakeCamera()
    await _safety(camera).check_camera()
    assert camera.checks == 1


async def test_missing_camera_is_skipped_not_failed(capsys):
    # SafetyManager must stay constructible without a camera -- the SITL
    # scenarios and several unit tests build one that way.
    await SafetyManager(drone=None, telemetry=None).check_camera()
    assert "skipped" in capsys.readouterr().out


async def test_failure_blocks_the_mission_when_required(
    require_camera_preflight,
):
    require_camera_preflight(True)
    camera = FakeCamera(error=RuntimeError("no cameras available"))

    with pytest.raises(ex.CameraPreflightError) as excinfo:
        await _safety(camera).check_camera()

    # The original fault must survive on the exception chain.
    assert isinstance(excinfo.value.__cause__, RuntimeError)
    assert "no cameras available" in str(excinfo.value)


async def test_failure_only_warns_when_not_required(
    require_camera_preflight, capsys
):
    require_camera_preflight(False)
    camera = FakeCamera(error=RuntimeError("card full"))

    await _safety(camera).check_camera()  # must not raise

    output = capsys.readouterr().out
    assert "WARNING" in output
    assert "card full" in output


async def test_camera_preflight_error_is_a_no_flight_command_exception():
    # The vehicle is still disarmed on the ground when this is raised,
    # so Emergency must not respond to it with RTL.
    from flight.emergency import NO_FLIGHT_COMMAND_EXCEPTIONS

    assert ex.CameraPreflightError in NO_FLIGHT_COMMAND_EXCEPTIONS
