import pytest

from config import settings
from flight import exceptions as ex
from flight.emergency import Emergency


class FakeDrone:
    def __init__(self, rtl_error=None, land_error=None):
        self.rtl_calls = 0
        self.land_calls = 0
        self.hold_calls = 0
        self._rtl_error = rtl_error
        self._land_error = land_error

    async def return_to_launch(self):
        self.rtl_calls += 1
        if self._rtl_error is not None:
            raise self._rtl_error

    async def land(self):
        self.land_calls += 1
        if self._land_error is not None:
            raise self._land_error

    async def hold(self):
        self.hold_calls += 1


class FakeTelemetry:
    def __init__(self, battery=None, raise_on_read=False):
        self._battery = battery
        self._raise_on_read = raise_on_read

    async def get_battery(self):
        if self._raise_on_read:
            raise ex.TelemetryTimeoutError("stalled")
        return self._battery


# Acceptance test for D1/D2: link loss, telemetry stalls, arming failures,
# and unhealthy GPS/home state must never trigger a flight command --
# PX4's own failsafe (if airborne) or the ground state (if not armed)
# governs the aircraft in every one of these cases.
NO_COMMAND_EXCEPTIONS = [
    ex.PX4ConnectionError("x"),
    ex.LinkLostError("x"),
    ex.TelemetryTimeoutError("x"),
    ex.ArmingError("x"),
    ex.GPSNotReadyError("x"),
    ex.HomePositionNotReadyError("x"),
]


@pytest.mark.parametrize("exception", NO_COMMAND_EXCEPTIONS)
async def test_no_flight_command_exceptions_issue_no_command(exception):
    drone = FakeDrone()
    emergency = Emergency(drone=drone, telemetry=FakeTelemetry())

    await emergency.handle_exception(exception)

    assert drone.rtl_calls == 0
    assert drone.land_calls == 0
    assert drone.hold_calls == 0


async def test_low_battery_above_critical_requests_rtl():
    drone = FakeDrone()
    emergency = Emergency(drone=drone, telemetry=FakeTelemetry())

    battery = settings.BATTERY_CRITICAL_THRESHOLD + 1
    await emergency.handle_exception(
        ex.LowBatteryError("low", battery_percent=battery)
    )

    assert drone.rtl_calls == 1
    assert drone.land_calls == 0


async def test_low_battery_at_or_below_critical_lands_immediately():
    drone = FakeDrone()
    emergency = Emergency(drone=drone, telemetry=FakeTelemetry())

    await emergency.handle_exception(
        ex.LowBatteryError("critical", battery_percent=settings.BATTERY_CRITICAL_THRESHOLD)
    )

    assert drone.land_calls == 1
    assert drone.rtl_calls == 0


async def test_low_battery_without_carried_value_falls_back_to_reread():
    drone = FakeDrone()
    telemetry = FakeTelemetry(battery=settings.BATTERY_CRITICAL_THRESHOLD - 1)
    emergency = Emergency(drone=drone, telemetry=telemetry)

    # No battery_percent carried on the exception -- must fall back to a
    # guarded re-read rather than assuming either outcome.
    await emergency.handle_exception(ex.LowBatteryError("low"))

    assert drone.land_calls == 1


async def test_low_battery_reread_failure_defaults_to_landing():
    drone = FakeDrone()
    telemetry = FakeTelemetry(raise_on_read=True)
    emergency = Emergency(drone=drone, telemetry=telemetry)

    await emergency.handle_exception(ex.LowBatteryError("low"))

    # If we cannot even confirm the battery level, land now rather than
    # trust an RTL flight over an uncertain/possibly failing link.
    assert drone.land_calls == 1
    assert drone.rtl_calls == 0


@pytest.mark.parametrize(
    "exception",
    [ex.TakeoffError("x"), ex.GotoError("x"), ex.MissionError("x"), ex.LandingError("x")],
)
async def test_flight_maneuver_failures_request_rtl(exception):
    drone = FakeDrone()
    emergency = Emergency(drone=drone, telemetry=FakeTelemetry())

    await emergency.handle_exception(exception)

    assert drone.rtl_calls == 1


async def test_unexpected_exception_defaults_to_rtl():
    drone = FakeDrone()
    emergency = Emergency(drone=drone, telemetry=FakeTelemetry())

    await emergency.handle_exception(RuntimeError("something unforeseen"))

    assert drone.rtl_calls == 1


async def test_failed_emergency_command_raises_and_preserves_original_cause():
    original_error = ActionErrorLike("PX4 rejected RTL")
    drone = FakeDrone(rtl_error=original_error)
    emergency = Emergency(drone=drone, telemetry=FakeTelemetry())

    with pytest.raises(ex.EmergencyCommandError) as excinfo:
        await emergency.handle_exception(ex.TakeoffError("takeoff failed"))

    assert excinfo.value.__cause__ is original_error


class ActionErrorLike(Exception):
    pass
