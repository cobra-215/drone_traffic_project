import time
from dataclasses import dataclass

import pytest

from config import settings
from flight import exceptions as ex
from flight.monitor import FlightMonitor


@dataclass
class FakePosition:
    relative_altitude_m: float
    latitude_deg: float = 47.0
    longitude_deg: float = 8.0


@dataclass
class FakeHome:
    latitude_deg: float = 47.0
    longitude_deg: float = 8.0


@dataclass
class FakeHealth:
    is_global_position_ok: bool = True
    is_local_position_ok: bool = True
    is_home_position_ok: bool = True


@dataclass
class FakeVelocity:
    north_m_s: float = 0.0
    east_m_s: float = 0.0
    down_m_s: float = 0.0


class FakeTelemetry:
    def __init__(
        self,
        battery=100.0,
        altitude=settings.TAKEOFF_ALTITUDE_M,
        home_offset_deg=0.0,
        health=None,
        velocity=None,
    ):
        self.battery = battery
        self.altitude = altitude
        self.home_offset_deg = home_offset_deg
        self.health = health or FakeHealth()
        self.velocity = velocity or FakeVelocity()

    async def get_battery(self):
        return self.battery

    async def get_position(self):
        return FakePosition(relative_altitude_m=self.altitude)

    async def get_home(self):
        return FakeHome(
            latitude_deg=47.0 + self.home_offset_deg, longitude_deg=8.0
        )

    async def get_health(self):
        return self.health

    async def get_velocity(self):
        return self.velocity


async def test_check_battery_raises_and_carries_value_below_threshold():
    telemetry = FakeTelemetry(battery=settings.BATTERY_RTL_THRESHOLD)
    monitor = FlightMonitor(telemetry)

    with pytest.raises(ex.LowBatteryError) as excinfo:
        await monitor.check_battery()

    assert excinfo.value.battery_percent == settings.BATTERY_RTL_THRESHOLD


async def test_check_battery_passes_above_threshold():
    telemetry = FakeTelemetry(battery=settings.BATTERY_RTL_THRESHOLD + 1)
    monitor = FlightMonitor(telemetry)
    await monitor.check_battery()  # must not raise


async def test_check_altitude_raises_above_maximum():
    telemetry = FakeTelemetry(altitude=settings.MAX_FLIGHT_ALTITUDE_M + 1)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.MissionError):
        await monitor.check_altitude()


async def test_check_altitude_raises_below_minimum():
    telemetry = FakeTelemetry(altitude=settings.MIN_FLIGHT_ALTITUDE_M - 1)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.MissionError):
        await monitor.check_altitude()


async def test_check_altitude_passes_within_bounds():
    telemetry = FakeTelemetry(altitude=settings.TAKEOFF_ALTITUDE_M)
    monitor = FlightMonitor(telemetry)
    await monitor.check_altitude()  # must not raise


async def test_check_distance_from_home_raises_beyond_limit():
    # A large offset in degrees is guaranteed to exceed
    # MAX_DISTANCE_FROM_HOME_M regardless of its exact configured value.
    telemetry = FakeTelemetry(home_offset_deg=-5.0)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.MissionError):
        await monitor.check_distance_from_home()


async def test_check_distance_from_home_passes_when_close():
    telemetry = FakeTelemetry(home_offset_deg=0.0)
    monitor = FlightMonitor(telemetry)
    await monitor.check_distance_from_home()  # must not raise


@pytest.mark.parametrize(
    "health",
    [
        FakeHealth(is_global_position_ok=False),
        FakeHealth(is_local_position_ok=False),
        FakeHealth(is_home_position_ok=False),
    ],
)
async def test_check_navigation_health_raises_on_unhealthy_estimate(health):
    telemetry = FakeTelemetry(health=health)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.GPSNotReadyError):
        await monitor.check_navigation_health()


async def test_check_velocity_raises_on_excess_horizontal_speed():
    velocity = FakeVelocity(north_m_s=settings.MAX_HORIZONTAL_SPEED_M_S + 5)
    telemetry = FakeTelemetry(velocity=velocity)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.MissionError):
        await monitor.check_velocity()


async def test_check_velocity_raises_on_excess_descent_speed():
    # Positive down_m_s (NED) is descending.
    velocity = FakeVelocity(down_m_s=settings.MAX_DESCENT_SPEED_M_S + 5)
    telemetry = FakeTelemetry(velocity=velocity)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.MissionError):
        await monitor.check_velocity()


async def test_check_velocity_raises_on_excess_ascent_speed():
    # Negative down_m_s (NED) is ascending.
    velocity = FakeVelocity(down_m_s=-(settings.MAX_ASCENT_SPEED_M_S + 5))
    telemetry = FakeTelemetry(velocity=velocity)
    monitor = FlightMonitor(telemetry)
    with pytest.raises(ex.MissionError):
        await monitor.check_velocity()


async def test_check_velocity_passes_at_descent_speed_within_ascent_limit():
    # A descent speed that would exceed the (higher) ascent limit but not
    # the (lower) descent limit must not be mistakenly checked against
    # the wrong direction's threshold.
    assert settings.MAX_ASCENT_SPEED_M_S > settings.MAX_DESCENT_SPEED_M_S
    velocity = FakeVelocity(down_m_s=settings.MAX_DESCENT_SPEED_M_S - 0.1)
    telemetry = FakeTelemetry(velocity=velocity)
    monitor = FlightMonitor(telemetry)
    await monitor.check_velocity()  # must not raise


async def test_check_velocity_passes_within_limits():
    telemetry = FakeTelemetry(velocity=FakeVelocity(north_m_s=1.0, down_m_s=0.5))
    monitor = FlightMonitor(telemetry)
    await monitor.check_velocity()  # must not raise


def test_check_mission_timeout_raises_after_elapsed():
    monitor = FlightMonitor(FakeTelemetry())
    monitor.mission_start_time = time.monotonic() - settings.MISSION_TIMEOUT - 1
    with pytest.raises(ex.MissionError):
        monitor.check_mission_timeout()


def test_check_mission_timeout_passes_before_started():
    monitor = FlightMonitor(FakeTelemetry())
    monitor.check_mission_timeout()  # mission_start_time is None; must not raise


def test_stop_is_idempotent():
    monitor = FlightMonitor(FakeTelemetry())
    monitor.start()
    monitor.stop()
    monitor.stop()  # must not raise or duplicate output
    assert monitor.monitoring is False


def test_start_twice_raises():
    monitor = FlightMonitor(FakeTelemetry())
    monitor.start()
    with pytest.raises(RuntimeError):
        monitor.start()
    monitor.stop()
