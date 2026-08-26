import asyncio
import math
import time
from config import settings
from . import exceptions


class FlightMonitor:

  def __init__(self, telemetry):
    self.telemetry = telemetry
    self.monitoring = False
    self.mission_start_time = None

  def start(self):
    if self.monitoring:
      raise RuntimeError("Flight monitor is already running.")
    self.monitoring = True
    self.mission_start_time = time.monotonic()
    print("Flight monitor started.")

  def stop(self):
    if not self.monitoring:
      return
    self.monitoring = False
    print("Flight monitor stopped.")

  async def check_battery(self):
    battery = await self.telemetry.get_battery()
    if battery <= settings.BATTERY_RTL_THRESHOLD:
      raise exceptions.LowBatteryError(f"Battery level is {battery:.1f}%")

  async def check_altitude(self):
    position = await self.telemetry.get_position()
    if position is None:
        raise exceptions.GPSNotReadyError(
            "Position telemetry unavailable."
        )

    altitude = position.relative_altitude_m
    if altitude > settings.MAX_FLIGHT_ALTITUDE:
      raise exceptions.MissionError(
          f"Maximum altitude exceeded: {altitude:.1f} m (limit:"
          f" {settings.MAX_FLIGHT_ALTITUDE:.1f} m)."
      )

  async def check_navigation_health(self):
      """Verify that navigation and home-position estimates remain healthy."""

      health = await self.telemetry.get_health()

      if not health.is_global_position_ok:
          raise exceptions.GPSNotReadyError(
              "Global position estimate is unhealthy."
          )

      if not health.is_local_position_ok:
          raise exceptions.GPSNotReadyError(
              "Local position estimate is unhealthy."
          )

      if not health.is_home_position_ok:
          raise exceptions.GPSNotReadyError(
              "Home position is no longer healthy."
          )

  async def check_velocity(self):
    velocity = await self.telemetry.get_velocity()
    if velocity is None:
      raise exceptions.MissionError("Velocity telemetry unavailable.")

    horizontal_speed = math.sqrt(
        velocity.north_m_s**2 + velocity.east_m_s**2
    )
    vertical_speed = abs(velocity.down_m_s)

    if horizontal_speed > settings.MAX_HORIZONTAL_SPEED:
      raise exceptions.MissionError(
          f"Horizontal speed limit exceeded: {horizontal_speed:.1f} m/s"
          f" (limit: {settings.MAX_HORIZONTAL_SPEED:.1f} m/s)."
      )

    if vertical_speed > settings.MAX_VERTICAL_SPEED:
      raise exceptions.MissionError(
          f"Vertical speed limit exceeded: {vertical_speed:.1f} m/s (limit:"
          f" {settings.MAX_VERTICAL_SPEED:.1f} m/s)."
      )

  def check_mission_timeout(self):
    if self.mission_start_time is None:
      return
    elapsed = time.monotonic() - self.mission_start_time
    if elapsed > settings.MISSION_TIMEOUT:
      raise exceptions.MissionError("Mission timeout reached.")

  async def check_all(self):
    await self.check_battery()
    await self.check_altitude()
    await self.check_navigation_health()
    await self.check_velocity()
    self.check_mission_timeout()

  async def monitor(self):
    self.start()
    try:
      while self.monitoring:
        await self.check_all()
        await asyncio.sleep(settings.MONITOR_INTERVAL)
    finally:
      self.stop()