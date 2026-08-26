import asyncio
import math
import time
from config import settings
from . import exceptions
from . import geo


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
      raise exceptions.LowBatteryError(
          f"Battery level is {battery:.1f}%",
          battery_percent=battery,
      )

  async def check_altitude(self):
    position = await self.telemetry.get_position()
    if position is None:
        raise exceptions.GPSNotReadyError(
            "Position telemetry unavailable."
        )

    altitude = position.relative_altitude_m

    if altitude > settings.MAX_FLIGHT_ALTITUDE_M:
      raise exceptions.MissionError(
          f"Maximum altitude exceeded: {altitude:.1f} m (limit:"
          f" {settings.MAX_FLIGHT_ALTITUDE_M:.1f} m)."
      )

    # The monitor only runs once airborne (started after takeoff, stopped
    # before RTL/landing -- see MissionManager), so a reading below the
    # configured floor here means an unexpected descent, not a normal
    # takeoff/landing transition.
    if altitude < settings.MIN_FLIGHT_ALTITUDE_M:
      raise exceptions.MissionError(
          f"Below minimum flight altitude: {altitude:.1f} m (limit:"
          f" {settings.MIN_FLIGHT_ALTITUDE_M:.1f} m)."
      )

  async def check_distance_from_home(self):
    """Verify the vehicle has not drifted beyond the configured geofence."""

    position = await self.telemetry.get_position()
    home = await self.telemetry.get_home()

    distance = geo.horizontal_distance_m(
        position.latitude_deg,
        position.longitude_deg,
        home.latitude_deg,
        home.longitude_deg,
    )

    if distance > settings.MAX_DISTANCE_FROM_HOME_M:
      raise exceptions.MissionError(
          f"Distance from home exceeded: {distance:.1f} m (limit:"
          f" {settings.MAX_DISTANCE_FROM_HOME_M:.1f} m)."
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
    # MAVSDK VelocityNed is NED: positive down_m_s is descending, negative
    # is ascending. Checked separately against PX4's own asymmetric
    # MPC_Z_VEL_MAX_UP / MPC_Z_VEL_MAX_DN -- see settings.py.
    is_descending = velocity.down_m_s > 0

    if horizontal_speed > settings.MAX_HORIZONTAL_SPEED_M_S:
      raise exceptions.MissionError(
          f"Horizontal speed limit exceeded: {horizontal_speed:.1f} m/s"
          f" (limit: {settings.MAX_HORIZONTAL_SPEED_M_S:.1f} m/s)."
      )

    if is_descending and velocity.down_m_s > settings.MAX_DESCENT_SPEED_M_S:
      raise exceptions.MissionError(
          f"Descent speed limit exceeded: {velocity.down_m_s:.1f} m/s"
          f" (limit: {settings.MAX_DESCENT_SPEED_M_S:.1f} m/s)."
      )

    if not is_descending and -velocity.down_m_s > settings.MAX_ASCENT_SPEED_M_S:
      raise exceptions.MissionError(
          f"Ascent speed limit exceeded: {-velocity.down_m_s:.1f} m/s"
          f" (limit: {settings.MAX_ASCENT_SPEED_M_S:.1f} m/s)."
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
    await self.check_distance_from_home()
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
