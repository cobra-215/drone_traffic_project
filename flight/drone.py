import asyncio
import math
from mavsdk import System
from mavsdk.action import ActionError
from config.settings import PX4_CONNECTION_ADDRESS
from . import exceptions as ex


class Drone:

  def __init__(self, takeoff_altitude=10.0):
    self.system = System()
    self.takeoff_altitude = takeoff_altitude

  @staticmethod
  def _horizontal_distance_m(
      latitude_a, longitude_a, latitude_b, longitude_b
  ):
    earth_radius_m = 6_371_000.0
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    latitude_difference = math.radians(latitude_b - latitude_a)
    longitude_difference = math.radians(longitude_b - longitude_a)

    haversine = (
        math.sin(latitude_difference / 2) ** 2
        + math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(longitude_difference / 2) ** 2
    )

    return 2 * earth_radius_m * math.asin(math.sqrt(haversine))

  async def connect(self, timeout=15.0):
    """Connect to PX4 with a strict timeout."""
    try:
      print(f"Connecting to PX4 at {PX4_CONNECTION_ADDRESS}...")
      await self.system.connect(system_address=PX4_CONNECTION_ADDRESS)

      async def _wait_connection():
        async for state in self.system.core.connection_state():
          if state.is_connected:
            return

      await asyncio.wait_for(_wait_connection(), timeout=timeout)
      print("PX4 connected!")
    except Exception as e:
      raise ex.ConnectionError(
          f"Connection failed or timed out: {e}"
      ) from e

  async def wait_until_ready(self):
    """Wait until all sensors pass health checks."""
    print("Waiting for vehicle to become ready...")
    stream = self.system.telemetry.health()
    try:
      async for health in stream:
        print(
            f"GPS: {health.is_global_position_ok} | Home:"
            f" {health.is_home_position_ok} | Gyro:"
            f" {health.is_gyrometer_calibration_ok}"
        )
        if (
            health.is_global_position_ok
            and health.is_home_position_ok
            and health.is_gyrometer_calibration_ok
            and health.is_accelerometer_calibration_ok
            and health.is_magnetometer_calibration_ok
        ):
          print("Vehicle is ready!")
          return
    finally:
      await stream.aclose()

  async def arm(self):
    try:
      print("Arming...")
      await self.system.action.arm()
      print("Arm command sent.")
    except Exception as e:
      raise ex.ArmingError(f"Arming failed: {e}") from e

  async def takeoff(self, timeout=30):
    print(f"Taking off to {self.takeoff_altitude:.1f} m...")
    try:
      await self.system.action.set_takeoff_altitude(self.takeoff_altitude)
      await self.system.action.takeoff()

      start_time = asyncio.get_running_loop().time()
      position_stream = self.system.telemetry.position()

      try:
        async for position in position_stream:
          current_altitude = position.relative_altitude_m
          print(f"Current altitude: {current_altitude:.1f} m")

          if current_altitude >= self.takeoff_altitude * 0.90:
            print(f"Takeoff complete at {current_altitude:.1f} m.")
            return

          if asyncio.get_running_loop().time() - start_time > timeout:
            raise ex.TakeoffError("Takeoff timed out.")
      finally:
        await position_stream.aclose()

    except ex.TakeoffError:
      raise
    except Exception as e:
      raise ex.TakeoffError(f"Takeoff failed: {e}") from e

  async def goto(
      self, latitude, longitude, altitude, yaw=0.0, timeout=60
  ):
    if not -90.0 <= latitude <= 90.0:
      raise ValueError("Latitude must be between -90 and 90.")
    if not -180.0 <= longitude <= 180.0:
      raise ValueError("Longitude must be between -180 and 180.")

    position_stream = self.system.telemetry.position()
    try:
      current_position = await asyncio.wait_for(
          anext(position_stream), timeout=5.0
      )
      home_absolute_altitude = (
          current_position.absolute_altitude_m
          - current_position.relative_altitude_m
      )
      target_absolute_altitude = home_absolute_altitude + altitude

      print(
          f"Going to: Lat {latitude:.7f}, Lon {longitude:.7f}, Alt"
          f" {altitude:.1f}m"
      )
      await self.system.action.goto_location(
          latitude, longitude, target_absolute_altitude, yaw
      )

      deadline = asyncio.get_running_loop().time() + timeout
      while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
          raise ex.GotoError("Goto operation timed out.")

        position = await asyncio.wait_for(
            anext(position_stream), timeout=min(remaining, 5.0)
        )
        horizontal_distance = self._horizontal_distance_m(
            position.latitude_deg, position.longitude_deg, latitude, longitude
        )
        altitude_difference = abs(position.relative_altitude_m - altitude)

        print(
            f"Distance: {horizontal_distance:.1f}m | Alt error:"
            f" {altitude_difference:.1f}m"
        )
        if horizontal_distance <= 2.0 and altitude_difference <= 1.0:
          print("Waypoint reached.")
          return
    except ex.GotoError:
      raise
    except Exception as error:
      raise ex.GotoError(f"Goto failed: {error}") from error
    finally:
      await position_stream.aclose()

  async def land(self, timeout=60):
    """Cleanly land and verify disarm state without nested stream deadlocks."""
    print("Landing...")
    try:
      await self.system.action.land()
      print("Landing command sent.")

      deadline = asyncio.get_running_loop().time() + timeout
      pos_stream = self.system.telemetry.position()
      arm_stream = self.system.telemetry.armed()

      try:
        while asyncio.get_running_loop().time() < deadline:
          pos = await asyncio.wait_for(anext(pos_stream), timeout=5.0)
          armed = await asyncio.wait_for(anext(arm_stream), timeout=5.0)

          print(
              f"Landing progress: Alt={pos.relative_altitude_m:.2f}m,"
              f" Armed={armed}"
          )

          if pos.relative_altitude_m <= 0.30 and not armed:
            print("Landing complete and disarm confirmed.")
            return

          await asyncio.sleep(0.5)

        raise ex.LandingError("Landing timed out.")
      finally:
        await pos_stream.aclose()
        await arm_stream.aclose()

    except ex.LandingError:
      raise
    except Exception as e:
      raise ex.LandingError(f"Landing failed: {e}") from e

  async def return_to_launch(self):
    print("Returning to launch...")
    await self.system.action.return_to_launch()
    print("RTL command sent.")

  async def hold(self):
    try:
      print("Hold mode initializing...")
      await self.system.action.hold()
      print("Drone is holding position.")
    except ActionError as e:
      raise ex.MissionError(f"Failed to hold drone: {e}")