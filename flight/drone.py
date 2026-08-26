import asyncio
import contextlib

from mavsdk import System
from mavsdk.telemetry import LandedState

from config import settings
from . import exceptions as ex
from . import geo
from .telemetry import Telemetry


class Drone:

  def __init__(self, takeoff_altitude=None):
    self.system = System()
    self.takeoff_altitude = (
        takeoff_altitude
        if takeoff_altitude is not None
        else settings.TAKEOFF_ALTITUDE_M
    )
    # Internal bounded-read helper, distinct from any application-facing
    # Telemetry instance the caller may separately construct around this
    # same Drone. Used only for Drone's own command-completion polling
    # (takeoff/land), so every read Drone performs is bounded the same
    # way instead of duplicating ad hoc stream handling.
    self._telemetry = Telemetry(self)

  async def connect(self, timeout=None):
    """Connect to PX4 with a strict timeout covering the whole operation."""
    timeout = timeout if timeout is not None else settings.CONNECTION_TIMEOUT

    async def _connect_and_wait():
      # mavsdk.System.connect() itself (mavsdk_server startup + gRPC
      # channel setup) can hang indefinitely if PX4/mavsdk_server never
      # becomes reachable -- observed live when PX4 died mid-session and
      # a subsequent connect() attempt blocked past its intended timeout
      # because only the "wait for is_connected" half used to be bounded.
      # Wrapping this whole coroutine in one asyncio.wait_for() below
      # bounds the entire operation, not just the second half of it.
      await self.system.connect(system_address=settings.PX4_CONNECTION_ADDRESS)

      connected = False

      async for state in self.system.core.connection_state():
        if state.is_connected:
          connected = True
          break

      if not connected:
        # The connection_state stream ended without ever reporting a
        # connection. Treat this as failure rather than reporting a
        # false success.
        raise ex.PX4ConnectionError(
            "Connection state stream ended without reporting a connection."
        )

    try:
      print(f"Connecting to PX4 at {settings.PX4_CONNECTION_ADDRESS}...")
      await asyncio.wait_for(_connect_and_wait(), timeout=timeout)
      print("PX4 connected!")
    except ex.PX4ConnectionError:
      raise
    except Exception as e:
      raise ex.PX4ConnectionError(
          f"Connection failed or timed out: {e}"
      ) from e

  async def arm(self):
    try:
      print("Arming...")
      await self.system.action.arm()
      print("Arm command sent.")
    except Exception as e:
      raise ex.ArmingError(f"Arming failed: {e}") from e

  async def takeoff(self, timeout=None):
    """Command takeoff and wait for the target altitude, with a hard bound."""
    timeout = timeout if timeout is not None else settings.TAKEOFF_TIMEOUT_S
    print(f"Taking off to {self.takeoff_altitude:.1f} m...")
    try:
      await self.system.action.set_takeoff_altitude(self.takeoff_altitude)
      await self.system.action.takeoff()

      deadline = asyncio.get_running_loop().time() + timeout

      while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
          raise ex.TakeoffError("Takeoff timed out.")

        # Each read is individually bounded by Telemetry's own timeout
        # (raises TelemetryTimeoutError on a stall), so this loop can
        # never block past `timeout` + one telemetry read even if
        # position telemetry stops entirely -- unlike the previous bare
        # `async for`, which had no per-sample bound at all.
        position = await self._telemetry.get_position()
        current_altitude = position.relative_altitude_m
        print(f"Current altitude: {current_altitude:.1f} m")

        if current_altitude >= self.takeoff_altitude * 0.90:
          print(f"Takeoff complete at {current_altitude:.1f} m.")
          return

        await asyncio.sleep(0.5)

    except ex.TakeoffError:
      raise
    except Exception as e:
      raise ex.TakeoffError(f"Takeoff failed: {e}") from e

  async def goto(
      self,
      latitude,
      longitude,
      altitude,
      yaw=0.0,
      acceptance_radius_m=None,
      altitude_tolerance_m=None,
      timeout=None,
  ):
    if not -90.0 <= latitude <= 90.0:
      raise ValueError("Latitude must be between -90 and 90.")
    if not -180.0 <= longitude <= 180.0:
      raise ValueError("Longitude must be between -180 and 180.")

    acceptance_radius_m = (
        acceptance_radius_m
        if acceptance_radius_m is not None
        else settings.WAYPOINT_ACCEPTANCE_RADIUS_M
    )
    altitude_tolerance_m = (
        altitude_tolerance_m
        if altitude_tolerance_m is not None
        else settings.WAYPOINT_ALTITUDE_TOLERANCE_M
    )
    timeout = timeout if timeout is not None else settings.WAYPOINT_TIMEOUT

    position_stream = self.system.telemetry.position()
    try:
      current_position = await asyncio.wait_for(
          anext(position_stream), timeout=settings.TELEMETRY_TIMEOUT
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
            anext(position_stream),
            timeout=min(remaining, settings.TELEMETRY_TIMEOUT),
        )
        horizontal_distance = geo.horizontal_distance_m(
            position.latitude_deg, position.longitude_deg, latitude, longitude
        )
        altitude_difference = abs(position.relative_altitude_m - altitude)

        print(
            f"Distance: {horizontal_distance:.1f}m | Alt error:"
            f" {altitude_difference:.1f}m"
        )
        if (
            horizontal_distance <= acceptance_radius_m
            and altitude_difference <= altitude_tolerance_m
        ):
          print("Waypoint reached.")
          return
    except ex.GotoError:
      raise
    except Exception as error:
      raise ex.GotoError(f"Goto failed: {error}") from error
    finally:
      # This finally can run while the surrounding task is itself being
      # cancelled (goto is cancelled by MissionManager when the monitor
      # raises). Shield the close so it always completes, and suppress
      # only the CancelledError the shielded await raises in *this* task
      # -- the original cancellation that brought us here is preserved
      # separately by Python's exception-propagation rules and continues
      # to propagate once this finally block finishes.
      with contextlib.suppress(asyncio.CancelledError):
        await asyncio.shield(position_stream.aclose())

  async def land(self, timeout=None):
    """Land and confirm ground contact + disarm without long-lived streams."""
    timeout = timeout if timeout is not None else settings.LANDING_TIMEOUT_S
    print("Landing...")
    try:
      await self.system.action.land()
      print("Landing command sent.")

      deadline = asyncio.get_running_loop().time() + timeout

      while asyncio.get_running_loop().time() < deadline:
        # Fresh one-sample reads each iteration (via Telemetry), rather
        # than pulling alternately from two long-lived streams -- a
        # long-lived `armed()` stream only re-emits on change, so a
        # successful landing could otherwise time out waiting for an
        # armed-state sample that never arrives.
        position = await self._telemetry.get_position()
        armed = await self._telemetry.get_armed()
        landed_state = await self._telemetry.get_landed_state()

        print(
            f"Landing progress: Alt={position.relative_altitude_m:.2f}m,"
            f" Armed={armed}, LandedState={landed_state}"
        )

        on_ground = landed_state == LandedState.ON_GROUND
        if position.relative_altitude_m <= 0.30 and not armed and on_ground:
          print("Landing complete and disarm confirmed.")
          return

        await asyncio.sleep(0.5)

      raise ex.LandingError("Landing timed out.")

    except ex.LandingError:
      raise
    except Exception as e:
      raise ex.LandingError(f"Landing failed: {e}") from e

  async def return_to_launch(self):
    print("Returning to launch...")
    try:
      await self.system.action.return_to_launch()
    except Exception as e:
      raise ex.MissionError(f"Return to launch failed: {e}") from e
    print("RTL command sent.")

  async def hold(self):
    try:
      print("Hold mode initializing...")
      await self.system.action.hold()
      print("Drone is holding position.")
    except Exception as e:
      raise ex.MissionError(f"Failed to hold drone: {e}") from e

  def disconnect(self):
    """
    Explicitly stop this Drone's mavsdk_server subprocess.

    MAVSDK's System registers both a __del__ hook and an atexit handler
    to kill its mavsdk_server subprocess, but neither is guaranteed to
    run promptly: __del__ depends on garbage-collection timing, and this
    project's own SITL testing showed the atexit handler does not
    reliably fire either (repeated Drone construction across separate
    process runs left mavsdk_server subprocesses running and caused a
    "bind error: Address in use" on the next connect(), since every
    Drone defaults to the same gRPC/UDP ports).

    Call this once a Drone is done being used -- callers should wrap
    their Drone usage in try/finally to guarantee it runs even on
    failure. Best-effort and never raises: cleanup failing must not be
    allowed to affect flight-critical code or mask the exception that
    triggered cleanup in the first place.
    """

    stop = getattr(self.system, "_stop_mavsdk_server", None)
    if stop is None:
      # MAVSDK's internal API changed; nothing we can do here without
      # depending further on undocumented internals.
      print(
          "Drone.disconnect: mavsdk.System has no _stop_mavsdk_server; "
          "skipping explicit cleanup."
      )
      return

    try:
      stop()
    except Exception as e:
      print(f"Drone.disconnect: could not stop mavsdk_server cleanly: {e}")
