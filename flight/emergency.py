import asyncio

from config import settings
from . import exceptions


# Exceptions where a flight command cannot safely be trusted to reach the
# vehicle, or where PX4's own configured failsafe (not this application) is
# responsible for deciding the aircraft's response. In every one of these
# cases the application aborts its own mission bookkeeping WITHOUT issuing
# RTL, Hold, or any other command.
NO_FLIGHT_COMMAND_EXCEPTIONS = (
    exceptions.PX4ConnectionError,
    exceptions.LinkLostError,
    exceptions.TelemetryTimeoutError,
    exceptions.ArmingError,
    exceptions.GPSNotReadyError,
    exceptions.HomePositionNotReadyError,
)


class Emergency:

  def __init__(self, drone, telemetry):
    self.drone = drone
    self.telemetry = telemetry

  async def _run_emergency_command(self, coro, description):
    """
    Run a single emergency flight command with a bound timeout.

    A failed emergency command must never be swallowed, but it also must
    not silently replace the exception that triggered the emergency in
    the first place -- so it is re-raised as EmergencyCommandError
    chained `from` the original failure, preserving both on the
    exception's __cause__ chain.
    """

    try:
      await asyncio.wait_for(
          coro, timeout=settings.EMERGENCY_COMMAND_TIMEOUT
      )
    except Exception as e:
      raise exceptions.EmergencyCommandError(
          f"Emergency command '{description}' failed: {e}"
      ) from e

  async def rtl(self):
    print("Emergency: Activating return to launch")
    await self._run_emergency_command(
        self.drone.return_to_launch(), "return_to_launch"
    )
    print("Emergency: RTL command accepted by PX4.")

  async def emergency_landing(self):
    print("Emergency: Initiating immediate landing")
    await self._run_emergency_command(self.drone.land(), "land")

  async def hold_position(self):
    print("Emergency: Holding position")
    await self._run_emergency_command(self.drone.hold(), "hold")

  async def abort_mission(self, reason, request_rtl=True):
    """Mark mission as aborted and optionally command RTL."""
    print(f"Emergency: Mission aborted. Reason: {reason}")
    if request_rtl:
      await self.rtl()

  async def handle_exception(self, exception):
      """Apply the application's emergency-response policy."""

      print(
          f"Emergency triggered: "
          f"{type(exception).__name__}: {exception}"
      )

      # Python cannot trust a flight command to reach the vehicle after a
      # connection failure, a lost link, a telemetry stall, an arming
      # failure, or an unhealthy GPS/home-position estimate. In every one
      # of these cases PX4's own configured failsafe (data-link-loss,
      # RC-loss, position-loss) is responsible for the aircraft's
      # response, or the vehicle is not yet flying at all (arming
      # failure, home-position-not-ready during preflight).
      if isinstance(exception, NO_FLIGHT_COMMAND_EXCEPTIONS):
          print(
              f"Emergency: {type(exception).__name__} detected. "
              "Aborting application mission without a flight command; "
              "PX4's own failsafe (if airborne) or the ground state (if "
              "not yet armed) governs the aircraft."
          )

          await self.abort_mission(
              reason=str(exception),
              request_rtl=False,
          )

      elif isinstance(exception, exceptions.LowBatteryError):
          battery = exception.battery_percent

          if battery is None:
              # The exception did not carry a measured value (e.g. raised
              # from somewhere other than FlightMonitor.check_battery) --
              # fall back to a guarded re-read rather than assuming the
              # worst or the best.
              try:
                  battery = await asyncio.wait_for(
                      self.telemetry.get_battery(),
                      timeout=settings.EMERGENCY_COMMAND_TIMEOUT,
                  )
              except Exception as read_error:
                  print(
                      "Emergency: could not re-read battery level "
                      f"({read_error}). Treating as critical."
                  )
                  await self.emergency_landing()
                  return

          if battery <= settings.BATTERY_CRITICAL_THRESHOLD:
              print(
                  f"Emergency: critically low battery "
                  f"({battery:.1f}%). Landing now."
              )

              await self.emergency_landing()

          else:
              await self.abort_mission(
                  reason=f"Low battery ({battery:.1f}%)",
                  request_rtl=True,
              )

      elif isinstance(
              exception,
              (
                      exceptions.TakeoffError,
                      exceptions.GotoError,
                      exceptions.MissionError,
                      exceptions.LandingError,
              ),
      ):
          print(
              "Emergency: flight maneuver or mission failure. "
              "Aborting mission and initiating RTL."
          )

          await self.abort_mission(
              reason=str(exception),
              request_rtl=True,
          )

      else:
          print(
              f"Emergency: unexpected "
              f"{type(exception).__name__}. "
              "Aborting mission and initiating RTL."
          )

          await self.abort_mission(
              reason=str(exception),
              request_rtl=True,
          )
