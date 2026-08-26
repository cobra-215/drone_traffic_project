from config import settings
from . import exceptions


class Emergency:

  def __init__(self, drone, telemetry):
    self.drone = drone
    self.telemetry = telemetry

  async def rtl(self):
    print("Emergency: Activating return to launch")
    await self.drone.return_to_launch()
    print("Emergency: RTL command accepted by PX4.")

  async def emergency_landing(self):
    print("Emergency: Initiating immediate landing")
    await self.drone.land()

  async def hold_position(self):
    print("Emergency: Holding position")
    await self.drone.hold()

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

      # Python cannot command PX4 after a connection failure.
      # PX4 must handle any airborne link loss using its configured failsafe.
      if isinstance(exception, exceptions.ConnectionError):
          print(
              "Emergency: MAVLink connection failed or was lost. "
              "Aborting application mission without a flight command."
          )

          await self.abort_mission(
              reason=str(exception),
              request_rtl=False,
          )

      # Arming failure occurs before normal flight begins.
      elif isinstance(exception, exceptions.ArmingError):
          print(
              "Emergency: pre-arm or arming failure detected. "
              "Aborting mission without requesting flight maneuvers."
          )

          await self.abort_mission(
              reason=str(exception),
              request_rtl=False,
          )

      elif isinstance(exception, exceptions.LowBatteryError):
          battery = await self.telemetry.get_battery()

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

      # Do not command Hold or RTL when navigation/home health is invalid.
      # PX4's configured position-loss failsafe decides the aircraft response.
      elif isinstance(exception, exceptions.GPSNotReadyError):
          print(
              "Emergency: navigation or home position is unhealthy. "
              "Aborting application mission without a flight command."
          )

          await self.abort_mission(
              reason=str(exception),
              request_rtl=False,
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