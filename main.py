import asyncio

from config import settings
from camera.factory import build_recorder
from flight.drone import Drone
from flight.emergency import Emergency
from flight.monitor import FlightMonitor
from flight.safety import SafetyManager
from flight.telemetry import Telemetry
from mission.mission import Mission
from mission.mission_manager import MissionManager
from mission.position import Position
from mission.waypoint import Waypoint


def build_mission():
    """
    Build the single observation mission this application currently
    flies: transit to the configured observation site, hold there for
    OBSERVATION_DURATION while recording, then return.

    Mission.home is intentionally left unset here -- MissionManager fills
    it in from PX4's actual reported home position once connected, since
    it is not known before that.
    """

    observation_waypoint = Waypoint(
        position=Position(
            latitude_deg=settings.OBSERVATION_LATITUDE,
            longitude_deg=settings.OBSERVATION_LONGITUDE,
            altitude_m=settings.OBSERVATION_ALTITUDE_M,
        ),
        name="observation",
        hold_time_s=settings.OBSERVATION_DURATION,
    )

    return Mission(waypoints=[observation_waypoint])


async def main():
    """Create the flight system and run one observation mission."""

    settings.validate_settings()

    drone = Drone(
        takeoff_altitude=settings.TAKEOFF_ALTITUDE_M
    )

    telemetry = Telemetry(drone)

    safety = SafetyManager(
        drone=drone,
        telemetry=telemetry,
    )

    monitor = FlightMonitor(
        telemetry=telemetry,
    )

    emergency = Emergency(
        drone=drone,
        telemetry=telemetry,
    )

    camera = build_recorder()
    mission = build_mission()

    mission_manager = MissionManager(
        drone=drone,
        telemetry=telemetry,
        safety=safety,
        monitor=monitor,
        emergency=emergency,
        camera=camera,
        mission=mission,
    )

    try:
        try:
            await mission_manager.run()

        except asyncio.CancelledError:
            # mission_manager.run() already logged and handled this;
            # nothing further to do here besides letting cancellation
            # continue to propagate cleanly out of main().
            raise

        except Exception as exception:
            # MissionManager already passed the failure to Emergency.
            # Do not call Emergency.handle_exception() again here.
            print(
                f"Mission ended unsuccessfully: "
                f"{type(exception).__name__}: {exception}"
            )
            raise
    finally:
        # Explicit, best-effort cleanup of the mavsdk_server subprocess --
        # see Drone.disconnect() for why this cannot be left to MAVSDK's
        # own __del__/atexit handling.
        drone.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        # On Python 3.11+, asyncio.run() converts a first Ctrl+C into
        # cancellation of the running task, which mission_manager.run()
        # already handles explicitly (stopping the monitor/camera and
        # issuing NO flight command). A raw KeyboardInterrupt only reaches
        # here on a second/forceful interrupt, once cleanup has already
        # been attempted.
        print(
            "\nMission interrupted by operator. The aircraft may still "
            "be airborne -- PX4, the RC pilot, or QGroundControl are "
            "responsible for it from this point."
        )
