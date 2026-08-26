import asyncio

from config import settings
from flight.drone import Drone
from flight.emergency import Emergency
from flight.monitor import FlightMonitor
from flight.safety import SafetyManager
from flight.telemetry import Telemetry
from mission.mission import MissionManager


async def main():
    """Create the flight system and run one observation mission."""

    drone = Drone(
        takeoff_altitude=settings.MIN_FLIGHT_ALTITUDE
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

    mission = MissionManager(
        drone=drone,
        telemetry=telemetry,
        safety=safety,
        monitor=monitor,
        emergency=emergency,
        target_latitude=settings.OBSERVATION_LATITUDE,
        target_longitude=settings.OBSERVATION_LONGITUDE,
        target_altitude=settings.OBSERVATION_ALTITUDE,
        observation_duration=settings.OBSERVATION_DURATION,
    )

    try:
        await mission.run()

    except Exception as exception:
        # MissionManager already passed the failure to Emergency.
        # Do not call Emergency.handle_exception() again here.
        print(
            f"Mission ended unsuccessfully: "
            f"{type(exception).__name__}: {exception}"
        )
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        print("\nMission interrupted by operator.")