from flight.telemetry import Telemetry
from . import exceptions


class SafetyManager:

    def __init__(self, drone, telemetry: Telemetry):
        """Receive the telemetry manager."""
        self.drone = drone
        self.telemetry = telemetry
        self.minimum_battery = 30.0

    async def check_battery(self) -> float:
        """Verify that the battery level is above the minimum."""

        battery = await self.telemetry.get_battery()

        if battery < self.minimum_battery:
            raise exceptions.LowBatteryError(
                f"Battery is too low "
                f"({battery:.1f}%)"
            )

        print(
            f"Battery is OK "
            f"Battery value: ({battery:.1f}%) "
        )

    async def check_gps(self):
        """Verify that GPS/global position is ready."""

        gps_ready = await self.telemetry.gps_ready()

        if not gps_ready:
            raise exceptions.GPSNotReadyError(
                "GPS is not ready yet"
            )

        print("GPS is OK")

    async def check_home_position(self):
        """Verify that the home position has been established."""

        home_ready = (
            await self.telemetry.is_home_position_ready()
        )

        if not home_ready:
            raise exceptions.HomePositionNotReadyError(
                "Home position is not ready yet"
            )

        print("Home position is OK")

    async def check_flight_mode(self):
        """Get and display the current flight mode."""

        mode = await self.telemetry.get_flight_mode()

        print(f"Current flight mode: {mode}")

        return mode


    async def preflight_check(self):
        """Run all safety checks before flight."""

        print("Starting preflight safety check...")

        await self.check_gps()
        await self.check_home_position()
        await self.check_battery()

        print("All preflight checks passed!")

        return True