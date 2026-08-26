from config import settings
from .telemetry import Telemetry
from .px4_params import PX4ParameterAudit
from . import exceptions


class SafetyManager:

    def __init__(self, drone, telemetry: Telemetry):
        """Receive the drone (for the read-only PX4 parameter audit) and
        the telemetry manager."""
        self.drone = drone
        self.telemetry = telemetry
        self.param_audit = PX4ParameterAudit(drone)

    async def check_battery(self) -> float:
        """Verify that the battery level is above the preflight minimum."""

        battery = await self.telemetry.get_battery()

        if battery < settings.BATTERY_PREFLIGHT_MIN_PERCENT:
            raise exceptions.LowBatteryError(
                f"Battery is too low for preflight "
                f"({battery:.1f}%, minimum "
                f"{settings.BATTERY_PREFLIGHT_MIN_PERCENT:.1f}%)",
                battery_percent=battery,
            )

        print(
            f"Battery is OK "
            f"Battery value: ({battery:.1f}%) "
        )

        return battery

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

        if not settings.REQUIRE_PREFLIGHT_CHECKS:
            print(
                "WARNING: REQUIRE_PREFLIGHT_CHECKS is False -- skipping "
                "all preflight safety checks. This should never be set "
                "for a real flight."
            )
            return True

        print("Starting preflight safety check...")

        await self.check_gps()

        if settings.REQUIRE_HOME_POSITION:
            await self.check_home_position()
        else:
            print(
                "WARNING: REQUIRE_HOME_POSITION is False -- skipping the "
                "home-position check."
            )

        await self.check_battery()

        # Read-only; logs PX4's own failsafe/limit configuration and warns
        # (never fails) if an application limit is inconsistent with it.
        # Must never be able to block arming -- see PX4ParameterAudit.run().
        await self.param_audit.run()

        print("All preflight checks passed!")

        return True
