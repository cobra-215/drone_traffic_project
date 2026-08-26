import asyncio

from config import settings


class Telemetry:
    """Provide bounded, one-sample access to PX4 MAVSDK telemetry."""

    def __init__(self, drone):
        self.drone = drone

    async def _get_single_sample(self, stream_func):
        """
        Read one sample from a MAVSDK telemetry stream.

        The stream is always closed, and waiting is limited by the configured
        telemetry timeout.
        """

        stream = stream_func()

        try:
            return await asyncio.wait_for(
                anext(stream),
                timeout=settings.TELEMETRY_TIMEOUT,
            )
        finally:
            await stream.aclose()

    async def get_position(self):
        """
        Return one current MAVSDK Position sample.

        Includes latitude, longitude, absolute altitude, and relative altitude.
        """

        return await self._get_single_sample(
            self.drone.system.telemetry.position
        )

    async def get_velocity(self):
        """Return one current MAVSDK VelocityNed sample."""

        return await self._get_single_sample(
            self.drone.system.telemetry.velocity_ned
        )

    async def get_battery(self):
        """Return remaining battery percentage as a float."""

        battery = await self._get_single_sample(
            self.drone.system.telemetry.battery
        )

        return battery.remaining_percent

    async def get_flight_mode(self):
        """Return the current PX4 flight mode."""

        return await self._get_single_sample(
            self.drone.system.telemetry.flight_mode
        )

    async def get_heading(self):
        """Return one current MAVSDK Heading sample."""

        return await self._get_single_sample(
            self.drone.system.telemetry.heading
        )

    async def get_health(self):
        """Return one current MAVSDK Health sample."""

        return await self._get_single_sample(
            self.drone.system.telemetry.health
        )

    async def gps_ready(self):
        """Return whether PX4 reports a healthy global-position estimate."""

        health = await self.get_health()

        return health.is_global_position_ok

    async def is_home_position_ready(self):
        """Return whether PX4 reports a healthy home position."""

        health = await self.get_health()

        return health.is_home_position_ok

    async def get_armed(self):
        """Return whether the vehicle is currently armed."""

        return await self._get_single_sample(
            self.drone.system.telemetry.armed
        )