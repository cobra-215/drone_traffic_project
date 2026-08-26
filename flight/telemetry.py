import asyncio

from config import settings
from . import exceptions as ex


class Telemetry:
    """Provide bounded, one-sample access to PX4 MAVSDK telemetry."""

    def __init__(self, drone):
        self.drone = drone

    async def _get_single_sample(self, stream_func, stream_name=None):
        """
        Read one sample from a MAVSDK telemetry stream.

        The stream is always closed, and waiting is limited by the configured
        telemetry timeout. A stall is raised as TelemetryTimeoutError rather
        than a raw asyncio.TimeoutError, so callers (in particular Emergency)
        can recognise a stalled link and abort without commanding flight over
        a link that has just demonstrated it is not working.
        """

        stream = stream_func()
        name = stream_name or getattr(stream_func, "__name__", "unknown")

        try:
            return await asyncio.wait_for(
                anext(stream),
                timeout=settings.TELEMETRY_TIMEOUT,
            )
        except asyncio.TimeoutError as e:
            raise ex.TelemetryTimeoutError(
                f"Telemetry stream '{name}' produced no sample within "
                f"{settings.TELEMETRY_TIMEOUT:.1f}s.",
                stream_name=name,
            ) from e
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

    async def get_home(self):
        """
        Return one current MAVSDK Position sample for the home position.

        Used to validate/monitor distance from home; distinct from
        get_position(), which returns the vehicle's current position.
        """

        return await self._get_single_sample(
            self.drone.system.telemetry.home
        )

    async def get_landed_state(self):
        """
        Return the current MAVSDK LandedState (ON_GROUND / TAKING_OFF /
        IN_AIR / LANDING / UNKNOWN).

        This is PX4's own assessment of whether the vehicle is on the
        ground, and is a stronger landing-confirmation signal than a
        relative-altitude threshold alone.
        """

        return await self._get_single_sample(
            self.drone.system.telemetry.landed_state
        )