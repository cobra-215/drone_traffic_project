import asyncio
import contextlib

from flight import exceptions


class MissionManager:
    """
    Coordinate one complete observation mission.

    FlightMonitor detects safety violations.
    Emergency handles abnormal mission outcomes.
    MissionManager owns the normal mission sequence and normal RTL.
    """

    def __init__(
        self,
        drone,
        telemetry,
        safety,
        monitor,
        emergency,
        target_latitude,
        target_longitude,
        target_altitude,
        observation_duration,
    ):
        self.drone = drone
        self.telemetry = telemetry
        self.safety = safety
        self.monitor = monitor
        self.emergency = emergency

        self.target_latitude = target_latitude
        self.target_longitude = target_longitude
        self.target_altitude = target_altitude
        self.observation_duration = observation_duration

    async def _run_while_monitored(
        self,
        operation,
        monitor_task,
    ):
        """
        Run one mission operation while FlightMonitor runs concurrently.

        If the monitor detects a safety issue, cancel the active operation and
        propagate the monitor exception to the emergency layer.
        """

        operation_task = asyncio.create_task(operation)

        done, _ = await asyncio.wait(
            {operation_task, monitor_task},
            return_when=asyncio.FIRST_COMPLETED,
        )

        if monitor_task in done:
            monitor_exception = monitor_task.exception()

            if not operation_task.done():
                operation_task.cancel()

                with contextlib.suppress(
                    asyncio.CancelledError
                ):
                    await operation_task

            if monitor_exception is not None:
                raise monitor_exception

            raise RuntimeError(
                "Flight monitor stopped unexpectedly during "
                "an active mission phase."
            )

        return await operation_task

    async def _stop_monitor(self, monitor_task):
        """Stop a healthy monitor cleanly after normal mission completion."""

        if monitor_task.done():
            monitor_exception = monitor_task.exception()

            if monitor_exception is not None:
                raise monitor_exception

            raise RuntimeError(
                "Flight monitor stopped unexpectedly before "
                "mission completion."
            )

        self.monitor.stop()
        await monitor_task

    async def _wait_for_landing(self, timeout=120):
        """Wait for PX4 RTL to land and disarm the vehicle."""

        deadline = asyncio.get_running_loop().time() + timeout

        while asyncio.get_running_loop().time() < deadline:
            position = await self.telemetry.get_position()
            armed = await self.telemetry.get_armed()

            print(
                "RTL landing check: "
                f"altitude={position.relative_altitude_m:.2f} m, "
                f"armed={armed}"
            )

            if position.relative_altitude_m <= 0.30 and not armed:
                print("Landing and disarm confirmed.")
                return

            await asyncio.sleep(2.0)

        raise exceptions.LandingError(
            "RTL landing confirmation timed out."
        )

    async def run(self):
        """Run one complete observation mission."""

        monitor_task = None

        try:
            print("\n[1/6] Connecting to PX4...")
            await self.drone.connect()

            print("\n[2/6] Running preflight safety checks...")
            await self.safety.preflight_check()

            print("\n[3/6] Arming and taking off...")
            await self.drone.arm()
            await self.drone.takeoff()

            print("\n[4/6] Starting continuous flight monitor...")
            monitor_task = asyncio.create_task(
                self.monitor.monitor()
            )

            print("\n[5/6] Flying to observation waypoint...")
            await self._run_while_monitored(
                self.drone.goto(
                    latitude=self.target_latitude,
                    longitude=self.target_longitude,
                    altitude=self.target_altitude,
                ),
                monitor_task,
            )

            print(
                "Observation phase started for "
                f"{self.observation_duration} seconds..."
            )

            await self._run_while_monitored(
                asyncio.sleep(self.observation_duration),
                monitor_task,
            )

            await self._stop_monitor(monitor_task)
            monitor_task = None

            print(
                "\n[6/6] Mission completed normally. "
                "Returning to launch..."
            )

            await self.drone.return_to_launch()
            await self._wait_for_landing()

            print("Mission complete. Aircraft safely on ground.")

        except Exception as exception:
            if monitor_task is not None:
                self.monitor.stop()

                if not monitor_task.done():
                    with contextlib.suppress(Exception):
                        await monitor_task

            await self.emergency.handle_exception(exception)
            raise