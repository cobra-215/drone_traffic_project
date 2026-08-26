import asyncio
import contextlib

from mavsdk.telemetry import LandedState

from config import settings
from flight import exceptions
from .position import AltitudeReference, Position


class MissionManager:
    """
    Coordinate one complete observation mission.

    FlightMonitor detects safety violations.
    Emergency handles abnormal mission outcomes.
    MissionManager owns the normal mission sequence and normal RTL.

    The flight monitor deliberately does NOT run during takeoff or during
    RTL/landing:
      - Takeoff has its own bounded internal timeout (Drone.takeoff()),
        and FlightMonitor's minimum-altitude check assumes the vehicle is
        already at or above TAKEOFF_ALTITUDE_M when monitoring starts --
        starting the monitor during the climb would spuriously trip that
        check on every single flight.
      - RTL/landing is PX4's own responsibility once commanded; this is
        an intentional, accepted gap (see the project plan), not an
        oversight.
    """

    def __init__(
        self,
        drone,
        telemetry,
        safety,
        monitor,
        emergency,
        camera,
        mission,
    ):
        self.drone = drone
        self.telemetry = telemetry
        self.safety = safety
        self.monitor = monitor
        self.emergency = emergency
        self.camera = camera
        self.mission = mission

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

        try:
            done, _ = await asyncio.wait(
                {operation_task, monitor_task},
                return_when=asyncio.FIRST_COMPLETED,
            )
        except asyncio.CancelledError:
            # This task itself was cancelled (operator abort). asyncio.wait()
            # does not cancel the tasks it was waiting on, so operation_task
            # would otherwise be left running as an orphan -- e.g. a
            # Drone.goto() whose MAVSDK stream would never be closed. Cancel
            # and await it here before letting cancellation continue to
            # propagate.
            if not operation_task.done():
                operation_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await operation_task
            raise

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

    async def _wait_for_landing(self, timeout=None):
        """Wait for PX4 RTL to land and disarm the vehicle."""

        timeout = timeout if timeout is not None else settings.LANDING_TIMEOUT_S
        deadline = asyncio.get_running_loop().time() + timeout

        while asyncio.get_running_loop().time() < deadline:
            position = await self.telemetry.get_position()
            armed = await self.telemetry.get_armed()
            landed_state = await self.telemetry.get_landed_state()

            print(
                "RTL landing check: "
                f"altitude={position.relative_altitude_m:.2f} m, "
                f"armed={armed}, landed_state={landed_state}"
            )

            on_ground = landed_state == LandedState.ON_GROUND
            if position.relative_altitude_m <= 0.30 and not armed and on_ground:
                print("Landing and disarm confirmed.")
                return

            await asyncio.sleep(2.0)

        raise exceptions.LandingError(
            "RTL landing confirmation timed out."
        )

    async def _read_home_position(self):
        """Read PX4's reported home position as a mission.Position."""

        home_sample = await self.telemetry.get_home()
        return Position(
            latitude_deg=home_sample.latitude_deg,
            longitude_deg=home_sample.longitude_deg,
            altitude_m=home_sample.absolute_altitude_m,
            altitude_reference=AltitudeReference.AMSL,
        )

    async def _fly_waypoint(self, waypoint, monitor_task):
        """Fly to one waypoint and, if it has a hold time, observe there."""

        position = waypoint.position

        if position.altitude_reference != AltitudeReference.RELATIVE_TO_HOME:
            # Drone.goto() only accepts a relative-to-home altitude and
            # performs the relative->AMSL conversion itself. Flying an
            # AMSL-referenced waypoint would require a different Drone
            # entry point that does not exist yet.
            raise NotImplementedError(
                f"Waypoint {waypoint.name or ''} has "
                f"altitude_reference={position.altitude_reference}, but "
                "MissionManager only supports RELATIVE_TO_HOME waypoints."
            )

        print(f"Flying to waypoint {waypoint.name or position!r}...")
        await self._run_while_monitored(
            self.drone.goto(
                latitude=position.latitude_deg,
                longitude=position.longitude_deg,
                altitude=position.altitude_m,
                yaw=waypoint.yaw_deg,
                acceptance_radius_m=waypoint.acceptance_radius_m,
                altitude_tolerance_m=waypoint.altitude_tolerance_m,
                timeout=waypoint.timeout_s,
            ),
            monitor_task,
        )

        if waypoint.hold_time_s <= 0:
            return

        recording = False
        try:
            await self.camera.start_recording()
            recording = True
        except Exception as e:
            # A camera failure must never abort a healthy flight -- it
            # degrades the mission's data-collection outcome, not the
            # flight itself.
            print(
                f"Camera failed to start recording: {e}. Continuing "
                "mission without recording at this waypoint."
            )

        try:
            print(f"Observing at waypoint for {waypoint.hold_time_s:.0f}s...")
            await self._run_while_monitored(
                asyncio.sleep(waypoint.hold_time_s),
                monitor_task,
            )
        finally:
            if recording:
                try:
                    await self.camera.stop_recording()
                except Exception as e:
                    print(f"Camera failed to stop recording cleanly: {e}")

    async def run(self):
        """Run one complete observation mission."""

        monitor_task = None

        try:
            print("\n[1/7] Connecting to PX4...")
            await self.drone.connect()

            print("\n[2/7] Running preflight safety checks...")
            await self.safety.preflight_check()

            print("\n[3/7] Reading home position and validating mission...")
            self.mission.home = await self._read_home_position()
            self.mission.validate()
            self.mission.start()

            print("\n[4/7] Arming and taking off...")
            await self.drone.arm()
            await self.drone.takeoff()

            print("\n[5/7] Starting continuous flight monitor...")
            monitor_task = asyncio.create_task(
                self.monitor.monitor()
            )

            print("\n[6/7] Flying mission waypoints...")
            for waypoint in self.mission.waypoints:
                await self._fly_waypoint(waypoint, monitor_task)

            await self._stop_monitor(monitor_task)
            monitor_task = None

            print(
                "\n[7/7] Mission completed normally. "
                "Returning to launch..."
            )

            await self.drone.return_to_launch()
            await self._wait_for_landing()

            self.mission.complete()
            print("Mission complete. Aircraft safely on ground.")

        except asyncio.CancelledError:
            # Decided operator-abort policy: Ctrl+C may mean the operator
            # is about to take manual control, and an automatic RTL mode
            # change would fight them for authority. Stop only this
            # application's own bookkeeping and issue NO flight command --
            # PX4 / the RC pilot / QGroundControl own the aircraft from
            # here.
            print(
                "\nMission cancelled by operator. THE AIRCRAFT MAY STILL "
                "BE AIRBORNE. This application is issuing NO further "
                "flight commands -- PX4, the RC pilot, or QGroundControl "
                "are responsible for the aircraft from this point."
            )

            if monitor_task is not None:
                self.monitor.stop()
                if not monitor_task.done():
                    with contextlib.suppress(Exception):
                        await monitor_task

            with contextlib.suppress(Exception):
                await self.camera.stop_recording()

            with contextlib.suppress(RuntimeError):
                self.mission.abort("Cancelled by operator.")

            raise

        except Exception as exception:
            if monitor_task is not None:
                self.monitor.stop()

                if not monitor_task.done():
                    with contextlib.suppress(Exception):
                        await monitor_task

            with contextlib.suppress(Exception):
                await self.camera.stop_recording()

            with contextlib.suppress(RuntimeError):
                self.mission.fail(str(exception))

            await self.emergency.handle_exception(exception)
            raise
