"""
Gazebo/SITL integration scenarios for this project's flight and mission
layers.

Requires a running PX4 SITL + Gazebo instance (e.g. `make px4_sitl
gz_x500`) reachable at config.settings.PX4_CONNECTION_ADDRESS, with
QGroundControl optionally observing on UDP 14550.

These are integration tests, not unit tests: each scenario drives a real
MAVSDK connection and (for most scenarios) a real flight. They are not
collected by `pytest tests/unit` and are not part of this project's fast
test loop -- run them individually via tests/sitl/run.py.

Two scenarios (link_loss, operator_abort) simulate their fault by
monkeypatching this application's own Telemetry/Drone wrappers rather
than killing the shared PX4 process or the operator's own terminal, so
the same SITL instance can keep being used for other scenarios
afterward. Both intentionally leave the vehicle airborne and hovering
when they pass (issuing no RTL/land/hold is the entire point) -- recover
the vehicle manually via QGroundControl, or restart SITL, before running
a scenario that assumes a grounded vehicle. Disconnecting our own
Drone/mavsdk_server (which every scenario does on exit, see
components() below) does not affect PX4 or the aircraft itself.
"""

import asyncio
import contextlib
import time

from config import settings
from camera.factory import build_recorder
from flight import exceptions as ex
from flight.drone import Drone
from flight.emergency import Emergency
from flight.monitor import FlightMonitor
from flight.safety import SafetyManager
from flight.telemetry import Telemetry
from mission.mission import Mission
from mission.mission_manager import MissionManager
from mission.position import Position
from mission.waypoint import Waypoint


@contextlib.asynccontextmanager
async def components():
    """
    Build one set of flight/mission collaborators and guarantee the
    Drone's mavsdk_server subprocess is explicitly stopped on the way
    out, regardless of how the scenario ends.

    Without this, running scenarios back-to-back (each a fresh Python
    process construction of Drone -> System) can leak mavsdk_server
    subprocesses and cause "bind error: Address in use" on the next
    connect() -- see Drone.disconnect() for why this can't be left to
    MAVSDK's own __del__/atexit handling.
    """

    drone = Drone(takeoff_altitude=settings.TAKEOFF_ALTITUDE_M)
    telemetry = Telemetry(drone)
    safety = SafetyManager(drone=drone, telemetry=telemetry)
    monitor = FlightMonitor(telemetry=telemetry)
    emergency = Emergency(drone=drone, telemetry=telemetry)
    camera = build_recorder()

    try:
        yield drone, telemetry, safety, monitor, emergency, camera
    finally:
        drone.disconnect()


def build_short_mission(hold_time_s=10.0):
    """A single nearby waypoint with a short hold, for fast scenario runs."""

    waypoint = Waypoint(
        position=Position(
            latitude_deg=settings.OBSERVATION_LATITUDE,
            longitude_deg=settings.OBSERVATION_LONGITUDE,
            altitude_m=settings.OBSERVATION_ALTITUDE_M,
        ),
        name="observation",
        hold_time_s=hold_time_s,
    )
    return Mission(waypoints=[waypoint])


async def wait_for_landing(telemetry, timeout=None):
    timeout = timeout if timeout is not None else settings.LANDING_TIMEOUT_S
    deadline = asyncio.get_event_loop().time() + timeout

    while asyncio.get_event_loop().time() < deadline:
        position = await telemetry.get_position()
        armed = await telemetry.get_armed()
        print(
            f"    landing check: alt={position.relative_altitude_m:.2f}m "
            f"armed={armed}"
        )
        if position.relative_altitude_m <= 0.30 and not armed:
            return
        await asyncio.sleep(2.0)

    raise TimeoutError("Vehicle did not land within the timeout.")


async def scenario_nominal():
    """
    Full mission via MissionManager: connect -> preflight (incl. PX4
    parameter audit) -> validate -> arm -> takeoff -> fly to waypoint ->
    observe (short, with simulated recording) -> RTL -> land -> disarm.
    """

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=10.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        await manager.run()

        assert mission.status.value == "completed", (
            f"expected completed, got {mission.status}"
        )
        assert camera.is_recording is False, (
            "camera left recording after mission completion"
        )
        print("SCENARIO nominal: PASSED")


async def scenario_multi_waypoint():
    """Two waypoints in sequence, each with its own short observation hold."""

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        wp1 = Waypoint(
            position=Position(
                latitude_deg=settings.OBSERVATION_LATITUDE,
                longitude_deg=settings.OBSERVATION_LONGITUDE,
                altitude_m=settings.OBSERVATION_ALTITUDE_M,
            ),
            name="wp1",
            hold_time_s=5.0,
        )
        wp2 = Waypoint(
            position=Position(
                latitude_deg=settings.OBSERVATION_LATITUDE + 0.0005,
                longitude_deg=settings.OBSERVATION_LONGITUDE,
                altitude_m=settings.OBSERVATION_ALTITUDE_M,
            ),
            name="wp2",
            hold_time_s=5.0,
        )
        mission = Mission(waypoints=[wp1, wp2])
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        await manager.run()

        assert mission.status.value == "completed"
        print("SCENARIO multi-waypoint: PASSED")


async def scenario_low_battery():
    """
    Injected low battery during the observation hold must:
      FlightMonitor raises LowBatteryError (carrying the measured value)
      -> MissionManager hands off to Emergency
      -> Emergency requests RTL (battery above critical threshold)
      -> PX4 lands.
    """

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=60.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        original_get_battery = telemetry.get_battery
        injected_value = settings.BATTERY_RTL_THRESHOLD  # <= RTL, > critical

        async def injected_low_battery():
            return injected_value

        async def inject_after_delay():
            await asyncio.sleep(8.0)
            telemetry.get_battery = injected_low_battery
            print("    [injected low battery]")

        injector = asyncio.create_task(inject_after_delay())

        try:
            try:
                await manager.run()
                raise AssertionError(
                    "expected LowBatteryError; mission completed normally"
                )
            except ex.LowBatteryError as e:
                print(f"    correctly raised LowBatteryError: {e}")
        finally:
            telemetry.get_battery = original_get_battery
            injector.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await injector

        assert mission.status.value == "failed", (
            f"expected failed, got {mission.status}"
        )

        await wait_for_landing(telemetry)
        print("SCENARIO low-battery: PASSED")


async def scenario_home_not_ready():
    """
    Acceptance test for D2: HomePositionNotReadyError during preflight
    (on the ground, disarmed) must abort WITHOUT any flight command.
    """

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=5.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        original_is_home_position_ready = telemetry.is_home_position_ready

        async def always_not_ready():
            return False

        telemetry.is_home_position_ready = always_not_ready

        try:
            try:
                await manager.run()
                raise AssertionError("expected HomePositionNotReadyError")
            except ex.HomePositionNotReadyError as e:
                print(f"    correctly raised HomePositionNotReadyError: {e}")
        finally:
            telemetry.is_home_position_ready = original_is_home_position_ready

        assert mission.status.value == "failed"

        armed = await telemetry.get_armed()
        assert armed is False, (
            "vehicle must not be armed when home position is not ready"
        )

        print("SCENARIO home-not-ready: PASSED (no flight command issued)")


async def scenario_altitude_violation():
    """Injected excessive altitude during the observation hold must abort
    and RTL via the flight-maneuver-failure branch of Emergency."""

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=60.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        original_get_position = telemetry.get_position

        async def spoofed_get_position():
            real = await original_get_position()
            real.relative_altitude_m = settings.MAX_FLIGHT_ALTITUDE_M + 10
            return real

        async def inject_after_delay():
            await asyncio.sleep(8.0)
            telemetry.get_position = spoofed_get_position
            print("    [injected altitude violation]")

        injector = asyncio.create_task(inject_after_delay())

        try:
            try:
                await manager.run()
                raise AssertionError(
                    "expected MissionError for altitude violation"
                )
            except ex.MissionError as e:
                print(f"    correctly raised MissionError: {e}")
        finally:
            telemetry.get_position = original_get_position
            injector.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await injector

        assert mission.status.value == "failed"

        await wait_for_landing(telemetry)
        print("SCENARIO altitude-violation: PASSED")


async def scenario_mission_timeout():
    """
    Forces FlightMonitor's own mission_start_time into the past (rather
    than shrinking settings.MISSION_TIMEOUT, which Mission.validate()
    also depends on) so check_mission_timeout() fires during the
    observation hold.
    """

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=30.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        async def force_timeout_after_monitor_starts():
            while not monitor.monitoring:
                await asyncio.sleep(0.2)
            monitor.mission_start_time = (
                time.monotonic() - settings.MISSION_TIMEOUT - 1
            )
            print("    [forced mission_start_time into the past]")

        forcer = asyncio.create_task(force_timeout_after_monitor_starts())

        try:
            try:
                await manager.run()
                raise AssertionError("expected MissionError for mission timeout")
            except ex.MissionError as e:
                print(f"    correctly raised MissionError: {e}")
        finally:
            forcer.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await forcer

        assert mission.status.value == "failed"

        await wait_for_landing(telemetry)
        print("SCENARIO mission-timeout: PASSED")


async def scenario_link_loss():
    """
    Acceptance test for D1: once telemetry stalls, the mission must abort
    WITHOUT issuing RTL -- PX4's own data-link-loss failsafe (not this
    application) is responsible for the aircraft in that state.

    Simulates the stall at the telemetry layer (every read raises
    TelemetryTimeoutError) rather than killing the shared PX4 process.
    Leaves the vehicle airborne and hovering when it passes.
    """

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=60.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        rtl_calls = []
        original_rtl = drone.return_to_launch

        async def spying_rtl():
            rtl_calls.append(True)
            return await original_rtl()

        drone.return_to_launch = spying_rtl

        async def sever_link_after_delay():
            await asyncio.sleep(8.0)

            async def severed(*args, **kwargs):
                raise ex.TelemetryTimeoutError(
                    "simulated link loss", stream_name="simulated"
                )

            for name in (
                "get_battery",
                "get_position",
                "get_home",
                "get_health",
                "get_velocity",
                "get_armed",
                "get_landed_state",
            ):
                setattr(telemetry, name, severed)

            print("    [simulated telemetry link loss]")

        severer = asyncio.create_task(sever_link_after_delay())

        try:
            try:
                await manager.run()
                raise AssertionError("expected TelemetryTimeoutError")
            except ex.TelemetryTimeoutError as e:
                print(f"    correctly raised TelemetryTimeoutError: {e}")
        finally:
            severer.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await severer
            drone.return_to_launch = original_rtl

        assert rtl_calls == [], (
            f"RTL must NOT be issued on link loss, but was called "
            f"{len(rtl_calls)} time(s)"
        )
        assert mission.status.value == "failed"

        print("SCENARIO link-loss: PASSED (no RTL issued, as required)")
        print(
            "    NOTE: the vehicle is still airborne by design -- recover "
            "manually via QGroundControl, or restart SITL, before running "
            "a scenario that assumes a grounded vehicle."
        )


async def scenario_operator_abort():
    """
    Acceptance test for D10: cancelling the mission mid-flight (simulating
    operator Ctrl+C) must issue NO further flight command -- PX4, the RC
    pilot, or QGroundControl own the aircraft from that point.

    Leaves the vehicle airborne and hovering when it passes.
    """

    async with components() as (drone, telemetry, safety, monitor, emergency, camera):
        mission = build_short_mission(hold_time_s=60.0)
        manager = MissionManager(
            drone, telemetry, safety, monitor, emergency, camera, mission
        )

        commands = []
        original_rtl = drone.return_to_launch
        original_hold = drone.hold
        original_land = drone.land

        async def spy_rtl():
            commands.append("rtl")
            return await original_rtl()

        async def spy_hold():
            commands.append("hold")
            return await original_hold()

        async def spy_land():
            commands.append("land")
            return await original_land()

        drone.return_to_launch = spy_rtl
        drone.hold = spy_hold
        drone.land = spy_land

        mission_task = asyncio.create_task(manager.run())

        await asyncio.sleep(8.0)  # let it get airborne and into the observation hold
        mission_task.cancel()

        try:
            await mission_task
            raise AssertionError("expected CancelledError")
        except asyncio.CancelledError:
            print("    mission task cancelled, as expected")

        assert commands == [], (
            f"expected NO flight commands after cancellation, got {commands}"
        )
        assert mission.status.value == "aborted"

        print(
            "SCENARIO operator-abort: PASSED (no flight command issued "
            "after cancellation)"
        )
        print(
            "    NOTE: the vehicle is still airborne by design -- recover "
            "manually via QGroundControl, or restart SITL, before running "
            "a scenario that assumes a grounded vehicle."
        )


SCENARIOS = {
    "nominal": scenario_nominal,
    "multi-waypoint": scenario_multi_waypoint,
    "low-battery": scenario_low_battery,
    "home-not-ready": scenario_home_not_ready,
    "altitude-violation": scenario_altitude_violation,
    "mission-timeout": scenario_mission_timeout,
    "link-loss": scenario_link_loss,
    "operator-abort": scenario_operator_abort,
}
