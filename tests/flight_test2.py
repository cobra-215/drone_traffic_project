import argparse
import asyncio
import time

from config import settings
from flight import exceptions
from flight.drone import Drone
from flight.emergency import Emergency
from flight.monitor import FlightMonitor
from flight.safety import SafetyManager
from flight.telemetry import Telemetry


HEALTHY_MONITOR_SECONDS = 15
LAND_ALTITUDE_TOLERANCE_M = 0.30
LAND_TIMEOUT_SECONDS = 120


async def wait_for_monitor_to_remain_healthy(monitor_task, seconds):
    """
    Confirm that the monitor stays alive and raises no exception during the
    given healthy-flight period.
    """

    deadline = time.monotonic() + seconds

    while time.monotonic() < deadline:
        if monitor_task.done():
            exception = monitor_task.exception()

            if exception is not None:
                raise exception

            raise RuntimeError(
                "FlightMonitor stopped unexpectedly before the mission ended."
            )

        await asyncio.sleep(0.2)


async def wait_for_landing(telemetry):
    """Wait until PX4 has returned and the vehicle is near ground level."""

    deadline = time.monotonic() + LAND_TIMEOUT_SECONDS

    while time.monotonic() < deadline:
        position = await telemetry.get_position()
        altitude = position.relative_altitude_m

        print(f"RTL landing check: altitude={altitude:.2f} m")

        if altitude <= LAND_ALTITUDE_TOLERANCE_M:
            print("Landing confirmed.")
            return

        await asyncio.sleep(1)

    raise TimeoutError(
        f"Vehicle did not land within {LAND_TIMEOUT_SECONDS} seconds."
    )


async def run_healthy_scenario(
    drone,
    telemetry,
    monitor,
):
    """
    Test that the monitor remains active during a stable hover, then stop it
    cleanly and perform normal mission completion RTL.
    """

    print("\n[Scenario: healthy]")
    print(
        f"Monitoring normally for {HEALTHY_MONITOR_SECONDS} seconds..."
    )

    monitor_task = asyncio.create_task(monitor.monitor())

    try:
        await wait_for_monitor_to_remain_healthy(
            monitor_task,
            HEALTHY_MONITOR_SECONDS,
        )

        print("Healthy monitor test passed.")

    finally:
        # Normal mission completion: stop monitor first.
        monitor.stop()
        await monitor_task

    # This is normal mission RTL, not an emergency action.
    print("Healthy mission complete. Requesting normal RTL.")
    await drone.return_to_launch()
    await wait_for_landing(telemetry)


async def run_low_battery_scenario(
    telemetry,
    monitor,
    emergency,
):
    """
    Verify the complete chain:

    injected low battery
        -> FlightMonitor raises LowBatteryError
        -> EmergencyManager aborts mission
        -> EmergencyManager requests PX4 RTL
        -> PX4 lands
    """

    print("\n[Scenario: low-battery]")
    monitor_task = asyncio.create_task(monitor.monitor())

    original_get_battery = telemetry.get_battery

    try:
        print(
            f"Running normally for {HEALTHY_MONITOR_SECONDS} seconds "
            "before injecting the test condition..."
        )

        await wait_for_monitor_to_remain_healthy(
            monitor_task,
            HEALTHY_MONITOR_SECONDS,
        )

        async def injected_low_battery():
            return settings.BATTERY_RTL_THRESHOLD

        telemetry.get_battery = injected_low_battery

        print(
            "Injected low battery into application telemetry. "
            "Waiting for FlightMonitor to react..."
        )

        try:
            await asyncio.wait_for(
                monitor_task,
                timeout=(settings.MONITOR_INTERVAL * 3) + 5,
            )

        except exceptions.LowBatteryError as exception:
            print(f"Monitor correctly detected fault: {exception}")
            await emergency.handle_exception(exception)

        else:
            raise AssertionError(
                "FlightMonitor did not raise LowBatteryError."
            )

        await wait_for_landing(telemetry)
        print("Low-battery emergency test passed.")

    finally:
        telemetry.get_battery = original_get_battery

        # Safe even when monitor already stopped in its finally block.
        monitor.stop()

        if not monitor_task.done():
            await monitor_task


async def main(scenario):
    print("=" * 60)
    print("FLIGHT MONITOR GAZEBO INTEGRATION TEST")
    print("=" * 60)

    drone = Drone(
        takeoff_altitude=settings.MIN_FLIGHT_ALTITUDE
    )
    telemetry = Telemetry(drone)
    safety = SafetyManager(
        drone=drone,
        telemetry=telemetry,
    )
    monitor = FlightMonitor(telemetry)
    emergency = Emergency(
        drone=drone,
        telemetry=telemetry,
    )

    print("\n[1/4] Connecting to PX4...")
    await drone.connect()

    print("\n[2/4] Running preflight safety checks...")
    await safety.preflight_check()

    print("\n[3/4] Arming and taking off...")
    await drone.arm()
    await drone.takeoff()

    print(
        "\n[4/4] Takeoff confirmed. "
        f"Running '{scenario}' monitor scenario..."
    )

    current_position = await telemetry.get_position()

    await drone.goto(
        latitude=current_position.latitude_deg + 0.00090,
        longitude=current_position.longitude_deg,
        altitude=settings.MIN_FLIGHT_ALTITUDE,
    )

    if scenario == "healthy":
        await run_healthy_scenario(
            drone=drone,
            telemetry=telemetry,
            monitor=monitor,
        )

    elif scenario == "low-battery":
        await run_low_battery_scenario(
            telemetry=telemetry,
            monitor=monitor,
            emergency=emergency,
        )

    print("\n" + "=" * 60)
    print("TEST PASSED")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        choices=("healthy", "low-battery"),
        required=True,
    )

    arguments = parser.parse_args()
    asyncio.run(main(arguments.scenario))