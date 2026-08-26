import asyncio

from config import settings
from flight.drone import Drone
from flight.telemetry import Telemetry
from flight.safety import SafetyManager
from flight.monitor import FlightMonitor
from flight.emergency import Emergency


async def main():

    print("=" * 60)
    print("FLIGHT LAYER SITL INTEGRATION TEST")
    print("=" * 60)

    drone = Drone(takeoff_altitude=settings.MIN_FLIGHT_ALTITUDE)

    telemetry = Telemetry(drone)

    safety = SafetyManager(
        drone=drone,
        telemetry=telemetry
    )

    monitor = FlightMonitor(telemetry)

    emergency = Emergency(
        drone=drone,
        telemetry=telemetry
    )

    # ========================================================
    # 1. CONNECT
    # ========================================================

    print("\n[1/6] Connecting to PX4...")

    await drone.connect()

    print("PX4 connection established.")

    # ========================================================
    # 2. TELEMETRY TEST
    # ========================================================

    print("\n[2/6] Testing telemetry...")

    battery = await telemetry.get_battery()
    position = await telemetry.get_position()
    velocity = await telemetry.get_velocity()
    flight_mode = await telemetry.get_flight_mode()
    heading = await telemetry.get_heading()
    gps_ready = await telemetry.gps_ready()
    home_ready = await telemetry.is_home_position_ready()
    health = await telemetry.get_health()

    print(f"Battery: {battery:.1f}%")
    print(f"Position: {position}")
    print(f"Velocity: {velocity}")
    print(f"Flight mode: {flight_mode}")
    print(f"Heading: {heading}")
    print(f"GPS ready: {gps_ready}")
    print(f"Home position ready: {home_ready}")
    print(f"Health: {health}")

    # ========================================================
    # 3. PREFLIGHT SAFETY
    # ========================================================

    print("\n[3/6] Running preflight safety check...")

    await safety.preflight_check()

    print("Preflight safety check PASSED.")

    # ========================================================
    # 4. ARM + TAKEOFF
    # ========================================================

    print("\n[4/6] Arming and taking off...")

    await drone.arm()

    print("Armed.")

    await drone.takeoff()

    print("Takeoff command completed.")

    # Give PX4 time to stabilize.
    await asyncio.sleep(5)

    # ========================================================
    # 5. FLIGHT MONITOR
    # ========================================================

    print("\n[5/6] Starting flight monitor...")

    monitor_task = asyncio.create_task(
        monitor.monitor()
    )

    monitor_error = None

    try:
        await asyncio.sleep(10)

    finally:
        monitor.stop()

    try:
        await monitor_task

    except Exception as e:
        monitor_error = e
        print(f"Flight monitor detected an exception: {e}")
        await emergency.handle_exception(e)

    if monitor_error is None:
        print("\n[6/6] Mission complete. Returning to home...")
        await emergency.rtl()
        print("RTL command sent.")

    # ========================================================
    # 6. RTL
    # ========================================================

    print("\n[6/6] Returning to launch...")

    await emergency.rtl()

    print("RTL command sent.")

    # Give PX4 time to complete the return/landing.
    await asyncio.sleep(20)

    print("\n" + "=" * 60)
    print("FLIGHT LAYER SITL TEST COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())