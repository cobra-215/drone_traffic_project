"""
CLI entry point for the Gazebo/SITL integration scenarios in
tests/sitl/scenarios.py.

Usage:
    python -m tests.sitl.run --scenario nominal
    python -m tests.sitl.run --scenario low-battery

Requires a running PX4 SITL + Gazebo instance reachable at
config.settings.PX4_CONNECTION_ADDRESS (e.g. `make px4_sitl gz_x500` from
a PX4-Autopilot checkout). QGroundControl may optionally observe on UDP
14550.

link-loss and operator-abort intentionally leave the vehicle airborne
when they pass -- see their docstrings in scenarios.py before running
another scenario afterward.
"""

import argparse
import asyncio
import sys

from .scenarios import SCENARIOS


async def main(scenario_name):
    print("=" * 60)
    print(f"SITL SCENARIO: {scenario_name}")
    print("=" * 60)

    scenario = SCENARIOS[scenario_name]
    await scenario()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    arguments = parser.parse_args()

    try:
        asyncio.run(main(arguments.scenario))
    except AssertionError as e:
        print(f"\nSCENARIO {arguments.scenario} FAILED: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nScenario interrupted by operator.")
        sys.exit(1)
