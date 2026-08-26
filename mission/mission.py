"""
Mission: an ordered sequence of waypoints plus identity, status, and
timing -- the record of what a flight intended to do. Execution lives in
mission_manager.py; this module intentionally carries no MAVSDK import so
it is fully unit-testable without a Drone.
"""

import enum
import time
import uuid
from dataclasses import dataclass, field
from typing import List, Optional

from config import settings
from .position import Position
from .waypoint import Waypoint


class MissionStatus(enum.Enum):
    """
    A mission's lifecycle status.

    Deliberately has NO "paused" state. Pausing in the air is a PX4 mode
    change (Hold) that competes with RTL and with PX4's own failsafes for
    control authority, and resuming afterward has undefined state (which
    waypoint, at what altitude, with how much battery reserve). This
    project's plan explicitly defers pause/resume until that airborne
    semantics and control-authority policy is defined -- do not add a
    paused state to work around that decision.
    """

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    ABORTED = "aborted"
    FAILED = "failed"


@dataclass
class Mission:
    """
    waypoints: the ordered flight plan.
    home: the vehicle's actual home Position, filled in by MissionManager
        once PX4 reports it (not known until after connect/preflight), and
        required by validate() to check each waypoint's distance from home.
    """

    waypoints: List[Waypoint]
    home: Optional[Position] = None
    mission_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    status: MissionStatus = MissionStatus.PENDING
    started_at: Optional[float] = None
    ended_at: Optional[float] = None
    failure_reason: Optional[str] = None

    def validate(self):
        """
        Check the mission against application-level limits before any PX4
        command is issued. Raises ValueError on the first problem found.

        This does not duplicate Drone.goto()'s own lat/lon range check --
        that check stays in Drone as a last-resort assertion. This is the
        pre-arming gate: a bad waypoint aborts here, before the vehicle
        ever leaves the ground, instead of raising mid-flight where it
        would be handled as a flight failure by Emergency.
        """

        if not self.waypoints:
            raise ValueError("Mission must contain at least one waypoint.")

        total_hold_time = 0.0

        for index, waypoint in enumerate(self.waypoints):
            position = waypoint.position
            label = waypoint.name or f"#{index}"

            if position.altitude_m < settings.MIN_FLIGHT_ALTITUDE_M:
                raise ValueError(
                    f"Waypoint {label} altitude {position.altitude_m:.1f} m "
                    "is below MIN_FLIGHT_ALTITUDE_M "
                    f"({settings.MIN_FLIGHT_ALTITUDE_M:.1f} m)."
                )

            if position.altitude_m > settings.MAX_FLIGHT_ALTITUDE_M:
                raise ValueError(
                    f"Waypoint {label} altitude {position.altitude_m:.1f} m "
                    "exceeds MAX_FLIGHT_ALTITUDE_M "
                    f"({settings.MAX_FLIGHT_ALTITUDE_M:.1f} m)."
                )

            if self.home is not None:
                distance = self.home.horizontal_distance_m(position)
                if distance > settings.MAX_DISTANCE_FROM_HOME_M:
                    raise ValueError(
                        f"Waypoint {label} is {distance:.1f} m from home, "
                        "exceeding MAX_DISTANCE_FROM_HOME_M "
                        f"({settings.MAX_DISTANCE_FROM_HOME_M:.1f} m)."
                    )

            total_hold_time += waypoint.hold_time_s

        if total_hold_time > settings.MISSION_TIMEOUT:
            raise ValueError(
                f"Total waypoint hold time ({total_hold_time:.0f}s) alone "
                f"exceeds MISSION_TIMEOUT ({settings.MISSION_TIMEOUT:.0f}s), "
                "before takeoff, transit, RTL, and landing are even "
                "accounted for."
            )

    def start(self):
        if self.status != MissionStatus.PENDING:
            raise RuntimeError(f"Cannot start a mission in status {self.status}.")
        self.status = MissionStatus.RUNNING
        self.started_at = time.monotonic()

    def complete(self):
        if self.status != MissionStatus.RUNNING:
            raise RuntimeError(
                f"Cannot complete a mission in status {self.status}."
            )
        self.status = MissionStatus.COMPLETED
        self.ended_at = time.monotonic()

    def abort(self, reason):
        if self.status not in (MissionStatus.PENDING, MissionStatus.RUNNING):
            raise RuntimeError(f"Cannot abort a mission in status {self.status}.")
        self.status = MissionStatus.ABORTED
        self.failure_reason = reason
        self.ended_at = time.monotonic()

    def fail(self, reason):
        if self.status not in (MissionStatus.PENDING, MissionStatus.RUNNING):
            raise RuntimeError(f"Cannot fail a mission in status {self.status}.")
        self.status = MissionStatus.FAILED
        self.failure_reason = reason
        self.ended_at = time.monotonic()
