"""
One mission target: where to go, how long to linger there, and the
tolerances that decide when PX4 is considered to have arrived.

Per-waypoint tolerances exist because a transit leg and an observation
hold do not want the same acceptance radius, and because hard-coding
"2.0 m" / "1.0 m" / "60 s" inside flight/drone.py (as the original code
did) left config and behaviour disconnected from each other.
"""

from dataclasses import dataclass
from typing import Optional

from config import settings
from .position import Position


@dataclass(frozen=True)
class Waypoint:
    position: Position
    name: str = ""
    yaw_deg: float = 0.0

    # Time to hold/observe at this waypoint once reached, in seconds. A
    # waypoint with hold_time_s == 0 is a pure transit point -- no camera
    # recording is started there (see MissionManager._fly_waypoint()).
    hold_time_s: float = 0.0

    acceptance_radius_m: Optional[float] = None
    altitude_tolerance_m: Optional[float] = None
    timeout_s: Optional[float] = None

    def __post_init__(self):
        # The dataclass is frozen (waypoints should not be mutated once
        # built), so settings-derived defaults are filled in here via
        # object.__setattr__ rather than as ordinary mutable defaults.
        if self.acceptance_radius_m is None:
            object.__setattr__(
                self,
                "acceptance_radius_m",
                settings.WAYPOINT_ACCEPTANCE_RADIUS_M,
            )
        if self.altitude_tolerance_m is None:
            object.__setattr__(
                self,
                "altitude_tolerance_m",
                settings.WAYPOINT_ALTITUDE_TOLERANCE_M,
            )
        if self.timeout_s is None:
            object.__setattr__(self, "timeout_s", settings.WAYPOINT_TIMEOUT)

        if self.hold_time_s < 0:
            raise ValueError(
                f"hold_time_s must be >= 0, got {self.hold_time_s}."
            )
        if self.acceptance_radius_m <= 0:
            raise ValueError("acceptance_radius_m must be positive.")
        if self.altitude_tolerance_m <= 0:
            raise ValueError("altitude_tolerance_m must be positive.")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive.")
