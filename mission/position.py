"""
Explicit geographic position model.

Deliberately carries no MAVSDK import so it is fully unit-testable without
a Drone/System. An altitude-reference mistake on hardware is a
controlled-flight-into-terrain mistake, so every Position states which
frame its altitude is measured in rather than leaving that implicit.
"""

import enum
from dataclasses import dataclass

from flight import geo


class AltitudeReference(enum.Enum):
    """Which reference frame a Position's altitude is measured against."""

    # Metres above the vehicle's home/takeoff point (MAVSDK
    # relative_altitude_m). This is what config/settings.py's altitude
    # values mean, and what Drone.goto() ultimately accepts as `altitude`.
    RELATIVE_TO_HOME = "relative_to_home"

    # Metres above mean sea level (MAVSDK absolute_altitude_m). The
    # relative-to-home -> AMSL conversion happens in exactly one place:
    # flight.drone.Drone.goto(), using the vehicle's current home altitude
    # at the moment the command is issued.
    AMSL = "amsl"


@dataclass(frozen=True)
class Position:
    """An immutable WGS84 geographic point with an explicit altitude frame."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    altitude_reference: AltitudeReference = AltitudeReference.RELATIVE_TO_HOME

    def __post_init__(self):
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError(
                f"latitude_deg must be between -90 and 90, got "
                f"{self.latitude_deg}."
            )
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError(
                f"longitude_deg must be between -180 and 180, got "
                f"{self.longitude_deg}."
            )

    def horizontal_distance_m(self, other):
        """Great-circle distance in metres to another Position."""

        return geo.horizontal_distance_m(
            self.latitude_deg,
            self.longitude_deg,
            other.latitude_deg,
            other.longitude_deg,
        )
