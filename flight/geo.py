"""
Pure geographic math shared by the flight and mission packages.

Deliberately dependency-free (stdlib `math` only) so it can be unit-tested
without constructing a Drone/MAVSDK System, and so `mission/` can validate
waypoint distances without importing `flight.drone` and creating a cycle.
"""

import math

EARTH_RADIUS_M = 6_371_000.0


def horizontal_distance_m(latitude_a, longitude_a, latitude_b, longitude_b):
    """Return the great-circle distance in metres between two WGS84 points."""

    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    latitude_difference = math.radians(latitude_b - latitude_a)
    longitude_difference = math.radians(longitude_b - longitude_a)

    haversine = (
        math.sin(latitude_difference / 2) ** 2
        + math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(longitude_difference / 2) ** 2
    )

    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(haversine))
