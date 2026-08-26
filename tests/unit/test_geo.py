import pytest

from flight import geo


def test_zero_distance_for_identical_points():
    assert geo.horizontal_distance_m(47.0, 8.0, 47.0, 8.0) == pytest.approx(0.0, abs=1e-6)


def test_one_degree_longitude_at_equator():
    # Circumference / 360 with the module's own Earth radius, so this
    # tracks EARTH_RADIUS_M rather than assuming a different constant.
    expected = 2 * geo.EARTH_RADIUS_M * 3.141592653589793 / 360
    distance = geo.horizontal_distance_m(0.0, 0.0, 0.0, 1.0)
    assert distance == pytest.approx(expected, rel=1e-6)


def test_short_hop_matches_flat_earth_approximation():
    # For a short displacement, the great-circle distance must agree
    # closely with the standard equirectangular (flat-Earth) approximation
    # computed independently here -- this cross-checks the haversine
    # implementation against a different formula rather than against a
    # single hardcoded "known" value.
    import math

    lat, lon = 47.398, 8.546
    d_lat_deg, d_lon_deg = 0.001, 0.0009  # ~100-150 m scale, matching the
    # magnitude of the Gazebo test waypoint used elsewhere in this project.

    lat_rad = math.radians(lat)
    dx = math.radians(d_lon_deg) * math.cos(lat_rad) * geo.EARTH_RADIUS_M
    dy = math.radians(d_lat_deg) * geo.EARTH_RADIUS_M
    flat_earth_distance = math.hypot(dx, dy)

    distance = geo.horizontal_distance_m(
        lat, lon, lat + d_lat_deg, lon + d_lon_deg
    )

    assert distance == pytest.approx(flat_earth_distance, rel=1e-3)


def test_distance_is_symmetric():
    a = (47.398, 8.546)
    b = (47.400, 8.550)
    assert geo.horizontal_distance_m(*a, *b) == pytest.approx(
        geo.horizontal_distance_m(*b, *a)
    )
