import pytest

from mission.position import AltitudeReference, Position


def test_defaults_to_relative_to_home():
    p = Position(latitude_deg=47.0, longitude_deg=8.0, altitude_m=20.0)
    assert p.altitude_reference == AltitudeReference.RELATIVE_TO_HOME


def test_amsl_reference_is_explicit():
    p = Position(
        latitude_deg=47.0,
        longitude_deg=8.0,
        altitude_m=450.0,
        altitude_reference=AltitudeReference.AMSL,
    )
    assert p.altitude_reference == AltitudeReference.AMSL


@pytest.mark.parametrize("bad_latitude", [90.1, -90.1, 1000.0])
def test_rejects_invalid_latitude(bad_latitude):
    with pytest.raises(ValueError):
        Position(latitude_deg=bad_latitude, longitude_deg=8.0, altitude_m=10.0)


@pytest.mark.parametrize("bad_longitude", [180.1, -180.1, 1000.0])
def test_rejects_invalid_longitude(bad_longitude):
    with pytest.raises(ValueError):
        Position(latitude_deg=47.0, longitude_deg=bad_longitude, altitude_m=10.0)


def test_is_immutable():
    p = Position(latitude_deg=47.0, longitude_deg=8.0, altitude_m=10.0)
    with pytest.raises(Exception):
        p.latitude_deg = 48.0


def test_horizontal_distance_to_matches_geo_module():
    from flight import geo

    a = Position(latitude_deg=47.0, longitude_deg=8.0, altitude_m=10.0)
    b = Position(latitude_deg=47.001, longitude_deg=8.001, altitude_m=10.0)

    assert a.horizontal_distance_m(b) == pytest.approx(
        geo.horizontal_distance_m(47.0, 8.0, 47.001, 8.001)
    )
