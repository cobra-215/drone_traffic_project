"""
Tests for camera/analyze_video.py helpers. Requires requirements-vision.txt.
Run separately from the flight-critical suite: `pytest tests/vision`.
"""

import pytest

from camera.analyze_video import (
    _centroids_from_xyxy,
    parse_lines,
    parse_pcu_overrides,
)


# --- parse_pcu_overrides ---------------------------------------------------


def test_parses_single_override():
    assert parse_pcu_overrides(["van=1.4"]) == {"van": 1.4}


def test_parses_multiple_overrides():
    assert parse_pcu_overrides(["van=1.4", "bus=2.2"]) == {"van": 1.4, "bus": 2.2}


def test_empty_and_none_yield_empty_dict():
    assert parse_pcu_overrides([]) == {}
    assert parse_pcu_overrides(None) == {}


def test_rejects_missing_equals():
    with pytest.raises(ValueError):
        parse_pcu_overrides(["van"])


def test_rejects_non_numeric_weight():
    with pytest.raises(ValueError):
        parse_pcu_overrides(["van=heavy"])


def test_rejects_empty_class_name():
    with pytest.raises(ValueError):
        parse_pcu_overrides(["=1.4"])


# --- parse_lines ---------------------------------------------------------


def test_parses_named_line():
    (line,) = parse_lines(["northbound:640,0,640,720"])
    assert line.name == "northbound"
    assert line.a == (640.0, 0.0)
    assert line.b == (640.0, 720.0)


def test_auto_names_unnamed_lines():
    lines = parse_lines(["0,0,10,10", "5,0,5,10"])
    assert [line.name for line in lines] == ["line1", "line2"]


def test_rejects_wrong_coordinate_count():
    with pytest.raises(ValueError):
        parse_lines(["0,0,10"])


def test_rejects_non_numeric_coordinates():
    with pytest.raises(ValueError):
        parse_lines(["a,b,c,d"])


def test_rejects_degenerate_line():
    with pytest.raises(ValueError):
        parse_lines(["5,5,5,5"])


def test_empty_and_none_yield_no_lines():
    assert parse_lines([]) == []
    assert parse_lines(None) == []


# --- _centroids_from_xyxy ----------------------------------------------------


def test_centroids_from_boxes():
    assert _centroids_from_xyxy([(0, 0, 10, 20), (10, 10, 30, 30)]) == [
        (5.0, 10.0),
        (20.0, 20.0),
    ]


def test_centroids_none_passthrough():
    assert _centroids_from_xyxy(None) is None


# --- annotated_scale validation ------------------------------------------


def test_analyze_rejects_bad_annotated_scale(tmp_path):
    from camera.analyze_video import analyze

    for bad in (0.0, -0.5, 1.5):
        with pytest.raises(ValueError):
            analyze(
                model_path="unused",
                video_path="unused",
                output_dir=str(tmp_path),
                window_seconds=10,
                confidence_threshold=0.4,
                annotated_scale=bad,
            )
