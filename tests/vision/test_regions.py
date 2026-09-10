"""
Tests for camera/regions.py -- the region-file loader/saver.

camera/regions.py is stdlib-only by design, so these tests need neither
OpenCV nor a display. Run with the rest of the vision suite:
`pytest tests/vision`.
"""

import json

import pytest

from camera.regions import RegionFileError, load_regions, save_regions
from camera.traffic_metrics import CountingLine, Zone


def write(tmp_path, document, name="regions.json"):
    path = tmp_path / name
    path.write_text(json.dumps(document))
    return str(path)


# --- round trip ------------------------------------------------------------


def test_save_load_round_trip(tmp_path):
    path = str(tmp_path / "regions.json")
    save_regions(
        path,
        lines=[CountingLine("north_gate", 640, 0, 640, 720)],
        zones=[Zone("north_arm", [(0, 0), (100, 0), (100, 100), (0, 100)])],
        video="traffic.mp4",
        frame_size=(1280, 720),
    )

    lines, zones = load_regions(path)

    assert [line.name for line in lines] == ["north_gate"]
    assert lines[0].a == (640.0, 0.0)
    assert lines[0].b == (640.0, 720.0)
    assert [zone.name for zone in zones] == ["north_arm"]
    assert zones[0].points == [
        (0.0, 0.0),
        (100.0, 0.0),
        (100.0, 100.0),
        (0.0, 100.0),
    ]


def test_saves_frame_size_and_video(tmp_path):
    path = str(tmp_path / "regions.json")
    save_regions(
        path,
        lines=[CountingLine("a", 0, 0, 10, 10)],
        video="clip.mp4",
        frame_size=(1920, 1080),
    )

    document = json.loads((tmp_path / "regions.json").read_text())
    assert document["frame_size"] == [1920, 1080]
    assert document["video"] == "clip.mp4"


def test_empty_file_loads_as_empty_lists(tmp_path):
    path = write(tmp_path, {"lines": [], "zones": []})
    assert load_regions(path) == ([], [])


def test_missing_keys_are_tolerated(tmp_path):
    # A file with only zones must not fail for having no "lines" key.
    path = write(
        tmp_path,
        {"zones": [{"name": "z", "points": [[0, 0], [1, 0], [1, 1]]}]},
    )
    lines, zones = load_regions(path)
    assert lines == []
    assert len(zones) == 1


def test_unnamed_shapes_get_auto_names(tmp_path):
    path = write(
        tmp_path,
        {
            "lines": [{"points": [[0, 0], [10, 10]]}],
            "zones": [{"points": [[0, 0], [1, 0], [1, 1]]}],
        },
    )
    lines, zones = load_regions(path)
    assert lines[0].name == "line1"
    assert zones[0].name == "zone1"


# --- frame-size mismatch ---------------------------------------------------


def test_frame_size_mismatch_warns(tmp_path, capsys):
    path = write(
        tmp_path,
        {
            "frame_size": [1920, 1080],
            "lines": [{"name": "a", "points": [[0, 0], [10, 10]]}],
        },
    )
    load_regions(path, frame_size=(1280, 720))

    output = capsys.readouterr().out
    assert "WARNING" in output
    assert "1920x1080" in output
    assert "1280x720" in output


def test_matching_frame_size_does_not_warn(tmp_path, capsys):
    path = write(
        tmp_path,
        {
            "frame_size": [1280, 720],
            "lines": [{"name": "a", "points": [[0, 0], [10, 10]]}],
        },
    )
    load_regions(path, frame_size=(1280, 720))
    assert "WARNING" not in capsys.readouterr().out


# --- malformed files -------------------------------------------------------


def test_rejects_invalid_json(tmp_path):
    path = tmp_path / "regions.json"
    path.write_text("{not json")
    with pytest.raises(RegionFileError):
        load_regions(str(path))


def test_rejects_non_object_document(tmp_path):
    path = write(tmp_path, ["not", "an", "object"])
    with pytest.raises(RegionFileError):
        load_regions(path)


def test_rejects_line_without_two_points(tmp_path):
    path = write(tmp_path, {"lines": [{"name": "a", "points": [[0, 0]]}]})
    with pytest.raises(RegionFileError):
        load_regions(path)


def test_rejects_zone_with_fewer_than_three_points(tmp_path):
    path = write(tmp_path, {"zones": [{"name": "z", "points": [[0, 0], [1, 1]]}]})
    with pytest.raises(RegionFileError):
        load_regions(path)


def test_rejects_shape_without_points(tmp_path):
    path = write(tmp_path, {"lines": [{"name": "a"}]})
    with pytest.raises(RegionFileError):
        load_regions(path)


def test_rejects_malformed_point(tmp_path):
    path = write(tmp_path, {"lines": [{"name": "a", "points": [[0, 0], [1]]}]})
    with pytest.raises(RegionFileError):
        load_regions(path)


def test_rejects_non_list_shape_collection(tmp_path):
    path = write(tmp_path, {"zones": {"name": "z"}})
    with pytest.raises(RegionFileError):
        load_regions(path)
