"""
Tests for camera/traffic_metrics.py. Requires requirements-vision.txt
(matplotlib) -- run separately from the flight-critical suite:
`pytest tests/vision`.
"""

import csv

import pytest

from camera.traffic_metrics import (
    PCU_DEFAULT,
    PCU_FACTORS,
    CountingLine,
    DensityAnalyzer,
    PcuResolver,
    ScreenlineAnalyzer,
    _segments_intersect,
)

CLASS_NAMES = {0: "car", 1: "van", 2: "truck", 3: "bus"}


# --------------------------------------------------------------------------
# PcuResolver
# --------------------------------------------------------------------------


def test_pcu_resolver_known_class():
    assert PcuResolver().factor_for("truck") == PCU_FACTORS["truck"]


def test_pcu_resolver_is_case_insensitive():
    assert PcuResolver().factor_for("Van") == PCU_FACTORS["van"]


def test_pcu_resolver_override_wins():
    assert PcuResolver({"van": 1.4}).factor_for("van") == 1.4


def test_pcu_resolver_unknown_class_defaults_and_warns_once(capsys):
    resolver = PcuResolver()
    assert resolver.factor_for("hovercraft") == PCU_DEFAULT
    assert resolver.factor_for("hovercraft") == PCU_DEFAULT
    assert capsys.readouterr().out.count("hovercraft") == 1


def test_pcu_resolver_table_covers_every_class():
    table = PcuResolver({"van": 1.4}).table(CLASS_NAMES)
    assert table == {"bus": 3.0, "car": 1.0, "truck": 3.0, "van": 1.4}


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------


def test_segments_intersect_true_when_crossing():
    assert _segments_intersect((0, 0), (10, 10), (0, 10), (10, 0))


def test_segments_intersect_false_when_parallel():
    assert not _segments_intersect((0, 0), (10, 0), (0, 1), (10, 1))


def test_segments_intersect_false_when_disjoint():
    assert not _segments_intersect((0, 0), (1, 1), (5, 5), (6, 6))


# --------------------------------------------------------------------------
# CountingLine
# --------------------------------------------------------------------------


def test_counting_line_rejects_degenerate_line():
    with pytest.raises(ValueError):
        CountingLine("bad", 5, 5, 5, 5)


def test_counting_line_detects_crossing_and_direction():
    # Vertical line drawn top-to-bottom, from A=(5,0) to B=(5,10).
    line = CountingLine("gate", 5, 0, 5, 10)
    # Left -> right and right -> left must be detected as opposite
    # directions (the exact +/- label is a documented convention).
    left_to_right = line.crossing_direction((2, 5), (8, 5))
    right_to_left = line.crossing_direction((8, 5), (2, 5))
    assert {left_to_right, right_to_left} == {"+", "-"}
    # Moving parallel, never touching the line -> no crossing.
    assert line.crossing_direction((0, 1), (0, 9)) is None
    # Moving on the right side only -> no crossing.
    assert line.crossing_direction((6, 1), (9, 9)) is None


# --------------------------------------------------------------------------
# DensityAnalyzer
# --------------------------------------------------------------------------


def test_density_mean_and_peak_vehicles_in_frame():
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    d.record(0.0, track_ids=[1, 2], class_ids=[0, 0])  # 2 in frame
    d.record(1.0, track_ids=[1, 2, 3, 4], class_ids=[0, 0, 0, 0])  # 4 in frame
    d.record(2.0, track_ids=[1], class_ids=[0])  # 1 in frame
    row = d.to_rows()[0]
    assert row["frames"] == 3
    assert row["mean_vehicles_in_frame"] == pytest.approx((2 + 4 + 1) / 3, abs=0.01)
    assert row["max_vehicles_in_frame"] == 4
    assert row["distinct_vehicles"] == 4  # ids 1..4 seen once each


def test_density_has_no_flow_rate_column():
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    d.record(0.0, track_ids=[1], class_ids=[0])
    row = d.to_rows()[0]
    assert not any("flow_rate" in key for key in row)


def test_density_per_class_means_and_distinct_counts():
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    d.record(0.0, track_ids=[1, 2], class_ids=[0, 3])  # 1 car, 1 bus
    d.record(1.0, track_ids=[1, 2], class_ids=[0, 3])  # same vehicles
    row = d.to_rows()[0]
    assert row["mean_car"] == pytest.approx(1.0)
    assert row["mean_bus"] == pytest.approx(1.0)
    assert row["distinct_car"] == 1
    assert row["distinct_bus"] == 1


def test_density_mean_pcu_uses_overrides():
    d = DensityAnalyzer(
        class_names=CLASS_NAMES, window_seconds=60, pcu_factors={"van": 1.4}
    )
    d.record(0.0, track_ids=[1], class_ids=[1])  # one van, one frame
    assert d.to_rows()[0]["mean_pcu_in_frame"] == pytest.approx(1.4)


def test_density_windows_by_frame_time():
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=10)
    d.record(5.0, track_ids=[1], class_ids=[0])
    d.record(15.0, track_ids=[2], class_ids=[0])
    rows = d.to_rows()
    assert [r["window_start_s"] for r in rows] == [0, 10]


def test_density_handles_frames_with_no_detections():
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    d.record(0.0, track_ids=[1, 2], class_ids=[0, 0])
    d.record(1.0, track_ids=None, class_ids=None)  # empty frame counts as 0
    row = d.to_rows()[0]
    assert row["frames"] == 2
    assert row["mean_vehicles_in_frame"] == pytest.approx(1.0)


def test_density_empty_when_nothing_recorded():
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    assert d.to_rows() == []


def test_density_writes_csv_and_plot(tmp_path):
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    d.record(0.0, track_ids=[1, 2], class_ids=[0, 3])
    csv_path = tmp_path / "density.csv"
    png_path = tmp_path / "density.png"
    d.write_csv(str(csv_path))
    d.plot(str(png_path))
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1
    assert "mean_vehicles_in_frame" in rows[0]
    assert png_path.stat().st_size > 0


# --------------------------------------------------------------------------
# ScreenlineAnalyzer
# --------------------------------------------------------------------------


def _line():
    return CountingLine("gate", 5, 0, 5, 10)


def test_screenline_requires_a_line():
    with pytest.raises(ValueError):
        ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=[], window_seconds=60)


def test_screenline_counts_a_crossing_once_with_direction():
    s = ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=[_line()], window_seconds=60)
    # frame 1: establish position on the left of the line
    s.record(0.0, track_ids=[1], class_ids=[0], centroids=[(2, 5)])
    # frame 2: now on the right -> one crossing
    s.record(1.0, track_ids=[1], class_ids=[0], centroids=[(8, 5)])
    # frame 3: still on the right -> no new crossing
    s.record(2.0, track_ids=[1], class_ids=[0], centroids=[(9, 5)])
    assert s.total_crossings == 1
    row = s.to_rows()[0]
    assert row["line"] == "gate"
    assert row["direction"] in {"+", "-"}
    assert row["count_car"] == 1
    assert row["total_count"] == 1


def test_screenline_does_not_count_vehicles_that_never_cross():
    s = ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=[_line()], window_seconds=60)
    s.record(0.0, track_ids=[1], class_ids=[0], centroids=[(1, 5)])
    s.record(1.0, track_ids=[1], class_ids=[0], centroids=[(2, 5)])  # still left
    assert s.total_crossings == 0
    assert s.to_rows() == []


def test_screenline_flow_rate_uses_q_equals_n_over_t():
    s = ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=[_line()], window_seconds=60)
    for track_id in range(1, 6):  # 5 vehicles cross in the first minute
        s.record(0.0, track_ids=[track_id], class_ids=[0], centroids=[(2, 5)])
        s.record(1.0, track_ids=[track_id], class_ids=[0], centroids=[(8, 5)])
    row = s.to_rows()[0]
    assert row["total_count"] == 5
    assert row["flow_rate_vph"] == pytest.approx(300.0)  # 5 * 3600 / 60


def test_screenline_pcu_flow_rate_uses_overrides():
    s = ScreenlineAnalyzer(
        class_names=CLASS_NAMES,
        lines=[_line()],
        window_seconds=60,
        pcu_factors={"van": 1.4},
    )
    s.record(0.0, track_ids=[1], class_ids=[1], centroids=[(2, 5)])  # van
    s.record(1.0, track_ids=[1], class_ids=[1], centroids=[(8, 5)])
    row = s.to_rows()[0]
    assert row["pcu_weighted_count"] == pytest.approx(1.4)
    assert row["pcu_flow_rate_vph"] == pytest.approx(1.4 * 3600 / 60)


def test_screenline_separates_lines_and_directions():
    lines = [CountingLine("v", 5, 0, 5, 10), CountingLine("h", 0, 5, 10, 5)]
    s = ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=lines, window_seconds=60)
    # A diagonal move that crosses both lines at once.
    s.record(0.0, track_ids=[1], class_ids=[0], centroids=[(2, 2)])
    s.record(1.0, track_ids=[1], class_ids=[0], centroids=[(8, 8)])
    crossed_lines = {r["line"] for r in s.to_rows()}
    assert crossed_lines == {"v", "h"}
    assert s.total_crossings == 2


def test_screenline_writes_csv_and_plot(tmp_path):
    s = ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=[_line()], window_seconds=60)
    s.record(0.0, track_ids=[1], class_ids=[0], centroids=[(2, 5)])
    s.record(1.0, track_ids=[1], class_ids=[0], centroids=[(8, 5)])
    csv_path = tmp_path / "screenline.csv"
    png_path = tmp_path / "screenline.png"
    s.write_csv(str(csv_path))
    s.plot(str(png_path))
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    # one crossing in a 60s window -> 1 * 3600 / 60 == 60 veh/h
    assert rows[0]["flow_rate_vph"] == "60.0"
    assert png_path.stat().st_size > 0


def test_window_seconds_must_be_positive():
    with pytest.raises(ValueError):
        DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=0)
    with pytest.raises(ValueError):
        ScreenlineAnalyzer(class_names=CLASS_NAMES, lines=[_line()], window_seconds=-1)
