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
    Zone,
    ZoneAnalyzer,
    _point_in_polygon,
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
    with pytest.raises(ValueError):
        ZoneAnalyzer(class_names=CLASS_NAMES, zones=[_north()], window_seconds=0)


def test_density_record_still_accepts_three_arguments():
    # analyze_video.py now calls every analyzer with centroids; the
    # 3-argument form must keep working for existing callers.
    d = DensityAnalyzer(class_names=CLASS_NAMES, window_seconds=60)
    d.record(0.0, [1], [0])
    assert d.total_vehicles == 1


# --------------------------------------------------------------------------
# _point_in_polygon / Zone
# --------------------------------------------------------------------------

SQUARE = [(0, 0), (10, 0), (10, 10), (0, 10)]
# An L-shape: x in [0,10] y in [0,4], plus x in [0,4] y in [4,10].
L_SHAPE = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]


def test_point_inside_convex_polygon():
    assert _point_in_polygon((5, 5), SQUARE)


def test_point_outside_convex_polygon():
    assert not _point_in_polygon((15, 5), SQUARE)
    assert not _point_in_polygon((5, 15), SQUARE)
    assert not _point_in_polygon((-1, 5), SQUARE)


def test_concave_polygon_notch_is_outside():
    assert _point_in_polygon((2, 2), L_SHAPE)  # in the base
    assert _point_in_polygon((2, 8), L_SHAPE)  # in the upright
    assert not _point_in_polygon((8, 8), L_SHAPE)  # in the notch


def test_zone_requires_three_points():
    with pytest.raises(ValueError):
        Zone("degenerate", [(0, 0), (1, 1)])


def test_zone_contains_and_centroid():
    zone = Zone("square", SQUARE)
    assert zone.contains((5, 5))
    assert not zone.contains((50, 50))
    assert zone.centroid() == (5.0, 5.0)


# --------------------------------------------------------------------------
# ZoneAnalyzer
# --------------------------------------------------------------------------
#
# Two arms with a gap between them, standing in for an intersection:
#   north_arm  x in [0, 10]
#   (no zone)  x in (10, 20)   <- the intersection box itself
#   east_arm   x in [20, 30]


def _north():
    return Zone("north_arm", [(0, 0), (10, 0), (10, 10), (0, 10)])


def _east():
    return Zone("east_arm", [(20, 0), (30, 0), (30, 10), (20, 10)])


def _zone_analyzer(**kwargs):
    kwargs.setdefault("class_names", CLASS_NAMES)
    kwargs.setdefault("zones", [_north(), _east()])
    kwargs.setdefault("window_seconds", 60)
    return ZoneAnalyzer(**kwargs)


def test_zones_requires_a_zone():
    with pytest.raises(ValueError):
        ZoneAnalyzer(class_names=CLASS_NAMES, zones=[], window_seconds=60)


def test_zone_occupancy_mean_and_peak():
    z = _zone_analyzer()
    # frame 0: two cars in north, one in east
    z.record(0.0, [1, 2, 3], [0, 0, 0], [(1, 1), (2, 2), (21, 1)])
    # frame 1: four cars in north
    z.record(1.0, [1, 2, 4, 5], [0, 0, 0, 0], [(1, 1), (2, 2), (3, 3), (4, 4)])
    # frame 2: nobody in either zone
    z.record(2.0, [], [], [])

    rows = {row["zone"]: row for row in z.occupancy_rows()}

    assert rows["north_arm"]["frames"] == 3
    assert rows["north_arm"]["mean_vehicles_in_zone"] == pytest.approx(
        (2 + 4 + 0) / 3, abs=0.01
    )
    assert rows["north_arm"]["max_vehicles_in_zone"] == 4
    assert rows["east_arm"]["max_vehicles_in_zone"] == 1


def test_zone_occupancy_is_per_class():
    z = _zone_analyzer()
    z.record(0.0, [1, 2], [0, 3], [(1, 1), (2, 2)])  # a car and a bus
    row = z.occupancy_rows()[0]
    assert row["mean_car"] == pytest.approx(1.0)
    assert row["mean_bus"] == pytest.approx(1.0)
    # PCU: car 1.0 + bus 3.0
    assert row["mean_pcu_in_zone"] == pytest.approx(4.0)


def test_movement_detected_across_the_zoneless_gap():
    # The whole point of tracking the LAST named zone rather than the
    # previous frame's zone: a vehicle crossing an intersection spends
    # several frames inside no zone at all.
    z = _zone_analyzer()
    z.record(0.0, [1], [0], [(5, 5)])  # north_arm
    z.record(1.0, [1], [0], [(15, 5)])  # in the middle -- no zone
    z.record(2.0, [1], [0], [(16, 5)])  # still no zone
    z.record(3.0, [1], [0], [(25, 5)])  # east_arm

    assert z.total_movements == 1
    row = z.movement_rows()[0]
    assert row["movement"] == "north_arm->east_arm"
    assert row["from_zone"] == "north_arm"
    assert row["to_zone"] == "east_arm"
    assert row["count_car"] == 1


def test_movement_counted_once_while_the_vehicle_stays_in_the_new_zone():
    z = _zone_analyzer()
    z.record(0.0, [1], [0], [(5, 5)])
    z.record(1.0, [1], [0], [(25, 5)])
    z.record(2.0, [1], [0], [(26, 5)])
    z.record(3.0, [1], [0], [(27, 5)])
    assert z.total_movements == 1


def test_no_movement_when_the_vehicle_returns_to_the_same_zone():
    z = _zone_analyzer()
    z.record(0.0, [1], [0], [(5, 5)])  # north_arm
    z.record(1.0, [1], [0], [(15, 5)])  # gap
    z.record(2.0, [1], [0], [(6, 5)])  # back in north_arm
    assert z.total_movements == 0
    assert z.movement_rows() == []


def test_no_movement_for_a_vehicle_that_never_leaves_its_zone():
    z = _zone_analyzer()
    for index, x in enumerate((1, 2, 3, 4)):
        z.record(float(index), [1], [0], [(x, 5)])
    assert z.total_movements == 0


def test_movements_are_directional():
    z = _zone_analyzer()
    z.record(0.0, [1], [0], [(5, 5)])
    z.record(1.0, [1], [0], [(25, 5)])  # north -> east
    z.record(2.0, [2], [0], [(25, 5)])
    z.record(3.0, [2], [0], [(5, 5)])  # east -> north
    movements = {row["movement"] for row in z.movement_rows()}
    assert movements == {"north_arm->east_arm", "east_arm->north_arm"}


def test_movement_flow_rate_uses_q_equals_n_over_t():
    z = _zone_analyzer()
    for track_id in range(1, 6):  # 5 vehicles turn in the first minute
        z.record(0.0, [track_id], [0], [(5, 5)])
        z.record(1.0, [track_id], [0], [(25, 5)])
    row = z.movement_rows()[0]
    assert row["total_count"] == 5
    assert row["flow_rate_vph"] == pytest.approx(300.0)  # 5 * 3600 / 60


def test_movement_pcu_uses_overrides():
    z = _zone_analyzer(pcu_factors={"van": 1.4})
    z.record(0.0, [1], [1], [(5, 5)])  # van
    z.record(1.0, [1], [1], [(25, 5)])
    row = z.movement_rows()[0]
    assert row["pcu_weighted_count"] == pytest.approx(1.4)
    assert row["pcu_flow_rate_vph"] == pytest.approx(1.4 * 3600 / 60)


def test_overlapping_zones_resolve_to_the_first_listed():
    overlapping = Zone("outer", [(0, 0), (30, 0), (30, 10), (0, 10)])
    z = ZoneAnalyzer(
        class_names=CLASS_NAMES,
        zones=[_north(), overlapping],
        window_seconds=60,
    )
    z.record(0.0, [1], [0], [(5, 5)])  # inside both
    zones_seen = {row["zone"] for row in z.occupancy_rows()}
    assert zones_seen == {"north_arm"}


def test_zones_split_into_time_windows():
    z = _zone_analyzer(window_seconds=10)
    z.record(0.0, [1], [0], [(5, 5)])
    z.record(5.0, [1], [0], [(25, 5)])  # movement in window 0
    z.record(11.0, [2], [0], [(5, 5)])
    z.record(15.0, [2], [0], [(25, 5)])  # movement in window 1
    windows = [row["window"] for row in z.movement_rows()]
    assert windows == ["0:00-0:10", "0:10-0:20"]


def test_zones_ignores_frames_with_no_detections():
    z = _zone_analyzer()
    z.record(0.0, track_ids=None, class_ids=None, centroids=None)
    assert z.total_movements == 0
    assert z.occupancy_rows() == []


def test_zones_write_csv_and_plot_produce_both_files(tmp_path):
    z = _zone_analyzer()
    z.record(0.0, [1], [0], [(5, 5)])
    z.record(1.0, [1], [0], [(25, 5)])

    z.write_csv(str(tmp_path / "zones.csv"))
    z.plot(str(tmp_path / "zones.png"))

    occupancy_csv = tmp_path / "zones_occupancy.csv"
    movements_csv = tmp_path / "zones_movements.csv"
    assert occupancy_csv.exists()
    assert movements_csv.exists()
    assert (tmp_path / "zones_occupancy.png").stat().st_size > 0
    assert (tmp_path / "zones_movements.png").stat().st_size > 0

    with open(movements_csv, newline="") as f:
        rows = list(csv.DictReader(f))
    # one movement in a 60s window -> 1 * 3600 / 60 == 60 veh/h
    assert rows[0]["movement"] == "north_arm->east_arm"
    assert rows[0]["flow_rate_vph"] == "60.0"
