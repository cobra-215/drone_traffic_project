"""
Tests for camera/traffic_metrics.py. Requires requirements-vision.txt
(matplotlib) -- run separately from the flight-critical suite:
`pytest tests/vision`.
"""

import csv

import pytest

from camera.traffic_metrics import PCU_DEFAULT, PCU_FACTORS, TrafficAnalyzer

CLASS_NAMES = {0: "car", 1: "truck", 2: "motorcycle"}


def test_each_track_id_counted_once():
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    # Same track_id=1 seen in 5 consecutive frames -- must count once,
    # not five times. This is the entire point of first-seen tracking.
    for t in [0.0, 1.0, 2.0, 3.0, 4.0]:
        analyzer.record(t, track_ids=[1], class_ids=[0])
    assert analyzer.total_vehicles == 1


def test_binning_groups_by_first_seen_time():
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    analyzer.record(5.0, track_ids=[1], class_ids=[0])  # bin 0
    analyzer.record(65.0, track_ids=[2], class_ids=[0])  # bin 1
    rows = analyzer.to_rows()
    assert len(rows) == 2
    assert rows[0]["bin_start_s"] == 0
    assert rows[1]["bin_start_s"] == 60


def test_pcu_weighting_uses_class_specific_factor():
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    analyzer.record(0.0, track_ids=[1], class_ids=[1])  # truck
    rows = analyzer.to_rows()
    assert rows[0]["pcu_weighted_count"] == pytest.approx(PCU_FACTORS["truck"])


def test_unmapped_class_uses_default_and_warns(capsys):
    analyzer = TrafficAnalyzer(class_names={0: "spaceship"}, bin_seconds=60)
    analyzer.record(0.0, track_ids=[1], class_ids=[0])
    rows = analyzer.to_rows()
    assert rows[0]["pcu_weighted_count"] == pytest.approx(PCU_DEFAULT)
    assert "spaceship" in capsys.readouterr().out


def test_flow_rate_normalises_to_vehicles_per_hour():
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    for track_id in range(5):
        analyzer.record(0.0, track_ids=[track_id], class_ids=[0])
    rows = analyzer.to_rows()
    # 5 vehicles in a 60s bin == 300 vehicles/hour equivalent (q = n/T).
    assert rows[0]["flow_rate_vph"] == pytest.approx(300.0)


def test_custom_pcu_factors_override_defaults():
    analyzer = TrafficAnalyzer(
        class_names=CLASS_NAMES, bin_seconds=60, pcu_factors={"car": 2.0}
    )
    analyzer.record(0.0, track_ids=[1], class_ids=[0])  # car
    rows = analyzer.to_rows()
    assert rows[0]["pcu_weighted_count"] == pytest.approx(2.0)


def test_to_rows_empty_when_nothing_recorded():
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    assert analyzer.to_rows() == []


def test_record_ignores_none_track_ids():
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    analyzer.record(0.0, track_ids=None, class_ids=None)  # must not raise
    assert analyzer.total_vehicles == 0


def test_write_csv_produces_expected_columns(tmp_path):
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    analyzer.record(0.0, track_ids=[1], class_ids=[0])
    analyzer.record(0.0, track_ids=[2], class_ids=[1])

    csv_path = tmp_path / "report.csv"
    analyzer.write_csv(str(csv_path))

    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

    assert len(rows) == 1
    assert "count_car" in rows[0]
    assert "count_truck" in rows[0]
    assert rows[0]["total_count"] == "2"


def test_write_csv_skips_when_no_data(tmp_path):
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    csv_path = tmp_path / "report.csv"
    analyzer.write_csv(str(csv_path))
    assert not csv_path.exists()


def test_bin_seconds_must_be_positive():
    with pytest.raises(ValueError):
        TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=0)


def test_plot_produces_a_file(tmp_path):
    analyzer = TrafficAnalyzer(class_names=CLASS_NAMES, bin_seconds=60)
    analyzer.record(0.0, track_ids=[1], class_ids=[0])
    plot_path = tmp_path / "report.png"
    analyzer.plot(str(plot_path))
    assert plot_path.exists()
    assert plot_path.stat().st_size > 0
