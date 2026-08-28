"""
Post-flight traffic analysis: reduces per-frame tracked vehicle
detections into a report split into fixed-length time windows (e.g. one
row per 10 seconds of video), in one of two modes.

- DensityAnalyzer (mode="density"): road occupancy / congestion. How
  many vehicles are visible per frame, on average and at peak, per
  class. The right measure for a wide-area (drone hover) view where many
  vehicles are in frame at once. Does NOT report a vehicles/hour flow
  rate -- "vehicles that appeared somewhere in a wide field of view" is
  not a cross-section count and annualising it is meaningless.

- ScreenlineAnalyzer (mode="screenline"): actual traffic FLOW across one
  or more user-defined counting lines. A vehicle is counted once, when
  its tracked centroid crosses a line between two frames, attributed to
  a direction ("+"/"-") and to the vehicle's class. Because this IS a
  cross-section count, the standard flow-rate formula q = n / T applies
  and the reported vehicles/hour is meaningful.

Part of the offline vision/analysis stack -- see requirements-vision.txt.
Never imported by flight/ or mission/, and must never run on the
Raspberry Pi's flight process.
"""

import csv
from collections import defaultdict


# Passenger Car Unit (PCU) equivalency factors: the standard traffic-
# engineering method for converting a mixed stream of vehicle classes
# into one comparable "traffic volume" figure, since a bus or truck
# occupies far more road capacity than a motorcycle. Values below follow
# IRC:106-1990 ("Guidelines for Capacity of Urban Roads in Plain Areas",
# Indian Roads Congress), a widely cited reference for urban mixed-
# traffic PCU values; the US Highway Capacity Manual (HCM) uses the same
# passenger-car-equivalent concept with its own tabulated values for
# freeway/highway contexts. Keys are matched case-insensitively against
# the trained model's class names.
#
# This is almost certainly NOT a complete or correct mapping for a
# custom-trained model's actual class names -- extend or override it
# (via the pcu_factors argument) to match your model's real class list.
# Any class not found here defaults to PCU_DEFAULT (1.0) and is reported
# once via a printed warning, so a missing mapping is never silent.
PCU_FACTORS = {
    "car": 1.0,
    "van": 1.0,
    "jeep": 1.0,
    "taxi": 1.0,
    "small vehicle": 1.0,
    "small-vehicle": 1.0,
    "large vehicle": 3.0,
    "large-vehicle": 3.0,
    "motorcycle": 0.5,
    "motorbike": 0.5,
    "scooter": 0.5,
    "bicycle": 0.5,
    "auto-rickshaw": 0.75,
    "rickshaw": 0.75,
    "tricycle": 0.75,
    "bus": 3.0,
    "truck": 3.0,
    "trailer": 3.0,
    "lorry": 3.0,
}
PCU_DEFAULT = 1.0


class PcuResolver:
    """
    Resolves a vehicle class name to its PCU weight, warning once per
    unknown class so a missing mapping surfaces instead of silently
    being counted at the default weight.
    """

    def __init__(self, pcu_factors=None):
        self._factors = {
            name.lower(): weight for name, weight in PCU_FACTORS.items()
        }
        if pcu_factors:
            self._factors.update(
                {name.lower(): weight for name, weight in pcu_factors.items()}
            )
        self._warned = set()

    def factor_for(self, class_name):
        weight = self._factors.get(class_name.lower())
        if weight is None:
            if class_name not in self._warned:
                print(
                    f"PCU: no factor for class '{class_name}'; using default "
                    f"{PCU_DEFAULT}. Add it to PCU_FACTORS, or pass "
                    "pcu_factors=, to calibrate this for your model's classes."
                )
                self._warned.add(class_name)
            return PCU_DEFAULT
        return weight

    def table(self, class_names):
        """{class_name: weight} for every class in a YOLO `.names` dict."""
        return {
            name: self.factor_for(name)
            for name in sorted(set(class_names.values()))
        }


class CountingLine:
    """
    A named line segment in pixel coordinates, from point A to point B,
    used as a traffic counting screen-line.
    """

    def __init__(self, name, x1, y1, x2, y2):
        if (x1, y1) == (x2, y2):
            raise ValueError(f"CountingLine {name!r} is a single point.")
        self.name = name
        self.a = (float(x1), float(y1))
        self.b = (float(x2), float(y2))

    def crossing_direction(self, prev_pt, curr_pt):
        """
        "+" , "-", or None.

        None  -- the motion prev_pt -> curr_pt does not cross this line.
        "+"   -- the 2D cross product of the line vector (A->B) with the
                 vehicle's motion vector is positive.
        "-"   -- it is negative (the opposite way across the line).

        Which physical heading "+" corresponds to depends on how you drew
        the line (A->B order and orientation); one test run makes it
        obvious, and the report keeps the two directions separate
        regardless, so you can label them afterwards.
        """

        if not _segments_intersect(self.a, self.b, prev_pt, curr_pt):
            return None
        line_x, line_y = self.b[0] - self.a[0], self.b[1] - self.a[1]
        move_x, move_y = curr_pt[0] - prev_pt[0], curr_pt[1] - prev_pt[1]
        return "+" if (line_x * move_y - line_y * move_x) > 0 else "-"


class DensityAnalyzer:
    """
    Road occupancy / congestion from per-frame detections, one row per
    fixed-length time window.

    Call record() once per frame. For each window the report gives the
    mean and peak number of vehicles visible per frame (overall and per
    class), the mean PCU-weighted occupancy, and how many DISTINCT
    vehicles (by track ID) were first seen during the window.
    """

    def __init__(self, class_names, pcu_factors=None, window_seconds=60.0):
        if window_seconds <= 0:
            raise ValueError("window_seconds must be positive.")

        self.class_names = class_names
        self.window_seconds = window_seconds
        self.pcu = PcuResolver(pcu_factors)
        self._seen_ids = set()
        self._windows = defaultdict(_new_density_window)

    @property
    def total_vehicles(self):
        """Distinct tracked vehicles seen across the whole video."""
        return len(self._seen_ids)

    def pcu_table(self):
        return self.pcu.table(self.class_names)

    def record(self, frame_time_s, track_ids, class_ids):
        window_index = int(frame_time_s // self.window_seconds)
        w = self._windows[window_index]
        w["n_frames"] += 1

        if track_ids is None:
            return

        per_class = defaultdict(int)
        for track_id, class_id in zip(track_ids, class_ids):
            track_id = int(track_id)
            class_name = _resolve_class_name(self.class_names, class_id)
            per_class[class_name] += 1
            if track_id not in self._seen_ids:
                self._seen_ids.add(track_id)
                w["distinct_ids"].add(track_id)
                w["distinct_class_counts"][class_name] += 1

        total = sum(per_class.values())
        w["total_sum"] += total
        w["total_max"] = max(w["total_max"], total)
        for class_name, count in per_class.items():
            w["class_sum"][class_name] += count

    def to_rows(self):
        if not self._windows:
            return []

        class_names_seen = sorted(
            {
                name
                for w in self._windows.values()
                for name in list(w["class_sum"])
                + list(w["distinct_class_counts"])
            }
        )

        rows = []
        for window_index in sorted(self._windows):
            w = self._windows[window_index]
            n = w["n_frames"] or 1
            mean_pcu = sum(
                (w["class_sum"].get(name, 0) / n) * self.pcu.factor_for(name)
                for name in class_names_seen
            )
            start_s = window_index * self.window_seconds
            end_s = start_s + self.window_seconds
            row = {
                "window": _format_window(start_s, end_s),
                "window_start_s": start_s,
                "window_end_s": end_s,
                "frames": w["n_frames"],
                **{
                    f"mean_{name}": round(w["class_sum"].get(name, 0) / n, 2)
                    for name in class_names_seen
                },
                "mean_vehicles_in_frame": round(w["total_sum"] / n, 2),
                "max_vehicles_in_frame": w["total_max"],
                "mean_pcu_in_frame": round(mean_pcu, 2),
                **{
                    f"distinct_{name}": w["distinct_class_counts"].get(name, 0)
                    for name in class_names_seen
                },
                "distinct_vehicles": len(w["distinct_ids"]),
            }
            rows.append(row)
        return rows

    def write_csv(self, path):
        _write_rows_csv(self.to_rows(), path, "DensityAnalyzer")

    def plot(self, path, title="Road occupancy over time"):
        rows = self.to_rows()
        if not rows:
            print(f"DensityAnalyzer: no data; not plotting {path}.")
            return

        plt = _plt()
        class_names_seen = sorted(
            key[len("mean_") :]
            for key in rows[0]
            if key.startswith("mean_")
            and key not in ("mean_vehicles_in_frame", "mean_pcu_in_frame")
        )
        labels = [row["window"] for row in rows]
        x = list(range(len(rows)))

        fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        _grouped_bars(
            ax_top,
            x,
            [
                (name, [row[f"mean_{name}"] for row in rows])
                for name in class_names_seen
            ],
        )
        ax_top.set_ylabel("Mean vehicles in frame")
        ax_top.set_title(title)
        ax_top.legend(loc="upper right", fontsize="small")

        ax_bot.plot(
            x,
            [row["max_vehicles_in_frame"] for row in rows],
            marker="o",
            label="Peak vehicles in frame",
        )
        ax_bot.plot(
            x,
            [row["distinct_vehicles"] for row in rows],
            marker="o",
            label="New vehicles seen in window",
        )
        ax_bot.set_ylabel("Vehicles")
        ax_bot.set_xlabel("Time into video (mm:ss)")
        ax_bot.set_xticks(x)
        ax_bot.set_xticklabels(labels, rotation=45, ha="right")
        ax_bot.legend(loc="upper right", fontsize="small")

        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"DensityAnalyzer: wrote plot to {path}")


class ScreenlineAnalyzer:
    """
    Traffic flow across one or more CountingLines, one row per
    (time window, line, direction).

    Call record() once per frame with each tracked vehicle's centroid. A
    vehicle is counted when the segment between its previous and current
    centroid crosses a line; the crossing is attributed to that line, a
    direction ("+"/"-"), and the vehicle's class. q = n / T then gives a
    meaningful vehicles/hour, since this is a genuine cross-section count.
    """

    def __init__(
        self, class_names, lines, pcu_factors=None, window_seconds=60.0
    ):
        if window_seconds <= 0:
            raise ValueError("window_seconds must be positive.")
        if not lines:
            raise ValueError(
                "ScreenlineAnalyzer needs at least one CountingLine."
            )

        self.class_names = class_names
        self.lines = list(lines)
        self.window_seconds = window_seconds
        self.pcu = PcuResolver(pcu_factors)
        self._prev_centroid = {}  # track_id -> (x, y)
        # (window_index, line_name, direction) -> {class_name: count}
        self._counts = defaultdict(lambda: defaultdict(int))
        self._total_crossings = 0

    @property
    def total_crossings(self):
        return self._total_crossings

    def pcu_table(self):
        return self.pcu.table(self.class_names)

    def record(self, frame_time_s, track_ids, class_ids, centroids):
        if track_ids is None:
            return

        window_index = int(frame_time_s // self.window_seconds)
        for track_id, class_id, centroid in zip(track_ids, class_ids, centroids):
            track_id = int(track_id)
            curr = (float(centroid[0]), float(centroid[1]))
            prev = self._prev_centroid.get(track_id)
            self._prev_centroid[track_id] = curr
            if prev is None:
                continue

            class_name = _resolve_class_name(self.class_names, class_id)
            for line in self.lines:
                direction = line.crossing_direction(prev, curr)
                if direction is not None:
                    self._counts[(window_index, line.name, direction)][
                        class_name
                    ] += 1
                    self._total_crossings += 1

    def to_rows(self):
        if not self._counts:
            return []

        class_names_seen = sorted(
            {
                name
                for class_counts in self._counts.values()
                for name in class_counts
            }
        )

        rows = []
        for key in sorted(self._counts):
            window_index, line_name, direction = key
            counts = self._counts[key]
            total = sum(counts.values())
            pcu_total = sum(
                counts.get(name, 0) * self.pcu.factor_for(name)
                for name in class_names_seen
            )
            start_s = window_index * self.window_seconds
            end_s = start_s + self.window_seconds
            rows.append(
                {
                    "window": _format_window(start_s, end_s),
                    "window_start_s": start_s,
                    "window_end_s": end_s,
                    "line": line_name,
                    "direction": direction,
                    **{
                        f"count_{name}": counts.get(name, 0)
                        for name in class_names_seen
                    },
                    "total_count": total,
                    "pcu_weighted_count": round(pcu_total, 2),
                    "flow_rate_vph": round(
                        total * 3600.0 / self.window_seconds, 1
                    ),
                    "pcu_flow_rate_vph": round(
                        pcu_total * 3600.0 / self.window_seconds, 1
                    ),
                }
            )
        return rows

    def write_csv(self, path):
        _write_rows_csv(self.to_rows(), path, "ScreenlineAnalyzer")

    def plot(self, path, title="Traffic flow across counting lines"):
        rows = self.to_rows()
        if not rows:
            print(f"ScreenlineAnalyzer: no crossings; not plotting {path}.")
            return

        plt = _plt()
        class_names_seen = sorted(
            key[len("count_") :] for key in rows[0] if key.startswith("count_")
        )

        # Top panel: total crossings per time window (all lines and
        # directions summed), one grouped bar per class.
        window_labels = _sorted_windows(rows)
        per_window = defaultdict(lambda: defaultdict(int))
        for row in rows:
            for class_name in class_names_seen:
                per_window[row["window"]][class_name] += row[
                    f"count_{class_name}"
                ]
        x = list(range(len(window_labels)))

        fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        _grouped_bars(
            ax_top,
            x,
            [
                (
                    name,
                    [per_window[label][name] for label in window_labels],
                )
                for name in class_names_seen
            ],
        )
        ax_top.set_ylabel("Line crossings")
        ax_top.set_title(title)
        ax_top.legend(loc="upper right", fontsize="small")

        # Bottom panel: flow rate per (line, direction) series.
        series = defaultdict(dict)
        for row in rows:
            series[(row["line"], row["direction"])][row["window"]] = row[
                "flow_rate_vph"
            ]
        for (line_name, direction), by_window in sorted(series.items()):
            ax_bot.plot(
                x,
                [by_window.get(label, 0) for label in window_labels],
                marker="o",
                label=f"{line_name} {direction}",
            )
        ax_bot.set_ylabel("Flow rate (veh/h)")
        ax_bot.set_xlabel("Time into video (mm:ss)")
        ax_bot.set_xticks(x)
        ax_bot.set_xticklabels(window_labels, rotation=45, ha="right")
        ax_bot.legend(loc="upper right", fontsize="small")

        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"ScreenlineAnalyzer: wrote plot to {path}")


def _new_density_window():
    return {
        "n_frames": 0,
        "total_sum": 0,
        "total_max": 0,
        "class_sum": defaultdict(int),
        "distinct_ids": set(),
        "distinct_class_counts": defaultdict(int),
    }


def _resolve_class_name(class_names, class_id):
    return class_names.get(int(class_id), str(int(class_id)))


def _sorted_windows(rows):
    """Distinct window labels from screenline rows, in chronological order."""
    seen = {}
    for row in rows:
        seen.setdefault(row["window_start_s"], row["window"])
    return [seen[start] for start in sorted(seen)]


def _grouped_bars(ax, x, series):
    """
    Draw one cluster of side-by-side bars per x position: `series` is a
    list of (label, [values]) with one value per x position, rendered as
    one bar colour per label.
    """
    n = len(series)
    if n == 0:
        return
    group_width = 0.8
    bar_width = group_width / n
    for i, (label, values) in enumerate(series):
        offset = (i - (n - 1) / 2) * bar_width
        ax.bar([xi + offset for xi in x], values, bar_width, label=label)


def _write_rows_csv(rows, path, who):
    if not rows:
        print(f"{who}: no data; not writing {path}.")
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"{who}: wrote {len(rows)} rows to {path}")


def _plt():
    import matplotlib

    matplotlib.use("Agg")  # headless-safe; no display assumed
    import matplotlib.pyplot as plt

    return plt


def _cross(o, a, b):
    """
    Cross product (a - o) x (b - o). Its sign tells which side of the
    directed line o->a the point b lies on.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_intersect(p1, p2, p3, p4):
    """
    True if segment p1-p2 properly intersects segment p3-p4. Collinear
    and endpoint-touching cases are treated as non-crossings, which is
    fine for vehicle-centroid motion between frames.
    """
    d1 = _cross(p3, p4, p1)
    d2 = _cross(p3, p4, p2)
    d3 = _cross(p1, p2, p3)
    d4 = _cross(p1, p2, p4)
    return ((d1 > 0 > d2) or (d1 < 0 < d2)) and (
        (d3 > 0 > d4) or (d3 < 0 < d4)
    )


def _format_clock(seconds):
    """Seconds -> 'm:ss', or 'h:mm:ss' once past an hour."""
    seconds = int(seconds)
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes}:{secs:02d}"


def _format_window(start_s, end_s):
    """A time window as an inclusive-looking range label, e.g. '0:10-0:20'."""
    return f"{_format_clock(start_s)}-{_format_clock(end_s)}"
