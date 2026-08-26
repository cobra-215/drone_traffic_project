"""
Post-flight traffic-volume analysis: reduces per-frame tracked vehicle
detections into a time-binned, class-weighted traffic report.

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
# (via the pcu_factors constructor argument) to match your model's real
# class list. Any class not found here defaults to PCU_DEFAULT (1.0) and
# is reported once via a printed warning, so a missing mapping is never
# silent.
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


class TrafficAnalyzer:
    """
    Accumulates unique-vehicle sightings across a video and reduces them
    to a time-binned traffic report.

    Each tracked vehicle (by its tracker-assigned ID) is counted exactly
    ONCE, at the first frame it is seen. Naive per-frame counting would
    count the same vehicle dozens of times as it crosses the frame,
    which is not a traffic count of anything meaningful.
    """

    def __init__(self, class_names, pcu_factors=None, bin_seconds=60.0):
        """
        class_names: dict[int, str], e.g. a YOLO model's `.names`.
        pcu_factors: optional dict overriding/extending PCU_FACTORS,
            keyed the same way (case-insensitive class name -> weight).
        bin_seconds: width of each time bin in the report, in seconds
            (e.g. 60.0 for "vehicles per minute").
        """

        if bin_seconds <= 0:
            raise ValueError("bin_seconds must be positive.")

        self.class_names = class_names
        self.bin_seconds = bin_seconds
        self._pcu_factors = {
            name.lower(): weight for name, weight in PCU_FACTORS.items()
        }
        if pcu_factors:
            self._pcu_factors.update(
                {name.lower(): weight for name, weight in pcu_factors.items()}
            )

        self._first_seen = {}  # track_id -> (frame_time_s, class_id)
        self._warned_classes = set()

    @property
    def total_vehicles(self):
        """Total number of unique tracked vehicles recorded so far."""
        return len(self._first_seen)

    def _pcu_for(self, class_name):
        weight = self._pcu_factors.get(class_name.lower())
        if weight is None:
            if class_name not in self._warned_classes:
                print(
                    f"TrafficAnalyzer: no PCU factor for class "
                    f"'{class_name}'; using default {PCU_DEFAULT}. Add it "
                    "to PCU_FACTORS, or pass pcu_factors=, to calibrate "
                    "this for your model's actual classes."
                )
                self._warned_classes.add(class_name)
            return PCU_DEFAULT
        return weight

    def record(self, frame_time_s, track_ids, class_ids):
        """
        Record one frame's tracked detections.

        track_ids / class_ids: parallel arrays as returned by
        camera.detections.get_detections() -- either may be None if
        nothing was detected/tracked in this frame.
        """

        if track_ids is None:
            return

        for track_id, class_id in zip(track_ids, class_ids):
            track_id = int(track_id)
            if track_id not in self._first_seen:
                self._first_seen[track_id] = (frame_time_s, int(class_id))

    def to_rows(self):
        """
        Reduce recorded sightings to one row per time bin, sorted by bin
        start time. Each row reports, for that bin:
          - a raw count per class actually seen,
          - the total raw vehicle count and a running cumulative total,
          - the PCU-weighted traffic volume,
          - the equivalent hourly flow rate (raw and PCU-weighted),
            using the standard traffic-flow-rate formula q = n / T
            (Highway Capacity Manual), normalised to vehicles/hour
            regardless of the configured bin width -- so a 5-vehicle
            count in a 60s bin is reported as a 300 veh/h flow rate,
            comparable across differently-sized bins.
        """

        if not self._first_seen:
            return []

        class_names_seen = sorted(
            {
                self.class_names.get(class_id, str(class_id))
                for _, class_id in self._first_seen.values()
            }
        )

        bins = defaultdict(lambda: defaultdict(int))
        for frame_time_s, class_id in self._first_seen.values():
            bin_index = int(frame_time_s // self.bin_seconds)
            class_name = self.class_names.get(class_id, str(class_id))
            bins[bin_index][class_name] += 1

        rows = []
        cumulative = 0
        for bin_index in sorted(bins):
            counts = bins[bin_index]
            total = sum(counts.values())
            pcu_total = sum(
                counts.get(name, 0) * self._pcu_for(name)
                for name in class_names_seen
            )
            cumulative += total

            bin_start_s = bin_index * self.bin_seconds
            row = {
                "bin_start_s": bin_start_s,
                "bin_end_s": bin_start_s + self.bin_seconds,
                "bin_start_hms": _format_hms(bin_start_s),
                **{
                    f"count_{name}": counts.get(name, 0)
                    for name in class_names_seen
                },
                "total_count": total,
                "cumulative_count": cumulative,
                "pcu_weighted_count": round(pcu_total, 2),
                "flow_rate_vph": round(total * 3600.0 / self.bin_seconds, 1),
                "pcu_flow_rate_vph": round(
                    pcu_total * 3600.0 / self.bin_seconds, 1
                ),
            }
            rows.append(row)

        return rows

    def write_csv(self, path):
        """Write the time-binned traffic report to a CSV file."""

        rows = self.to_rows()
        if not rows:
            print(f"TrafficAnalyzer: no vehicles recorded; not writing {path}.")
            return

        fieldnames = list(rows[0].keys())
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        print(f"TrafficAnalyzer: wrote {len(rows)} time bins to {path}")

    def plot(self, path, title="Traffic volume over time"):
        """
        Save a two-panel chart: per-class vehicle counts per time bin
        (stacked bars, directly answering "vehicles per time unit") on
        top, and the hourly-equivalent flow rate (raw and PCU-weighted)
        on the bottom.
        """

        import matplotlib

        matplotlib.use("Agg")  # headless-safe; no display assumed
        import matplotlib.pyplot as plt

        rows = self.to_rows()
        if not rows:
            print(f"TrafficAnalyzer: no vehicles recorded; not plotting {path}.")
            return

        class_names_seen = sorted(
            key[len("count_") :] for key in rows[0] if key.startswith("count_")
        )

        labels = [row["bin_start_hms"] for row in rows]

        fig, (ax_counts, ax_flow) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

        bottom = [0] * len(rows)
        for class_name in class_names_seen:
            values = [row[f"count_{class_name}"] for row in rows]
            ax_counts.bar(labels, values, bottom=bottom, label=class_name)
            bottom = [b + v for b, v in zip(bottom, values)]

        ax_counts.set_ylabel("Vehicles per bin")
        ax_counts.set_title(title)
        ax_counts.legend(loc="upper right", fontsize="small")

        flow = [row["flow_rate_vph"] for row in rows]
        pcu_flow = [row["pcu_flow_rate_vph"] for row in rows]
        ax_flow.plot(labels, flow, marker="o", label="Flow rate (veh/h)")
        ax_flow.plot(
            labels, pcu_flow, marker="o", label="PCU-weighted flow rate (PCU/h)"
        )
        ax_flow.set_ylabel("Equivalent hourly rate")
        ax_flow.set_xlabel(f"Time bin start (bin width = {self.bin_seconds:.0f}s)")
        ax_flow.legend(loc="upper right", fontsize="small")
        ax_flow.tick_params(axis="x", rotation=45)

        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)

        print(f"TrafficAnalyzer: wrote plot to {path}")


def _format_hms(seconds):
    seconds = int(seconds)
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"
