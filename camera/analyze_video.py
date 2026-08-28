"""
Offline traffic analysis CLI.

Runs a trained YOLO model (standard or OBB) over a recorded video with
tracking enabled, using ALL of the model's trained classes, and produces
a report split into fixed-length time windows, in one of two modes:

  --mode density     (default) road occupancy / congestion: mean and
                     peak vehicles in frame per time window, per class.
                     The right measure for a wide-area drone-hover view.

  --mode screenline  actual traffic flow: vehicles/hour across one or
                     more counting lines you define with --line. A
                     vehicle is counted once, when its track crosses a
                     line. Requires at least one --line.

See camera/traffic_metrics.py for the methodology behind each.

This is entirely offline post-flight analysis. It is never imported by
flight/ or mission/, and must not run on the Raspberry Pi flight process
-- see requirements-vision.txt, installed separately from the
flight-critical requirements.txt.

Usage:
    python -m camera.analyze_video \\
        --model models/best.pt --video clip.mp4 --window-seconds 10

    python -m camera.analyze_video \\
        --model models/best.pt --video clip.mp4 --mode screenline \\
        --line northbound:640,0,640,720 --line eastbound:0,360,1280,360 \\
        --pcu van=1.4
"""

import argparse
import os

import cv2

from .detections import get_detections
from .detector import TrafficDetector
from .traffic_metrics import (
    CountingLine,
    DensityAnalyzer,
    ScreenlineAnalyzer,
)


def parse_pcu_overrides(pairs):
    """
    Turn a list of "class=weight" strings (from --pcu) into a
    {class_name: float} dict for the analyzer's pcu_factors argument.
    """

    overrides = {}
    for pair in pairs or []:
        if "=" not in pair:
            raise ValueError(
                f"--pcu expects NAME=WEIGHT (e.g. van=1.4), got: {pair!r}"
            )
        name, _, weight = pair.partition("=")
        name = name.strip()
        if not name:
            raise ValueError(f"--pcu entry has an empty class name: {pair!r}")
        try:
            overrides[name] = float(weight)
        except ValueError:
            raise ValueError(
                f"--pcu weight for {name!r} is not a number: {weight!r}"
            )
    return overrides


def parse_lines(specs):
    """
    Turn a list of "[NAME:]X1,Y1,X2,Y2" strings (from --line) into a list
    of CountingLine. Pixel coordinates, origin top-left. Unnamed lines
    are auto-named line1, line2, ...
    """

    lines = []
    for index, spec in enumerate(specs or [], start=1):
        name = f"line{index}"
        body = spec
        if ":" in spec:
            name, _, body = spec.partition(":")
            name = name.strip()
        parts = [p.strip() for p in body.split(",")]
        if len(parts) != 4:
            raise ValueError(
                f"--line expects [NAME:]X1,Y1,X2,Y2, got: {spec!r}"
            )
        try:
            x1, y1, x2, y2 = (float(p) for p in parts)
        except ValueError:
            raise ValueError(
                f"--line coordinates must be numbers, got: {spec!r}"
            )
        lines.append(CountingLine(name, x1, y1, x2, y2))
    return lines


def _centroids_from_xyxy(xyxy):
    """(N, 4) xyxy boxes -> (N, 2) centroids. None passes through."""
    if xyxy is None:
        return None
    return [
        ((x1 + x2) / 2.0, (y1 + y2) / 2.0) for x1, y1, x2, y2 in xyxy
    ]


def build_analyzer(mode, class_names, window_seconds, pcu_overrides, lines):
    if mode == "density":
        return DensityAnalyzer(
            class_names=class_names,
            window_seconds=window_seconds,
            pcu_factors=pcu_overrides or None,
        )
    if mode == "screenline":
        return ScreenlineAnalyzer(
            class_names=class_names,
            lines=lines,
            window_seconds=window_seconds,
            pcu_factors=pcu_overrides or None,
        )
    raise ValueError(f"Unknown mode: {mode!r}")


def analyze(
    model_path,
    video_path,
    output_dir,
    window_seconds,
    confidence_threshold,
    mode="density",
    pcu_overrides=None,
    lines=None,
):
    os.makedirs(output_dir, exist_ok=True)

    detector = TrafficDetector(
        model_path=model_path,
        confidence_threshold=confidence_threshold,
        target_classes=None,  # track every class the model was trained on
    )

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open video source: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    analyzer = build_analyzer(
        mode, detector.model.names, window_seconds, pcu_overrides, lines
    )

    print(f"Mode: {mode}")
    print("Model classes and the PCU weight each will be counted with:")
    for class_name, weight in analyzer.pcu_table().items():
        print(f"  {class_name}: {weight}")

    frame_index = 0
    print(f"Analyzing {video_path} at {fps:.1f} fps...")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            results = detector.process_frame(frame)
            xyxy, track_ids, class_ids, _confs = get_detections(results)
            frame_time_s = frame_index / fps

            if mode == "screenline":
                analyzer.record(
                    frame_time_s,
                    track_ids,
                    class_ids,
                    _centroids_from_xyxy(xyxy),
                )
            else:
                analyzer.record(frame_time_s, track_ids, class_ids)

            frame_index += 1
            if frame_index % 100 == 0:
                print(f"  processed {frame_index} frames ({frame_time_s:.1f}s)...")
    finally:
        cap.release()

    if mode == "screenline":
        print(
            f"Done: {frame_index} frames processed, "
            f"{analyzer.total_crossings} line crossings counted."
        )
    else:
        print(
            f"Done: {frame_index} frames processed, "
            f"{analyzer.total_vehicles} distinct vehicles seen."
        )

    csv_path = os.path.join(output_dir, f"{mode}_report.csv")
    plot_path = os.path.join(output_dir, f"{mode}_report.png")
    analyzer.write_csv(csv_path)
    analyzer.plot(plot_path)

    return analyzer


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--model", required=True, help="Path to trained YOLO weights (.pt)."
    )
    parser.add_argument(
        "--video", required=True, help="Path to the recorded video to analyze."
    )
    parser.add_argument(
        "--mode",
        choices=["density", "screenline"],
        default="density",
        help=(
            "density (default): occupancy/congestion, no flow rate. "
            "screenline: vehicles/hour across --line counting lines."
        ),
    )
    parser.add_argument(
        "--line",
        metavar="[NAME:]X1,Y1,X2,Y2",
        action="append",
        default=[],
        help=(
            "A counting line in pixel coordinates (origin top-left), for "
            "--mode screenline. Repeat for multiple lines, e.g. "
            "--line northbound:640,0,640,720 . Required with --mode screenline."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default="./traffic_report",
        help="Directory for the CSV and plot output (default: ./traffic_report).",
    )
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=60.0,
        help="Length of each time window in the report, in seconds (default: 60).",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.4,
        help="Minimum detection confidence (default: 0.4).",
    )
    parser.add_argument(
        "--pcu",
        metavar="CLASS=WEIGHT",
        nargs="*",
        default=[],
        help=(
            "Override the PCU weight for one or more classes, e.g. "
            "--pcu van=1.4 bus=2.2 . Classes not overridden keep their "
            "default from camera/traffic_metrics.py:PCU_FACTORS."
        ),
    )
    args = parser.parse_args()

    lines = parse_lines(args.line)
    if args.mode == "screenline" and not lines:
        parser.error("--mode screenline requires at least one --line")

    analyze(
        model_path=args.model,
        video_path=args.video,
        output_dir=args.output_dir,
        window_seconds=args.window_seconds,
        confidence_threshold=args.confidence,
        mode=args.mode,
        pcu_overrides=parse_pcu_overrides(args.pcu),
        lines=lines,
    )
