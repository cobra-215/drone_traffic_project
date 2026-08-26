"""
Offline traffic-volume analysis CLI.

Runs a trained YOLO model (standard or OBB) over a recorded video with
tracking enabled, using ALL of the model's trained classes, and produces
a time-binned traffic report: a CSV of vehicle counts / PCU-weighted
volume / flow rate per time bin, and a PNG plot of the same. See
camera/traffic_metrics.py for the counting and flow-rate methodology.

This is entirely offline post-flight analysis. It is never imported by
flight/ or mission/, and must not run on the Raspberry Pi flight process
-- see requirements-vision.txt, which is installed separately from the
flight-critical requirements.txt.

Usage:
    python -m camera.analyze_video \\
        --model /path/to/best.pt \\
        --video /path/to/recording.mp4 \\
        --output-dir ./traffic_report \\
        --bin-seconds 60
"""

import argparse
import os

import cv2

from .detections import get_detections
from .detector import TrafficDetector
from .traffic_metrics import TrafficAnalyzer


def analyze(model_path, video_path, output_dir, bin_seconds, confidence_threshold):
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

    analyzer = TrafficAnalyzer(
        class_names=detector.model.names,
        bin_seconds=bin_seconds,
    )

    frame_index = 0
    print(f"Analyzing {video_path} at {fps:.1f} fps...")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            results = detector.process_frame(frame)
            _xyxy, track_ids, class_ids, _confs = get_detections(results)

            frame_time_s = frame_index / fps
            analyzer.record(frame_time_s, track_ids, class_ids)

            frame_index += 1
            if frame_index % 100 == 0:
                print(f"  processed {frame_index} frames ({frame_time_s:.1f}s)...")
    finally:
        cap.release()

    print(
        f"Done: {frame_index} frames processed, "
        f"{analyzer.total_vehicles} unique vehicles tracked."
    )

    csv_path = os.path.join(output_dir, "traffic_report.csv")
    plot_path = os.path.join(output_dir, "traffic_report.png")
    analyzer.write_csv(csv_path)
    analyzer.plot(plot_path, title=f"Traffic volume -- {os.path.basename(video_path)}")

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
        "--output-dir",
        default="./traffic_report",
        help="Directory for the CSV and plot output (default: ./traffic_report).",
    )
    parser.add_argument(
        "--bin-seconds",
        type=float,
        default=60.0,
        help="Time bin width for the report, in seconds (default: 60).",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.4,
        help="Minimum detection confidence (default: 0.4).",
    )
    args = parser.parse_args()

    analyze(
        model_path=args.model,
        video_path=args.video,
        output_dir=args.output_dir,
        bin_seconds=args.bin_seconds,
        confidence_threshold=args.confidence,
    )
