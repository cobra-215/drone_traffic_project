import time
import cv2
from .detector import TrafficDetector
from .processor import TrafficProcessor


class VisionPipeline:

    def __init__(
            self,
            video_source,
            model_path='yolov8n.pt',
            roi_polygons=None,
            output_path=None,
    ):
        self.video_source = video_source
        self.output_path = output_path
        self.detector = TrafficDetector(model_path=model_path)
        self.processor = TrafficProcessor(roi_polygons=roi_polygons)

    def run(self, show_preview=True):
        """Execute video analysis pipeline."""
        cap = cv2.VideoCapture(self.video_source)
        if not cap.isOpened():
            raise FileNotFoundError(
                f"Could not open video source: {self.video_source}"
            )

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30

        writer = None
        if self.output_path:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(self.output_path, fourcc, fps, (width, height))

        print(f"Starting analysis on video: {self.video_source} ({width}x{height} @ {fps}fps)")

        start_time = time.time()
        frame_count = 0

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            # 1. Detection and Tracking
            tracking_results = self.detector.process_frame(frame)

            # 2. ROI Counting and Annotation
            annotated_frame, counts = self.processor.process_tracks(
                tracking_results, frame
            )

            # 3. Output handling
            if writer:
                writer.write(annotated_frame)

            if show_preview:
                cv2.imshow("Drone Traffic Analysis", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0

        print("\n" + "=" * 50)
        print("ANALYSIS COMPLETE")
        print(f"Processed {frame_count} frames in {elapsed:.2f}s ({avg_fps:.1f} FPS)")
        print(f"Total Unique Vehicles Tracked: {len(self.processor.counted_ids)}")
        print(f"Counts per lane: {self.processor.lane_counts}")
        print("=" * 50)

        return self.processor.lane_counts