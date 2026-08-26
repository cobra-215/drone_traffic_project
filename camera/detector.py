import os
from ultralytics import YOLO


class TrafficDetector:

  def __init__(
      self,
      model_path="best.pt",
      confidence_threshold=0.4,
      target_classes=None,
  ):
    """Initialize custom-trained YOLO model for vehicle detection and tracking.

    model_path: Path to your custom trained weights (e.g., 'best.pt' or
    'runs/detect/train/weights/best.pt'). confidence_threshold: Minimum
    confidence score to accept a detection. target_classes: List of class IDs to
    filter (e.g., [0, 1]). Set to None to track ALL classes trained in your
    model.
    """
    if not os.path.exists(model_path):
      raise FileNotFoundError(
          f"Custom YOLO weights file not found at: {model_path}. "
          "Please check the path to your best.pt file."
      )

    print(f"Loading custom trained YOLO model from: {model_path}")
    self.model = YOLO(model_path)
    self.conf_threshold = confidence_threshold
    self.target_classes = target_classes  # None means keep all trained classes

  def process_frame(self, frame):
    """Run object detection and spatial tracking on a single frame using custom weights.

    Uses ByteTrack by default to maintain persistent IDs across frames.
    """
    tracking_kwargs = {
        "source": frame,
        "conf": self.conf_threshold,
        "persist": True,  # Maintain persistent object IDs across video frames
        "verbose": False,
    }

    # Only filter classes if target_classes is explicitly specified
    if self.target_classes is not None:
      tracking_kwargs["classes"] = self.target_classes

    # Execute tracking loop
    results = self.model.track(**tracking_kwargs)
    return results[0]