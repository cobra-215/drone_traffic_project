import cv2
import numpy as np

from .detections import get_detections


class TrafficProcessor:

  def __init__(self, roi_polygons=None):
    """roi_polygons: Dict mapping lane names to polygon coordinates [[x1, y1],

    [x2, y2], ...]
    """
    self.roi_polygons = roi_polygons or {}
    self.counted_ids = set()
    self.lane_counts = {lane: 0 for lane in self.roi_polygons.keys()}

  def is_point_in_polygon(self, point, polygon):
    """Check if a coordinate (x, y) falls inside a lane ROI polygon."""
    return (
        cv2.pointPolygonTest(
            np.array(polygon, np.int32), (float(point[0]), float(point[1])), False
        )
        >= 0
    )

  def process_tracks(self, tracking_results, frame):
    """Extract vehicle centroids, check ROI intersections, and annotate frame."""
    # get_detections() reads whichever of .boxes / .obb the model
    # populated -- standard detectors use .boxes, OBB (oriented
    # bounding box) detectors use .obb instead, and .boxes is always
    # None for an OBB model. Both expose the same xyxy/id/cls/conf
    # attribute names, so this is the only branch needed.
    boxes, track_ids, class_ids, _confs = get_detections(tracking_results)
    if boxes is None:
      return frame, self.lane_counts

    for box, track_id, cls_id in zip(boxes, track_ids, class_ids):
      x1, y1, x2, y2 = box
      # Calculate centroid of bounding box
      cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

      # Check which lane polygon contains the vehicle centroid
      for lane_name, polygon in self.roi_polygons.items():
        if self.is_point_in_polygon((cx, cy), polygon):
          if track_id not in self.counted_ids:
            self.counted_ids.add(track_id)
            self.lane_counts[lane_name] += 1

      # Draw Bounding Box & Centroid
      cv2.rectangle(
          frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2
      )
      cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
      cv2.putText(
          frame,
          f"ID: {track_id}",
          (int(x1), int(y1) - 10),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (0, 255, 0),
          2,
      )

    # Overlay ROI Polygons on Video
    for lane_name, polygon in self.roi_polygons.items():
      pts = np.array(polygon, np.int32).reshape((-1, 1, 2))
      cv2.polylines(
          frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2
      )
      cv2.putText(
          frame,
          f"{lane_name}: {self.lane_counts[lane_name]}",
          (polygon[0][0], polygon[0][1] - 10),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.6,
          (255, 255, 0),
          2,
      )

    return frame, self.lane_counts