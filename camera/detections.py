"""
Shared helper for reading Ultralytics tracking results, whether the
underlying model is a standard (axis-aligned) detector or an OBB
(oriented bounding box) detector -- both expose the same
xyxy / id / cls / conf attribute names on their respective .boxes /
.obb containers, so callers only need one code path.

Part of the offline vision/analysis stack -- see requirements-vision.txt.
Never imported by flight/ or mission/.
"""


def get_detections(result):
    """
    Return (xyxy, track_ids, class_ids, confs) as numpy arrays for one
    Ultralytics Results object, or (None, None, None, None) if there are
    no tracked detections in this frame (nothing detected, or the
    tracker has not yet assigned IDs).
    """

    detections = result.boxes if result.boxes is not None else result.obb

    if detections is None or detections.id is None:
        return None, None, None, None

    xyxy = detections.xyxy.cpu().numpy()
    track_ids = detections.id.int().cpu().numpy()
    class_ids = detections.cls.int().cpu().numpy()
    confs = detections.conf.cpu().numpy()
    return xyxy, track_ids, class_ids, confs
