"""
Tests for camera/detections.py. Requires requirements-vision.txt (torch)
-- run separately from the flight-critical suite: `pytest tests/vision`.
"""

import torch

from camera.detections import get_detections


class FakeBoxes:
    def __init__(self, xyxy, ids, cls, conf):
        self.xyxy = torch.tensor(xyxy, dtype=torch.float32)
        self.id = torch.tensor(ids, dtype=torch.float32) if ids is not None else None
        self.cls = torch.tensor(cls, dtype=torch.float32)
        self.conf = torch.tensor(conf, dtype=torch.float32)


class FakeResult:
    def __init__(self, boxes=None, obb=None):
        self.boxes = boxes
        self.obb = obb


def make_fake_boxes(n=2):
    xyxy = [[10, 10, 20, 20], [30, 30, 40, 40]][:n]
    ids = list(range(1, n + 1))
    cls = [0, 1][:n]
    conf = [0.9, 0.8][:n]
    return FakeBoxes(xyxy, ids, cls, conf)


def test_reads_boxes_when_present():
    result = FakeResult(boxes=make_fake_boxes(2), obb=None)
    xyxy, track_ids, class_ids, confs = get_detections(result)
    assert xyxy.shape == (2, 4)
    assert list(track_ids) == [1, 2]
    assert list(class_ids) == [0, 1]
    assert confs.shape == (2,)


def test_reads_obb_when_boxes_absent():
    # OBB models populate .obb instead of .boxes; .boxes is None for them
    # -- this is the actual shape of a YOLO-OBB model's Results object.
    result = FakeResult(boxes=None, obb=make_fake_boxes(2))
    xyxy, track_ids, class_ids, confs = get_detections(result)
    assert xyxy.shape == (2, 4)
    assert list(track_ids) == [1, 2]


def test_returns_none_when_no_detections():
    result = FakeResult(boxes=None, obb=None)
    xyxy, track_ids, class_ids, confs = get_detections(result)
    assert xyxy is None
    assert track_ids is None
    assert class_ids is None
    assert confs is None


def test_returns_none_when_tracking_ids_not_yet_assigned():
    boxes = make_fake_boxes(1)
    boxes.id = None  # tracker has not assigned IDs yet (e.g. first frame)
    result = FakeResult(boxes=boxes, obb=None)
    xyxy, track_ids, class_ids, confs = get_detections(result)
    assert xyxy is None
