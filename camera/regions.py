"""
Load and save the counting lines and zones used by the offline traffic
analysis, as a small JSON file.

Kept separate from camera/draw_regions.py (the interactive editor that
writes these files) so that reading a region file needs neither OpenCV
nor a display -- this module is stdlib-only and unit-testable anywhere,
exactly like camera/traffic_metrics.py.

File format:

    {
      "video": "traffic.mp4",
      "frame_size": [1920, 1080],
      "lines": [
        {"name": "north_gate", "points": [[640, 0], [640, 720]]}
      ],
      "zones": [
        {"name": "north_arm",
         "points": [[100, 50], [400, 50], [420, 300], [80, 300]]}
      ]
    }

All coordinates are pixels in the SOURCE video's full resolution, origin
top-left. "frame_size" records the resolution they were drawn against so
that running them over a differently-sized video can warn instead of
silently producing wrong counts.
"""

import json

from .traffic_metrics import CountingLine, Zone


class RegionFileError(ValueError):
    """A region file exists but cannot be interpreted."""


def save_regions(path, lines=None, zones=None, video=None, frame_size=None):
    """
    Write CountingLine and Zone objects to `path` as JSON.

    `frame_size` is the (width, height) of the video the shapes were
    drawn on; it is stored so load_regions() can warn about a mismatch.
    """

    document = {
        "video": video,
        "frame_size": list(frame_size) if frame_size else None,
        "lines": [
            {"name": line.name, "points": [list(line.a), list(line.b)]}
            for line in lines or []
        ],
        "zones": [
            {"name": zone.name, "points": [list(point) for point in zone.points]}
            for zone in zones or []
        ],
    }

    with open(path, "w") as f:
        json.dump(document, f, indent=2)

    print(
        f"Regions: wrote {len(document['lines'])} line(s) and "
        f"{len(document['zones'])} zone(s) to {path}"
    )


def load_regions(path, frame_size=None):
    """
    Read a region file and return (lines, zones) as CountingLine and Zone
    objects.

    Pass the analysed video's actual (width, height) as `frame_size` to
    get a warning when it differs from the resolution the shapes were
    drawn against -- pixel coordinates from a 4K frame mean something
    entirely different on a 720p one, and that mistake is otherwise
    invisible in the output.
    """

    try:
        with open(path) as f:
            document = json.load(f)
    except json.JSONDecodeError as e:
        raise RegionFileError(f"{path} is not valid JSON: {e}") from e

    if not isinstance(document, dict):
        raise RegionFileError(
            f"{path} must contain a JSON object, got {type(document).__name__}."
        )

    stored_size = document.get("frame_size")
    if frame_size and stored_size and list(stored_size) != list(frame_size):
        print(
            f"WARNING: {path} was drawn against a "
            f"{stored_size[0]}x{stored_size[1]} frame, but this video is "
            f"{frame_size[0]}x{frame_size[1]}. The pixel coordinates will "
            "not line up with the road. Re-draw the regions on this video."
        )

    lines = [
        CountingLine(name, *points[0], *points[1])
        for name, points in _shapes(document, "lines", path, expected_points=2)
    ]
    zones = [
        Zone(name, points)
        for name, points in _shapes(document, "zones", path, expected_points=None)
    ]

    return lines, zones


def _shapes(document, key, path, expected_points):
    """
    Validate and yield (name, points) for document[key].

    `expected_points` is an exact required count, or None for "at least
    3" (a polygon). Validation lives here rather than in the callers so
    a malformed file fails with a message naming the file and the shape,
    instead of an IndexError somewhere in the analysis.
    """

    shapes = document.get(key) or []
    if not isinstance(shapes, list):
        raise RegionFileError(f"{path}: {key!r} must be a list.")

    for index, shape in enumerate(shapes):
        if not isinstance(shape, dict):
            raise RegionFileError(f"{path}: {key}[{index}] must be an object.")

        name = shape.get("name") or f"{key[:-1]}{index + 1}"
        points = shape.get("points")

        if not isinstance(points, list):
            raise RegionFileError(
                f"{path}: {key} entry {name!r} has no 'points' list."
            )

        if expected_points is not None and len(points) != expected_points:
            raise RegionFileError(
                f"{path}: {key} entry {name!r} needs exactly "
                f"{expected_points} points, got {len(points)}."
            )

        if expected_points is None and len(points) < 3:
            raise RegionFileError(
                f"{path}: {key} entry {name!r} needs at least 3 points to be "
                f"a polygon, got {len(points)}."
            )

        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise RegionFileError(
                    f"{path}: {key} entry {name!r} has a malformed point "
                    f"{point!r}; expected [x, y]."
                )

        yield name, [(float(x), float(y)) for x, y in points]
