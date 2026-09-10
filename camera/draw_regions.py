"""
Interactive editor for counting lines and intersection zones.

Opens one frame of a video in a window and lets you click the shapes you
want to count with, then writes them to a JSON region file that
camera/analyze_video.py reads via --regions. This exists because the
alternative -- typing --line X1,Y1,X2,Y2 pixel coordinates you have no
way of knowing by looking at the footage -- is unusable in practice.

Usage:
    python -m camera.draw_regions \\
        --video traffic.mp4 --output regions.json --at-seconds 5 --scale 0.6

Controls:
    left click       add a point
    u / backspace    undo the last point
    l                finish the current shape as a LINE   (exactly 2 points)
    z                finish the current shape as a ZONE   (3+ points)
    d                delete the last completed shape
    s                save to --output and quit
    q / ESC          quit without saving

Draw a LINE across a road to count vehicles crossing it (--mode
screenline). Draw one ZONE polygon over each approach arm of an
intersection to get per-arm occupancy and turning movements (--mode
zones).

Needs a display: on WSL that means WSLg or an X server. This is an
offline authoring tool -- it is never imported by flight/ or mission/
and must never run on the Raspberry Pi's flight process.
"""

import argparse

import cv2
import numpy as np

from .regions import save_regions
from .traffic_metrics import CountingLine, Zone

WINDOW = "draw regions"

LINE_COLOR = (0, 255, 255)  # yellow, matching analyze_video's overlay
POINT_COLOR = (255, 255, 255)
ZONE_COLORS = [
    (0, 200, 0),
    (255, 128, 0),
    (200, 0, 200),
    (0, 128, 255),
    (0, 0, 220),
    (180, 180, 0),
]

HELP_LINES = [
    "click: add point   u: undo point   d: delete last shape",
    "l: finish LINE (2 pts)   z: finish ZONE (3+ pts)",
    "s: save and quit   q/ESC: quit without saving",
]


def grab_frame(video_path, at_seconds=0.0):
    """
    Return one BGR frame from `video_path`, and the video's (width,
    height).

    Seeks by timestamp, then falls back to reading frames sequentially:
    CAP_PROP_POS_MSEC seeking is unreliable on some codecs and silently
    lands on frame 0 (or fails) rather than erroring.
    """

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open video source: {video_path}")

    try:
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        frame = None
        if at_seconds > 0:
            cap.set(cv2.CAP_PROP_POS_MSEC, at_seconds * 1000.0)
            ret, frame = cap.read()
            if not ret:
                frame = None

        if frame is None:
            fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            target_index = int(at_seconds * fps)
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            for _ in range(target_index + 1):
                ret, candidate = cap.read()
                if not ret:
                    break
                frame = candidate

        if frame is None:
            raise RuntimeError(
                f"Could not read any frame from {video_path}."
            )

        return frame, (width or frame.shape[1], height or frame.shape[0])
    finally:
        cap.release()


class RegionEditor:
    """
    Editor state and rendering.

    Points are held in DISPLAY coordinates while editing and converted
    back to full-resolution source pixels once, in shapes_full_res(), so
    that drawing on a scaled-down window still produces coordinates
    valid for the original video.
    """

    def __init__(self, frame, scale=1.0):
        if not 0 < scale <= 1.0:
            raise ValueError("scale must be in (0, 1].")

        self.scale = scale
        self.display = (
            frame.copy()
            if scale == 1.0
            else cv2.resize(
                frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        )
        self.points = []  # in-progress shape, display coordinates
        self.shapes = []  # (kind, name, [display points])
        self.cursor = None

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.cursor = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))

    def undo_point(self):
        if self.points:
            self.points.pop()

    def delete_last_shape(self):
        if self.shapes:
            kind, name, _ = self.shapes.pop()
            print(f"Deleted {kind} {name!r}.")

    def finish(self, kind):
        """Close the in-progress shape as 'line' or 'zone'."""

        required = 2 if kind == "line" else 3
        if len(self.points) < required or (
            kind == "line" and len(self.points) != 2
        ):
            print(
                f"A {kind} needs "
                f"{'exactly 2' if kind == 'line' else 'at least 3'} points; "
                f"you have {len(self.points)}."
            )
            return

        default = f"{kind}{sum(1 for k, _, _ in self.shapes if k == kind) + 1}"
        name = input(f"Name for this {kind} [{default}]: ").strip() or default
        self.shapes.append((kind, name, list(self.points)))
        self.points = []
        print(f"Added {kind} {name!r}.")

    def shapes_full_res(self):
        """(lines, zones) as CountingLine/Zone in source-video pixels."""

        lines, zones = [], []
        for kind, name, points in self.shapes:
            scaled = [(x / self.scale, y / self.scale) for x, y in points]
            if kind == "line":
                lines.append(CountingLine(name, *scaled[0], *scaled[1]))
            else:
                zones.append(Zone(name, scaled))
        return lines, zones

    def render(self):
        canvas = self.display.copy()
        overlay = canvas.copy()
        zone_index = 0

        for kind, name, points in self.shapes:
            if kind == "line":
                cv2.line(canvas, points[0], points[1], LINE_COLOR, 2)
                _label(canvas, name, points[0], LINE_COLOR)
            else:
                color = ZONE_COLORS[zone_index % len(ZONE_COLORS)]
                zone_index += 1
                polygon = np.array(points, dtype=np.int32)
                cv2.fillPoly(overlay, [polygon], color)
                cv2.polylines(canvas, [polygon], True, color, 2)
                _label(canvas, name, _centroid(points), color)

        cv2.addWeighted(overlay, 0.25, canvas, 0.75, 0, canvas)

        for point in self.points:
            cv2.circle(canvas, point, 4, POINT_COLOR, -1)
        if len(self.points) > 1:
            cv2.polylines(
                canvas,
                [np.array(self.points, dtype=np.int32)],
                False,
                POINT_COLOR,
                1,
            )
        if self.points and self.cursor:
            cv2.line(canvas, self.points[-1], self.cursor, POINT_COLOR, 1)

        _draw_help(canvas, len(self.points))
        return canvas


def _centroid(points):
    n = len(points)
    return (
        int(sum(x for x, _ in points) / n),
        int(sum(y for _, y in points) / n),
    )


def _label(image, text, origin, color):
    cv2.putText(
        image,
        text,
        (int(origin[0]) + 5, int(origin[1]) + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
    )


def _draw_help(image, pending_points):
    lines = list(HELP_LINES)
    if pending_points:
        lines.append(f"{pending_points} point(s) in progress")

    for index, text in enumerate(lines):
        position = (10, 22 + index * 22)
        # Black underlay first so the text stays readable over both a
        # bright road surface and a dark one.
        cv2.putText(
            image, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3
        )
        cv2.putText(
            image,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )


def draw_regions(video_path, output_path, at_seconds=0.0, scale=1.0):
    """Run the editor; returns True if regions were saved."""

    frame, frame_size = grab_frame(video_path, at_seconds)
    editor = RegionEditor(frame, scale)

    print(f"Editing a {frame_size[0]}x{frame_size[1]} frame at {at_seconds:.1f}s.")
    for text in HELP_LINES:
        print(f"  {text}")

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, editor.on_mouse)

    saved = False
    try:
        while True:
            cv2.imshow(WINDOW, editor.render())
            key = cv2.waitKey(20) & 0xFF

            if key in (ord("q"), 27):  # 27 = ESC
                print("Quit without saving.")
                break
            if key in (ord("u"), 8, 127):  # 8/127 = backspace/delete
                editor.undo_point()
            elif key == ord("d"):
                editor.delete_last_shape()
            elif key == ord("l"):
                editor.finish("line")
            elif key == ord("z"):
                editor.finish("zone")
            elif key == ord("s"):
                lines, zones = editor.shapes_full_res()
                if not lines and not zones:
                    print("Nothing to save yet -- draw at least one shape.")
                    continue
                save_regions(
                    output_path,
                    lines=lines,
                    zones=zones,
                    video=video_path,
                    frame_size=frame_size,
                )
                saved = True
                break
    finally:
        cv2.destroyAllWindows()

    return saved


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--video", required=True, help="Video to draw the regions on."
    )
    parser.add_argument(
        "--output",
        default="regions.json",
        help="Region file to write (default: regions.json).",
    )
    parser.add_argument(
        "--at-seconds",
        type=float,
        default=0.0,
        help=(
            "Which moment of the video to draw on (default: 0). Pick a "
            "frame where the roads are clearly visible."
        ),
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help=(
            "Shrink the editing window by this factor, in (0, 1], for "
            "footage larger than your screen. Saved coordinates are always "
            "in the video's full resolution. Default 1.0."
        ),
    )
    args = parser.parse_args()

    draw_regions(
        video_path=args.video,
        output_path=args.output,
        at_seconds=args.at_seconds,
        scale=args.scale,
    )
