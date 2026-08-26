"""
Real Raspberry Pi Camera Module 3 recorder, using Picamera2/libcamera.

NOT YET RUNNABLE OR TESTED -- the camera hardware is not available yet.
This file is written and reviewed now so the project is complete once
hardware arrives, but it must never be imported at module load time by
any flight-path code. camera/factory.py only imports this module when
config.settings.CAMERA_BACKEND == "picamera2", and even then the
Picamera2/libcamera import itself happens inside the constructor below,
not at the top of this file -- so importing this module never fails (or
even touches hardware) on a machine without a Pi camera, including this
development machine and CI.
"""

import time
from pathlib import Path
from typing import Optional

from config import settings
from .recorder import Recorder


class PiCamera2Recorder(Recorder):
    """
    Picamera2/libcamera-backed recorder for the Raspberry Pi Camera
    Module 3.

    HARDWARE-GATED: every line below is unverified until real hardware
    testing (see the project plan's "Final-for-SITL vs not-final-until-
    hardware" section) -- encoder throughput, thermal throttling, SD card
    write speed, vibration/rolling-shutter behaviour, and the accuracy of
    the recorded start timestamp for later telemetry<->video alignment
    all require the physical Pi + Camera Module 3 to validate.
    """

    def __init__(
        self,
        output_dir="/home/pi/recordings",
        resolution=(1920, 1080),
        framerate=30,
        bitrate=10_000_000,
    ):
        # Deferred import: must not be a module-level import, so that
        # `import camera.pi_camera` never fails (or requires libcamera) on
        # a machine with no camera hardware.
        from picamera2 import Picamera2
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FfmpegOutput

        self._Picamera2 = Picamera2
        self._H264Encoder = H264Encoder
        self._FfmpegOutput = FfmpegOutput

        self.output_dir = Path(output_dir)
        self.resolution = resolution
        self.framerate = framerate
        self.bitrate = bitrate

        self._camera = None
        self._encoder = None
        self._output = None
        self._recording = False
        self._started_at: Optional[float] = None
        self._current_output_path: Optional[Path] = None

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def output_path(self) -> Optional[Path]:
        return self._current_output_path

    def _check_free_disk_space(self):
        """
        Refuse to start a recording that is unlikely to fit.

        Estimates the worst-case file size from VIDEO_DURATION and the
        configured bitrate, and checks it against free space on
        output_dir's filesystem. This runs before recording starts, not
        during it -- a disk-full failure must be caught before the
        mission commits to an observation hold, not discovered partway
        through one.
        """

        import shutil

        estimated_bytes = (self.bitrate / 8) * settings.VIDEO_DURATION
        free_bytes = shutil.disk_usage(self.output_dir).free

        if free_bytes < estimated_bytes:
            raise RuntimeError(
                "Insufficient disk space for a "
                f"{settings.VIDEO_DURATION:.0f}s recording: need ~"
                f"{estimated_bytes / 1e6:.0f} MB, have "
                f"{free_bytes / 1e6:.0f} MB free in {self.output_dir}."
            )

    async def start_recording(self):
        if self._recording:
            print(
                "PiCamera2Recorder: already recording; ignoring duplicate "
                "start."
            )
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._check_free_disk_space()

        timestamp = time.strftime("%Y%m%dT%H%M%S")
        self._current_output_path = (
            self.output_dir / f"observation_{timestamp}.mp4"
        )

        self._camera = self._Picamera2()
        video_config = self._camera.create_video_configuration(
            main={"size": self.resolution},
            controls={"FrameRate": self.framerate},
        )
        self._camera.configure(video_config)

        self._encoder = self._H264Encoder(bitrate=self.bitrate)
        self._output = self._FfmpegOutput(str(self._current_output_path))

        self._camera.start_recording(self._encoder, self._output)

        self._recording = True
        # Monotonic start time, recorded so an offline step can later
        # align this recording against the mission's telemetry log (which
        # uses time.monotonic() throughout -- see flight/monitor.py).
        self._started_at = time.monotonic()

        print(
            f"PiCamera2Recorder: recording started -> "
            f"{self._current_output_path}"
        )

    async def stop_recording(self):
        if not self._recording:
            return

        elapsed = (
            time.monotonic() - self._started_at if self._started_at else 0.0
        )

        try:
            self._camera.stop_recording()
        finally:
            self._camera.close()
            self._recording = False
            self._camera = None
            self._encoder = None
            self._output = None

        print(
            f"PiCamera2Recorder: recording stopped after {elapsed:.1f}s -> "
            f"{self._current_output_path}"
        )
