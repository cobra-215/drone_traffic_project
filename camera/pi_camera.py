"""
Real Raspberry Pi Camera Module 3 recorder, using Picamera2/libcamera.

HARDWARE-GATED. The Picamera2/libcamera import happens inside the
constructor, not at the top of this file, so importing this module never
fails (or touches hardware) on a machine without a Pi camera -- including
this project's development machine and CI. camera/factory.py only imports
it at all when config.settings.CAMERA_BACKEND == "picamera2".

The picamera2 API calls below are written from its documented interface
but have NOT been executed against the installed package. Before running
this on the Pi for the first time, verify the real signatures -- this
project's standing rule is never to assume a hardware API:

    rpicam-hello --list-cameras
    python3 -c "from picamera2 import Picamera2; help(Picamera2.create_video_configuration)"
    python3 -c "from picamera2.encoders import H264Encoder; help(H264Encoder.__init__)"
    python3 -c "from picamera2.outputs import FfmpegOutput; help(FfmpegOutput.__init__)"
    python3 -c "from picamera2 import Picamera2; help(Picamera2.start_recording)"

See docs/raspberry_pi_setup.md, and camera/camera_check.py for a
standalone bring-up run that needs no PX4 and no flight controller.
"""

import asyncio
import contextlib
import json
import shutil
import time
from pathlib import Path
from typing import Optional

from config import settings
from .recorder import Recorder


class PiCamera2Recorder(Recorder):
    """
    Picamera2/libcamera-backed recorder for the Raspberry Pi Camera
    Module 3.

    Every blocking Picamera2 call is dispatched through
    asyncio.to_thread(). Those calls are synchronous and can take
    hundreds of milliseconds to seconds; awaiting them directly on the
    event loop would stall FlightMonitor's safety checks and every
    telemetry read for that whole time. A camera must never be able to
    do that to the flight loop.

    Still unverified until real hardware testing: encoder throughput,
    thermal throttling, SD-card write speed, vibration/rolling-shutter
    behaviour, and the accuracy of the recorded start timestamp for
    later telemetry<->video alignment.
    """

    def __init__(
        self,
        output_dir=None,
        resolution=None,
        framerate=None,
        bitrate=None,
        max_duration_s=None,
    ):
        # Deferred import: must not be a module-level import, so that
        # `import camera.pi_camera` never fails (or requires libcamera)
        # on a machine with no camera hardware.
        from picamera2 import Picamera2
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FfmpegOutput

        self._Picamera2 = Picamera2
        self._H264Encoder = H264Encoder
        self._FfmpegOutput = FfmpegOutput

        # None-sentinel defaults rather than settings values frozen into
        # the signature at import time -- the same pattern Drone uses for
        # takeoff_altitude.
        self.output_dir = Path(
            output_dir
            if output_dir is not None
            else settings.RECORDING_OUTPUT_DIR
        ).expanduser()
        self.resolution = tuple(
            resolution if resolution is not None else settings.CAMERA_RESOLUTION
        )
        self.framerate = (
            framerate if framerate is not None else settings.CAMERA_FRAMERATE
        )
        self.bitrate = (
            bitrate if bitrate is not None else settings.CAMERA_BITRATE
        )
        self.max_duration_s = (
            max_duration_s
            if max_duration_s is not None
            else settings.VIDEO_DURATION
        )

        self._camera = None
        self._encoder = None
        self._output = None
        self._recording = False
        self._started_at: Optional[float] = None
        self._started_wall_clock: Optional[float] = None
        self._current_output_path: Optional[Path] = None
        self._watchdog_task: Optional[asyncio.Task] = None

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def output_path(self) -> Optional[Path]:
        return self._current_output_path

    @property
    def metadata_path(self) -> Optional[Path]:
        """The sidecar JSON written alongside the current recording."""
        if self._current_output_path is None:
            return None
        return self._current_output_path.with_suffix(".json")

    # -- preflight ---------------------------------------------------------

    async def preflight_check(self):
        """
        Prove on the ground that a recording could actually be made:
        the output directory is writable, there is room for a full
        recording, and libcamera can see the camera.
        """

        await asyncio.to_thread(self._check_output_dir_writable)
        await asyncio.to_thread(self._check_free_disk_space)
        await asyncio.to_thread(self._probe_camera)

        print(
            f"PiCamera2Recorder: preflight OK -- camera detected, "
            f"{self.output_dir} writable, disk space sufficient for a "
            f"{self.max_duration_s:.0f}s recording."
        )

    def _check_output_dir_writable(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # Actually write, rather than trusting os.access(): the failure
        # this needs to catch is a read-only or full SD card, which
        # permission bits alone do not reveal.
        probe = self.output_dir / ".write_probe"
        try:
            probe.write_bytes(b"")
        finally:
            with contextlib.suppress(OSError):
                probe.unlink()

    def _check_free_disk_space(self):
        """
        Refuse to start a recording that is unlikely to fit.

        Estimates the worst-case file size from the recording duration
        cap and the configured bitrate, and checks it against free space
        on output_dir's filesystem. A disk-full failure must be caught
        before the mission commits to an observation hold, not
        discovered partway through one.
        """

        estimated_bytes = (self.bitrate / 8) * self.max_duration_s
        free_bytes = shutil.disk_usage(self.output_dir).free

        if free_bytes < estimated_bytes:
            raise RuntimeError(
                "Insufficient disk space for a "
                f"{self.max_duration_s:.0f}s recording: need ~"
                f"{estimated_bytes / 1e6:.0f} MB, have "
                f"{free_bytes / 1e6:.0f} MB free in {self.output_dir}."
            )

    def _probe_camera(self):
        """
        Construct and immediately close a Picamera2 to prove libcamera
        detects the camera. Deliberately does nothing more: any richer
        probe would depend on picamera2 attributes this project has not
        verified against the installed package.
        """

        camera = self._Picamera2()
        camera.close()

    # -- recording ---------------------------------------------------------

    async def start_recording(self):
        if self._recording:
            print(
                "PiCamera2Recorder: already recording; ignoring duplicate "
                "start."
            )
            return

        await asyncio.to_thread(self.output_dir.mkdir, parents=True, exist_ok=True)
        await asyncio.to_thread(self._check_free_disk_space)

        timestamp = time.strftime("%Y%m%dT%H%M%S")
        self._current_output_path = (
            self.output_dir / f"observation_{timestamp}.mp4"
        )

        await asyncio.to_thread(self._open_and_start)

        self._recording = True
        # Monotonic start time, recorded so an offline step can later
        # align this recording against the mission's telemetry log
        # (which uses time.monotonic() throughout -- see
        # flight/monitor.py). Wall clock is recorded alongside it purely
        # so a human can tell when the footage was taken.
        self._started_at = time.monotonic()
        self._started_wall_clock = time.time()

        await asyncio.to_thread(self._write_metadata)

        # settings.VIDEO_DURATION is documented as being enforced by the
        # recorder independently of the mission's observation hold,
        # precisely so a missed stop_recording() cannot leave the camera
        # running until the card fills. Nothing enforced it before this.
        self._watchdog_task = asyncio.create_task(self._duration_watchdog())

        print(
            f"PiCamera2Recorder: recording started -> "
            f"{self._current_output_path}"
        )

    def _open_and_start(self):
        """Blocking Picamera2 startup; always called via to_thread."""

        self._camera = self._Picamera2()
        video_config = self._camera.create_video_configuration(
            main={"size": self.resolution},
            controls={"FrameRate": self.framerate},
        )
        self._camera.configure(video_config)

        self._encoder = self._H264Encoder(bitrate=self.bitrate)
        self._output = self._FfmpegOutput(str(self._current_output_path))

        self._camera.start_recording(self._encoder, self._output)

    async def _duration_watchdog(self):
        await asyncio.sleep(self.max_duration_s)

        print(
            f"PiCamera2Recorder: recording duration cap "
            f"({self.max_duration_s:.0f}s) reached; stopping."
        )
        # Clear the handle before stopping so stop_recording() does not
        # try to cancel the task it is being called from.
        self._watchdog_task = None
        await self.stop_recording()

    async def stop_recording(self):
        await self._cancel_watchdog()

        if not self._recording:
            return

        elapsed = (
            time.monotonic() - self._started_at if self._started_at else 0.0
        )

        try:
            await asyncio.to_thread(self._stop_and_close)
        finally:
            # State is cleared even if the camera teardown raised.
            # Recorder.stop_recording()'s contract is that it must not
            # raise, and MissionManager calls it unconditionally on
            # every abort path -- a teardown failure must never mask the
            # exception that triggered the abort.
            self._recording = False
            self._camera = None
            self._encoder = None
            self._output = None

        with contextlib.suppress(OSError):
            await asyncio.to_thread(self._write_metadata, elapsed)

        print(
            f"PiCamera2Recorder: recording stopped after {elapsed:.1f}s -> "
            f"{self._current_output_path}"
        )

    def _stop_and_close(self):
        """Blocking Picamera2 teardown; always called via to_thread."""

        camera = self._camera
        if camera is None:
            # start_recording() failed partway through, leaving nothing
            # to tear down.
            return

        try:
            camera.stop_recording()
        finally:
            camera.close()

    async def _cancel_watchdog(self):
        watchdog = self._watchdog_task
        self._watchdog_task = None
        if watchdog is None or watchdog.done():
            return
        watchdog.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await watchdog

    # -- metadata ----------------------------------------------------------

    def _write_metadata(self, duration_s=None):
        """
        Write the sidecar JSON beside the video.

        Written once at start (so an interrupted run still leaves the
        capture settings and start time on disk) and rewritten at stop
        with the achieved duration. Offline analysis reports time as
        seconds into the video; this file is what maps that onto real
        clock time, and later onto the mission's telemetry log.
        """

        path = self.metadata_path
        if path is None:
            return

        document = {
            "video": self._current_output_path.name,
            "resolution": list(self.resolution),
            "framerate": self.framerate,
            "bitrate": self.bitrate,
            "max_duration_s": self.max_duration_s,
            "started_wall_clock": self._started_wall_clock,
            "started_wall_clock_iso": (
                time.strftime(
                    "%Y-%m-%dT%H:%M:%S", time.localtime(self._started_wall_clock)
                )
                if self._started_wall_clock
                else None
            ),
            "started_monotonic": self._started_at,
            "duration_s": round(duration_s, 2) if duration_s is not None else None,
        }

        with open(path, "w") as f:
            json.dump(document, f, indent=2)
