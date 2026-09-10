"""
Tests for camera/pi_camera.py, against a fake picamera2 injected into
sys.modules.

PiCamera2Recorder defers its picamera2 import into __init__ precisely so
that importing the module needs no hardware; that same deferral is what
lets these tests run on any machine, with no Pi and no libcamera. They
belong in tests/unit (not tests/vision) because nothing here needs the
vision stack -- this is flight-path code.
"""

import asyncio
import json
import sys
import types
from pathlib import Path

import pytest

from camera.pi_camera import PiCamera2Recorder
from config import settings


# --- fake picamera2 --------------------------------------------------------


class FakeCamera:
    """Records what was asked of it; writes bytes when told to record."""

    instances = []
    fail_on_construct = False

    def __init__(self):
        if FakeCamera.fail_on_construct:
            raise RuntimeError("no cameras available")
        FakeCamera.instances.append(self)
        self.configured = None
        self.recording = False
        self.closed = False
        self.output = None

    def create_video_configuration(self, main=None, controls=None):
        return {"main": main, "controls": controls}

    def configure(self, config):
        self.configured = config

    def start_recording(self, encoder, output):
        self.recording = True
        self.output = output
        # Stand in for the encoder actually producing a file.
        output.path.write_bytes(b"\0" * 2048)

    def stop_recording(self):
        self.recording = False

    def close(self):
        self.closed = True


class FakeEncoder:
    def __init__(self, bitrate=None):
        self.bitrate = bitrate


class FakeOutput:
    def __init__(self, path):
        self.path = Path(path)


@pytest.fixture
def fake_picamera2(monkeypatch):
    FakeCamera.instances = []
    FakeCamera.fail_on_construct = False

    picamera2 = types.ModuleType("picamera2")
    encoders = types.ModuleType("picamera2.encoders")
    outputs = types.ModuleType("picamera2.outputs")

    picamera2.Picamera2 = FakeCamera
    encoders.H264Encoder = FakeEncoder
    outputs.FfmpegOutput = FakeOutput
    picamera2.encoders = encoders
    picamera2.outputs = outputs

    monkeypatch.setitem(sys.modules, "picamera2", picamera2)
    monkeypatch.setitem(sys.modules, "picamera2.encoders", encoders)
    monkeypatch.setitem(sys.modules, "picamera2.outputs", outputs)

    return FakeCamera


@pytest.fixture
def recorder(fake_picamera2, tmp_path):
    return PiCamera2Recorder(
        output_dir=str(tmp_path / "recordings"),
        resolution=(640, 480),
        framerate=30,
        bitrate=1_000_000,
        max_duration_s=60,
    )


# --- construction ----------------------------------------------------------


def test_defaults_come_from_settings(fake_picamera2, tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RECORDING_OUTPUT_DIR", str(tmp_path))
    monkeypatch.setattr(settings, "CAMERA_RESOLUTION", (1280, 720))
    monkeypatch.setattr(settings, "CAMERA_FRAMERATE", 24)
    monkeypatch.setattr(settings, "CAMERA_BITRATE", 5_000_000)
    monkeypatch.setattr(settings, "VIDEO_DURATION", 120)

    r = PiCamera2Recorder()

    assert r.output_dir == tmp_path
    assert r.resolution == (1280, 720)
    assert r.framerate == 24
    assert r.bitrate == 5_000_000
    assert r.max_duration_s == 120


def test_output_dir_expands_user(fake_picamera2):
    r = PiCamera2Recorder(output_dir="~/recordings")
    assert "~" not in str(r.output_dir)
    assert r.output_dir.is_absolute()


def test_starts_not_recording(recorder):
    assert recorder.is_recording is False


# --- preflight -------------------------------------------------------------


async def test_preflight_passes_and_leaves_no_camera_open(recorder):
    await recorder.preflight_check()
    assert recorder.output_dir.exists()
    # The probe camera must be closed again -- leaving it open would
    # make the real start_recording() fail on a busy device.
    assert all(camera.closed for camera in FakeCamera.instances)
    assert recorder.is_recording is False


async def test_preflight_leaves_no_probe_file_behind(recorder):
    await recorder.preflight_check()
    assert list(recorder.output_dir.iterdir()) == []


async def test_preflight_fails_when_camera_is_absent(recorder):
    FakeCamera.fail_on_construct = True
    with pytest.raises(RuntimeError):
        await recorder.preflight_check()


async def test_preflight_fails_when_disk_space_is_insufficient(
    fake_picamera2, tmp_path
):
    # A bitrate/duration combination no filesystem could satisfy.
    r = PiCamera2Recorder(
        output_dir=str(tmp_path),
        bitrate=10**15,
        max_duration_s=10**6,
    )
    with pytest.raises(RuntimeError, match="Insufficient disk space"):
        await r.preflight_check()


# --- recording -------------------------------------------------------------


async def test_start_recording_writes_video_and_metadata(recorder):
    await recorder.start_recording()
    try:
        assert recorder.is_recording is True
        assert recorder.output_path.exists()
        assert recorder.metadata_path.exists()
    finally:
        await recorder.stop_recording()


async def test_capture_settings_reach_the_camera(recorder):
    await recorder.start_recording()
    try:
        camera = FakeCamera.instances[-1]
        assert camera.configured["main"] == {"size": (640, 480)}
        assert camera.configured["controls"] == {"FrameRate": 30}
    finally:
        await recorder.stop_recording()


async def test_metadata_gains_duration_after_stop(recorder):
    await recorder.start_recording()
    await recorder.stop_recording()

    document = json.loads(recorder.metadata_path.read_text())
    assert document["resolution"] == [640, 480]
    assert document["framerate"] == 30
    assert document["bitrate"] == 1_000_000
    assert document["duration_s"] is not None
    assert document["started_wall_clock"] is not None
    assert document["started_monotonic"] is not None
    assert document["started_wall_clock_iso"]


async def test_double_start_does_not_open_a_second_camera(recorder):
    await recorder.start_recording()
    try:
        opened = len(FakeCamera.instances)
        await recorder.start_recording()
        assert len(FakeCamera.instances) == opened
        assert recorder.is_recording is True
    finally:
        await recorder.stop_recording()


async def test_stop_closes_the_camera_and_clears_state(recorder):
    await recorder.start_recording()
    camera = FakeCamera.instances[-1]
    await recorder.stop_recording()

    assert camera.recording is False
    assert camera.closed is True
    assert recorder.is_recording is False


async def test_stop_without_start_does_not_raise(recorder):
    await recorder.stop_recording()
    assert recorder.is_recording is False


async def test_stop_is_idempotent(recorder):
    await recorder.start_recording()
    await recorder.stop_recording()
    await recorder.stop_recording()
    assert recorder.is_recording is False


async def test_stop_survives_a_half_failed_start(recorder):
    # MissionManager calls stop_recording() unconditionally on every
    # abort path; a teardown failure must never mask the exception that
    # triggered the abort.
    recorder._recording = True
    recorder._camera = None
    await recorder.stop_recording()
    assert recorder.is_recording is False


async def test_stop_clears_state_even_if_teardown_raises(recorder):
    await recorder.start_recording()
    camera = FakeCamera.instances[-1]

    def explode():
        raise RuntimeError("encoder wedged")

    camera.stop_recording = explode

    with pytest.raises(RuntimeError):
        await recorder.stop_recording()

    assert recorder.is_recording is False
    assert recorder._camera is None
    assert camera.closed is True


# --- duration watchdog -----------------------------------------------------


async def test_watchdog_stops_recording_at_the_duration_cap(
    fake_picamera2, tmp_path
):
    # settings.VIDEO_DURATION is documented as enforced by the recorder
    # independently of the mission, so a missed stop_recording() cannot
    # leave the camera running until the card fills.
    r = PiCamera2Recorder(
        output_dir=str(tmp_path),
        bitrate=1_000_000,
        max_duration_s=0.05,
    )
    await r.start_recording()
    assert r.is_recording is True

    await asyncio.sleep(0.3)

    assert r.is_recording is False
    assert FakeCamera.instances[-1].closed is True


async def test_watchdog_is_cancelled_by_a_normal_stop(recorder):
    await recorder.start_recording()
    watchdog = recorder._watchdog_task
    await recorder.stop_recording()

    assert recorder._watchdog_task is None
    assert watchdog.cancelled() or watchdog.done()


async def test_watchdog_does_not_outlive_the_recording(fake_picamera2, tmp_path):
    r = PiCamera2Recorder(
        output_dir=str(tmp_path), bitrate=1_000_000, max_duration_s=0.05
    )
    await r.start_recording()
    await r.stop_recording()

    # If the watchdog were still pending it would fire here and try to
    # stop an already-stopped recorder.
    await asyncio.sleep(0.2)
    assert r.is_recording is False
