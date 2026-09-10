"""
Standalone camera bring-up check for the Raspberry Pi.

Runs the recorder's preflight check, records for a few seconds, stops,
and reports what landed on disk. No PX4, no flight controller, no
mission -- but it drives the *exact* code path MissionManager uses, so a
pass here means the recorder works, not that a parallel test script
works.

This is the first thing to run on a new Pi, after confirming the OS
itself sees the camera (`rpicam-hello --list-cameras`). See
docs/raspberry_pi_setup.md.

Usage:
    # Whatever config.settings.CAMERA_BACKEND selects:
    python -m camera.camera_check --seconds 10

    # Force the real camera without editing settings:
    python -m camera.camera_check --seconds 10 --backend picamera2

Exits non-zero if the check fails, so it is usable in a setup script.
"""

import argparse
import asyncio
import sys

from config import settings
from .factory import build_recorder


async def run_check(seconds, backend=None, output_dir=None):
    """Preflight, record for `seconds`, stop, and report. Returns a Recorder."""

    if backend is not None:
        # Overriding the module attribute rather than taking a backend
        # argument through build_recorder(): settings is the single
        # source of truth for which backend is live, and this keeps that
        # true even while forcing one for a bring-up run.
        settings.CAMERA_BACKEND = backend

    settings.validate_settings()

    print(f"Camera backend: {settings.CAMERA_BACKEND}")
    recorder = build_recorder()

    if output_dir is not None and hasattr(recorder, "output_dir"):
        from pathlib import Path

        recorder.output_dir = Path(output_dir).expanduser()

    print("\n[1/3] Preflight check...")
    await recorder.preflight_check()

    print(f"\n[2/3] Recording for {seconds:.1f}s...")
    await recorder.start_recording()
    try:
        await asyncio.sleep(seconds)
    finally:
        print("\n[3/3] Stopping...")
        await recorder.stop_recording()

    _report(recorder)
    return recorder


def _report(recorder):
    output_path = getattr(recorder, "output_path", None)
    if output_path is None:
        print(
            "\nNo video file was produced. That is expected for the "
            "simulation backend; with CAMERA_BACKEND='picamera2' it means "
            "the recording failed."
        )
        return

    if not output_path.exists():
        raise RuntimeError(
            f"Recorder reported {output_path} but the file does not exist."
        )

    size_mb = output_path.stat().st_size / 1e6
    print(f"\nVideo:    {output_path} ({size_mb:.1f} MB)")

    if size_mb == 0:
        raise RuntimeError(
            f"{output_path} is empty -- the encoder produced no data."
        )

    metadata_path = getattr(recorder, "metadata_path", None)
    if metadata_path is not None and metadata_path.exists():
        print(f"Metadata: {metadata_path}")
        print(metadata_path.read_text())

    print(
        "Camera check passed. Copy the file off the Pi and run it through "
        "camera/analyze_video.py to close the loop from capture to report."
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=10.0,
        help="How long to record (default: 10).",
    )
    parser.add_argument(
        "--backend",
        choices=["simulation", "picamera2"],
        default=None,
        help=(
            "Override config.settings.CAMERA_BACKEND for this run only. "
            "Use 'picamera2' to test real hardware without editing settings."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Override settings.RECORDING_OUTPUT_DIR for this run only, e.g. "
            "to record onto a USB drive."
        ),
    )
    args = parser.parse_args()

    try:
        asyncio.run(
            run_check(
                seconds=args.seconds,
                backend=args.backend,
                output_dir=args.output_dir,
            )
        )
    except Exception as error:
        print(f"\nCamera check FAILED: {type(error).__name__}: {error}")
        sys.exit(1)
