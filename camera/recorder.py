"""
Recorder is the contract every camera backend must satisfy.
SimulationRecorder is the current, clearly-labelled no-op used for
Gazebo/SITL mission testing. The real Picamera2 implementation lives in
pi_camera.py and is selected through factory.py, so mission code never
has to change when the Raspberry Pi Camera Module 3 becomes available.
"""

import time
from abc import ABC, abstractmethod
from typing import Optional


class Recorder(ABC):
    """Interface every camera backend (simulated or real) must implement."""

    @abstractmethod
    async def start_recording(self):
        """Begin recording. Safe to call when already recording (no-op)."""
        raise NotImplementedError

    @abstractmethod
    async def stop_recording(self):
        """
        Stop recording if currently recording; otherwise a no-op.

        Must NOT raise when called while not recording -- MissionManager
        calls this unconditionally during cleanup and abort paths.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def is_recording(self) -> bool:
        raise NotImplementedError


class SimulationRecorder(Recorder):
    """
    Simulation/no-op recording interface for Gazebo/SITL mission testing.

    Tracks recording state and timing so mission code can be written and
    tested against the real call sequence (start on arrival at an
    observation waypoint, stop on departure or on mission abort) before
    the Raspberry Pi Camera Module 3 is physically available. Produces no
    video file.
    """

    def __init__(self):
        self._recording = False
        self._started_at: Optional[float] = None

    @property
    def is_recording(self) -> bool:
        return self._recording

    async def start_recording(self):
        if self._recording:
            print(
                "SimulationRecorder: already recording; ignoring duplicate "
                "start."
            )
            return

        self._recording = True
        self._started_at = time.monotonic()
        print("SimulationRecorder: recording started (simulated, no video file).")

    async def stop_recording(self):
        if not self._recording:
            return

        elapsed = (
            time.monotonic() - self._started_at if self._started_at else 0.0
        )
        self._recording = False
        self._started_at = None
        print(
            f"SimulationRecorder: recording stopped after {elapsed:.1f}s "
            "(simulated)."
        )
