"""
Selects which Recorder implementation MissionManager uses, based on
config.settings.CAMERA_BACKEND. This is the one place mission code needs
to touch when the real camera becomes available -- flight/ and mission/
never import camera.pi_camera or camera.recorder directly.
"""

from config import settings
from .recorder import SimulationRecorder


def build_recorder():
    if settings.CAMERA_BACKEND == "simulation":
        return SimulationRecorder()

    if settings.CAMERA_BACKEND == "picamera2":
        # Imported here, not at module load time, so a machine without
        # Picamera2/libcamera installed can still import camera.factory
        # and run in simulation mode.
        from .pi_camera import PiCamera2Recorder

        return PiCamera2Recorder()

    raise ValueError(
        f"Unknown CAMERA_BACKEND={settings.CAMERA_BACKEND!r}; expected "
        "'simulation' or 'picamera2'."
    )
