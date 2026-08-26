class Camera:
    """
    Temporary camera interface for Gazebo mission testing.

    This version tracks recording state but does not create video files.
    Replace its internals with Picamera2 when the Raspberry Pi camera is
    available.
    """

    def __init__(self):
        self.recording = False

    async def start_recording(self):
        """Start the observation recording."""

        if self.recording:
            raise RuntimeError("Camera is already recording.")

        self.recording = True
        print("Camera: recording started.")

    async def stop_recording(self):
        """Stop the observation recording."""

        if not self.recording:
            return

        self.recording = False
        print("Camera: recording stopped.")