class DroneError(Exception):
    """The Base DroneError."""
    pass


class PX4ConnectionError(DroneError):
    """When the initial PX4 connection fails or times out."""
    pass


# Deprecated alias. The builtin `ConnectionError` shadows this name, which
# meant a genuine builtin ConnectionError raised elsewhere in the stack was
# never caught by isinstance checks against this class. Use
# PX4ConnectionError in new code; this alias exists only so any external
# caller importing the old name keeps working.
ConnectionError = PX4ConnectionError


class LinkLostError(DroneError):
    """
    When the MAVLink connection is lost after having been established.

    This is distinct from PX4ConnectionError, which is only for the initial
    connection attempt. PX4's own data-link-loss failsafe is responsible for
    the aircraft's response; the application must abort without commanding
    flight when this is raised.
    """
    pass


class TelemetryTimeoutError(DroneError):
    """
    When a bounded telemetry read does not produce a sample in time.

    Raised by Telemetry._get_single_sample() instead of letting a raw
    asyncio.TimeoutError escape, so Emergency can classify a stalled
    telemetry stream the same way it classifies a lost link: abort without
    commanding flight, since a command sent over the same link cannot be
    trusted to arrive.
    """

    def __init__(self, message, stream_name=None):
        super().__init__(message)
        self.stream_name = stream_name


class GPSNotReadyError(DroneError):
    """When GPS is unavailable."""
    pass


class LowBatteryError(DroneError):
    """
    When battery is at or below an application threshold.

    Carries the measured percentage so Emergency does not need to re-read
    telemetry (which may be what is failing) to decide between RTL and
    emergency landing.
    """

    def __init__(self, message, battery_percent=None):
        super().__init__(message)
        self.battery_percent = battery_percent


class ArmingError(DroneError):
    """When Arming fails."""
    pass


class MissionError(DroneError):
    """When Mission fails."""
    pass


class HomePositionNotReadyError(DroneError):
    """When HomePosition fails."""
    pass


class TakeoffError(DroneError):
    """When Takeoff fails."""
    pass


class GotoError(DroneError):
    """When Goto fails."""
    pass


class LandingError(DroneError):
    """When Landing fails."""
    pass


class MissionAbortError(DroneError):
    """When Mission fails."""
    pass


class EmergencyCommandError(DroneError):
    """
    When an emergency-response flight command (RTL / land / hold) itself
    fails.

    The emergency-response policy in Emergency must never swallow a failed
    emergency command, but it also must not let the failure silently replace
    the exception that triggered the emergency in the first place. This type
    is raised via `raise EmergencyCommandError(...) from original_exception`
    so both the original cause and the emergency-command failure are
    preserved on the exception chain.
    """
    pass
