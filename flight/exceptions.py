class DroneError(Exception):
    """The Base DroneError."""
    pass

class ConnectionError(DroneError):
    """ When PX4 connection fails. """
    pass

class GPSNotReadyError(DroneError):
    """When GPS is unavailable."""
    pass

class LowBatteryError(DroneError):
    """When Battery is low."""
    pass

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