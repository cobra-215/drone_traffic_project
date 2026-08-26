# ============================================================
# config/settings.py
#
# Central configuration for the drone traffic-monitoring
# application.
#
# IMPORTANT:
# These values configure OUR APPLICATION.
# PX4's own safety/failsafe parameters remain configured
# inside PX4. See flight/px4_params.py for a read-only audit
# of the PX4 side and docs/px4_parameter_checklist.md for the
# human-reviewed checklist.
#
# Unless otherwise noted, all altitudes in this file are
# RELATIVE TO HOME/TAKEOFF ALTITUDE (MAVSDK "relative_altitude_m"),
# not AMSL. The relative-to-AMSL conversion happens in exactly one
# place: flight/drone.py Drone.goto().
# ============================================================


# ============================================================
# PX4 / MAVSDK CONNECTION
# ============================================================

# UDP endpoint used by MAVSDK to communicate with PX4 SITL.
#
# For the standard PX4 SITL configuration this is normally
# 14540.
#
# For a real vehicle, this value depends on the actual
# telemetry/network configuration.
PX4_CONNECTION_ADDRESS = "udpin://0.0.0.0:14540"

# Maximum time allowed while establishing the connection.
CONNECTION_TIMEOUT = 30.0


# ============================================================
# TELEMETRY
# ============================================================

# How often FlightMonitor checks the aircraft state.
#
# 1 second is a reasonable starting point for application-level
# monitoring. PX4 itself operates much faster internally.
MONITOR_INTERVAL = 1.0

# Maximum time we wait for an individual telemetry request.
TELEMETRY_TIMEOUT = 5.0


# ============================================================
# BATTERY (percent, 0-100, per MAVSDK 3.x Battery.remaining_percent)
# ============================================================

# Minimum battery required to begin a mission. Must be strictly greater
# than BATTERY_RTL_THRESHOLD -- otherwise a vehicle could pass preflight
# and immediately trip the in-flight RTL threshold on the monitor's first
# check. Enforced by validate_settings().
BATTERY_PREFLIGHT_MIN_PERCENT = 50.0

# Application-level battery threshold.
#
# When the mission detects that battery is below this level,
# the application should stop the mission and request RTL.
BATTERY_RTL_THRESHOLD = 30.0

# Critical battery threshold.
#
# At or below this level, the application must not continue
# the mission and must land immediately rather than attempt RTL.
BATTERY_CRITICAL_THRESHOLD = 20.0


# ============================================================
# ALTITUDE (metres, relative to home -- see module docstring)
# ============================================================

# Minimum altitude allowed during the mission.
#
# This is an APPLICATION constraint, not a replacement for
# PX4's altitude/failsafe configuration.
MIN_FLIGHT_ALTITUDE_M = 5.0

# Maximum altitude our application allows the mission to use.
MAX_FLIGHT_ALTITUDE_M = 50.0

# Altitude commanded during takeoff. This is a TARGET, not a limit --
# keep it distinct from MIN_FLIGHT_ALTITUDE_M, which is the floor the
# monitor should never let the mission fall below once airborne.
TAKEOFF_ALTITUDE_M = 10.0

# Altitude used for the observation hold. Must be a usable altitude for
# traffic observation, not simply the minimum floor.
OBSERVATION_ALTITUDE_M = 20.0


# ============================================================
# FLIGHT MONITORING LIMITS
# ============================================================

# Maximum horizontal speed allowed by our application, in m/s.
#
# This must stay AT OR BELOW the PX4 MPC_XY_VEL_MAX parameter on the
# target airframe, or this check can never fire because PX4 itself will
# never command a faster horizontal speed. flight/px4_params.py checks
# this consistency at preflight and warns (does not fail) if violated.
MAX_HORIZONTAL_SPEED_M_S = 10.0

# Maximum ascent / descent speed allowed by our application, in m/s.
#
# PX4 itself throttles ascent and descent asymmetrically (SITL defaults:
# MPC_Z_VEL_MAX_UP=3.0, MPC_Z_VEL_MAX_DN=1.5 -- descent is throttled
# harder than climb, a common multicopter safety convention). A single
# symmetric app-level limit can't safely sit below both without either
# being a dead check on ascent or false-tripping normal descent, so this
# project tracks them separately, mirroring PX4's own split. Each must
# stay at or below its corresponding PX4 parameter (flight/px4_params.py
# checks this consistency at preflight and warns, does not fail, if
# violated); the values below sit with margin under PX4 SITL's defaults
# so a normal transit climb/descent for this project's low, slow
# observation profile does not trip them, while a genuine runaway still
# would.
MAX_ASCENT_SPEED_M_S = 2.5
MAX_DESCENT_SPEED_M_S = 1.3

# Maximum horizontal distance from the home position that any waypoint
# may target, and that the monitor allows the vehicle to drift to. This
# is an application-level geofence in addition to (never instead of)
# PX4's own GF_MAX_HOR_DIST geofence.
MAX_DISTANCE_FROM_HOME_M = 500.0


# ============================================================
# WAYPOINT ARRIVAL
# ============================================================

# Default horizontal distance from a waypoint's target position that
# counts as "arrived". Individual Waypoint objects may override this.
WAYPOINT_ACCEPTANCE_RADIUS_M = 2.0

# Default altitude error that counts as "arrived" for a waypoint.
WAYPOINT_ALTITUDE_TOLERANCE_M = 1.0

# Time allowed for the drone to reach a mission waypoint before the
# application considers the operation failed.
WAYPOINT_TIMEOUT = 120.0

# Time allowed for the takeoff climb to reach its target altitude.
TAKEOFF_TIMEOUT_S = 30.0

# Time allowed for PX4 to complete a landing and disarm once a land/RTL
# command has been accepted.
LANDING_TIMEOUT_S = 120.0


# ============================================================
# OBSERVATION MISSION
# ============================================================

# Desired time spent recording traffic at the intersection.
#
# 15 minutes = 900 seconds.
OBSERVATION_DURATION = 15 * 60

# Maximum amount of time allowed for the complete mission.
#
# This includes:
#     takeoff
#     travelling to the intersection
#     observation
#     return
#     landing
#
# This prevents an application-level mission from running
# indefinitely.
MISSION_TIMEOUT = 30 * 60


# ============================================================
# VIDEO
# ============================================================

# Maximum recording duration, enforced by the camera recorder
# independently of OBSERVATION_DURATION. Kept separate because the
# camera may start/stop on a slightly different schedule than the
# mission's observation hold, and because a stuck recorder must not be
# allowed to run indefinitely if a stop_recording() call is ever missed.
VIDEO_DURATION = 15 * 60


# ============================================================
# CAMERA
# ============================================================

# Which Recorder implementation camera/factory.py constructs.
#
# "simulation" -- camera/recorder.py SimulationRecorder. No hardware, no
#     video file. This must remain the default until the Raspberry Pi
#     Camera Module 3 is physically available and validated.
# "picamera2"  -- camera/pi_camera.py PiCamera2Recorder. Real hardware
#     capture. Do not select this on a machine without Picamera2/libcamera
#     installed; the import is deferred into the constructor for exactly
#     this reason.
CAMERA_BACKEND = "simulation"


# ============================================================
# EMERGENCY RESPONSE
# ============================================================

# Maximum time our application waits for an emergency command
# to complete/respond.
EMERGENCY_COMMAND_TIMEOUT = 5.0


# ============================================================
# HOME POSITION / PREFLIGHT
# ============================================================

# Whether the application requires PX4 to report a valid
# home position before starting the mission.
REQUIRE_HOME_POSITION = True

# Whether the application requires all expected preflight
# health checks to pass before starting the mission.
REQUIRE_PREFLIGHT_CHECKS = True


# ============================================================
# DEBUGGING
# ============================================================

# Reserved for future verbose-logging control. Not currently consumed
# anywhere -- flagged here rather than silently wired into ad hoc print
# statements, since that would touch every module for no present benefit.
DEBUG = True


# ============================================================
# OBSERVATION WAYPOINT / MISSION SITE
# ============================================================
#
# MISSION_SITE selects which named site below the mission targets.
# Real observation sites must be added as their own named entries in
# MISSION_SITES -- never edit GAZEBO_TEST_WAYPOINT itself. That way, if
# MISSION_SITE is ever left unset or misconfigured, the application falls
# back to the harmless Gazebo default instead of a stale real-world
# coordinate silently being used.

# Gazebo test waypoint: about 100 m north of the default PX4 SITL home.
GAZEBO_TEST_WAYPOINT = {
    "latitude": 47.3988708,
    "longitude": 8.5461636,
}

MISSION_SITES = {
    "gazebo_test": GAZEBO_TEST_WAYPOINT,
    # "<real_site_name>": {"latitude": ..., "longitude": ...},
}

# Which entry of MISSION_SITES is currently active. Must never default to
# a real-world site in version control.
MISSION_SITE = "gazebo_test"

OBSERVATION_LATITUDE = MISSION_SITES[MISSION_SITE]["latitude"]
OBSERVATION_LONGITUDE = MISSION_SITES[MISSION_SITE]["longitude"]


def validate_settings():
    """
    Assert that the configured thresholds are mutually consistent.

    Intended to be called once at application startup (see main.py) so a
    misconfiguration is caught before any PX4 command is issued, rather
    than discovered mid-flight as an unreachable safety check.
    """

    if not (
        BATTERY_PREFLIGHT_MIN_PERCENT
        > BATTERY_RTL_THRESHOLD
        > BATTERY_CRITICAL_THRESHOLD
    ):
        raise ValueError(
            "Battery thresholds must satisfy "
            "BATTERY_PREFLIGHT_MIN_PERCENT > BATTERY_RTL_THRESHOLD > "
            "BATTERY_CRITICAL_THRESHOLD, got "
            f"{BATTERY_PREFLIGHT_MIN_PERCENT} > {BATTERY_RTL_THRESHOLD} > "
            f"{BATTERY_CRITICAL_THRESHOLD}."
        )

    if not (
        MIN_FLIGHT_ALTITUDE_M
        < TAKEOFF_ALTITUDE_M
        <= OBSERVATION_ALTITUDE_M
        < MAX_FLIGHT_ALTITUDE_M
    ):
        raise ValueError(
            "Altitude settings must satisfy MIN_FLIGHT_ALTITUDE_M < "
            "TAKEOFF_ALTITUDE_M <= OBSERVATION_ALTITUDE_M < "
            f"MAX_FLIGHT_ALTITUDE_M, got {MIN_FLIGHT_ALTITUDE_M} < "
            f"{TAKEOFF_ALTITUDE_M} <= {OBSERVATION_ALTITUDE_M} < "
            f"{MAX_FLIGHT_ALTITUDE_M}."
        )

    if MAX_DISTANCE_FROM_HOME_M <= 0:
        raise ValueError("MAX_DISTANCE_FROM_HOME_M must be positive.")

    if WAYPOINT_ACCEPTANCE_RADIUS_M <= 0:
        raise ValueError("WAYPOINT_ACCEPTANCE_RADIUS_M must be positive.")

    if WAYPOINT_ALTITUDE_TOLERANCE_M <= 0:
        raise ValueError("WAYPOINT_ALTITUDE_TOLERANCE_M must be positive.")

    if MISSION_SITE not in MISSION_SITES:
        raise ValueError(
            f"MISSION_SITE={MISSION_SITE!r} is not a key in MISSION_SITES."
        )

    if CAMERA_BACKEND not in ("simulation", "picamera2"):
        raise ValueError(
            f"CAMERA_BACKEND={CAMERA_BACKEND!r} is not a recognised "
            "backend (expected 'simulation' or 'picamera2')."
        )
