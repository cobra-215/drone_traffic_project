# ============================================================
# config/settings.py
#
# Central configuration for the drone traffic-monitoring
# application.
#
# IMPORTANT:
# These values configure OUR APPLICATION.
# PX4's own safety/failsafe parameters remain configured
# inside PX4.
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
# BATTERY
# ============================================================

# Application-level battery threshold.
#
# When the mission detects that battery is below this level,
# the application should stop the mission and request RTL.
BATTERY_RTL_THRESHOLD = 30.0

# Critical battery threshold.
#
# At or below this level, the application must not continue
# the mission.
BATTERY_CRITICAL_THRESHOLD = 20.0


# ============================================================
# ALTITUDE
# ============================================================

# Minimum altitude allowed during the mission.
#
# This is an APPLICATION constraint, not a replacement for
# PX4's altitude/failsafe configuration.
MIN_FLIGHT_ALTITUDE = 5.0

# Maximum altitude our application allows the mission to use.
MAX_FLIGHT_ALTITUDE = 50.0



# ============================================================
# FLIGHT MONITORING LIMITS
# ============================================================

# Maximum horizontal speed allowed by our application.
MAX_HORIZONTAL_SPEED = 15.0

# Maximum absolute vertical speed allowed by our application.
MAX_VERTICAL_SPEED = 5.0



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

# Maximum recording duration.
#
# Keep this separate from OBSERVATION_DURATION because later
# the camera may start/stop independently of the mission.
VIDEO_DURATION = 15 * 60


# ============================================================
# EMERGENCY RESPONSE
# ============================================================

# Maximum time our application waits for an emergency command
# to complete/respond.
EMERGENCY_COMMAND_TIMEOUT = 5.0


# ============================================================
# MISSION TIMEOUT
# ============================================================

# Time allowed for the drone to reach a mission waypoint
# before the application considers the operation failed.
WAYPOINT_TIMEOUT = 120.0


# ============================================================
# HOME POSITION
# ============================================================

# Whether the application requires PX4 to report a valid
# home position before starting the mission.
REQUIRE_HOME_POSITION = True


# ============================================================
# PREFLIGHT
# ============================================================

# Whether the application requires all expected preflight
# health checks to pass before starting the mission.
REQUIRE_PREFLIGHT_CHECKS = True


# ============================================================
# DEBUGGING
# ============================================================

# Enable additional application diagnostic output.
DEBUG = True


# ============================================================
# OBSERVATION WAYPOINT
# ============================================================

# Gazebo test waypoint: about 100 m north of the default PX4 SITL home.
# Replace these with the approved real observation location later.
OBSERVATION_LATITUDE = 47.3988708
OBSERVATION_LONGITUDE = 8.5461636

# Metres relative to home/takeoff altitude.
OBSERVATION_ALTITUDE = MIN_FLIGHT_ALTITUDE