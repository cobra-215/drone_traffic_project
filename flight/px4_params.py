"""
Read-only audit of PX4 parameters relevant to this application's safety
assumptions.

STRICTLY READ-ONLY: this module must never call set_param_* on the MAVSDK
param plugin. It exists to (a) record the aircraft's actual failsafe
configuration alongside each mission for post-flight review, and (b) warn
-- never fail -- when an application-level limit in config/settings.py
cannot actually fire because it is looser than PX4's own configured limit
(see defect D5 in the project's implementation plan).

The parameter names below are PX4's standard documented parameters. Only
the MAVSDK param plugin API itself (get_param_float/get_param_int) has
been verified against the installed MAVSDK package by inspecting it
directly; the individual PX4 parameter names have NOT been confirmed
against a live PX4 instance by this code. Firmware versions do
rename/remove parameters, so every read here is individually guarded: a
missing parameter or a read timeout produces a warning, never an
exception, and never blocks arming. Re-confirm this list against
`param show` on the actual target firmware before hardware flights -- see
docs/px4_parameter_checklist.md for the human-reviewed counterpart of
this automated check.
"""

import asyncio

from config import settings


FLOAT_PARAMETERS = [
    "BAT_LOW_THR",
    "BAT_CRIT_THR",
    "BAT_EMERGEN_THR",
    "COM_RC_LOSS_T",
    "COM_DL_LOSS_T",
    "GF_MAX_HOR_DIST",
    "GF_MAX_VER_DIST",
    "RTL_RETURN_ALT",
    "RTL_DESCEND_ALT",
    "RTL_LAND_DELAY",
    "MPC_XY_VEL_MAX",
    "MPC_Z_VEL_MAX_UP",
    "MPC_Z_VEL_MAX_DN",
    "MIS_TAKEOFF_ALT",
    "COM_DISARM_LAND",
]

INT_PARAMETERS = [
    "COM_LOW_BAT_ACT",
    "NAV_RCL_ACT",
    "NAV_DLL_ACT",
    "GF_ACTION",
]


class PX4ParameterAudit:
    """Read a fixed set of PX4 parameters and log them. Never raises."""

    def __init__(self, drone, timeout=None):
        self.drone = drone
        self.timeout = (
            timeout if timeout is not None else settings.TELEMETRY_TIMEOUT
        )

    async def _read_float(self, name):
        try:
            return await asyncio.wait_for(
                self.drone.system.param.get_param_float(name),
                timeout=self.timeout,
            )
        except Exception as e:
            print(f"PX4 parameter audit: could not read {name} ({e}). Skipping.")
            return None

    async def _read_int(self, name):
        try:
            return await asyncio.wait_for(
                self.drone.system.param.get_param_int(name),
                timeout=self.timeout,
            )
        except Exception as e:
            print(f"PX4 parameter audit: could not read {name} ({e}). Skipping.")
            return None

    async def read_all(self):
        """Read every configured parameter. Never raises."""

        values = {}
        for name in FLOAT_PARAMETERS:
            values[name] = await self._read_float(name)
        for name in INT_PARAMETERS:
            values[name] = await self._read_int(name)
        return values

    def check_consistency(self, values):
        """
        Compare application limits against the PX4 values just read.

        Returns a list of human-readable warning strings; never raises. A
        parameter that could not be read is simply skipped rather than
        warned about, since we have no value to compare.
        """

        warnings = []

        px4_xy_max = values.get("MPC_XY_VEL_MAX")
        if (
            px4_xy_max is not None
            and settings.MAX_HORIZONTAL_SPEED_M_S > px4_xy_max
        ):
            warnings.append(
                "MAX_HORIZONTAL_SPEED_M_S "
                f"({settings.MAX_HORIZONTAL_SPEED_M_S:.1f}) exceeds PX4's "
                f"MPC_XY_VEL_MAX ({px4_xy_max:.1f}); this application "
                "check can never fire."
            )

        px4_z_up = values.get("MPC_Z_VEL_MAX_UP")
        if px4_z_up is not None and settings.MAX_ASCENT_SPEED_M_S > px4_z_up:
            warnings.append(
                "MAX_ASCENT_SPEED_M_S "
                f"({settings.MAX_ASCENT_SPEED_M_S:.1f}) exceeds PX4's "
                f"MPC_Z_VEL_MAX_UP ({px4_z_up:.1f}); this application "
                "check can never fire on ascent."
            )

        px4_z_dn = values.get("MPC_Z_VEL_MAX_DN")
        if px4_z_dn is not None and settings.MAX_DESCENT_SPEED_M_S > px4_z_dn:
            warnings.append(
                "MAX_DESCENT_SPEED_M_S "
                f"({settings.MAX_DESCENT_SPEED_M_S:.1f}) exceeds PX4's "
                f"MPC_Z_VEL_MAX_DN ({px4_z_dn:.1f}); this application "
                "check can never fire on descent."
            )

        gf_ver = values.get("GF_MAX_VER_DIST")
        if (
            gf_ver is not None
            and gf_ver > 0
            and settings.MAX_FLIGHT_ALTITUDE_M >= gf_ver
        ):
            warnings.append(
                "MAX_FLIGHT_ALTITUDE_M "
                f"({settings.MAX_FLIGHT_ALTITUDE_M:.1f}) is at or beyond "
                f"PX4's geofence GF_MAX_VER_DIST ({gf_ver:.1f}); the "
                "geofence -- not this application check -- would act "
                "first."
            )

        gf_hor = values.get("GF_MAX_HOR_DIST")
        if (
            gf_hor is not None
            and gf_hor > 0
            and settings.MAX_DISTANCE_FROM_HOME_M >= gf_hor
        ):
            warnings.append(
                "MAX_DISTANCE_FROM_HOME_M "
                f"({settings.MAX_DISTANCE_FROM_HOME_M:.1f}) is at or "
                f"beyond PX4's geofence GF_MAX_HOR_DIST ({gf_hor:.1f}); "
                "the geofence -- not this application check -- would act "
                "first."
            )

        return warnings

    async def run(self):
        """Read all parameters, log them, warn on inconsistency. Never raises."""

        try:
            values = await self.read_all()
        except Exception as e:
            # Belt-and-suspenders: read_all() already guards every
            # individual read, but the audit must never be able to block
            # arming even if something unexpected happens here.
            print(f"PX4 parameter audit failed unexpectedly ({e}). Continuing.")
            return {}, []

        print("PX4 parameter audit (read-only):")
        for name, value in values.items():
            print(f"  {name} = {value if value is not None else 'unavailable'}")

        warnings = self.check_consistency(values)
        for warning in warnings:
            print(f"PX4 parameter audit WARNING: {warning}")

        return values, warnings
