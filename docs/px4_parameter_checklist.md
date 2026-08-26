# PX4 parameter pre-flight checklist

This is the human-signed counterpart to the automated, read-only audit in
`flight/px4_params.py`. The automated audit proves what PX4 *is currently
configured to do*; this checklist is where a person confirms what it
*should* be configured to do before an outdoor flight. Neither replaces
the other, and this project never writes PX4 parameters from Python --
any correction found here is made in QGroundControl.

Do not fly outdoors until every row has been checked against the actual
airframe's parameters (e.g. via QGroundControl's parameter list, or the
PX4 shell `param show <name>`), and the values have been recorded below
with the date and the reviewer.

| Parameter | Purpose | Expected / reviewed value | Reviewed by | Date |
|---|---|---|---|---|
| `BAT_LOW_THR` | Battery low threshold | | | |
| `BAT_CRIT_THR` | Battery critical threshold | | | |
| `BAT_EMERGEN_THR` | Battery emergency threshold | | | |
| `COM_LOW_BAT_ACT` | Action on low battery | | | |
| `NAV_RCL_ACT` | Action on RC loss | | | |
| `COM_RC_LOSS_T` | RC loss timeout | | | |
| `COM_RC_IN_MODE` | RC input requirement | | | |
| `NAV_DLL_ACT` | Action on data-link loss | | | |
| `COM_DL_LOSS_T` | Data-link loss timeout | | | |
| `COM_OBL_ACT` | Action on offboard loss (if used) | | | |
| `GF_ACTION` | Geofence breach action | | | |
| `GF_MAX_HOR_DIST` | Geofence horizontal radius | | | |
| `GF_MAX_VER_DIST` | Geofence vertical limit | | | |
| `RTL_TYPE` | RTL path behaviour | | | |
| `RTL_RETURN_ALT` | RTL return altitude | | | |
| `RTL_DESCEND_ALT` | RTL descent altitude | | | |
| `RTL_LAND_DELAY` | Hover-before-land delay | | | |
| `MPC_XY_VEL_MAX` | Max horizontal speed | | | |
| `MPC_XY_CRUISE` | Cruise horizontal speed | | | |
| `MPC_Z_VEL_MAX_UP` | Max ascent speed | | | |
| `MPC_Z_VEL_MAX_DN` | Max descent speed | | | |
| `MIS_TAKEOFF_ALT` | Takeoff altitude | | | |
| `COM_DISARM_LAND` | Auto-disarm delay after landing | | | |
| `CBRK_*` (all) | Safety circuit breakers | must all be at their default (disarmed) value | | |

Reminders:

- `config/settings.py`'s `MAX_HORIZONTAL_SPEED_M_S` / `MAX_ASCENT_SPEED_M_S`
  / `MAX_DESCENT_SPEED_M_S` / `MAX_FLIGHT_ALTITUDE_M` /
  `MAX_DISTANCE_FROM_HOME_M` must stay at or below the corresponding PX4
  limits above (ascent against `MPC_Z_VEL_MAX_UP`, descent against the
  usually-stricter `MPC_Z_VEL_MAX_DN`), or the application-level check can
  never fire (see `flight/px4_params.py` and defect D5 in the project
  plan). The automated audit warns about this at every preflight; this
  checklist is where the values are actually corrected.
- SITL success never substitutes for this checklist. The parameter values
  above are specific to the real airframe and must be re-reviewed if the
  airframe, firmware, or PX4 version changes.
