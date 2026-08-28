# Drone Traffic Observation Project

A drone application that flies an observation mission over an
intersection (or similar site) in PX4 SITL/Gazebo, records the
observation with a (currently simulated) camera, and offline-analyzes
the recording with a trained YOLO model to produce a traffic-volume
report.

**Status: SITL/Gazebo only.** Final hardware (Raspberry Pi + Camera
Module 3) is not available yet. Nothing here has been validated on real
hardware — see [Safety status](#safety-status) before assuming any of
this is flight-ready outside simulation.

## Repository layout

```
main.py                  Entry point: builds and runs one observation mission
config/settings.py       All application-level tunables (units + reference frames documented inline)

flight/                  PX4/MAVSDK integration -- flight-safety-critical
  drone.py                 Owns the MAVSDK System: connect/arm/takeoff/goto/land/RTL/hold
  telemetry.py              Bounded, always-closed telemetry reads
  monitor.py                 Continuous in-flight safety checks (battery/altitude/speed/geofence/timeout)
  emergency.py               Exception -> response policy (RTL / land / abort-without-command)
  safety.py                   Preflight checks (GPS, home, battery, PX4 parameter audit)
  px4_params.py               Read-only PX4 parameter audit (never writes a PX4 parameter)
  geo.py                       Pure geographic distance math
  exceptions.py                 The exception taxonomy Emergency dispatches on

mission/                 Mission planning and execution
  position.py               Geographic point with an explicit altitude reference (relative-to-home vs AMSL)
  waypoint.py                 One target + arrival tolerances + hold/observation time
  mission.py                   Ordered waypoints, status, validation (no MAVSDK dependency)
  mission_manager.py            Executes a Mission: sequencing, monitor coordination, camera, RTL

camera/                  Camera + offline vision analysis
  recorder.py               Recorder interface + SimulationRecorder (current, no hardware)
  pi_camera.py               Real Picamera2 implementation (hardware-gated, not yet tested)
  factory.py                  Selects the recorder backend from config.settings.CAMERA_BACKEND
  detector.py / processor.py    YOLO detection+tracking / ROI lane counting (offline only)
  detections.py                 Shared helper: reads results.boxes OR results.obb (OBB models)
  traffic_metrics.py             DensityAnalyzer + ScreenlineAnalyzer: time-windowed PCU-weighted reports
  analyze_video.py                 CLI: run a model over a recording (--mode density|screenline), write CSV + plot

tests/
  unit/                    Fast, no-MAVSDK-no-hardware tests for flight/ and mission/
  vision/                    Tests for camera/ analysis code (needs requirements-vision.txt)
  sitl/                       Gazebo/SITL integration scenarios (needs a running PX4 SITL instance)

docs/px4_parameter_checklist.md   Human-reviewed PX4 parameter checklist (pairs with px4_params.py)
```

## Setup

Two separate dependency sets, installed separately on purpose: the
flight process must never depend on the vision/ML stack.

```bash
python3 -m venv .venv
source .venv/bin/activate

pip install -r requirements.txt          # flight-critical: mavsdk, pytest
pip install -r requirements-vision.txt   # offline analysis only: ultralytics, opencv, matplotlib
```

`requirements-vision.txt` is only needed if you're running the camera
analysis tools (`camera/analyze_video.py`, `tests/vision`). It is never
required to fly a mission or run the flight-critical tests.

## Running the fast test suite (no drone, no PX4, no GPU needed)

```bash
pytest tests/unit -q
```

99 tests covering flight/mission logic, the exception-dispatch policy,
mission validation, config consistency, and the PX4 parameter audit —
all against fakes, no MAVSDK connection required. This is the suite to
run on every change.

```bash
pytest tests/vision -q
```

44 tests for the offline traffic-analysis code (needs
`requirements-vision.txt` installed).

## Flying a mission in SITL/Gazebo

You need a PX4 SITL + Gazebo instance running. From a PX4-Autopilot
checkout:

```bash
make px4_sitl gz_x500
```

This starts PX4 SITL and Gazebo. MAVSDK connects on UDP `14540`
(`config/settings.PX4_CONNECTION_ADDRESS`); QGroundControl can observe
independently on UDP `14550`.

Then, in this project (with `requirements.txt` installed):

```bash
python main.py
```

This runs one complete mission: connect → preflight (incl. PX4 parameter
audit) → validate → arm → takeoff → fly to the configured observation
waypoint → record (simulated) → RTL → land → disarm.

### SITL integration scenarios

`tests/sitl/scenarios.py` covers specific fault scenarios end-to-end
against a live SITL instance (not run by `pytest tests/unit`):

```bash
python -m tests.sitl.run --scenario nominal
python -m tests.sitl.run --scenario low-battery
python -m tests.sitl.run --scenario link-loss        # leaves the vehicle airborne on pass -- see below
python -m tests.sitl.run --scenario operator-abort    # leaves the vehicle airborne on pass -- see below
```

Full list: `nominal`, `multi-waypoint`, `low-battery`, `home-not-ready`,
`altitude-violation`, `mission-timeout`, `link-loss`, `operator-abort`.

`link-loss` and `operator-abort` **intentionally leave the vehicle
airborne when they pass** — issuing no RTL/land command is exactly what
they're testing (PX4's own failsafe, or the RC pilot/QGroundControl, is
supposed to be responsible for the aircraft in those situations, not
this application). Recover the vehicle manually via QGroundControl (or
just restart SITL) before running a scenario that assumes the vehicle
starts on the ground.

## Configuration

Everything tunable lives in `config/settings.py`, with units and
altitude-reference frame documented on every value. Notable ones:

- `MISSION_SITE` / `MISSION_SITES` — which named waypoint the mission
  flies to. Defaults to the harmless Gazebo test site; add real sites as
  their own named entries rather than editing the Gazebo default in
  place, so an unset/misconfigured `MISSION_SITE` always falls back to
  something harmless instead of a stale real-world coordinate.
- `CAMERA_BACKEND` — `"simulation"` (default, no hardware) or
  `"picamera2"` (real Raspberry Pi Camera Module 3, once available).
- `TAKEOFF_ALTITUDE_M`, `OBSERVATION_ALTITUDE_M`,
  `MIN_/MAX_FLIGHT_ALTITUDE_M`, `MAX_DISTANCE_FROM_HOME_M`,
  `MAX_HORIZONTAL_SPEED_M_S`, `MAX_ASCENT_/MAX_DESCENT_SPEED_M_S`,
  `BATTERY_PREFLIGHT_MIN_PERCENT`/`BATTERY_RTL_THRESHOLD`/`BATTERY_CRITICAL_THRESHOLD`.

`config.settings.validate_settings()` (called at startup by `main.py`)
enforces that these are mutually consistent (e.g. thresholds ordered
correctly, altitude window sane). `flight/px4_params.py` separately
warns at every preflight if an application limit is looser than the
real PX4 airframe's own configured limit — read `docs/px4_parameter_checklist.md`
before ever flying real hardware.

## Offline traffic analysis

Once you have a recorded video (from a real flight, or any drone
footage) and a trained YOLO model (standard or OBB task), there are two
analysis modes — pick the one that matches your footage.

### `--mode density` (default) — occupancy / congestion

```bash
python -m camera.analyze_video \
    --model models/best.pt \
    --video /path/to/recording.mp4 \
    --window-seconds 10
```

Produces `density_report.csv` / `.png`: per time window, the mean and peak
number of vehicles visible per frame (overall and per class), the mean
PCU-weighted occupancy, and how many distinct vehicles were seen. This
is the right measure for a wide-area drone-hover view where many
vehicles are in frame at once. It deliberately does **not** report a
vehicles/hour flow rate — counting "vehicles that appeared somewhere in
a wide field of view" and annualising it is not a meaningful flow
measurement.

### `--mode screenline` — actual traffic flow

```bash
python -m camera.analyze_video \
    --model models/best.pt \
    --video /path/to/recording.mp4 \
    --mode screenline \
    --line northbound:640,0,640,720 \
    --line eastbound:0,360,1280,360 \
    --window-seconds 10
```

Produces `screenline_report.csv` / `.png`: for each counting line and
each direction across it, the number of vehicles whose track crossed the
line per time window, per class, plus the flow rate in vehicles/hour
(`q = n / T`, the standard Highway Capacity Manual formula — valid here
because a line crossing *is* a cross-section count). Each `--line` is
`[NAME:]X1,Y1,X2,Y2` in pixel coordinates (origin top-left); repeat it
for multiple lines. Direction is reported as `+`/`-` (a documented
convention — one run shows you which physical heading each is).

### PCU weights

Both modes weight vehicle classes by Passenger Car Unit factors (a bus
or truck occupies more road capacity than a car). Defaults are in
`camera/traffic_metrics.py:PCU_FACTORS`; override per-run without
editing code:

```bash
--pcu van=1.4 bus=2.2
```

Unknown class names fall back to 1.0 with a printed warning. The run
prints the full class→weight table it will use before starting.

### Seeing the detections (demos)

Add `--save-annotated` to also write `<output-dir>/<mode>_annotated.mp4`
with detection boxes, class labels, and track IDs drawn on every frame
(plus the counting lines, in screenline mode). `--show` opens a live
preview window instead (needs a display; on WSL, WSLg or an X server).
`--max-seconds N` stops early, and `--annotated-scale 0.5` shrinks the
output video (analysis unaffected) — handy for a shareable demo clip:

```bash
python -m camera.analyze_video --model models/best.pt --video clip.mp4 \
    --save-annotated --max-seconds 30 --annotated-scale 0.5
```

OpenCV's `mp4v` codec is not very space-efficient; re-encode the result
with ffmpeg (`ffmpeg -i in.mp4 -c:v libx264 -crf 23 out.mp4`) for a
~10x smaller file if you need to email it.

See `camera/traffic_metrics.py` for the methodology behind each mode.

### Model weights

Not committed to this repository (see `.gitignore` — `models/`, `*.pt`)
since they're large binary artifacts, not source. Get `best.pt` from
whoever trained it, place it in `models/` (or anywhere), and pass its
path via `--model`.

## Safety status

- **PX4 is the final flight-safety authority.** This application never
  attempts to replace PX4's own failsafes for battery, RC loss,
  data-link loss, GPS/navigation loss, geofence, or flight termination —
  it detects problems early and aborts its own mission bookkeeping, but
  the aircraft's actual response in a lost-link or unhealthy-navigation
  state is PX4's, not this code's. See `flight/emergency.py`.
- **SITL/Gazebo success does not prove hardware readiness.** Things SITL
  cannot validate: real battery percentage fidelity, GPS acquisition/fix
  quality, compass/EKF health, ground effect on landing detection,
  achievable waypoint accuracy in wind, the Pi↔PX4 companion-link's real
  reliability, and camera/encoder performance under real thermal and I/O
  load. Do not fly real hardware without working through
  `docs/px4_parameter_checklist.md` and a staged flight-test plan
  (bench → tethered hover → short low-altitude flight → full mission).
- `flight/px4_params.py` reads PX4 parameters read-only, purely to log
  them and warn about inconsistencies — it never writes a PX4 parameter.
