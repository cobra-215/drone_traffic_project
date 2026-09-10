# Raspberry Pi setup

Bringing up the Raspberry Pi + Camera Module 3 for this project.

**Scope: camera only.** This covers getting the Pi recording video
through this project's own code path. It does **not** cover connecting
the Pi to a flight controller — see [Later: the PX4
link](#later-the-px4-link) for what will need verifying when one exists.

Nothing here has been validated on real hardware yet. Treat every step
as something to confirm, not something to assume.

---

## 1. Confirm the camera at the OS level first

Before touching Python. If libcamera can't see the camera, no amount of
Python debugging will help.

Raspberry Pi OS **Bookworm 64-bit** is the assumed image.

```bash
rpicam-hello --list-cameras     # Bookworm
libcamera-hello --list-cameras  # older Bullseye name
```

You should see the Camera Module 3 listed with its modes. If not:

- **Ribbon cable orientation.** The blue backing faces the USB/Ethernet
  side of the board; the silver contacts face the HDMI side.
- **Pi 5 uses a different connector.** The Pi 5 has two narrow CAM/DISP
  connectors, not the Pi 4's single wide CSI one. A Camera Module 3
  shipped with a standard cable will **not** fit a Pi 5 — you need the
  narrow 22-pin adapter cable.
- Power the Pi down completely before reseating the cable.

## 2. Install picamera2 from apt, not pip

```bash
sudo apt update
sudo apt install -y python3-picamera2
```

**picamera2 must come from apt.** It does not pip-install cleanly: it
binds to the system libcamera stack, and a pip build will either fail or
produce a package that cannot open the camera.

## 3. Create the venv with `--system-site-packages`

This is the single most common way this setup fails. A normal venv
cannot see the apt-installed picamera2, so `from picamera2 import
Picamera2` raises `ModuleNotFoundError` inside a venv that looks
perfectly healthy from the outside.

```bash
cd ~/drone_traffic_project
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

python -c "from picamera2 import Picamera2; print('picamera2 OK')"
```

If that import fails, stop and fix it before going further.

## 4. Install the flight requirements — and only those

```bash
pip install -r requirements.txt
```

**Never install `requirements-vision.txt` on the Pi.** Keeping
torch/ultralytics/matplotlib off the flight process is precisely why
this project splits its dependencies into two files. The Pi records;
analysis happens afterwards on a laptop.

## 5. Verify the picamera2 API before first run

`camera/pi_camera.py` was written against picamera2's documented
interface but has never been executed against the installed package.
This project's standing rule is never to assume a hardware API — inspect
it first:

```bash
python3 -c "from picamera2 import Picamera2; help(Picamera2.create_video_configuration)"
python3 -c "from picamera2.encoders import H264Encoder; help(H264Encoder.__init__)"
python3 -c "from picamera2.outputs import FfmpegOutput; help(FfmpegOutput.__init__)"
python3 -c "from picamera2 import Picamera2; help(Picamera2.start_recording)"
```

Correct `camera/pi_camera.py` to match whatever the installed version
actually exposes. The calls to check are in `_open_and_start()` and
`_stop_and_close()`.

## 6. Point the settings at the real camera

In `config/settings.py`:

```python
CAMERA_BACKEND = "picamera2"
RECORDING_OUTPUT_DIR = "~/recordings"   # "~" is expanded at use
CAMERA_RESOLUTION = (1920, 1080)
CAMERA_FRAMERATE = 30
CAMERA_BITRATE = 10_000_000
```

`CAMERA_BACKEND` stays `"simulation"` in version control — this repo is
also checked out on machines with no camera. Changing it is a
deliberate, Pi-local edit.

`RECORDING_OUTPUT_DIR` is `~`-relative on purpose: Bookworm has no `pi`
user unless you create one, so a hardcoded `/home/pi/...` would be wrong
on most modern images.

## 7. Run the bring-up check

```bash
python -m camera.camera_check --seconds 10
```

This runs the recorder's preflight check, records for 10 seconds, stops,
and prints the resulting file, its size and its sidecar metadata. It
drives the *same* code path `MissionManager` uses, so a pass means the
recorder works — not that a parallel test script works.

To test the real camera without editing settings:

```bash
python -m camera.camera_check --seconds 10 --backend picamera2
```

Then copy the `.mp4` off the Pi and run it through
`camera/analyze_video.py` on the laptop. That closes the loop from
capture to report.

## 8. Run the flight-critical test suite

```bash
pytest tests/unit -q
```

These need no camera and no PX4 and must pass on the Pi exactly as they
do on the development machine.

---

## Storage

At `CAMERA_BITRATE = 10_000_000` (10 Mbit/s):

| Recording | Approximate size |
|---|---|
| 1 minute | 75 MB |
| 15 minutes (`VIDEO_DURATION`) | ~1.1 GB |

Use a fast A2-rated SD card, or better, record to a USB SSD:

```bash
python -m camera.camera_check --seconds 30 --output-dir /mnt/ssd/recordings
```

`PiCamera2Recorder.preflight_check()` refuses to start if there isn't
room for a full `VIDEO_DURATION` recording, so a full card is caught on
the ground rather than partway through an observation hold.

`VIDEO_DURATION` is also enforced as a hard cap by the recorder itself:
if a `stop_recording()` call is ever missed, a watchdog stops the camera
anyway rather than letting it run until the card fills.

## Thermals

Sustained 1080p30 H.264 encoding is a real thermal load, and a throttled
Pi drops frames. Use active cooling (the official Pi 5 Active Cooler, or
any case fan). Watch for it during a long test:

```bash
vcgencmd measure_temp
vcgencmd get_throttled     # 0x0 means no throttling has occurred
```

A 15-minute `--seconds 900` run on the bench, with the enclosure closed,
is the honest test — a 10-second run proves nothing about thermals.

---

## Later: the PX4 link

None of this is verified — there is no flight controller connected yet.
When there is:

**1. Confirm mavsdk works on ARM.** The `mavsdk` Python package bundles
a per-platform `mavsdk_server` binary. Confirm an arm64 one shipped and
that it actually executes:

```bash
python3 -c "import mavsdk, os; print(os.listdir(os.path.join(os.path.dirname(mavsdk.__file__), 'bin')))"
```

**2. Set the serial address.** `config/settings.py` keeps the SITL UDP
address as its default. A Pi wired to a flight controller's TELEM port
normally needs something like:

```python
PX4_CONNECTION_ADDRESS = "serial:///dev/serial0:921600"
```

Confirm the device path (`/dev/serial0`, `/dev/ttyAMA0`, `/dev/ttyUSB0`
for a USB-serial adapter) and the baud rate against the actual wiring
and the flight controller's `SER_TEL*_BAUD` parameter. Enable the UART
and disable the serial login shell in `raspi-config` if using the GPIO
pins.

**3. Do not skip the parameter checklist.** `docs/px4_parameter_checklist.md`
must be worked through before any real flight, and
`flight/px4_params.py` will warn at every preflight where an application
limit is looser than PX4's own.

**4. SITL success is not hardware readiness.** A green `tests/unit` run,
a passing `camera_check`, and a clean SITL mission establish that the
software is consistent — not that the aircraft is safe to fly. The
staged sequence (bench with no props → tethered hover → short
low-altitude flight → full mission) still applies in full.
