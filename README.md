# arm_control_webapp — unified FR3 control site

One local Flask site with three selectable modes for driving the Fairino
FR3 arm over a part mounted on the optical table:

1. **Target sphere centers** — upload an STL, pick the table hole the part's
   center-of-volume is mounted directly above, preview the planned poses,
   then execute a force-limited poke of every detected sphere along its
   surface normal. Starts with one vertical pre-probe at the selected hole,
   and ends by fitting the actual contacted points against the STL's
   nominal geometry to report where the part really is relative to the
   model.
2. **Drag-teach aim + gamepad release** — hand-guide the arm via drag-teach,
   press a gamepad button, force-limited poke forward.
3. **Height map** — vertically probes a configurable-spacing 11x11 grid
   centered on a selected table hole, force-limited at each point.

It uses `stl_geometry.py`, `table_calibration.py`, and `move_to_spheres.py`
as libraries — see those files' own docstrings for what they do.

## Run it

```bash
cd FHL26
pip install -r arm_control_webapp/requirements.txt   # flask; everything else is inherited
python3 arm_control_webapp/app.py
```

Open the printed URL. **Defaults to a fully simulated robot and sensor** —
safe to click through with zero hardware attached. Real hardware requires
an explicit opt-in:

```bash
FR3_SIMULATE_ROBOT=0 FR3_SIMULATE_SENSOR=0 python3 arm_control_webapp/app.py
```

Other env vars (all optional): `FR3_ROBOT_IP` (default `192.168.57.2`),
`FR3_SENSOR_PORT` (default `/dev/ttyUSB0`), `FR3_TABLE_CALIB` (default
`table_calibration.json`, alongside this app), `FR3_HARD_FLOOR_Z_MM`
(default `20.0` -- see "Before ever setting `FR3_SIMULATE_ROBOT=0`" below),
`FR3_WEB_HOST` / `FR3_WEB_PORT` (default `0.0.0.0:5000`).

## Table calibration

This app loads `table_calibration.json` (produced by
`../table_calibration.py capture`) — it does not build a capture UI of its
own. If that file doesn't exist yet, Mode 1's preview/execute and Mode 3
will return a 404 telling you to run `table_calibration.py capture` first.

For local dev without a robot, write a throwaway fixture instead of using
the real capture flow, e.g.:

```json
{
  "theta_deg": 0.0, "tx_mm": 400.0, "ty_mm": 0.0, "z0_mm": 100.0,
  "pitch_mm": 25.4, "n_points": 3, "residual_rms_mm": 0.1,
  "per_point_residual_mm": [0.1, 0.1, 0.1], "z_spread_mm": 0.1,
  "collinearity_ratio": 0.5, "reflection_detected": false
}
```

and point `FR3_TABLE_CALIB` at it. Keep this out of the real
`table_calibration.json` path so it never gets mistaken for a real fit.

## Testing without hardware

```bash
python3 arm_control_webapp/test_webapp_selftest.py
```

Plain-assert tests (same style as `../test_geometry_selftest.py`) covering
`probe_vertical()`, `SimulatedRobot`'s motion contract, and
`registration.fit_rigid_transform()` — no robot, sensor, or hardware SDK
needed.

With the dev table-calibration fixture above and the server running, you
can drive the whole pipeline via the GUI, or curl the JSON API directly
(`/api/mode1/upload-stl`, `/api/mode1/preview`, `/api/mode1/execute`,
`/api/mode3/start`, `/api/status`, `/api/confirm`, `/api/abort`,
`/api/simulate-collision`). With `FR3_SIMULATE_SENSOR=1` (the default),
`POST /api/simulate-collision` holds an elevated force reading for ~1s so
you can exercise the "contact reached" success path end to end without
real hardware.

## Known environment constraint (not this app's problem to fix)

Per the repo root README, `numpy`/`scipy` can fail to import with an
architecture-mismatch error on some dev machines — use a venv with a
matching-architecture Python.

## Before ever setting `FR3_SIMULATE_ROBOT=0`

The FR3 orientation convention (`ZYX` Euler, used throughout by
`move_to_spheres.target_to_pose`) is not yet hardware-verified. Run the
verification procedure documented in `../move_to_spheres.py`'s module
docstring once on the real robot before trusting any commanded orientation
this app produces — it's the same code path, so the same caveat applies.

### Running a live robot with no working force sensor

Every mode's actual motion is force-limited — it stops on the sensor's
force reading or on the sensor going stale, not on anything else. If
`FR3_SIMULATE_SENSOR=1` (or a real sensor has failed and is being run
simulated anyway) while `FR3_SIMULATE_ROBOT=0`, that stop condition can
never legitimately fire, so a force-limited move runs its *entire*
commanded distance no matter what's in the way. Mode 1's pre-probe and Mode
3's whole height-map grid are a particular case of this: their descent
target is deliberately set *past* where the surface is expected to be
(normally the force stop catches it first), which inverts into "drives
straight into the table" without one.

`FR3_HARD_FLOOR_Z_MM` (default `20.0`) is a backstop for exactly this: a
hard floor on every commanded target's Z, in the **robot's own base
frame**, checked by `move_to_spheres.z_floor_violation()` before any
force-limited move (and before Mode 1's standoff moves) is dispatched. The
default assumes the standard convention that the base frame origin sits at
the robot's mounting point with +Z up, so `20.0` means "never command the
TCP within 20mm of the robot's own base" — confirm that convention holds
for your setup (e.g. via the drag-teach + read-only pose check from the
orientation-verification procedure above) before trusting it. A target
that would violate the floor is refused and logged with an
`outcome`/`stopped_reason` explaining why, and that target/point is
skipped rather than aborting the whole run.

This is a last-resort backstop against a grossly wrong target (e.g. bad
calibration sending the tool toward the base or through the table), **not**
a substitute for a correct table calibration or for verifying targets are
sane, and it does nothing to protect against a collision above the floor
height. Test cautiously, with a hand on the E-stop, and prefer Mode 1's
default per-target standoff confirmation (`trusted_run: false`) — it pauses
above the part before every force-limited approach so you can inspect and
skip rather than letting a bad target run.

## Switching the force sensor from simulated to live

Once the real Bota sensor is working again, flip `FR3_SIMULATE_SENSOR` back
to `0`. `SIMULATE_ROBOT`/`SIMULATE_SENSOR` are independent env vars (see
`config.py`), so this doesn't touch the robot setting at all.

1. **Install the real sensor's dependencies** in the venv -- `Bota_sys.py`
   (imported lazily by `sensor_backend.RealSensorBackend`, so nothing above
   needed it while simulated) pulls in `pyserial`, `crc`, `plotly`,
   `ipython`, and `pandas`:
   ```bash
   source venv/bin/activate
   pip install pyserial crc plotly ipython pandas
   ```
2. **Find the sensor's serial port.** Plug it in and check
   `ls /dev/ttyUSB*`, or -- better, since a USB-serial device's `ttyUSB`
   number can shift depending on plug order/what else is attached -- use its
   stable path instead: `ls -l /dev/serial/by-id/` and point
   `FR3_SENSOR_PORT` at that instead of a bare `/dev/ttyUSBn`.
3. **Add the service user to the `dialout` group** so it's actually allowed
   to open the serial device (this is very likely why a first attempt would
   fail with a permissions error even with everything else correct):
   ```bash
   sudo usermod -a -G dialout pi
   ```
   Group membership only takes effect on that user's next login session --
   for a systemd service this means restarting the service is enough (it's
   a fresh session), but if you're testing by hand in an already-open
   terminal, log out and back in first.
4. **Update the systemd unit** (`/etc/systemd/system/arm-control.service`):
   change `Environment=FR3_SIMULATE_SENSOR=1` to `=0`, and add
   `Environment=FR3_SENSOR_PORT=/dev/serial/by-id/...` if it's not the
   default `/dev/ttyUSB0`.
5. **Apply it:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl restart arm-control.service
   journalctl -u arm-control.service -b | grep Sensor
   ```
   Confirm the startup banner now reads `Sensor: REAL (...)` with the
   correct port.

A few things worth knowing about real-sensor mode:
- `POST /api/simulate-collision` (and its GUI button) is unavailable --
  `app.py` refuses it whenever `SIMULATE_SENSOR` is false, since faking a
  collision on top of a real force reading would be actively misleading.
- The force sensor's reading is what actually stops a force-limited
  approach, in addition to `FR3_HARD_FLOOR_Z_MM` (harmless to leave in
  place either way -- it only ever fires on a grossly-out-of-range target).
  `force_threshold_n` (default `5.0`, settable per-run) is the number doing
  the real work -- sanity-check it against the sensor's actual noise floor
  before trusting it on a real approach, the same way the orientation
  convention above needs verifying before being trusted.
- Sensor-staleness detection (`sensor_stale_ms`, default `200`) is
  meaningful here in a way it isn't in simulated mode, where the noise loop
  can never go stale by construction.
