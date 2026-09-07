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
   press a gamepad button, force-limited poke forward. Ported from
   `../FR3_Aim&Poke/main.py` with its two known bugs fixed (see
   `hardware.py`'s `run_mode2`).
3. **Height map** — vertically probes a configurable-spacing 11x11 grid
   centered on a selected table hole, force-limited at each point.

It reuses `stl_geometry.py`, `table_calibration.py`, and `move_to_spheres.py`
from the repo root as libraries (unmodified) — see those files' own
docstrings/README for what they do.

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
`../table_calibration.json`), `FR3_WEB_HOST` / `FR3_WEB_PORT` (default
`0.0.0.0:5000`).

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
