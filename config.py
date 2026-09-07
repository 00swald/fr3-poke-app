"""config.py -- environment-driven settings for the unified arm control app.

SIMULATE_ROBOT / SIMULATE_SENSOR default to True: a lab safety default, not
a convenience one. Nobody should be able to copy this folder onto the Pi,
forget to set an env var, and have real motion happen on first run. Real
hardware requires an explicit opt-in:

    FR3_SIMULATE_ROBOT=0 FR3_SIMULATE_SENSOR=0 python3 arm_control_webapp/app.py

These are read once at process start. There is no live toggle from the
browser -- changing a safety-relevant setting mid-session is exactly the
kind of thing that should require a deliberate restart, not a button click.
"""

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP_ROOT = os.path.dirname(os.path.abspath(__file__))

# The sibling modules this app reuses (stl_geometry, table_calibration,
# move_to_spheres, Bota_sys) are plain top-level scripts at the repo root,
# not a package. Every module in this app imports `config` first (directly
# or transitively), so this is the one place that needs to put the repo
# root on sys.path for `import stl_geometry` etc. to work regardless of
# the working directory the app was launched from.
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def _env_bool(name, default):
    val = os.environ.get(name)
    if val is None:
        return default
    return val.strip() not in ("0", "false", "False", "")


ROBOT_IP = os.environ.get("FR3_ROBOT_IP", "192.168.57.2")
SENSOR_PORT = os.environ.get("FR3_SENSOR_PORT", "/dev/ttyUSB0")

SIMULATE_ROBOT = _env_bool("FR3_SIMULATE_ROBOT", True)
SIMULATE_SENSOR = _env_bool("FR3_SIMULATE_SENSOR", True)

# Hard Z-axis floor, in the ROBOT'S OWN BASE FRAME (mm) -- assumes the
# standard convention that the base frame origin sits at the robot's
# mounting point with +Z pointing up, so this is literally "how many mm
# above the base the TCP is allowed to go." Enforced by move_to_spheres.py
# on every commanded target, independent of and in addition to the force
# sensor's stop condition. This exists specifically for running with a real
# robot but no working force sensor (SIMULATE_SENSOR=1, SIMULATE_ROBOT=0):
# a force-limited approach's *only* real stop condition is then gone, so
# without this it would run the full commanded distance no matter what's in
# the way. It is NOT a substitute for a correct table calibration or a
# collision-avoidance system -- it only catches a target whose Z is grossly
# wrong (e.g. into the table or the robot's own base), not a target that is
# above this floor but still wrong. Verify the base-frame Z convention
# actually holds for this robot before trusting it (see move_to_spheres.py's
# orientation-verification docstring for the same read-only-first approach).
HARD_FLOOR_Z_MM = float(os.environ.get("FR3_HARD_FLOOR_Z_MM", "20.0"))

TABLE_CALIB_PATH = os.environ.get(
    "FR3_TABLE_CALIB", os.path.join(APP_ROOT, "table_calibration.json")
)

RUNS_DIR = os.path.join(APP_ROOT, "runs")

# Optic table geometry (holes spaced 1" apart, per the physical table this
# app targets -- see table_calibration.py, which owns pitch_mm inside the
# fitted calibration file; these are just the grid *extent* for the GUI).
TABLE_N_COLS = 60
TABLE_N_ROWS = 40

HOST = os.environ.get("FR3_WEB_HOST", "0.0.0.0")
PORT = int(os.environ.get("FR3_WEB_PORT", "5000"))
