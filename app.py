"""app.py -- unified FR3 arm control web app.

Three modes, one Flask server:
  1. Target sphere centers  -- upload an STL, pick a table hole, poke every
     detected sphere along its surface normal, force-limited.
  2. Drag-teach aim + release -- hand-guide the arm, press a gamepad
     button, force-limited poke forward.
  3. Height map -- vertically probe an 11x11 grid around a selected table
     point.

Run: `cd FHL26 && python3 arm_control_webapp/app.py`
Defaults to a fully simulated robot + sensor (see config.py) -- safe to run
and click through with no hardware attached at all. Real hardware requires
an explicit opt-in:
    FR3_SIMULATE_ROBOT=0 FR3_SIMULATE_SENSOR=0 python3 arm_control_webapp/app.py
"""

import os
import threading

from flask import Flask, jsonify, render_template, request

import config  # noqa: F401 -- import first: patches sys.path for sibling modules
import stl_geometry as sg
import table_calibration as tc
import hardware
from state import state

app = Flask(__name__)

# run_id -> run_dir, populated by mode1 STL uploads so preview/execute can
# write their artifacts (targets_preview.json, run_log.csv,
# registration_result.json) alongside the uploaded STL and spheres.json.
_upload_dirs = {}


def _load_table_calib_or_none():
    try:
        return tc.load_calibration(config.TABLE_CALIB_PATH)
    except Exception:
        return None


# ------------------------------------------------------------------
# general / shared routes
# ------------------------------------------------------------------

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/settings")
def api_settings():
    calib = _load_table_calib_or_none()
    return jsonify({
        "robot_ip": config.ROBOT_IP,
        "sensor_port": config.SENSOR_PORT,
        "simulate_robot": config.SIMULATE_ROBOT,
        "simulate_sensor": config.SIMULATE_SENSOR,
        "table_calib_path": config.TABLE_CALIB_PATH,
        "table_calib_loaded": calib is not None,
        "table_calib": calib,
    })


@app.route("/api/table-grid")
def api_table_grid():
    calib = _load_table_calib_or_none()
    pitch_mm = calib["pitch_mm"] if calib else 25.4
    return jsonify({"n_cols": config.TABLE_N_COLS, "n_rows": config.TABLE_N_ROWS, "pitch_mm": pitch_mm})


@app.route("/api/status")
def api_status():
    return jsonify(state.snapshot())


@app.route("/api/confirm", methods=["POST"])
def api_confirm():
    body = request.get_json(force=True, silent=True) or {}
    response = body.get("response")
    if response not in ("continue", "skip", "abort"):
        return jsonify({"error": "response must be continue|skip|abort"}), 400
    ok = state.respond_confirm(response)
    if not ok:
        return jsonify({"error": "no confirmation is currently pending"}), 409
    return jsonify({"ok": True})


@app.route("/api/abort", methods=["POST"])
def api_abort():
    state.request_stop()
    state.respond_confirm("abort")
    return jsonify({"ok": True})


@app.route("/api/simulate-collision", methods=["POST"])
def api_simulate_collision():
    if not config.SIMULATE_SENSOR:
        return jsonify({"error": "simulate-collision is only available with FR3_SIMULATE_SENSOR=1"}), 400
    sensor = state.active_sensor
    if sensor is None:
        return jsonify({"error": "no run is active / no sensor connected"}), 409
    body = request.get_json(force=True, silent=True) or {}
    force_n = float(body.get("force_n", 15.0))
    hold_s = body.get("hold_s")
    sensor.simulate_collision(force_n, hold_s=float(hold_s) if hold_s is not None else None)
    return jsonify({"ok": True})


# ------------------------------------------------------------------
# Mode 1 -- target sphere centers
# ------------------------------------------------------------------

@app.route("/api/mode1/upload-stl", methods=["POST"])
def api_mode1_upload_stl():
    if "stl" not in request.files:
        return jsonify({"error": "missing 'stl' file field"}), 400
    f = request.files["stl"]
    if not f.filename:
        return jsonify({"error": "empty filename"}), 400

    run_id, run_dir = hardware.new_run_dir("mode1")
    stl_path = os.path.join(run_dir, "upload.stl")
    f.save(stl_path)

    kwargs = {}
    for key, cast in (("crease_angle_deg", float), ("min_patch_faces", int),
                      ("max_rms_frac", float), ("min_radius", float), ("max_radius", float)):
        val = request.form.get(key)
        if val not in (None, ""):
            kwargs[key] = cast(val)

    try:
        spheres_data = sg.find_spheres(stl_path, **kwargs)
    except Exception as e:
        return jsonify({"error": f"STL processing failed: {e}"}), 400

    hardware.write_json(os.path.join(run_dir, "spheres.json"), spheres_data)
    _upload_dirs[run_id] = run_dir

    return jsonify({"run_id": run_id, "spheres_data": spheres_data})


@app.route("/api/mode1/preview", methods=["POST"])
def api_mode1_preview():
    import move_to_spheres as mts

    body = request.get_json(force=True, silent=True) or {}
    spheres_data = body.get("spheres_data")
    mount_hole = body.get("mount_hole")
    if not spheres_data or mount_hole is None:
        return jsonify({"error": "spheres_data and mount_hole are required"}), 400

    calib = _load_table_calib_or_none()
    if calib is None:
        return jsonify({"error": f"table calibration not found at {config.TABLE_CALIB_PATH} -- "
                                  f"run table_calibration.py capture first"}), 404

    mount_yaw_deg = float(body.get("mount_yaw_deg", 0.0))
    reach_to = body.get("reach_to", "surface")
    standoff_mm = float(body.get("standoff_mm", 30.0))
    mount_hole_t = tuple(float(v) for v in mount_hole)

    if not spheres_data.get("spheres"):
        return jsonify({"targets": [], "initial_probe_xy": None,
                         "warning": "no spheres in spheres_data -- nothing to preview"})

    targets = mts.compose_targets(
        spheres_data, reach_to=reach_to, table_calib=calib,
        mount_hole=mount_hole_t, mount_yaw_deg=mount_yaw_deg,
        mount_datum_raw=spheres_data["com"],
    )

    planned = []
    for t in targets:
        standoff_point = t["point_robot"] + standoff_mm * t["normal_robot"]
        standoff_pose = mts.target_to_pose(standoff_point, t["normal_robot"])
        contact_pose = mts.target_to_pose(t["point_robot"], t["normal_robot"])
        planned.append({"index": t["index"], "radius": t["radius"],
                         "standoff_pose": standoff_pose, "contact_pose": contact_pose})

    initial_probe_xy = list(tc.hole_to_robot_xyz(mount_hole_t[0], mount_hole_t[1], calib)[:2])

    run_id = body.get("run_id")
    if run_id in _upload_dirs:
        hardware.write_json(os.path.join(_upload_dirs[run_id], "targets_preview.json"), planned)

    return jsonify({"targets": planned, "initial_probe_xy": initial_probe_xy})


@app.route("/api/mode1/execute", methods=["POST"])
def api_mode1_execute():
    if state.running:
        return jsonify({"error": "a run is already active"}), 409

    body = request.get_json(force=True, silent=True) or {}
    if not body.get("spheres_data") or body.get("mount_hole") is None:
        return jsonify({"error": "spheres_data and mount_hole are required"}), 400

    run_id = body.get("run_id")
    if run_id in _upload_dirs:
        run_dir = _upload_dirs[run_id]
    else:
        run_id, run_dir = hardware.new_run_dir("mode1")

    state.start_run("mode1", run_id, run_dir)
    threading.Thread(target=hardware.run_mode1, args=(body, run_dir), daemon=True).start()
    return jsonify({"started": True, "run_id": run_id})


# ------------------------------------------------------------------
# Mode 2 -- drag-teach aim + gamepad release + poke
# ------------------------------------------------------------------

@app.route("/api/mode2/start", methods=["POST"])
def api_mode2_start():
    if state.running:
        return jsonify({"error": "a run is already active"}), 409

    body = request.get_json(force=True, silent=True) or {}
    run_id, run_dir = hardware.new_run_dir("mode2")
    state.start_run("mode2", run_id, run_dir)
    threading.Thread(target=hardware.run_mode2, args=(body, run_dir), daemon=True).start()
    return jsonify({"started": True, "run_id": run_id})


@app.route("/api/mode2/stop", methods=["POST"])
def api_mode2_stop():
    state.request_stop()
    return jsonify({"ok": True})


# ------------------------------------------------------------------
# Mode 3 -- height map
# ------------------------------------------------------------------

@app.route("/api/mode3/start", methods=["POST"])
def api_mode3_start():
    if state.running:
        return jsonify({"error": "a run is already active"}), 409

    body = request.get_json(force=True, silent=True) or {}
    if body.get("center_hole") is None:
        return jsonify({"error": "center_hole is required"}), 400

    calib = _load_table_calib_or_none()
    if calib is None:
        return jsonify({"error": f"table calibration not found at {config.TABLE_CALIB_PATH} -- "
                                  f"run table_calibration.py capture first"}), 404

    run_id, run_dir = hardware.new_run_dir("mode3")
    state.start_run("mode3", run_id, run_dir)
    threading.Thread(target=hardware.run_mode3, args=(body, run_dir), daemon=True).start()
    return jsonify({"started": True, "run_id": run_id})


if __name__ == "__main__":
    os.makedirs(config.RUNS_DIR, exist_ok=True)
    print(f"Robot: {'SIMULATED' if config.SIMULATE_ROBOT else 'REAL (' + config.ROBOT_IP + ')'}")
    print(f"Sensor: {'SIMULATED' if config.SIMULATE_SENSOR else 'REAL (' + config.SENSOR_PORT + ')'}")
    print(f"Table calibration: {config.TABLE_CALIB_PATH}"
          f"{'' if _load_table_calib_or_none() else '  <-- NOT FOUND'}")
    print(f"Starting web server at http://{config.HOST}:{config.PORT}")
    app.run(host=config.HOST, port=config.PORT, debug=False, use_reloader=False, threaded=True)
