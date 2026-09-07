"""hardware.py -- robot connection, SimulatedRobot, and the three modes'
background-thread orchestration (run_mode1/run_mode2/run_mode3).

Every function here that actually moves a robot runs in a background
thread spawned by a Flask route (see app.py) -- never in the request
thread. Progress is published through the shared state.state singleton
(state.py), which the browser polls via GET /api/status.
"""

import csv
import json
import math
import os
import time
import threading

import config  # noqa: F401 -- import first: patches sys.path for sibling modules
import table_calibration as tc
import move_to_spheres as mts
import registration
import sensor_backend
import probe as probe_mod
from state import state


# ---------------------------------------------------------------------
# SimulatedRobot -- implements just the fairino Robot.RPC surface that
# compose_targets/target_to_pose/force_limited_approach/probe_vertical
# actually call, so those functions run completely unmodified against it.
# ---------------------------------------------------------------------

class SimulatedRobot:
    DEFAULT_START_POSE = [400.0, 0.0, 300.0, 180.0, 0.0, 0.0]

    def __init__(self, start_pose=None):
        self._lock = threading.Lock()
        self.pose = list(start_pose) if start_pose else list(self.DEFAULT_START_POSE)
        self._move_start_pose = None
        self._target = None
        self._move_start_t = None
        self._move_duration = None
        self._moving = False
        self._drag_teach = False

    # -- safety/connect-pattern surface --------------------------------
    def ResetAllError(self):
        return 0

    def RobotEnable(self, value):
        return 0

    def IsInDragTeach(self):
        return (0, 1 if self._drag_teach else 0)

    def DragTeachSwitch(self, value):
        self._drag_teach = bool(value)
        return 0

    def CloseRPC(self):
        return 0

    # -- motion -----------------------------------------------------
    def MoveL(self, pose, tool=0, user=0, vel=20.0, acc=20.0):
        with self._lock:
            self._advance_locked()
            dist = math.dist(self.pose[:3], pose[:3])
            duration = max(dist / max(float(vel), 1e-6), 0.05)
            self._move_start_pose = list(self.pose)
            self._target = list(pose)
            self._move_start_t = time.time()
            self._move_duration = duration
            self._moving = True
        return 0

    def GetRobotMotionDone(self):
        with self._lock:
            self._advance_locked()
            return (0, 0 if self._moving else 1)

    def StopMotion(self):
        with self._lock:
            self._advance_locked()
            self._moving = False
            self._target = None
        return 0

    def GetActualTCPPose(self, tool=0):
        with self._lock:
            self._advance_locked()
            return (0, list(self.pose))

    def PointsOffsetEnable(self, coord_type, offset):
        return 0

    def PointsOffsetDisable(self):
        return 0

    def _advance_locked(self):
        """Recompute self.pose from elapsed wall-clock time. Call with
        self._lock held."""
        if not self._moving:
            return
        elapsed = time.time() - self._move_start_t
        if elapsed >= self._move_duration:
            self.pose = list(self._target)
            self._moving = False
            self._target = None
        else:
            frac = elapsed / self._move_duration
            self.pose = [
                self._move_start_pose[i] + frac * (self._target[i] - self._move_start_pose[i])
                for i in range(6)
            ]


def connect_robot():
    if config.SIMULATE_ROBOT:
        return SimulatedRobot()

    from fairino import Robot

    robot = Robot.RPC(config.ROBOT_IP)
    robot.ResetAllError()
    time.sleep(0.5)
    robot.RobotEnable(1)
    time.sleep(1.0)

    drag_res = robot.IsInDragTeach()
    if isinstance(drag_res, int):
        print(f"Warning: SDK returned error code {drag_res} for Drag Mode.")
    else:
        err, drag_state = drag_res
        if drag_state == 1:
            print("Safety check: disabling Drag Mode...")
            robot.DragTeachSwitch(0)
            time.sleep(0.5)
    return robot


def close_robot(robot):
    if robot is None:
        return
    try:
        robot.ResetAllError()
    except Exception:
        pass
    try:
        robot.CloseRPC()
    except Exception:
        pass


# ---------------------------------------------------------------------
# run directories / logging helpers
# ---------------------------------------------------------------------

def new_run_dir(mode):
    ts = time.strftime("%Y%m%d_%H%M%S")
    run_id = f"{ts}_{mode}"
    run_dir = os.path.join(config.RUNS_DIR, run_id)
    os.makedirs(run_dir, exist_ok=True)
    return run_id, run_dir


def write_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)


def write_records_csv(path, records):
    if not records:
        return
    fieldnames = []
    for r in records:
        for k in r.keys():
            if k not in fieldnames:
                fieldnames.append(k)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in records:
            writer.writerow({k: r.get(k) for k in fieldnames})


# ---------------------------------------------------------------------
# Mode 1 -- target sphere centers
# ---------------------------------------------------------------------

def run_mode1(payload, run_dir):
    robot = None
    sensor = None
    try:
        spheres_data = payload["spheres_data"]
        mount_hole = tuple(float(v) for v in payload["mount_hole"])
        mount_yaw_deg = float(payload.get("mount_yaw_deg", 0.0))
        reach_to = payload.get("reach_to", "surface")
        standoff_mm = float(payload.get("standoff_mm", 30.0))
        standoff_vel = float(payload.get("standoff_vel", 20.0))
        approach_vel = float(payload.get("approach_vel", 10.0))
        force_threshold_n = float(payload.get("force_threshold_n", 5.0))
        sensor_stale_ms = float(payload.get("sensor_stale_ms", 200.0))
        trusted_run = bool(payload.get("trusted_run", False))
        probe_start_z_offset = float(payload.get("probe_start_z_offset", 50.0))
        probe_min_z_offset = float(payload.get("probe_min_z_offset", -20.0))
        probe_vel = float(payload.get("probe_vel", 10.0))
        stale_s = sensor_stale_ms / 1000.0
        # Hard backstop on commanded z, independent of the force sensor --
        # see config.HARD_FLOOR_Z_MM. Always on, not just when the sensor is
        # simulated: a real sensor that's failed mid-run should get the same
        # protection a simulated one gets.
        z_floor_mm = float(payload.get("z_floor_mm", config.HARD_FLOOR_Z_MM))

        if not spheres_data.get("spheres"):
            raise RuntimeError("no spheres in spheres_data -- nothing to do")

        state.set_status("loading table calibration...")
        table_calib = tc.load_calibration(config.TABLE_CALIB_PATH)

        state.set_status("composing targets...")
        targets = mts.compose_targets(
            spheres_data, reach_to=reach_to, table_calib=table_calib,
            mount_hole=mount_hole, mount_yaw_deg=mount_yaw_deg,
            mount_datum_raw=spheres_data["com"],
        )

        state.set_status("connecting to robot...")
        robot = connect_robot()
        sensor = sensor_backend.get_backend()
        state.active_sensor = sensor
        sensor.start()
        time.sleep(0.5)

        hole_xy = tc.hole_to_robot_xyz(mount_hole[0], mount_hole[1], table_calib)[:2]
        z0 = table_calib["z0_mm"]

        state.set_status("pre-probe: descending straight down at mount hole...")
        pre = probe_mod.probe_vertical(
            robot, sensor, hole_xy,
            start_z=z0 + probe_start_z_offset, min_z=z0 + probe_min_z_offset,
            vel=probe_vel, force_threshold_n=force_threshold_n, stale_s=stale_s,
            z_floor_mm=z_floor_mm,
        )
        state.append_record({
            "index": "pre-probe", "hole": list(mount_hole),
            "z_contact": pre["z_contact"], "outcome": pre["outcome"], "reached": pre["reached"],
        })

        nominal_points = []
        measured_points = []

        for target in targets:
            if state.is_stop_requested():
                state.append_record({"index": target["index"], "outcome": "aborted before start", "reached": False})
                continue

            point, normal = target["point_robot"], target["normal_robot"]
            standoff_point = point + standoff_mm * normal
            standoff_pose = mts.target_to_pose(standoff_point, normal)
            contact_pose = mts.target_to_pose(point, normal)

            violation = mts.z_floor_violation(standoff_pose, z_floor_mm)
            if violation:
                state.append_record({"index": target["index"], "radius": target["radius"],
                                      "outcome": violation, "reached": False})
                continue

            state.set_status(f"target {target['index']}: moving to standoff")
            err = robot.MoveL(standoff_pose, tool=0, user=0, vel=standoff_vel, acc=standoff_vel)
            if err != 0:
                state.append_record({"index": target["index"], "radius": target["radius"],
                                      "outcome": f"standoff MoveL rejected, code {err}", "reached": False})
                continue
            mts._wait_motion_done(robot)

            if not trusted_run:
                state.set_status(f"target {target['index']}: at standoff, waiting for operator")
                resp = state.request_confirm({
                    "target_index": target["index"], "radius": target["radius"],
                    "standoff_pose": standoff_pose, "contact_pose": contact_pose,
                })
                if resp == "abort":
                    state.append_record({"index": target["index"], "radius": target["radius"],
                                          "outcome": "aborted by operator at standoff", "reached": False})
                    state.request_stop()
                    break
                if resp == "skip":
                    state.append_record({"index": target["index"], "radius": target["radius"],
                                          "outcome": "skipped by operator at standoff", "reached": False})
                    continue

            state.set_status(f"target {target['index']}: approaching (force-limited)")
            result = mts.force_limited_approach(
                robot, sensor, contact_pose, vel=approach_vel,
                force_threshold_n=force_threshold_n, stale_s=stale_s,
                z_floor_mm=z_floor_mm,
            )
            record = {
                "index": target["index"], "radius": target["radius"],
                "standoff_pose": standoff_pose, "contact_pose": contact_pose,
                "outcome": result["stopped_reason"], "reached": bool(result["reached"]),
            }
            if result["reached"]:
                err, actual_pose = robot.GetActualTCPPose(0)
                if err == 0 and actual_pose:
                    record["actual_contact_pose"] = list(actual_pose)
                    nominal_points.append(list(point))
                    measured_points.append(list(actual_pose[:3]))
            state.append_record(record)

            state.set_status(f"target {target['index']}: retracting")
            robot.MoveL(standoff_pose, tool=0, user=0, vel=standoff_vel, acc=standoff_vel)
            mts._wait_motion_done(robot)

        state.set_status("computing registration fit...")
        reg = registration.fit_rigid_transform(nominal_points, measured_points)
        state.set_registration_result(reg)

        snapshot = state.snapshot()
        write_records_csv(os.path.join(run_dir, "run_log.csv"), snapshot["records"])
        write_json(os.path.join(run_dir, "registration_result.json"), reg)

        state.set_status("run complete")
    except Exception as e:
        print(f"[mode1] error: {e}")
        state.finish_run(error=e)
        return
    finally:
        if sensor is not None:
            sensor.stop()
        close_robot(robot)

    state.finish_run()


# ---------------------------------------------------------------------
# Mode 2 -- drag-teach aim + gamepad release + force-limited poke
# ---------------------------------------------------------------------

SEARCH_DISTANCE_MM = 200.0


def _forward_target_pose(start_pose, distance_mm):
    """Absolute target pose `distance_mm` along the tool's own +Z from
    start_pose, using the same orientation basis as target_to_pose/
    build_approach_basis so this stays consistent with the rest of the app
    instead of relying on PointsOffsetEnable's relative-offset semantics."""
    R = mts.fairino_rpy_to_matrix(start_pose[3], start_pose[4], start_pose[5])
    tool_z = R[:, 2]
    point = [start_pose[i] + distance_mm * tool_z[i] for i in range(3)]
    return [point[0], point[1], point[2], start_pose[3], start_pose[4], start_pose[5]]


def run_mode2(payload, run_dir):
    robot = None
    sensor = None
    try:
        import inputs

        force_threshold_n = float(payload.get("force_threshold_n", 5.0))
        search_distance_mm = float(payload.get("search_distance_mm", SEARCH_DISTANCE_MM))
        approach_vel = float(payload.get("approach_vel", 20.0))
        sensor_stale_ms = float(payload.get("sensor_stale_ms", 200.0))
        z_floor_mm = float(payload.get("z_floor_mm", config.HARD_FLOOR_Z_MM))

        state.set_status("waiting for gamepad connection and button press...")
        button_pressed = False
        while not button_pressed:
            if state.is_stop_requested():
                state.set_status("stopped before gamepad press")
                state.finish_run()
                return
            try:
                events = inputs.get_gamepad()
                for event in events:
                    if event.ev_type == "Key" and event.state == 1:
                        button_pressed = True
                        break
            except inputs.UnpluggedError:
                state.finish_run(error="no gamepad detected -- connect and restart")
                return

        state.set_status("button pressed! connecting to robot...")
        robot = connect_robot()

        state.set_status("starting sensor...")
        sensor = sensor_backend.get_backend()
        state.active_sensor = sensor
        sensor.start()
        time.sleep(0.5)

        err, start_pose = robot.GetActualTCPPose(0)
        if err != 0 or not start_pose:
            state.finish_run(error=f"failed to read starting pose (code {err})")
            return

        target_pose = _forward_target_pose(start_pose, search_distance_mm)

        state.set_status("moving forward, recording force...")
        t0 = time.time()

        # Poll force in a side thread purely to feed the live graph at a
        # steady cadence -- force_limited_approach owns the actual
        # motion/stop decision, the one collision-detection primitive in
        # this codebase.
        stop_poll = threading.Event()

        def _poll_force_history():
            while not stop_poll.is_set():
                if len(sensor.fz_vals) > 0:
                    state.append_force_sample(time.time() - t0, abs(sensor.fz_vals[-1]))
                time.sleep(0.01)

        poll_thread = threading.Thread(target=_poll_force_history, daemon=True)
        poll_thread.start()

        result = mts.force_limited_approach(
            robot, sensor, target_pose, vel=approach_vel,
            force_threshold_n=force_threshold_n, stale_s=sensor_stale_ms / 1000.0,
            z_floor_mm=z_floor_mm,
        )
        stop_poll.set()
        poll_thread.join(timeout=1.0)

        state.set_status(result["stopped_reason"] or "movement finished")
        state.append_record({
            "reached": bool(result["reached"]), "outcome": result["stopped_reason"],
            "start_pose": list(start_pose), "target_pose": target_pose,
        })

        write_records_csv(os.path.join(run_dir, "last_run_data.csv"), [
            {"t": t, "fz": f} for t, f in zip(state.force_history["t"], state.force_history["f"])
        ])

        state.set_status("run complete. data saved.")
    except Exception as e:
        print(f"[mode2] error: {e}")
        state.finish_run(error=e)
        return
    finally:
        if sensor is not None:
            sensor.stop()
        close_robot(robot)

    state.finish_run()


# ---------------------------------------------------------------------
# Mode 3 -- height-map probing (11x11 grid around a selected table point)
# ---------------------------------------------------------------------

GRID_HALF_EXTENT = 5  # -> 11x11 (2*5 + 1 per axis)


def run_mode3(payload, run_dir):
    robot = None
    sensor = None
    try:
        center_hole = tuple(float(v) for v in payload["center_hole"])
        spacing_mm = float(payload.get("spacing_mm", 5.0))
        probe_start_z_offset = float(payload.get("probe_start_z_offset", 50.0))
        probe_min_z_offset = float(payload.get("probe_min_z_offset", -20.0))
        probe_vel = float(payload.get("probe_vel", 10.0))
        force_threshold_n = float(payload.get("force_threshold_n", 5.0))
        sensor_stale_ms = float(payload.get("sensor_stale_ms", 200.0))
        stale_s = sensor_stale_ms / 1000.0
        z_floor_mm = float(payload.get("z_floor_mm", config.HARD_FLOOR_Z_MM))

        state.set_status("loading table calibration...")
        table_calib = tc.load_calibration(config.TABLE_CALIB_PATH)
        pitch = table_calib["pitch_mm"]
        z0 = table_calib["z0_mm"]
        center_x_mm = center_hole[0] * pitch
        center_y_mm = center_hole[1] * pitch

        n = 2 * GRID_HALF_EXTENT + 1
        heightmap = {"n": n, "spacing_mm": spacing_mm, "center_hole": list(center_hole), "points": []}
        state.set_heightmap(heightmap)

        state.set_status("connecting to robot...")
        robot = connect_robot()
        sensor = sensor_backend.get_backend()
        state.active_sensor = sensor
        sensor.start()
        time.sleep(0.5)

        # Serpentine (boustrophedon) scan order to minimize XY travel
        # between adjacent probes instead of always snapping back to j=-5.
        js = list(range(-GRID_HALF_EXTENT, GRID_HALF_EXTENT + 1))
        total = n * n
        done = 0
        for i in range(-GRID_HALF_EXTENT, GRID_HALF_EXTENT + 1):
            row_js = js if (i % 2 == 0) else list(reversed(js))
            for j in row_js:
                if state.is_stop_requested():
                    state.set_status("aborted by operator")
                    break

                x_mm = center_x_mm + i * spacing_mm
                y_mm = center_y_mm + j * spacing_mm
                xy_robot = tc.table_point_to_robot(x_mm, y_mm, table_calib)[:2]

                done += 1
                state.set_status(f"probing point {done}/{total} (i={i}, j={j})")
                result = probe_mod.probe_vertical(
                    robot, sensor, xy_robot,
                    start_z=z0 + probe_start_z_offset, min_z=z0 + probe_min_z_offset,
                    vel=probe_vel, force_threshold_n=force_threshold_n, stale_s=stale_s,
                    z_floor_mm=z_floor_mm,
                )
                point_record = {
                    "i": i, "j": j, "x": float(xy_robot[0]), "y": float(xy_robot[1]),
                    "z_contact": result["z_contact"], "outcome": result["outcome"],
                }
                heightmap["points"].append(point_record)
                state.set_heightmap(dict(heightmap))
                state.append_record(point_record)
            else:
                continue
            break

        write_json(os.path.join(run_dir, "heightmap.json"), heightmap)
        write_records_csv(os.path.join(run_dir, "heightmap.csv"), heightmap["points"])

        state.set_status("run complete")
    except Exception as e:
        print(f"[mode3] error: {e}")
        state.finish_run(error=e)
        return
    finally:
        if sensor is not None:
            sensor.stop()
        close_robot(robot)

    state.finish_run()
