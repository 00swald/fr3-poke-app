"""
move_to_spheres.py — drive the FR3's TCP to STL-detected sphere contact
points, approaching along the local surface normal, with a force-limited
final approach (reusing Bota_sys.BotaSerialSensor, fixed relative to how
FR3_Aim&Poke/main.py uses it -- see FIXES VS EXISTING SCRIPTS below).

Pipeline: stl_geometry.py's JSON (sphere centers/contact points/normals, in
the STL's COM-centered frame) + table_calibration.json (table-hole-grid ->
robot-base mapping) + this run's part-placement args (which hole, what yaw,
where the mounting datum is) compose into absolute robot-frame target poses.

DEFAULT IS DRY RUN: prints every planned standoff+contact pose and does not
import fairino or connect to anything. Pass --execute to actually move the
robot, and only after doing the orientation-convention verification below.

ORIENTATION CONVENTION -- READ BEFORE --execute:
FR3 poses are [x,y,z,rx,ry,rz] (mm, degrees). Per FAIRINO's manual
(fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_brief_introduction.html
Sec 2.1), attitude uses "ZYX of the floating coordinate system", i.e.
intrinsic Z-Y-X Euler angles: R = Rz(rz) @ Ry(ry) @ Rx(rx). That's a
manual-prose claim, not something verified against this specific robot by
running code -- confirm it before trusting any commanded orientation:
  1. Zero-risk first: put the robot in drag-teach, hand-pose it into a few
     visually-unambiguous orientations, and only READ GetActualTCPPose at
     each (never command a target). Compare the read (rx,ry,rz) against what
     fairino_rpy_to_matrix() predicts for that pose. No commanded motion at
     all in this step.
  2. Only then: small (~10-15 deg) COMMANDED single-axis rotations from a
     safe pose, one axis at a time (not all three together -- a sign flip on
     one axis is easy to miss if they move together), checked against a
     physical reference mark on the flange/tool.
The pure code-level round-trip (matrix -> rpy -> matrix) is covered by
test_geometry_selftest.py and needs no hardware -- that only catches an
implementation bug (e.g. an angle-order transposition), not a wrong
real-world convention. Both matter; neither substitutes for the other.

FIXES VS EXISTING SCRIPTS (found while building this -- see the plan for
detail; not patched in place there, just not repeated here):
  - FR3_Aim&Poke/main.py never runs sensor.update_plot(), so sensor.fz_vals
    is likely always empty and its force stop may never fire. This script
    spawns that thread explicitly.
  - FR3_Aim&Poke/main.py's monitoring loop infers "done" from move-thread
    liveness, which goes false almost immediately after an async MoveL
    dispatch. This script polls GetRobotMotionDone() directly instead
    (move_test.py's two-stage pattern), in the same loop that watches force.
  - Bota_sys.py's background read thread has no watchdog -- a single bad
    serial frame silently freezes all sensor values. This script tracks
    time-since-last-new-sample and stops (fails toward stopping) if the
    sensor goes stale mid-approach, independent of the force value itself.

Usage (dry run):
    python3 move_to_spheres.py spheres.json --table-calib table_calibration.json \\
        --mount-hole 3,2 --mount-yaw-deg 90 --mount-datum-raw 10 5 0

Usage (execute, only after the verification above):
    python3 move_to_spheres.py spheres.json --table-calib table_calibration.json \\
        --mount-hole 3,2 --mount-yaw-deg 90 --mount-datum-raw 10 5 0 \\
        --execute --robot-ip 192.168.57.2
"""

import argparse
import json
import sys
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation

import table_calibration as tc


# --------------------------------------------------------------------------
# orientation math (pure -- no hardware import, unit-tested in
# test_geometry_selftest.py's round-trip test)
# --------------------------------------------------------------------------

def build_approach_basis(outward_normal, up_hint=(0.0, 0.0, 1.0)):
    """3x3 rotation whose Z axis is -outward_normal (tool points INTO the
    surface -- the direction of travel during a poke). The other two axes are
    built stably against up_hint via Gram-Schmidt, falling back to a
    different hint when the desired axis is nearly parallel to it (classic
    look-at singularity)."""
    outward_normal = np.asarray(outward_normal, dtype=np.float64)
    outward_normal = outward_normal / np.linalg.norm(outward_normal)
    tool_z = -outward_normal

    up = np.asarray(up_hint, dtype=np.float64)
    if abs(np.dot(tool_z, up)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(tool_z, up)) > 0.99:
            up = np.array([0.0, 1.0, 0.0])

    tool_x = np.cross(up, tool_z)
    tool_x /= np.linalg.norm(tool_x)
    tool_y = np.cross(tool_z, tool_x)  # already unit length: tool_z, tool_x orthonormal
    return np.column_stack([tool_x, tool_y, tool_z])


def fairino_rpy_to_matrix(rx, ry, rz):
    """[rx,ry,rz] degrees -> 3x3 rotation. Intrinsic Z-Y-X: R = Rz(rz) @
    Ry(ry) @ Rx(rx). See the module docstring -- NOT yet hardware-verified."""
    return Rotation.from_euler('ZYX', [rz, ry, rx], degrees=True).as_matrix()


def matrix_to_fairino_rpy(R):
    """Inverse of fairino_rpy_to_matrix. scipy's as_euler('ZYX', ...) returns
    [about_Z, about_Y, about_X] -- reversed here to [rx,ry,rz] pose order.
    Getting that reversal backwards is an easy, silent, purely-code-level bug
    distinct from the real-world-convention question; see the round-trip
    test in test_geometry_selftest.py."""
    z, y, x = Rotation.from_matrix(R).as_euler('ZYX', degrees=True)
    return np.array([x, y, z])


def target_to_pose(point_robot, normal_robot):
    R = build_approach_basis(normal_robot)
    rx, ry, rz = matrix_to_fairino_rpy(R)
    return [float(point_robot[0]), float(point_robot[1]), float(point_robot[2]),
            float(rx), float(ry), float(rz)]


# --------------------------------------------------------------------------
# frame composition: STL (COM-centered) -> table -> robot base
# --------------------------------------------------------------------------

def rz_matrix(yaw_deg):
    th = np.radians(yaw_deg)
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def compose_targets(spheres_data, *, reach_to, table_calib=None, mount_hole=None,
                     mount_yaw_deg=0.0, mount_datum_raw=None):
    """Returns a list of {index, radius, point_robot, normal_robot}.

    If table_calib is None, mount_* are ignored and sphere coordinates are
    used as-is (STL COM-centered frame) -- only valid for a preview; the CLI
    refuses this combination under --execute.
    """
    spheres = spheres_data["spheres"]

    if table_calib is None:
        targets = []
        for s in spheres:
            point = np.array(s["contact_point"] if reach_to == "surface" else s["center"])
            normal = np.array(s["outward_normal"])
            targets.append({"index": s["index"], "radius": s["radius"],
                             "point_robot": point, "normal_robot": normal})
        return targets

    com = np.array(spheres_data["com"])
    datum_recentered = np.array(mount_datum_raw, dtype=np.float64) - com
    Ryaw = rz_matrix(mount_yaw_deg)
    datum_rotated = Ryaw @ datum_recentered

    hole_xyz_robot = tc.hole_to_robot_xyz(mount_hole[0], mount_hole[1], table_calib)
    translation = hole_xyz_robot - datum_rotated

    targets = []
    for s in spheres:
        point_stl = np.array(s["contact_point"] if reach_to == "surface" else s["center"])
        normal_stl = np.array(s["outward_normal"])
        targets.append({
            "index": s["index"],
            "radius": s["radius"],
            "point_robot": Ryaw @ point_stl + translation,
            "normal_robot": Ryaw @ normal_stl,
        })
    return targets


# --------------------------------------------------------------------------
# motion (only these functions touch fairino/Bota_sys -- never imported in
# dry-run mode)
# --------------------------------------------------------------------------

def _wait_motion_done(robot, start_timeout_s=2.0, poll_s=0.02):
    t0 = time.time()
    started = False
    while time.time() - t0 < start_timeout_s:
        _, is_done = robot.GetRobotMotionDone()
        if is_done == 0:
            started = True
            break
        time.sleep(poll_s)
    if not started:
        print("  warning: robot never reported motion start (0mm move?)")
    while True:
        _, is_done = robot.GetRobotMotionDone()
        if is_done == 1:
            break
        time.sleep(poll_s)


def force_limited_approach(robot, sensor, target_pose, *, vel, force_threshold_n,
                            stale_s, poll_interval_s=0.02, start_timeout_s=2.0):
    """Dispatch MoveL once, then poll motion-done + force + sensor staleness
    together in a single loop, calling StopMotion() the instant any safety
    condition trips. The move is itself already distance-bounded (target_pose
    is exactly the standoff-to-contact distance away, by construction of the
    caller) -- unlike the existing scripts' PointsOffsetEnable relative-move
    pattern, an absolute-pose MoveL cannot travel further than the commanded
    point, so no separate forward-distance cap is needed on top of that."""
    outcome = {"reached": False, "stopped_reason": None}

    err = robot.MoveL(target_pose, tool=0, user=0, vel=vel, acc=vel)
    if err != 0:
        outcome["stopped_reason"] = f"MoveL rejected, code {err}"
        return outcome

    t0 = time.time()
    while time.time() - t0 < start_timeout_s:
        _, is_done = robot.GetRobotMotionDone()
        if is_done == 0:
            break
        time.sleep(poll_interval_s)

    last_len = len(sensor.fz_vals)
    last_change_t = time.time()
    while True:
        cur_len = len(sensor.fz_vals)
        now = time.time()
        if cur_len > last_len:
            last_len, last_change_t = cur_len, now
        elif now - last_change_t > stale_s:
            robot.StopMotion()
            outcome["stopped_reason"] = f"sensor stale for >{stale_s:.2f}s -- stopped as a fault"
            return outcome

        if cur_len > 0:
            force = abs(sensor.fz_vals[-1])
            if force >= force_threshold_n:
                robot.StopMotion()
                outcome["reached"] = True
                outcome["stopped_reason"] = f"force threshold hit ({force:.2f}N >= {force_threshold_n}N)"
                return outcome

        _, is_done = robot.GetRobotMotionDone()
        if is_done == 1:
            outcome["stopped_reason"] = "reached target pose without hitting force threshold"
            return outcome

        time.sleep(poll_interval_s)


def run_target(robot, sensor, target, *, standoff_mm, standoff_vel, approach_vel,
                force_threshold_n, stale_s, confirm):
    point, normal = target["point_robot"], target["normal_robot"]
    standoff_point = point + standoff_mm * normal
    standoff_pose_vec = target_to_pose(standoff_point, normal)
    contact_pose_vec = target_to_pose(point, normal)

    record = {"index": target["index"], "radius": target["radius"],
              "standoff_pose": standoff_pose_vec, "contact_pose": contact_pose_vec,
              "outcome": None, "reached": False}

    print(f"\n--- target {target['index']} (radius {target['radius']:.3f}mm) ---")
    print(f"  moving to standoff: {['%.3f' % v for v in standoff_pose_vec]}")
    err = robot.MoveL(standoff_pose_vec, tool=0, user=0, vel=standoff_vel, acc=standoff_vel)
    if err != 0:
        record["outcome"] = f"standoff MoveL rejected, code {err}"
        print(f"  {record['outcome']} -- skipping this target")
        return record
    _wait_motion_done(robot)

    if confirm:
        resp = input("  at standoff -- Enter to approach, 's' to skip, 'q' to abort run: ").strip().lower()
        if resp == "q":
            record["outcome"] = "aborted by operator at standoff"
            print(f"  {record['outcome']}")
            raise KeyboardInterrupt("aborted by operator")
        if resp == "s":
            record["outcome"] = "skipped by operator at standoff"
            print(f"  {record['outcome']}")
            return record

    print(f"  approaching (force-limited, threshold {force_threshold_n}N): "
          f"{['%.3f' % v for v in contact_pose_vec]}")
    result = force_limited_approach(robot, sensor, contact_pose_vec, vel=approach_vel,
                                     force_threshold_n=force_threshold_n, stale_s=stale_s)
    record["outcome"], record["reached"] = result["stopped_reason"], result["reached"]
    print(f"  {record['outcome']}")

    print("  retracting to standoff...")
    robot.MoveL(standoff_pose_vec, tool=0, user=0, vel=standoff_vel, acc=standoff_vel)
    _wait_motion_done(robot)
    return record


def execute(targets, args):
    from fairino import Robot
    from Bota_sys import BotaSerialSensor
    import pandas as pd

    robot, sensor, records = None, None, []
    try:
        print(f"Connecting to FR3 at {args.robot_ip}...")
        robot = Robot.RPC(args.robot_ip)
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

        print(f"Starting Bota sensor on {args.sensor_port}...")
        sensor = BotaSerialSensor(args.sensor_port)
        sensor.start()
        threading.Thread(target=sensor.update_plot, daemon=True).start()
        time.sleep(0.5)

        for target in targets:
            try:
                records.append(run_target(
                    robot, sensor, target, standoff_mm=args.standoff_mm,
                    standoff_vel=args.standoff_vel, approach_vel=args.approach_vel,
                    force_threshold_n=args.force_threshold_n,
                    stale_s=args.sensor_stale_ms / 1000.0, confirm=not args.no_confirm))
            except KeyboardInterrupt:
                print("Run aborted by operator.")
                break
    finally:
        if sensor:
            print("Stopping sensor...")
            sensor.stop()
        if robot:
            print("Closing RPC connection to FR3 arm...")
            robot.ResetAllError()
            robot.CloseRPC()

    if records:
        pd.DataFrame(records).to_csv(args.out_log, index=False)
        print(f"\nwrote {args.out_log}")
    return records


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _parse_hole(s):
    col, row = s.split(",")
    return (float(col), float(row))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("spheres_json", help="output of stl_geometry.py --json")
    ap.add_argument("--table-calib", help="table_calibration.json from table_calibration.py")
    ap.add_argument("--mount-hole", type=_parse_hole, help="'col,row' the part's datum sits at")
    ap.add_argument("--mount-yaw-deg", type=float, default=0.0)
    ap.add_argument("--mount-datum-raw", type=float, nargs=3, metavar=("X", "Y", "Z"),
                     help="datum point in the STL's ORIGINAL (pre-recenter) coordinates -- "
                          "exactly what you'd read off the CAD/STL directly; COM is subtracted "
                          "internally")
    ap.add_argument("--reach-to", choices=["surface", "center"], default="surface",
                     help="default 'surface': target = center + radius*normal (physically "
                          "reachable). 'center' targets the literal sphere center -- only sane "
                          "for e.g. a thin ball-on-a-stalk, not a solid sphere.")
    ap.add_argument("--standoff-mm", type=float, default=30.0)
    ap.add_argument("--standoff-vel", type=float, default=20.0)
    ap.add_argument("--approach-vel", type=float, default=10.0)
    ap.add_argument("--force-threshold-n", type=float, default=5.0)
    ap.add_argument("--sensor-port", default="/dev/ttyUSB0")
    ap.add_argument("--sensor-stale-ms", type=float, default=200.0)
    ap.add_argument("--no-confirm", action="store_true",
                     help="skip the per-point standoff confirmation prompt (trusted repeat runs only)")
    ap.add_argument("--out-log", default="move_to_spheres_log.csv")
    ap.add_argument("--out-poses-json", help="dry-run: also write planned poses to this JSON path")
    ap.add_argument("--execute", action="store_true", help="actually connect and move (default: dry run)")
    ap.add_argument("--robot-ip", default="192.168.57.2")
    args = ap.parse_args(argv)

    with open(args.spheres_json) as f:
        spheres_data = json.load(f)

    if not spheres_data["spheres"]:
        print("no spheres in this JSON -- nothing to do (rejections: "
              f"{spheres_data.get('rejections')})")
        return 0

    table_calib = None
    if args.table_calib:
        if args.mount_hole is None or args.mount_datum_raw is None:
            print("--table-calib requires --mount-hole and --mount-datum-raw too")
            return 2
        table_calib = tc.load_calibration(args.table_calib)
    elif args.execute:
        print("--execute requires --table-calib (+ --mount-hole/--mount-datum-raw) -- "
              "STL-frame coordinates alone are not robot coordinates")
        return 2

    targets = compose_targets(spheres_data, reach_to=args.reach_to, table_calib=table_calib,
                               mount_hole=args.mount_hole, mount_yaw_deg=args.mount_yaw_deg,
                               mount_datum_raw=args.mount_datum_raw)

    print(f"{len(targets)} target(s), frame: "
          f"{'robot base (via table calibration)' if table_calib else 'STL COM-centered (PREVIEW ONLY)'}")

    planned = []
    for t in targets:
        standoff_point = t["point_robot"] + args.standoff_mm * t["normal_robot"]
        standoff_pose_vec = target_to_pose(standoff_point, t["normal_robot"])
        contact_pose_vec = target_to_pose(t["point_robot"], t["normal_robot"])
        planned.append({"index": t["index"], "radius": t["radius"],
                         "standoff_pose": standoff_pose_vec, "contact_pose": contact_pose_vec})
        print(f"  target {t['index']} (r={t['radius']:.3f}mm): "
              f"standoff={['%.3f' % v for v in standoff_pose_vec]} "
              f"contact={['%.3f' % v for v in contact_pose_vec]}")

    if args.out_poses_json:
        with open(args.out_poses_json, "w") as f:
            json.dump(planned, f, indent=2)
        print(f"wrote {args.out_poses_json}")

    if not args.execute:
        print("\nDRY RUN -- no robot connection made. Pass --execute to move the real arm, "
              "after doing the orientation-verification procedure in this file's docstring.")
        return 0

    execute(targets, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
