"""
table_calibration.py — planar optical-table -> robot-base calibration.

The part gets bolted to an optical table with a 1" (25.4mm) hole grid; the
robot is bolted to the same table. That's a known DOF, not an unknown pose to
discover: rotation about Z (yaw) + XY translation + a constant table height,
NOT a general 6-DOF registration. This fits exactly that: 2D Procrustes (SVD)
on >=2 (col,row)-grid-hole <-> robot-XYZ correspondences, restricted to
yaw-only rotation, with a constant z0 = mean measured height.

Two independent things this file explicitly checks for and gates on, because
they've each caused real problems in similar setups:
  - Only 2 calibration points make the fit EXACT (zero residual) by
    construction, even for yaw-only rotation -- which means it can't catch a
    data-entry blunder (e.g. operator typed the wrong hole). >=3 points turns
    the residual into a real data-quality signal. --allow-two-point opts out
    explicitly.
  - Collinear points (even 3+ of them) don't fix that either -- a rotation
    fit from collinear points is barely more constrained than from 2, since
    there's no leverage perpendicular to the line. Points should be spread
    widely (e.g. an L-shape across the table).

Usage:
    # from a points file: [{"hole": [3, 2], "robot_xyz": [x, y, z]}, ...]
    python3 table_calibration.py fit --points points.json --out table_calibration.json

    # interactive capture via drag-teach (needs the robot; only this path
    # imports fairino)
    python3 table_calibration.py capture --robot-ip 192.168.57.2 --out table_calibration.json
"""

import argparse
import json
import sys
import time

import numpy as np


# --------------------------------------------------------------------------
# core fit
# --------------------------------------------------------------------------

def fit_table_transform(hole_coords, robot_xyz, pitch_mm=25.4):
    """hole_coords: list of (col, row) grid coordinates (may be fractional).
    robot_xyz: list of (x,y,z) mm TCP positions recorded touching each
    corresponding hole, same order, same length, length >= 2.

    Returns a dict with theta_deg/tx_mm/ty_mm/z0_mm (the fitted transform) plus
    diagnostics (residual_rms_mm, collinearity_ratio, z_spread_mm,
    reflection_detected) -- see check_calibration_quality() to turn those into
    pass/fail gates.
    """
    hole_coords = np.asarray(hole_coords, dtype=np.float64)
    robot_xyz = np.asarray(robot_xyz, dtype=np.float64)
    n = len(hole_coords)
    if n < 2:
        raise ValueError("need at least 2 calibration points")
    if robot_xyz.shape[0] != n:
        raise ValueError("hole_coords and robot_xyz must have the same length")

    table_xy = hole_coords * pitch_mm
    robot_xy = robot_xyz[:, :2]
    robot_z = robot_xyz[:, 2]

    p_centroid = table_xy.mean(axis=0)
    q_centroid = robot_xy.mean(axis=0)
    P = table_xy - p_centroid
    Q = robot_xy - q_centroid

    # Collinearity of the table-side points alone (independent of the robot
    # side): ratio of smallest to largest singular value of P. Near 0 means
    # the points don't really span 2D -- yaw is poorly determined regardless
    # of how good the touches were.
    sv_P = np.linalg.svd(P, compute_uv=False)
    collinearity_ratio = float(sv_P[-1] / sv_P[0]) if sv_P[0] > 1e-12 else 0.0

    H = P.T @ Q
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    reflection_detected = False
    if np.linalg.det(R) < 0:
        # Only a pure rotation is physically possible here (a table doesn't
        # get mirrored); a reflection popping out of the fit normally means
        # bad input data (e.g. a hole typo), not a real degree of freedom.
        # Corrected via the standard Kabsch determinant fix, but surfaced as
        # reflection_detected so the caller can warn loudly.
        reflection_detected = True
        Vt_fixed = Vt.copy()
        Vt_fixed[-1, :] *= -1
        R = Vt_fixed.T @ U.T

    theta_deg = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    t = q_centroid - R @ p_centroid

    predicted_xy = (R @ table_xy.T).T + t
    per_point_residual_mm = np.linalg.norm(predicted_xy - robot_xy, axis=1)
    residual_rms_mm = float(np.sqrt(np.mean(per_point_residual_mm ** 2)))

    z0_mm = float(robot_z.mean())
    z_spread_mm = float(robot_z.max() - robot_z.min()) if n > 1 else 0.0

    return {
        "theta_deg": theta_deg,
        "tx_mm": float(t[0]),
        "ty_mm": float(t[1]),
        "z0_mm": z0_mm,
        "pitch_mm": pitch_mm,
        "n_points": n,
        "residual_rms_mm": residual_rms_mm,
        "per_point_residual_mm": per_point_residual_mm.tolist(),
        "z_spread_mm": z_spread_mm,
        "collinearity_ratio": collinearity_ratio,
        "reflection_detected": reflection_detected,
    }


def check_calibration_quality(fit_result, *, max_z_spread_mm=1.0, min_points=3,
                               collinearity_warn_ratio=0.05):
    """Returns a list of human-readable problem strings (empty = looks fine).
    Does not raise -- caller decides what to do (CLI hard-gates on min_points
    unless --allow-two-point; everything else is a loud warning either way)."""
    problems = []
    if fit_result["n_points"] < min_points:
        problems.append(
            f"only {fit_result['n_points']} calibration point(s) -- need >= {min_points} for a "
            f"self-checking fit. With exactly 2 points the fit is EXACT by construction (zero "
            f"residual) and cannot detect a data-entry blunder like a mistyped hole.")
    if fit_result["collinearity_ratio"] < collinearity_warn_ratio:
        problems.append(
            f"calibration points are nearly collinear (spread ratio {fit_result['collinearity_ratio']:.4f}) "
            f"-- yaw angle is poorly constrained. Use points spread widely across the table "
            f"(e.g. an L-shape), not points along a single row/column.")
    if fit_result["z_spread_mm"] > max_z_spread_mm:
        problems.append(
            f"z spread across calibration touches is {fit_result['z_spread_mm']:.3f}mm "
            f"(> {max_z_spread_mm}mm) -- either the robot base isn't quite level relative to the "
            f"table, or a touch was inconsistent (different reference feature/pointer length). "
            f"z0 may not be reliable.")
    if fit_result["reflection_detected"]:
        problems.append(
            "fit required a reflection correction to stay a proper rotation -- this normally only "
            "happens with bad/mismatched input data (e.g. a hole typo). Double check your points.")
    return problems


# --------------------------------------------------------------------------
# applying a fitted calibration
# --------------------------------------------------------------------------

def _rot2d(theta_deg):
    th = np.radians(theta_deg)
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


def table_point_to_robot(x_mm, y_mm, calib):
    """table-local (x,y) mm, e.g. col*pitch/row*pitch or any point on the
    table plane -> robot-base (x,y,z) mm, at the table surface height."""
    xy = _rot2d(calib["theta_deg"]) @ np.array([x_mm, y_mm]) + \
        np.array([calib["tx_mm"], calib["ty_mm"]])
    return np.array([xy[0], xy[1], calib["z0_mm"]])


def hole_to_robot_xyz(col, row, calib):
    pitch = calib["pitch_mm"]
    return table_point_to_robot(col * pitch, row * pitch, calib)


def table_direction_to_robot(dx_mm, dy_mm, calib):
    """Rotate a direction/vector (no translation) from table-local into the
    robot base frame's XY -- for composing with a part's own placement yaw."""
    return _rot2d(calib["theta_deg"]) @ np.array([dx_mm, dy_mm])


def make_placeholder_calibration(z0_mm=0.0, pitch_mm=25.4):
    """A deliberately-inaccurate stand-in calibration, for when there's no real
    touch data yet (e.g. the robot base sits ~30mm above the table on a riser,
    so 'touch the table' calibration hasn't been done). Identity yaw/XY, and a
    z0 chosen to be SAFE rather than correct.

    Why z0=0 by default: with the base mounted above the table, the true table
    surface sits at a negative z (below the base origin) in the robot base
    frame. Defaulting z0 to 0 -- i.e. pretending the table is level with the
    base origin -- overstates the table height, so any move computed via
    table_point_to_robot()/hole_to_robot_xyz() targets a z that's above the
    real table surface. That means moves stop hovering in the air instead of
    plowing into the table: safe, but the tool will NOT actually reach the
    table until this is replaced with a real fit from touch data.

    If you know the riser height precisely (e.g. base sits 30mm above the
    table), pass z0_mm=-30 + <safety margin you want to keep>, not the exact
    -30 -- keep some conservative margin until it's verified against a real
    touch.
    """
    return {
        "theta_deg": 0.0,
        "tx_mm": 0.0,
        "ty_mm": 0.0,
        "z0_mm": float(z0_mm),
        "pitch_mm": pitch_mm,
        "n_points": 0,
        "residual_rms_mm": None,
        "per_point_residual_mm": [],
        "z_spread_mm": None,
        "collinearity_ratio": None,
        "reflection_detected": False,
        "placeholder": True,
        "warning": (
            "PLACEHOLDER -- not fitted from real touch data. theta/tx/ty are "
            "identity (unknown yaw/offset) and z0 was chosen conservatively "
            "(errs toward hovering above the table, not into it). Do not trust "
            "XY accuracy or run contact/insertion moves against this "
            "calibration. Replace with `fit`/`capture` using >=3 real "
            "non-collinear table touches as soon as possible."
        ),
    }


def save_calibration(fit_result, path):
    with open(path, "w") as f:
        json.dump(fit_result, f, indent=2)


def load_calibration(path):
    with open(path) as f:
        return json.load(f)


# --------------------------------------------------------------------------
# interactive capture (only code path in this file that touches the robot)
# --------------------------------------------------------------------------

def capture_points_interactive(robot_ip):
    from fairino import Robot  # local import: only this function needs it

    robot = Robot.RPC(robot_ip)
    try:
        print(f"Connecting to {robot_ip}...")
        robot.ResetAllError()
        time.sleep(0.5)
        robot.RobotEnable(1)
        time.sleep(1.0)

        drag_res = robot.IsInDragTeach()
        if isinstance(drag_res, int):
            print(f"Warning: SDK returned error code {drag_res} for Drag Mode.")
        else:
            err, drag_state = drag_res
            if drag_state != 1:
                print("Enabling drag-teach so you can hand-guide the arm...")
                robot.DragTeachSwitch(1)
                time.sleep(0.5)

        print("Drag-teach is on. For each calibration hole: hand-guide the TCP to it,")
        print("then confirm. Use >=3 widely-spread, non-collinear holes (e.g. an L-shape).")
        captured = []
        while True:
            raw = input("\nHole as 'col,row' (blank to finish): ").strip()
            if not raw:
                break
            try:
                col_s, row_s = raw.split(",")
                col, row = float(col_s), float(row_s)
            except ValueError:
                print("  could not parse -- expected e.g. '3,2'")
                continue
            input(f"  jog the TCP to hole ({col},{row}), then press Enter to capture...")
            pose_res = robot.GetActualTCPPose(0)
            if isinstance(pose_res, int):
                print(f"  Error: robot returned fault code {pose_res} -- point NOT captured")
                continue
            err, pose = pose_res
            if err != 0 or not pose:
                print(f"  Error reading pose (code {err}) -- point NOT captured")
                continue
            xyz = list(pose[:3])
            print(f"  captured hole ({col},{row}) -> robot xyz {xyz}")
            captured.append({"hole": [col, row], "robot_xyz": xyz})
        return captured
    finally:
        print("Disabling drag-teach, closing RPC...")
        try:
            robot.DragTeachSwitch(0)
        except Exception:
            pass
        robot.ResetAllError()
        robot.CloseRPC()


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _run_fit(points, args):
    hole_coords = [p["hole"] for p in points]
    robot_xyz = [p["robot_xyz"] for p in points]
    fit = fit_table_transform(hole_coords, robot_xyz, pitch_mm=args.pitch_mm)

    min_points = 2 if args.allow_two_point else 3
    problems = check_calibration_quality(fit, max_z_spread_mm=args.max_z_spread_mm,
                                          min_points=min_points)

    print(f"fitted: theta={fit['theta_deg']:.4f} deg  tx={fit['tx_mm']:.4f}mm  "
          f"ty={fit['ty_mm']:.4f}mm  z0={fit['z0_mm']:.4f}mm")
    print(f"  n_points={fit['n_points']}  residual_rms={fit['residual_rms_mm']:.4f}mm  "
          f"z_spread={fit['z_spread_mm']:.4f}mm  collinearity_ratio={fit['collinearity_ratio']:.4f}")
    print(f"  per-point residuals (mm): {['%.4f' % r for r in fit['per_point_residual_mm']]}")

    if problems:
        print("\nPROBLEMS (fit NOT saved):")
        for p in problems:
            print(f"  - {p}")
        if fit["n_points"] < min_points:
            print("\nAdd more points, or pass --allow-two-point if you explicitly accept the risk.")
        return 1

    save_calibration(fit, args.out)
    print(f"\nwrote {args.out}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    common = dict(
        pitch_mm=25.4, max_z_spread_mm=1.0, allow_two_point=False,
    )

    fit_p = sub.add_parser("fit", help="fit from a points JSON file (no robot needed)")
    fit_p.add_argument("--points", required=True, help="JSON: [{hole:[col,row], robot_xyz:[x,y,z]}, ...]")
    fit_p.add_argument("--out", default="table_calibration.json")
    fit_p.add_argument("--pitch-mm", type=float, default=common["pitch_mm"])
    fit_p.add_argument("--max-z-spread-mm", type=float, default=common["max_z_spread_mm"])
    fit_p.add_argument("--allow-two-point", action="store_true")

    cap_p = sub.add_parser("capture", help="interactively capture points via drag-teach, then fit")
    cap_p.add_argument("--robot-ip", required=True)
    cap_p.add_argument("--out", default="table_calibration.json")
    cap_p.add_argument("--pitch-mm", type=float, default=common["pitch_mm"])
    cap_p.add_argument("--max-z-spread-mm", type=float, default=common["max_z_spread_mm"])
    cap_p.add_argument("--allow-two-point", action="store_true")
    cap_p.add_argument("--save-points", help="also save the raw captured points to this JSON path")

    ph_p = sub.add_parser("placeholder",
                           help="write a deliberately-inaccurate but safe stand-in calibration "
                                "(no robot/points needed) -- use only until a real fit exists")
    ph_p.add_argument("--out", default="table_calibration.json")
    ph_p.add_argument("--pitch-mm", type=float, default=common["pitch_mm"])
    ph_p.add_argument("--z0-mm", type=float, default=0.0,
                       help="assumed table height in the robot base frame; default 0.0 "
                            "conservatively assumes the table is level with the base origin "
                            "(safe when the true table is below the base, e.g. base on a riser)")

    args = ap.parse_args(argv)

    if args.cmd == "fit":
        with open(args.points) as f:
            points = json.load(f)
        return _run_fit(points, args)

    if args.cmd == "capture":
        points = capture_points_interactive(args.robot_ip)
        if len(points) < 2:
            print("fewer than 2 points captured -- nothing to fit")
            return 1
        if args.save_points:
            with open(args.save_points, "w") as f:
                json.dump(points, f, indent=2)
            print(f"wrote raw points to {args.save_points}")
        return _run_fit(points, args)

    if args.cmd == "placeholder":
        placeholder = make_placeholder_calibration(z0_mm=args.z0_mm, pitch_mm=args.pitch_mm)
        save_calibration(placeholder, args.out)
        print(f"wrote PLACEHOLDER calibration to {args.out} (z0_mm={args.z0_mm}) -- "
              f"{placeholder['warning']}")
        return 0

    return 1


if __name__ == "__main__":
    sys.exit(main())
