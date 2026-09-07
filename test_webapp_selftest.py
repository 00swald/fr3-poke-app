"""
test_webapp_selftest.py -- plain-assert self-tests for arm_control_webapp's
new pieces (probe.py, hardware.SimulatedRobot, sensor_backend.NullSensorBackend,
registration.py). Same no-pytest, no-hardware-needed convention as the repo
root's test_geometry_selftest.py -- run directly:

    cd FHL26 && python3 arm_control_webapp/test_webapp_selftest.py

Everything here runs against SimulatedRobot + NullSensorBackend, so it needs
no real arm, sensor, or hardware SDK (fairino/Bota_sys/inputs) installed.
"""

import sys
import time

import numpy as np

import config  # noqa: F401 -- patches sys.path for stl_geometry/table_calibration/move_to_spheres
import hardware
import probe as probe_mod
import registration
import sensor_backend


# --------------------------------------------------------------------------
# probe_vertical against SimulatedRobot + NullSensorBackend
# --------------------------------------------------------------------------

def test_probe_vertical_no_contact_reaches_floor():
    robot = hardware.SimulatedRobot(start_pose=[0.0, 0.0, 100.0, 180.0, 0.0, 0.0])
    sensor = sensor_backend.NullSensorBackend()
    sensor.start()
    try:
        result = probe_mod.probe_vertical(
            robot, sensor, (0.0, 0.0), start_z=100.0, min_z=0.0,
            vel=200.0, force_threshold_n=5.0, stale_s=2.0,
        )
        assert result["outcome"] == "no_contact_at_min_z", result
        assert result["z_contact"] is None
        assert result["reached"] is False
        # retracted back to the safe start height
        assert abs(robot.pose[2] - 100.0) < 1e-6
    finally:
        sensor.stop()


def test_probe_vertical_detects_manual_collision():
    robot = hardware.SimulatedRobot(start_pose=[0.0, 0.0, 100.0, 180.0, 0.0, 0.0])
    sensor = sensor_backend.NullSensorBackend()
    sensor.start()
    try:
        def trigger_collision():
            time.sleep(0.3)
            sensor.simulate_collision(force_n=20.0)

        import threading
        threading.Thread(target=trigger_collision, daemon=True).start()

        result = probe_mod.probe_vertical(
            robot, sensor, (0.0, 0.0), start_z=100.0, min_z=0.0,
            vel=5.0, force_threshold_n=5.0, stale_s=2.0,
        )
        assert result["outcome"] == "contact", result
        assert result["reached"] is True
        assert result["z_contact"] is not None
        assert 0.0 <= result["z_contact"] <= 100.0
    finally:
        sensor.stop()


def test_probe_vertical_stale_sensor_fails_safe():
    robot = hardware.SimulatedRobot(start_pose=[0.0, 0.0, 100.0, 180.0, 0.0, 0.0])
    # No .start() -- fz_vals never grows at all, so staleness trips almost
    # immediately. This is the "sensor genuinely absent" fail-safe path.
    sensor = sensor_backend.NullSensorBackend()
    result = probe_mod.probe_vertical(
        robot, sensor, (0.0, 0.0), start_z=100.0, min_z=0.0,
        vel=1.0, force_threshold_n=5.0, stale_s=0.05,
    )
    assert result["reached"] is False
    assert "stale" in (result["outcome"] or "")


# --------------------------------------------------------------------------
# SimulatedRobot motion contract
# --------------------------------------------------------------------------

def test_simulated_robot_motion_done_contract():
    robot = hardware.SimulatedRobot(start_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    err = robot.MoveL([100.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=50.0, acc=50.0)
    assert err == 0
    _, done = robot.GetRobotMotionDone()
    assert done == 0, "should report not-done immediately after dispatch"
    while True:
        _, done = robot.GetRobotMotionDone()
        if done == 1:
            break
        time.sleep(0.01)
    err, pose = robot.GetActualTCPPose(0)
    assert err == 0
    assert abs(pose[0] - 100.0) < 1e-6


def test_simulated_robot_stop_motion_freezes_position():
    robot = hardware.SimulatedRobot(start_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.MoveL([1000.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=10.0, acc=10.0)  # ~100s move
    time.sleep(0.1)
    robot.StopMotion()
    _, done = robot.GetRobotMotionDone()
    assert done == 1
    _, pose = robot.GetActualTCPPose(0)
    assert 0.0 < pose[0] < 1000.0, "should have stopped partway, not at the target"


# --------------------------------------------------------------------------
# registration.fit_rigid_transform
# --------------------------------------------------------------------------

def test_fit_rigid_transform_recovers_known_transform():
    rng = np.random.default_rng(0)
    nominal = rng.uniform(-50, 50, size=(6, 3))

    axis = np.array([0.2, 0.7, 0.4])
    axis = axis / np.linalg.norm(axis)
    theta = np.radians(17.0)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R_true = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    t_true = np.array([5.0, -3.0, 2.0])

    measured = (R_true @ nominal.T).T + t_true

    result = registration.fit_rigid_transform(nominal, measured)
    assert result["degenerate"] is False
    assert result["residual_rms_mm"] < 1e-6
    assert abs(result["rotation_angle_deg"] - 17.0) < 1e-3
    assert np.allclose(result["translation"], t_true, atol=1e-6)


def test_fit_rigid_transform_degenerate_cases():
    empty = registration.fit_rigid_transform([], [])
    assert empty["degenerate"] is True
    assert empty["n_points"] == 0

    one = registration.fit_rigid_transform([[0, 0, 0]], [[1, 2, 3]])
    assert one["degenerate"] is True
    assert one["n_points"] == 1
    assert np.allclose(one["translation"], [1, 2, 3])

    two = registration.fit_rigid_transform([[0, 0, 0], [10, 0, 0]], [[0, 0, 0], [10, 0, 0]])
    assert two["degenerate"] is True
    assert two["n_points"] == 2

    collinear = registration.fit_rigid_transform(
        [[0, 0, 0], [1, 0, 0], [2, 0, 0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    )
    assert collinear["degenerate"] is True


TESTS = [
    test_probe_vertical_no_contact_reaches_floor,
    test_probe_vertical_detects_manual_collision,
    test_probe_vertical_stale_sensor_fails_safe,
    test_simulated_robot_motion_done_contract,
    test_simulated_robot_stop_motion_freezes_position,
    test_fit_rigid_transform_recovers_known_transform,
    test_fit_rigid_transform_degenerate_cases,
]


def main():
    failures = []
    for t in TESTS:
        print(f"{t.__name__} ...")
        try:
            t()
            print("  PASS")
        except AssertionError as e:
            failures.append(t.__name__)
            print(f"  FAIL: {e}")
        except Exception as e:
            failures.append(t.__name__)
            print(f"  ERROR: {type(e).__name__}: {e}")
    print()
    if failures:
        print(f"{len(failures)}/{len(TESTS)} FAILED: {failures}")
        return 1
    print(f"all {len(TESTS)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
