"""probe.py -- probe_vertical(): the "move straight down until collision,
record z" primitive the user asked for. Used both by Mode 1's single
pre-probe at the selected table hole and by Mode 3's 11x11 height-map grid
-- one shared implementation, not one poll loop reimplemented per mode.

Deliberately built as a thin wrapper around
move_to_spheres.force_limited_approach() rather than a second
implementation of its poll loop: the safety-relevant logic (poll
motion-done + force + sensor staleness together, stop the instant any trips)
lives in exactly one place in this codebase.
"""

import config  # noqa: F401 -- import first so sys.path is patched before mts import
import move_to_spheres as mts


def probe_vertical(robot, sensor, xy_robot, *, start_z, min_z, vel,
                    force_threshold_n, stale_s, poll_interval_s=0.02,
                    start_timeout_s=2.0):
    """Move the TCP down to (xy_robot, start_z) at a safe travel height,
    then straight down (robot/world -Z; tool orientation held at whatever
    it currently is -- this is a vertical probe, not a normal-to-surface
    approach) toward (xy_robot, min_z), stopping on force threshold or
    sensor staleness. Retracts back to start_z before returning either way.

    Returns {"z_contact": float or None, "outcome": str, "reached": bool}.
    z_contact is the actual TCP z (via GetActualTCPPose), not the commanded
    floor -- the whole point of force-limiting is it usually stops short of
    min_z. z_contact is None if the probe reached min_z or faulted without
    ever registering contact.
    """
    x, y = float(xy_robot[0]), float(xy_robot[1])

    err, current_pose = robot.GetActualTCPPose(0)
    if err != 0 or not current_pose:
        return {"z_contact": None, "outcome": f"could not read current pose (code {err})", "reached": False}
    rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]

    # 1. Travel to the safe start pose above the probe point (not
    # force-limited -- by contract of the caller, start_z is a safe height).
    start_pose = [x, y, float(start_z), rx, ry, rz]
    err = robot.MoveL(start_pose, tool=0, user=0, vel=vel, acc=vel)
    if err != 0:
        return {"z_contact": None, "outcome": f"travel-to-start MoveL rejected, code {err}", "reached": False}
    mts._wait_motion_done(robot)

    # 2. Force-limited descent toward the floor. An absolute-pose MoveL
    # can't travel further than min_z, so no separate distance cap is
    # needed on top of the force/staleness watchdogs.
    target_pose = [x, y, float(min_z), rx, ry, rz]
    result = mts.force_limited_approach(
        robot, sensor, target_pose, vel=vel, force_threshold_n=force_threshold_n,
        stale_s=stale_s, poll_interval_s=poll_interval_s, start_timeout_s=start_timeout_s,
    )

    err, contact_pose = robot.GetActualTCPPose(0)
    actual_z = contact_pose[2] if (err == 0 and contact_pose) else None

    if result["reached"]:
        outcome = "contact"
        z_contact = actual_z
    elif result["stopped_reason"] and "reached target pose" in result["stopped_reason"]:
        outcome = "no_contact_at_min_z"
        z_contact = None
    else:
        # MoveL rejected, or sensor went stale, or some other fault -- a
        # real problem the caller should surface/abort on, not silently
        # continue past.
        outcome = result["stopped_reason"] or "unknown fault"
        z_contact = None

    # 3. Retract to the safe start height before returning, so the caller
    # can move on to the next XY point without a floor-height collision risk.
    robot.MoveL(start_pose, tool=0, user=0, vel=vel, acc=vel)
    mts._wait_motion_done(robot)

    return {"z_contact": z_contact, "outcome": outcome, "reached": bool(result["reached"])}
