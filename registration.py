"""registration.py -- fit_rigid_transform(): full 3D Kabsch/Procrustes fit.

Generalizes table_calibration.fit_table_transform()'s 2D yaw-only Kabsch
fit to an unconstrained 3D rotation + translation -- same SVD structure,
same reflection-guard trick, same "don't hard-fail on too few points, warn
loudly instead" philosophy as that file, so it should read as a recognizable
sibling.

This is what answers the user's "use these [collision] data points to find
its [the part's] position relative to the STL": Mode 1 calls this at the
end of a run with nominal_points = the planned contact points from
compose_targets() (i.e. where the STL says each sphere should be, given the
selected mount hole/yaw) and measured_points = the actual TCP position
recorded at the moment force_limited_approach reported contact for each
target. The fitted transform is the discrepancy between "where the part was
supposed to be" and "where the arm actually found it."
"""

import numpy as np


def fit_rigid_transform(nominal_points, measured_points):
    """nominal_points, measured_points: (N,3) array-likes, robot-base-frame
    mm, same order/correspondence.

    Returns a dict with rotation_matrix (3x3 list), rotation_angle_deg,
    translation ([x,y,z]), n_points, residual_rms_mm, per_point_residual_mm,
    degenerate (bool), warnings (list of str).
    """
    nominal = np.asarray(nominal_points, dtype=np.float64).reshape(-1, 3)
    measured = np.asarray(measured_points, dtype=np.float64).reshape(-1, 3)
    n = len(nominal)
    warnings = []

    if measured.shape[0] != n:
        raise ValueError("nominal_points and measured_points must have the same length")

    if n == 0:
        return {
            "rotation_matrix": np.eye(3).tolist(),
            "rotation_angle_deg": 0.0,
            "translation": [0.0, 0.0, 0.0],
            "n_points": 0,
            "residual_rms_mm": None,
            "per_point_residual_mm": [],
            "degenerate": True,
            "warnings": ["no points to register -- nothing was reached during this run"],
        }

    if n == 1:
        translation = (measured[0] - nominal[0]).tolist()
        return {
            "rotation_matrix": np.eye(3).tolist(),
            "rotation_angle_deg": 0.0,
            "translation": translation,
            "n_points": 1,
            "residual_rms_mm": 0.0,
            "per_point_residual_mm": [0.0],
            "degenerate": True,
            "warnings": [
                "only 1 point -- rotation is undetermined; only a translation-only "
                "offset is reported (identity rotation assumed)."
            ],
        }

    p_centroid = nominal.mean(axis=0)
    q_centroid = measured.mean(axis=0)
    P = nominal - p_centroid
    Q = measured - q_centroid

    # Collinearity of the nominal points alone (independent of what was
    # measured): ratio of smallest to largest singular value. Near 0 means
    # the points don't really span 3D -- some rotational DOF is poorly
    # determined regardless of measurement quality. Mirrors
    # table_calibration.py's collinearity_ratio, generalized to 3D.
    sv_P = np.linalg.svd(P, compute_uv=False)
    collinearity_ratio = float(sv_P[-1] / sv_P[0]) if sv_P[0] > 1e-12 else 0.0

    H = P.T @ Q
    U, S, Vt = np.linalg.svd(H)
    d = 1.0 if np.linalg.det(Vt.T @ U.T) >= 0 else -1.0
    reflection_detected = d < 0
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = q_centroid - R @ p_centroid

    predicted = (R @ nominal.T).T + t
    per_point_residual_mm = np.linalg.norm(predicted - measured, axis=1)
    residual_rms_mm = float(np.sqrt(np.mean(per_point_residual_mm ** 2)))

    try:
        from scipy.spatial.transform import Rotation

        rotation_angle_deg = float(Rotation.from_matrix(R).magnitude() * 180.0 / np.pi)
    except Exception:
        # Fallback with no scipy: angle of rotation from trace(R) = 1 + 2cos(theta).
        cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
        rotation_angle_deg = float(np.degrees(np.arccos(cos_theta)))

    degenerate = False
    if n == 2:
        degenerate = True
        warnings.append(
            "only 2 points -- rotation about the line connecting them is "
            "unconstrained; treat rotation_angle_deg with caution, the "
            "translation is more trustworthy."
        )
    elif collinearity_ratio < 0.05:
        degenerate = True
        warnings.append(
            f"points are nearly collinear (spread ratio {collinearity_ratio:.4f}) -- "
            "rotation about that line is poorly constrained regardless of point count."
        )

    if reflection_detected:
        warnings.append(
            "fit required a reflection correction to stay a proper rotation -- "
            "this normally only happens with a bad correspondence (e.g. two "
            "targets swapped). Double check the per-point residuals."
        )

    return {
        "rotation_matrix": R.tolist(),
        "rotation_angle_deg": rotation_angle_deg,
        "translation": t.tolist(),
        "n_points": n,
        "residual_rms_mm": residual_rms_mm,
        "per_point_residual_mm": per_point_residual_mm.tolist(),
        "degenerate": degenerate,
        "warnings": warnings,
    }
