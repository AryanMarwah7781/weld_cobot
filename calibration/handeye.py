"""Camera intrinsics + eye-in-hand calibration with consistency checks (layer 3).

Frames: base (robot), flange, cam (OpenCV camera: +Z optical axis), board (ChArUco origin).
T_a_b maps points from frame b into frame a. Units: mm, degrees.

Unknowns: X = T_flange_cam (what we want) and B = T_base_board (board is fixed).
For every view i: B = F_i @ X @ C_i, where F_i = T_base_flange (from the arm) and
C_i = T_cam_board (from the image).
"""
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from common.geometry import (invert, matrix_to_pose, mean_transform, pose_to_matrix, rotation_angle_deg,
                             rt_to_matrix, rvec_tvec_to_matrix)

METHODS = {
    "park": cv2.CALIB_HAND_EYE_PARK,
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# ---- intrinsics ----------------------------------------------------------------
def calibrate_intrinsics(views, charuco, image_size, rational=False):
    """views: list of dicts with 'corners', 'ids'. Returns dict with K, dist, rms, per_view, poses."""
    flags = cv2.CALIB_RATIONAL_MODEL if rational else 0
    rms, K, dist, rvecs, tvecs, _, _, per_view = cv2.aruco.calibrateCameraCharucoExtended(
        [v["corners"] for v in views], [v["ids"] for v in views], charuco.board, image_size,
        None, None, flags=flags)
    return {"rms": float(rms), "K": K, "dist": dist, "per_view": np.ravel(per_view),
            "T_cam_board": [rvec_tvec_to_matrix(r, t) for r, t in zip(rvecs, tvecs)]}


def intrinsics_with_rejection(views, charuco, image_size, rational=False, factor=3.0, min_views=10):
    """Fit, drop views whose error exceeds factor x median, refit once."""
    res = calibrate_intrinsics(views, charuco, image_size, rational)
    limit = factor * float(np.median(res["per_view"]))
    keep = [i for i, e in enumerate(res["per_view"]) if e <= limit]
    rejected = [(views[i]["name"], float(res["per_view"][i])) for i in range(len(views)) if i not in keep]
    if rejected and len(keep) >= min_views:
        views = [views[i] for i in keep]
        res = calibrate_intrinsics(views, charuco, image_size, rational)
    else:
        rejected = []
    return res, views, rejected


# ---- hand-eye --------------------------------------------------------------------
def closed_form(F, C, method):
    R_x, t_x = cv2.calibrateHandEye([T[:3, :3] for T in F], [T[:3, 3] for T in F],
                                    [T[:3, :3] for T in C], [T[:3, 3] for T in C], method=METHODS[method])
    return rt_to_matrix(R_x, t_x)


def board_in_base(F, X, C):
    return [Fi @ X @ Ci for Fi, Ci in zip(F, C)]


def board_center(charuco):
    return charuco.object_points.mean(axis=0)


def consistency(F, X, C, center):
    """Per-view deviation of the board (centre point in mm, orientation in deg) from the mean."""
    Bs = board_in_base(F, X, C)
    B = mean_transform(Bs)
    c_mean = B[:3, :3] @ center + B[:3, 3]
    dt = np.array([np.linalg.norm(Bi[:3, :3] @ center + Bi[:3, 3] - c_mean) for Bi in Bs])
    dr = np.array([rotation_angle_deg(Bi[:3, :3], B[:3, :3]) for Bi in Bs])
    return B, dt, dr


def _project(K, dist, T_cam_board, pts):
    rvec = Rotation.from_matrix(T_cam_board[:3, :3]).as_rotvec()
    img, _ = cv2.projectPoints(pts, rvec, T_cam_board[:3, 3], K, dist)
    return img.reshape(-1, 2)


def reprojection_errors(F, X, B, views, K, dist, object_points):
    """Pixel RMS per view when the board is predicted from the arm pose + X + B (not from the image)."""
    errs = []
    for Fi, v in zip(F, views):
        C_pred = invert(X) @ invert(Fi) @ B
        pts = object_points[np.ravel(v["ids"])]
        d = _project(K, dist, C_pred, pts) - v["corners"].reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(d ** 2, axis=1)))))
    return np.array(errs)


def _to_params(T):
    return np.concatenate([Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


def _from_params(p):
    return rt_to_matrix(Rotation.from_rotvec(p[:3]).as_matrix(), p[3:6])


def refine(F, X0, B0, views, K, dist, object_points):
    """Jointly refine X and B by minimising pixel reprojection error over all views."""
    pts = [object_points[np.ravel(v["ids"])] for v in views]
    obs = [v["corners"].reshape(-1, 2) for v in views]
    Finv = [invert(Fi) for Fi in F]

    def residuals(p):
        Xinv, B = invert(_from_params(p[:6])), _from_params(p[6:])
        return np.concatenate([(_project(K, dist, Xinv @ Fi_inv @ B, P) - o).ravel()
                               for Fi_inv, P, o in zip(Finv, pts, obs)])

    sol = least_squares(residuals, np.concatenate([_to_params(X0), _to_params(B0)]), loss="huber", f_scale=1.0)
    return _from_params(sol.x[:6]), _from_params(sol.x[6:])


def solve(views, charuco, K, dist, primary="park", max_dev_mm=2.0, min_views=10):
    """Hand-eye with iterative outlier rejection.

    views: dicts with 'name', 'flange_pose', 'corners', 'ids', 'T_cam_board'.
    Drops the single worst view while its board deviation exceeds max(3 x robust sigma, max_dev_mm)."""
    views = list(views)
    center = board_center(charuco)
    dropped = []
    while True:
        F = [pose_to_matrix(v["flange_pose"]) for v in views]
        C = [v["T_cam_board"] for v in views]
        X = closed_form(F, C, primary)
        B, dt, dr = consistency(F, X, C, center)
        mad = 1.4826 * float(np.median(np.abs(dt - np.median(dt))))
        limit = max(3.0 * mad, max_dev_mm)
        worst = int(np.argmax(dt))
        if dt[worst] <= limit or len(views) <= min_views:
            break
        dropped.append((views[worst]["name"], float(dt[worst])))
        views.pop(worst)

    methods = {}
    for m in METHODS:
        Xm = closed_form(F, C, m)
        methods[m] = {"dt_mm": float(np.linalg.norm(Xm[:3, 3] - X[:3, 3])),
                      "dr_deg": rotation_angle_deg(Xm[:3, :3], X[:3, :3]),
                      "pose": matrix_to_pose(Xm)}

    X_ref, B_ref = refine(F, X, B, views, K, dist, charuco.object_points)
    _, dt_ref, dr_ref = consistency(F, X_ref, C, center)
    return {
        "views": views, "dropped": dropped,
        "X_closed": X, "B_closed": B,
        "X": X_ref, "B": B_ref,
        "dev_mm": dt_ref, "dev_deg": dr_ref,
        "reproj_px": reprojection_errors(F, X_ref, B_ref, views, K, dist, charuco.object_points),
        "reproj_px_closed": reprojection_errors(F, X, B, views, K, dist, charuco.object_points),
        "methods": methods,
        "refine_shift_mm": float(np.linalg.norm(X_ref[:3, 3] - X[:3, 3])),
        "refine_shift_deg": rotation_angle_deg(X_ref[:3, :3], X[:3, :3]),
    }
