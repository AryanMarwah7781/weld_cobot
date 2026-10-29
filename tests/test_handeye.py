"""Synthetic end-to-end check: simulate views of a fixed board from a camera with a known
mount, then verify intrinsics + hand-eye recover that mount and reject corrupted views."""
import cv2
import numpy as np
import pytest

from calibration.charuco import BoardSpec, Charuco
from calibration.handeye import intrinsics_with_rejection, solve
from calibration.poses import default_offsets, pose_about_pivot
from common.geometry import invert, matrix_to_pose, pose_to_matrix, rotation_angle_deg, rt_to_matrix
from scipy.spatial.transform import Rotation

K_TRUE = np.array([[1400.0, 0, 960], [0, 1400.0, 600], [0, 0, 1]])
DIST_TRUE = np.array([-0.12, 0.05, 0.0005, -0.0003, 0.0])
SIZE = (1920, 1200)
X_TRUE = rt_to_matrix(Rotation.from_rotvec([0.04, -0.03, 1.2]).as_matrix(), [30.0, -20.0, 60.0])
CENTER = [250.0, 0.0, 300.0, 180.0, 0.0, 0.0]


@pytest.fixture(scope="module")
def charuco():
    return Charuco(BoardSpec(7, 5, 31.79, 23.31, "DICT_5X5_100"))


def simulate(charuco, rng, noise_px=0.15):
    Tc = pose_to_matrix(CENTER)
    pts = charuco.object_points
    center = pts.mean(axis=0)
    # board faces the centre camera at 240 mm, centred in view
    B = Tc @ X_TRUE @ rt_to_matrix(np.eye(3), [-center[0], -center[1], 240.0])
    views = []
    for i, off in enumerate(default_offsets()):
        F = pose_about_pivot(Tc, 300.0, off)
        C = invert(X_TRUE) @ invert(F) @ B
        img, _ = cv2.projectPoints(pts, Rotation.from_matrix(C[:3, :3]).as_rotvec(), C[:3, 3], K_TRUE, DIST_TRUE)
        img = img.reshape(-1, 2) + rng.normal(0, noise_px, (len(pts), 2))
        inside = np.where((img[:, 0] > 5) & (img[:, 0] < SIZE[0] - 5) & (img[:, 1] > 5) & (img[:, 1] < SIZE[1] - 5))[0]
        if len(inside) < 12:
            continue
        views.append({"name": f"v{i:02d}", "flange_pose": matrix_to_pose(F),
                      "corners": img[inside].reshape(-1, 1, 2).astype(np.float32),
                      "ids": inside.reshape(-1, 1).astype(np.int32)})
    return views


def run_pipeline(charuco, views):
    intr, views, _ = intrinsics_with_rejection(views, charuco, SIZE)
    for v, T in zip(views, intr["T_cam_board"]):
        v["T_cam_board"] = T
    return intr, solve(views, charuco, intr["K"], intr["dist"])


def test_recovers_mount(charuco):
    views = simulate(charuco, np.random.default_rng(0))
    assert len(views) >= 18
    intr, res = run_pipeline(charuco, views)
    assert intr["rms"] < 0.3
    assert np.linalg.norm(res["X"][:3, 3] - X_TRUE[:3, 3]) < 0.5
    assert rotation_angle_deg(res["X"][:3, :3], X_TRUE[:3, :3]) < 0.1
    assert np.median(res["reproj_px"]) < 0.4
    assert res["dev_mm"].max() < 1.0
    assert not res["dropped"]


def test_rejects_views_with_bad_arm_pose(charuco):
    views = simulate(charuco, np.random.default_rng(1))
    for v in views[3:5]:   # e.g. the board was bumped / pose logged wrongly by 6 mm
        v["flange_pose"] = list(v["flange_pose"])
        v["flange_pose"][0] += 6.0
    _, res = run_pipeline(charuco, views)
    assert {n for n, _ in res["dropped"]} == {views[3]["name"], views[4]["name"]}
    assert np.linalg.norm(res["X"][:3, 3] - X_TRUE[:3, 3]) < 0.5
