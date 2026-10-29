"""Read-only: plan hand-eye poses around the arm's current (or a given hypothetical) joint pose. No motion.

    python3 -m tools.plan_check 250 ['[j1,j2,j3,j4,j5,j6]']
"""
import sys

import cv2

from calibration.capture import print_plan
from calibration.charuco import BoardSpec, Charuco
from calibration.poses import Limits, default_offsets, plan_poses
from common.arm import JOINT_LIMITS, SafeArm
from common.camera import Camera
from common.geometry import pose_to_matrix

import json

pivot = float(sys.argv[1]) if len(sys.argv) > 1 else 250.0
with SafeArm() as arm:
    j = json.loads(sys.argv[2]) if len(sys.argv) > 2 else arm.joints()   # optional hypothetical centre
    pose = arm.fk(j)
    print("current joints", [round(a, 2) for a in j])
    print("FK(current)   ", [round(a, 2) for a in pose], "\ncontroller    ", [round(a, 2) for a in arm.flange_pose()])
    T = pose_to_matrix(pose)
    lim = Limits(JOINT_LIMITS, min_flange_z_mm=T[2, 3] - 0.35 * pivot)
    planned = plan_poses(T, j, pivot, default_offsets(), arm.ik, arm.fk, lim)
    print_plan(planned)
    print(f"{sum(p.ok for p in planned)}/{len(planned)} ok (pivot {pivot} mm, min z {lim.min_flange_z_mm:.0f})")
with Camera() as cam:
    frame, _ = cam.latest()
    corners, ids = Charuco(BoardSpec.load()).detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    print("board corners visible now:", 0 if ids is None else len(ids))
