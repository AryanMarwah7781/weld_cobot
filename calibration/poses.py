"""Hand-eye viewpoint generation around a taught centre pose, plus pre-motion checks.

Every candidate is a rotation of the flange about a pivot point that lies pivot_mm
along the flange +Z axis from the taught centre pose (the point the camera looks at,
assuming the camera looks roughly along tool Z). Optionally the camera then moves
nearer/farther along its own axis. Nothing here talks to hardware: ik/fk are passed
in, so the checks can be tested offline.
"""
from dataclasses import dataclass, field

import numpy as np

from common.geometry import matrix_to_pose, pose_to_matrix, rot_axis_angle, translation_z

GOLDEN_ANGLE_DEG = 137.508


@dataclass
class Offset:
    tilt_deg: float      # rotation away from the centre viewing direction
    azimuth_deg: float   # direction of that tilt, in the flange XY plane
    roll_deg: float      # rotation about the viewing axis
    dist_scale: float    # pivot distance multiplier (1.0 = taught standoff)


def default_offsets(n=24):
    """Diverse tilt directions (golden-angle azimuths), rolls and distances."""
    tilts, rolls, scales = [12.0, 20.0, 28.0], [-40.0, -15.0, 15.0, 40.0], [0.85, 1.0, 1.15]
    return [Offset(tilts[i % 3], (i * GOLDEN_ANGLE_DEG) % 360.0, rolls[i % 4], scales[(i // 3) % 3])
            for i in range(n)]


def pose_about_pivot(T_center, pivot_mm, off: Offset):
    """Flange transform for one offset. The pivot point stays fixed in the base frame."""
    az = np.radians(off.azimuth_deg)
    R = rot_axis_angle([np.cos(az), np.sin(az), 0.0], off.tilt_deg) @ rot_axis_angle([0, 0, 1], off.roll_deg)
    R4 = np.eye(4)
    R4[:3, :3] = R
    return T_center @ translation_z(pivot_mm) @ R4 @ translation_z(-pivot_mm * off.dist_scale)


def unwrap_near(joints, ref, limits):
    """Shift each joint by ±360° when that lands closer to ref and stays within limits."""
    out = []
    for q, r, (lo, hi) in zip(joints, ref, limits):
        best = q
        for k in (-360.0, 360.0):
            if lo <= q + k <= hi and abs(q + k - r) < abs(best - r):
                best = q + k
        out.append(best)
    return out


@dataclass
class Limits:
    joint_limits: list
    joint_margin_deg: float = 5.0
    j5_min_abs_deg: float = 10.0        # keep away from the wrist singularity
    max_joint_delta_deg: float = 90.0   # vs the centre config, rejects IK branch flips
    min_flange_z_mm: float = 0.0
    max_radius_mm: float = 420.0
    fk_tol_mm: float = 0.5
    fk_tol_deg: float = 0.2
    max_tcp_speed_mm_s: float = 20.0
    max_joint_speed_deg_s: float = 10.0
    tool_reach_mm: float = 150.0        # camera body assumed within this distance along tool Z


@dataclass
class PlannedPose:
    index: int
    offset: Offset
    flange_pose: list
    joints: list = None
    speed_deg_s: float = None
    ok: bool = False
    reasons: list = field(default_factory=list)


def _path_points(fk, j_from, j_to, n=12):
    """Base-frame positions of the flange and a point tool_reach along tool Z, along a joint-space path."""
    flange, tool = [], []
    for s in np.linspace(0.0, 1.0, n):
        T = pose_to_matrix(fk([a + s * (b - a) for a, b in zip(j_from, j_to)]))
        flange.append(T[:3, 3])
        tool.append(T)
    return np.array(flange), tool


def check_path(fk, j_from, j_to, lim: Limits):
    """Check a joint-space move and pick its joint speed. Returns (reasons, speed_deg_s)."""
    reasons = []
    delta = max(abs(a - b) for a, b in zip(j_from, j_to))
    flange_path, Ts = _path_points(fk, j_from, j_to)
    if flange_path[:, 2].min() < lim.min_flange_z_mm:
        reasons.append(f"path dips to flange z {flange_path[:, 2].min():.0f}")
    tool_path = np.array([Tk[:3, :3] @ [0, 0, lim.tool_reach_mm] + Tk[:3, 3] for Tk in Ts])
    length = max(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) for path in (flange_path, tool_path))
    # Joint moves are synchronised: duration = delta / speed. Cap the mean Cartesian speed at
    # 80% of the limit to leave room for the non-uniform speed along a joint-space path.
    speed = lim.max_joint_speed_deg_s if length < 1e-6 else min(
        lim.max_joint_speed_deg_s, 0.8 * lim.max_tcp_speed_mm_s * delta / length)
    speed = round(speed, 2)
    if speed < 0.5:
        reasons.append(f"path too long for speed cap ({length:.0f} mm)")
    return reasons, speed


def plan_poses(T_center, center_joints, pivot_mm, offsets, ik, fk, lim: Limits):
    """Compute joints for every offset and run all pre-motion checks.

    The robot is only ever moved centre -> pose -> centre, so each path is checked
    from the centre configuration."""
    planned = []
    for i, off in enumerate(offsets):
        T = pose_about_pivot(T_center, pivot_mm, off)
        pose = matrix_to_pose(T)
        p = PlannedPose(i, off, pose)
        planned.append(p)

        if T[2, 3] < lim.min_flange_z_mm:
            p.reasons.append(f"flange z {T[2, 3]:.0f} < min {lim.min_flange_z_mm:.0f}")
        if np.hypot(T[0, 3], T[1, 3]) > lim.max_radius_mm:
            p.reasons.append(f"radius {np.hypot(T[0, 3], T[1, 3]):.0f} > {lim.max_radius_mm:.0f}")
        joints = ik(pose, center_joints)
        if joints is None:
            p.reasons.append("IK failed")
            continue
        # Note: a flipped-wrist IK answer cannot be mapped back with (J4+180, -J5, J6+180) on the
        # Lite 6; its wrist axes do not quite intersect (tested: ~2.3 mm FK error). Reject instead.
        joints = unwrap_near(joints, center_joints, lim.joint_limits)
        p.joints = joints

        for k, (q, (lo, hi)) in enumerate(zip(joints, lim.joint_limits)):
            if not lo + lim.joint_margin_deg <= q <= hi - lim.joint_margin_deg:
                p.reasons.append(f"J{k + 1}={q:.1f} near limit")
        if abs(joints[4]) < lim.j5_min_abs_deg:
            p.reasons.append(f"J5={joints[4]:.1f} near singularity")
        if np.sign(joints[4]) != np.sign(center_joints[4]):
            p.reasons.append(f"J5 sign flip ({center_joints[4]:.1f} -> {joints[4]:.1f}): wrist would cross singularity")
        deltas = [abs(a - b) for a, b in zip(joints, center_joints)]
        delta = max(deltas)
        if delta > lim.max_joint_delta_deg:
            p.reasons.append(f"J{deltas.index(delta) + 1} changes {delta:.0f} deg from centre (IK branch flip?)")

        T_fk = pose_to_matrix(fk(joints))
        pos_err = np.linalg.norm(T_fk[:3, 3] - T[:3, 3])
        ang_err = np.degrees(np.arccos(np.clip((np.trace(T_fk[:3, :3].T @ T[:3, :3]) - 1) / 2, -1, 1)))
        if pos_err > lim.fk_tol_mm or ang_err > lim.fk_tol_deg:
            p.reasons.append(f"FK mismatch {pos_err:.2f} mm / {ang_err:.2f} deg")

        path_reasons, p.speed_deg_s = check_path(fk, center_joints, joints, lim)
        p.reasons += path_reasons
        p.ok = not p.reasons
    return planned
