import numpy as np

from calibration.poses import Limits, Offset, default_offsets, plan_poses, pose_about_pivot, unwrap_near
from common.arm import JOINT_LIMITS
from common.geometry import pose_to_matrix

CENTER_POSE = [250.0, 0.0, 300.0, 180.0, 0.0, 0.0]
CENTER_JOINTS = [0.0, 20.0, 60.0, 0.0, 40.0, 0.0]
SCALE = np.array([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])   # fake robot: joints are a linear map of the pose


def fake_fk(j):
    return list(np.array(CENTER_POSE) + (np.array(j) - CENTER_JOINTS) / SCALE)


def fake_ik(pose, ref):
    d = np.array(pose) - CENTER_POSE
    d[3:] = (d[3:] + 180) % 360 - 180
    return list(np.array(CENTER_JOINTS) + d * SCALE)


def test_pivot_point_stays_fixed():
    Tc = pose_to_matrix(CENTER_POSE)
    target = (Tc @ [0, 0, 250, 1])[:3]
    for off in default_offsets():
        T = pose_about_pivot(Tc, 250, off)
        assert np.allclose((T @ [0, 0, 250 * off.dist_scale, 1])[:3], target, atol=1e-6)
        # the camera axis still points at the target
        z = T[:3, 2]
        assert np.allclose(T[:3, 3] + z * 250 * off.dist_scale, target, atol=1e-6)


def test_offsets_are_diverse():
    offs = default_offsets()
    assert len(offs) == 24
    assert len({round(o.azimuth_deg) for o in offs}) == 24
    assert {o.roll_deg for o in offs} == {-40, -15, 15, 40}
    assert {o.dist_scale for o in offs} == {0.85, 1.0, 1.15}


def test_unwrap_prefers_branch_near_reference():
    assert unwrap_near([-170, 0, 0, 170, 0, 0], [180, 0, 0, -180, 0, 0], JOINT_LIMITS)[0] == 190
    assert unwrap_near([-170, 0, 0, 170, 0, 0], [180, 0, 0, -180, 0, 0], JOINT_LIMITS)[3] == -190


def test_plan_accepts_safe_poses_and_caps_speed():
    lim = Limits(JOINT_LIMITS, min_flange_z_mm=100)
    planned = plan_poses(pose_to_matrix(CENTER_POSE), CENTER_JOINTS, 250, default_offsets(), fake_ik, fake_fk, lim)
    assert all(p.ok for p in planned), [p.reasons for p in planned if not p.ok]
    for p in planned:
        assert 0 < p.speed_deg_s <= lim.max_joint_speed_deg_s


def test_plan_rejects_low_ik_fail_and_singularity():
    lim = Limits(JOINT_LIMITS, min_flange_z_mm=CENTER_POSE[2] + 1)
    p = plan_poses(pose_to_matrix(CENTER_POSE), CENTER_JOINTS, 250, [Offset(0, 0, 0, 1.0)], fake_ik, fake_fk, lim)[0]
    assert not p.ok and any("flange z" in r for r in p.reasons)

    lim = Limits(JOINT_LIMITS)
    p = plan_poses(pose_to_matrix(CENTER_POSE), CENTER_JOINTS, 250, [Offset(10, 0, 0, 1.0)],
                   lambda pose, ref: None, fake_fk, lim)[0]
    assert not p.ok and p.reasons == ["IK failed"]

    flipped = list(CENTER_JOINTS)
    flipped[4] = -40.0
    p = plan_poses(pose_to_matrix(CENTER_POSE), CENTER_JOINTS, 250, [Offset(10, 0, 0, 1.0)],
                   lambda pose, ref: flipped, fake_fk, lim)[0]
    assert not p.ok and any("sign flip" in r for r in p.reasons)

    sing = list(CENTER_JOINTS)
    sing[4] = 3.0
    p = plan_poses(pose_to_matrix(CENTER_POSE), CENTER_JOINTS, 250, [Offset(10, 0, 0, 1.0)],
                   lambda pose, ref: sing, fake_fk, lim)[0]
    assert not p.ok and any("singularity" in r for r in p.reasons)
