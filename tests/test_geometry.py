import numpy as np

from common.geometry import invert, matrix_to_pose, mean_transform, pose_to_matrix


def _rx(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_rpy_matches_xarm_convention():
    roll, pitch, yaw = 175.3, 6.0, -2.0
    T = pose_to_matrix([1, 2, 3, roll, pitch, yaw])
    assert np.allclose(T[:3, :3], _rz(yaw) @ _ry(pitch) @ _rx(roll))
    assert np.allclose(T[:3, 3], [1, 2, 3])


def test_pose_roundtrip_and_invert():
    pose = [-43.4, -231.6, 164.6, 175.3, 6.0, -1.95]
    T = pose_to_matrix(pose)
    assert np.allclose(matrix_to_pose(T), pose, atol=1e-9)
    assert np.allclose(invert(T) @ T, np.eye(4))


def test_mean_transform_of_identical_is_identity_op():
    T = pose_to_matrix([10, 20, 30, 10, 20, 30])
    assert np.allclose(mean_transform([T, T, T]), T)
