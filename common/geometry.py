"""Rigid-transform helpers.

xArm poses are [x, y, z, roll, pitch, yaw] in mm and degrees, fixed-axis XYZ:
R = Rz(yaw) @ Ry(pitch) @ Rx(roll), which is SciPy's lowercase 'xyz' (extrinsic).
"""
import numpy as np
from scipy.spatial.transform import Rotation


def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", pose[3:6], degrees=True).as_matrix()
    T[:3, 3] = pose[:3]
    return T


def matrix_to_pose(T):
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    return [float(v) for v in T[:3, 3]] + [float(v) for v in rpy]


def rt_to_matrix(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.ravel(t)
    return T


def rvec_tvec_to_matrix(rvec, tvec):
    return rt_to_matrix(Rotation.from_rotvec(np.ravel(rvec)).as_matrix(), tvec)


def invert(T):
    R, t = T[:3, :3], T[:3, 3]
    return rt_to_matrix(R.T, -R.T @ t)


def rot_axis_angle(axis, deg):
    axis = np.asarray(axis, float)
    return Rotation.from_rotvec(axis / np.linalg.norm(axis) * np.radians(deg)).as_matrix()


def translation_z(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def rotation_angle_deg(Ra, Rb):
    """Angle of the relative rotation between two 3x3 rotation matrices."""
    return float(np.degrees(Rotation.from_matrix(Ra.T @ Rb).magnitude()))


def mean_transform(Ts):
    """Average rigid transform: mean translation, chordal-mean rotation."""
    R = Rotation.from_matrix(np.array([T[:3, :3] for T in Ts])).mean().as_matrix()
    return rt_to_matrix(R, np.mean([T[:3, 3] for T in Ts], axis=0))
