"""Quality gates for hand-eye samples.

Layer 1 (per capture): image sharpness, glare, how much of the board is seen.
Layer 2 (dataset): near-duplicate poses, rotation diversity.
Layer 3 (result consistency) lives in handeye.py.
"""
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from common.geometry import pose_to_matrix, rotation_angle_deg


@dataclass
class CaptureThresholds:
    min_corners: int = 12
    min_coverage: float = 0.30          # hull of seen corners / full inner-corner area
    max_saturated: float = 0.01         # fraction of board pixels at >= 250
    min_sharpness_ratio: float = 0.60   # vs the reference (centre) capture


def board_mask(shape, corners):
    hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.float32)).astype(np.int32)
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, hull, 255)
    return mask


def sharpness(gray, mask):
    """Variance of the Laplacian inside the board region."""
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    return float(lap[mask > 0].var())


def saturated_fraction(gray, mask):
    return float(np.mean(gray[mask > 0] >= 250))


def coverage(ids, object_points):
    """Area of the convex hull of the seen corners, as a fraction of the full corner grid."""
    if ids is None or len(ids) < 3:
        return 0.0
    pts = object_points[np.ravel(ids), :2].astype(np.float32)
    full = cv2.contourArea(cv2.convexHull(object_points[:, :2].astype(np.float32)))
    return float(cv2.contourArea(cv2.convexHull(pts)) / full)


def evaluate_capture(gray, corners, ids, object_points, ref_sharpness, th: CaptureThresholds):
    """Return (ok, reasons, metrics) for one frame."""
    n = 0 if ids is None else len(ids)
    metrics = {"corners": n}
    reasons = []
    if n < th.min_corners:
        reasons.append(f"only {n} corners (< {th.min_corners})")
        return False, reasons, metrics
    mask = board_mask(gray.shape, corners)
    metrics.update(coverage=coverage(ids, object_points),
                   sharpness=sharpness(gray, mask),
                   saturated=saturated_fraction(gray, mask))
    if metrics["coverage"] < th.min_coverage:
        reasons.append(f"board coverage {metrics['coverage']:.0%} (< {th.min_coverage:.0%})")
    if metrics["saturated"] > th.max_saturated:
        reasons.append(f"glare: {metrics['saturated']:.1%} saturated (> {th.max_saturated:.0%})")
    if ref_sharpness and metrics["sharpness"] < th.min_sharpness_ratio * ref_sharpness:
        reasons.append(f"blurry: sharpness {metrics['sharpness']:.0f} < {th.min_sharpness_ratio:.0%} of ref {ref_sharpness:.0f}")
    return not reasons, reasons, metrics


def dedupe(samples, min_rot_deg=5.0, min_trans_mm=10.0):
    """Drop samples whose flange pose is within both thresholds of an already-kept one.

    samples: dicts with 'flange_pose' and 'sharpness'. Sharper samples win.
    Returns (kept, dropped)."""
    order = sorted(samples, key=lambda s: -s.get("sharpness", 0.0))
    kept, dropped = [], []
    for s in order:
        T = pose_to_matrix(s["flange_pose"])
        dup = next((k for k in kept
                    if np.linalg.norm(T[:3, 3] - k["_T"][:3, 3]) < min_trans_mm
                    and rotation_angle_deg(T[:3, :3], k["_T"][:3, :3]) < min_rot_deg), None)
        if dup:
            dropped.append(s)
        else:
            s["_T"] = T
            kept.append(s)
    for s in kept:
        s.pop("_T")
    return kept, dropped


def rotation_diversity(flange_poses):
    """Spread of relative rotation axes between consecutive views, in [0, 1].

    Hand-eye needs rotations about at least two non-parallel axes; ~0 means all
    motions rotate about the same axis (degenerate). Returns smallest/largest
    singular value of the stacked axis*angle vectors."""
    Rs = [pose_to_matrix(p)[:3, :3] for p in flange_poses]
    vecs = [Rotation.from_matrix(Rs[0].T @ R).as_rotvec() for R in Rs[1:]]
    vecs = np.array([v for v in vecs if np.linalg.norm(v) > 1e-3])
    if len(vecs) < 3:
        return 0.0
    s = np.linalg.svd(vecs, compute_uv=False)
    return float(s[-1] / s[0])
