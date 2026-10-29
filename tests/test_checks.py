import numpy as np

from calibration.checks import (CaptureThresholds, coverage, dedupe, evaluate_capture, rotation_diversity,
                                saturated_fraction, sharpness)

GRID = np.array([[x * 30.0, y * 30.0, 0.0] for y in range(1, 5) for x in range(1, 7)])


def _square_corners(x0=100, y0=100, step=40):
    return np.array([[x0 + (i % 6) * step, y0 + (i // 6) * step] for i in range(24)], np.float32).reshape(-1, 1, 2)


def test_coverage_full_and_partial():
    assert abs(coverage(np.arange(24), GRID) - 1.0) < 1e-6
    assert coverage(np.array([0, 1, 6, 7]), GRID) < 0.2


def test_sharpness_and_glare():
    rng = np.random.default_rng(0)
    sharp = (rng.random((300, 400)) * 255).astype(np.uint8)
    blurry = np.full((300, 400), 128, np.uint8)
    mask = np.full((300, 400), 255, np.uint8)
    assert sharpness(sharp, mask) > 100 * (sharpness(blurry, mask) + 1)
    blown = blurry.copy()
    blown[:30] = 255
    assert abs(saturated_fraction(blown, mask) - 0.1) < 1e-6


def test_evaluate_capture_rejects_too_few_corners():
    gray = np.zeros((400, 500), np.uint8)
    ok, reasons, _ = evaluate_capture(gray, _square_corners()[:5], np.arange(5).reshape(-1, 1), GRID, None,
                                      CaptureThresholds())
    assert not ok and "corners" in reasons[0]


def test_evaluate_capture_flags_blur_vs_reference():
    gray = np.full((400, 500), 100, np.uint8)
    ok, reasons, m = evaluate_capture(gray, _square_corners(), np.arange(24).reshape(-1, 1), GRID, 500.0,
                                      CaptureThresholds())
    assert not ok and any("blurry" in r for r in reasons)


def test_dedupe_keeps_sharper():
    a = {"name": "a", "flange_pose": [0, 0, 0, 180, 0, 0], "sharpness": 10}
    b = {"name": "b", "flange_pose": [2, 0, 0, 181, 0, 0], "sharpness": 20}
    c = {"name": "c", "flange_pose": [0, 0, 0, 180, 30, 0], "sharpness": 5}
    kept, dropped = dedupe([a, b, c])
    assert {s["name"] for s in kept} == {"b", "c"} and [s["name"] for s in dropped] == ["a"]


def test_rotation_diversity():
    flat = [[0, 0, 0, 180, 0, y] for y in (0, 10, 20, 30)]          # all about one axis
    varied = [[0, 0, 0, 180, 0, 0], [0, 0, 0, 160, 0, 0], [0, 0, 0, 180, 20, 0], [0, 0, 0, 180, 0, 20]]
    assert rotation_diversity(flat) < 0.05
    assert rotation_diversity(varied) > 0.3
