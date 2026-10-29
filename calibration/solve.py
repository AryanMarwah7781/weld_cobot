"""Solve intrinsics + hand-eye from a capture session, report, and save on approval.

    python3 -m calibration.solve data/calib_YYYYmmdd_HHMMSS [--mount 0,40,60] [--rational] [--no-prompt]

--mount: your rough measurement of the camera lens position in the flange frame (mm),
used only as a sanity check. Reads only accepted samples; corners are re-detected from
the saved images so the solve can be re-run with different settings at any time.
Writes report.json into the session folder. On "yes", writes
calibration/results/handeye_<session>.json and calibration/results/current.json.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from calibration import checks
from calibration.charuco import BoardSpec, Charuco
from calibration.handeye import intrinsics_with_rejection, solve
from common.geometry import matrix_to_pose

RESULTS = Path(__file__).parent / "results"


def grade(value, good, ok):
    return "PASS" if value <= good else ("WARN" if value <= ok else "FAIL")


def load_views(session_dir, charuco):
    views = []
    for line in (session_dir / "samples.jsonl").read_text().splitlines():
        s = json.loads(line)
        if not s["accepted"]:
            continue
        gray = cv2.imread(str(session_dir / "images" / f"{s['name']}.png"), cv2.IMREAD_GRAYSCALE)
        corners, ids = charuco.detect(gray)
        if ids is None or len(ids) < checks.CaptureThresholds.min_corners:
            continue
        views.append({"name": s["name"], "flange_pose": s["flange_pose"], "corners": corners, "ids": ids,
                      "sharpness": s["metrics"].get("sharpness", 0.0)})
    return views


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--mount", help="rough camera position in flange frame, 'x,y,z' mm")
    ap.add_argument("--rational", action="store_true", help="8-coefficient distortion model (wide lenses)")
    ap.add_argument("--max-dev-mm", type=float, default=2.0)
    ap.add_argument("--no-prompt", action="store_true", help="write report only, do not ask to save")
    args = ap.parse_args()

    sdir = Path(args.session)
    meta = json.loads((sdir / "session.json").read_text())
    spec = BoardSpec(**meta["board"])
    charuco = Charuco(spec)
    size = tuple(meta["image_size"])
    lines, grades = [], []

    def out(tag, text):
        grades.append(tag)
        lines.append(f"[{tag}] {text}")

    # ---- layer 2: dataset ------------------------------------------------------
    views = load_views(sdir, charuco)
    views, dupes = checks.dedupe(views)
    div = checks.rotation_diversity([v["flange_pose"] for v in views])
    out("INFO", f"{len(views)} accepted views ({len(dupes)} near-duplicates dropped: {[d['name'] for d in dupes]})")
    out("PASS" if div >= 0.2 else "WARN", f"rotation diversity {div:.2f} (>= 0.20 wanted)")
    if len(views) < 10:
        out("FAIL", "fewer than 10 usable views; capture more")
        print("\n".join(lines))
        return

    intr, views, rej = intrinsics_with_rejection(views, charuco, size, rational=args.rational)
    out("INFO", f"intrinsics outliers dropped: {rej}")
    out(grade(intr["rms"], 0.5, 1.0), f"intrinsics RMS {intr['rms']:.3f} px over {len(views)} views")
    K, dist = intr["K"], intr["dist"]
    for v, T in zip(views, intr["T_cam_board"]):
        v["T_cam_board"] = T

    # ---- layer 3: hand-eye + consistency ---------------------------------------
    res = solve(views, charuco, K, dist, max_dev_mm=args.max_dev_mm)
    n = len(res["views"])
    out("PASS" if n >= 12 else ("WARN" if n >= 10 else "FAIL"), f"{n} views used by hand-eye; dropped {res['dropped']}")
    out(grade(float(res["dev_mm"].max()), 2.0, 4.0),
        f"board position consistency: max {res['dev_mm'].max():.2f} mm, median {np.median(res['dev_mm']):.2f} mm; "
        f"orientation max {res['dev_deg'].max():.2f} deg")
    out(grade(float(np.median(res["reproj_px"])), 1.0, 2.0),
        f"hand-eye reprojection: median {np.median(res['reproj_px']):.2f} px, max {res['reproj_px'].max():.2f} px "
        f"(closed-form before refinement: {np.median(res['reproj_px_closed']):.2f} px)")
    worst = max(res["methods"].items(), key=lambda kv: kv[1]["dt_mm"])
    out("PASS" if worst[1]["dt_mm"] <= 1.0 and max(m["dr_deg"] for m in res["methods"].values()) <= 0.3 else "WARN",
        "method agreement vs park: " + ", ".join(f"{m} {d['dt_mm']:.2f}mm/{d['dr_deg']:.2f}deg"
                                                for m, d in res["methods"].items()))
    out("INFO", f"refinement moved the result {res['refine_shift_mm']:.2f} mm / {res['refine_shift_deg']:.2f} deg")

    X = res["X"]
    cam_pos, cam_axis = X[:3, 3], X[:3, 2]
    tilt = float(np.degrees(np.arccos(np.clip(cam_axis[2], -1, 1))))
    out("INFO", f"camera position in flange frame: x {cam_pos[0]:.1f}, y {cam_pos[1]:.1f}, z {cam_pos[2]:.1f} mm")
    out("INFO", f"optical axis in flange frame: {np.round(cam_axis, 3).tolist()} ({tilt:.1f} deg from tool Z)")
    if args.mount:
        rough = np.array([float(v) for v in args.mount.split(",")])
        d = float(np.linalg.norm(rough - cam_pos))
        out("PASS" if d <= 15 else "WARN", f"mount check: {d:.1f} mm from your rough measurement {rough.tolist()}")
    dists = [float(np.linalg.norm(v["T_cam_board"][:3, 3] + v["T_cam_board"][:3, :3] @ charuco.object_points.mean(0)))
             for v in res["views"]]
    out("INFO", f"camera-to-board-centre distance over views: {min(dists):.0f}-{max(dists):.0f} mm")

    verdict = "FAIL" if "FAIL" in grades else ("WARN" if "WARN" in grades else "PASS")
    print(f"\nSession {sdir.name}  board square {spec.square_mm} mm\n" + "\n".join(lines) + f"\n\nOVERALL: {verdict}")

    result = {
        "session": str(sdir), "board": meta["board"], "image_size": list(size), "verdict": verdict,
        "K": K.tolist(), "dist": np.ravel(dist).tolist(), "rational": args.rational,
        "intrinsics_rms_px": intr["rms"],
        "T_flange_cam": X.tolist(), "flange_cam_pose": matrix_to_pose(X),
        "T_base_board": res["B"].tolist(),
        "views_used": [v["name"] for v in res["views"]], "views_dropped": res["dropped"],
        "board_dev_mm": res["dev_mm"].tolist(), "reproj_px": res["reproj_px"].tolist(),
        "methods": res["methods"], "report": lines,
    }
    (sdir / "report.json").write_text(json.dumps(result, indent=2))
    print(f"Report written to {sdir / 'report.json'}")
    if args.no_prompt:
        return
    if verdict == "FAIL":
        print("Not offering to save: fix the FAIL items first.")
        return
    if input("Type yes to save this as the current calibration: ").strip().lower() == "yes":
        RESULTS.mkdir(exist_ok=True)
        text = json.dumps(result, indent=2)
        (RESULTS / f"handeye_{sdir.name}.json").write_text(text)
        (RESULTS / "current.json").write_text(text)
        print(f"Saved {RESULTS / 'current.json'}")
    else:
        print("Not saved.")


if __name__ == "__main__":
    main()
