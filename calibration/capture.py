"""Hand-eye data capture (eye-in-hand, board fixed on the table).

Run ON THE JETSON, from ~/weld_cobot, in an interactive terminal, standing at the arm
with the emergency stop in reach:

    python3 -m calibration.capture --pivot-mm 250            # teach centre, plan, confirm, auto-capture
    python3 -m calibration.capture --pivot-mm 250 --dry-run  # teach centre, plan and print only, no motion
    python3 -m calibration.capture --manual                  # hand-guide every pose, no automatic motion
    python3 -m calibration.capture --pivot-mm 250 --center data/calib_X/session.json   # reuse a taught centre

--pivot-mm: rough distance from the flange to the board along the tool Z axis (the
point the camera looks at). Live preview with board detection: http://192.168.94.94:8080/

Every capture is kept (images/ + samples.jsonl) with accept/reject reasons; nothing is
deleted. The board must not move during a session.
"""
import argparse
import json
import threading
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from calibration.charuco import BoardSpec, Charuco
from calibration.checks import CaptureThresholds, evaluate_capture
from calibration.poses import Limits, check_path, default_offsets, plan_poses
from common.arm import JOINT_LIMITS, ArmError, SafeArm
from common.camera import Camera
from common.geometry import pose_to_matrix
from common.preview import Preview

REACH_TOL_DEG = 0.1
FRAMES_PER_CAPTURE = 5
MAX_ATTEMPTS = 3
MIN_GOOD_POSES = 15
MIN_CENTER_J5_DEG = 35.0


class Session:
    def __init__(self, root, args, spec, camera):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = Path(root) / f"calib_{stamp}"
        (self.dir / "images").mkdir(parents=True)
        self.meta = {"created": stamp, "args": vars(args), "board": asdict(spec), "image_size": camera.size,
                     "samples_file": "samples.jsonl"}
        self.save_meta()
        self.accepted, self.rejected = 0, 0

    def save_meta(self):
        (self.dir / "session.json").write_text(json.dumps(self.meta, indent=2))

    def record(self, rec, image):
        cv2.imwrite(str(self.dir / "images" / f"{rec['name']}.png"), image)
        with open(self.dir / "samples.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")
        if rec["accepted"]:
            self.accepted += 1
        else:
            self.rejected += 1


class LivePreview:
    """Background thread: detect the board on the latest frame and publish an annotated view."""

    def __init__(self, camera, charuco, port):
        self.camera, self.charuco, self.status = camera, charuco, ""
        self.preview = Preview(port)
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self._running:
            frame, _ = self.camera.latest()
            corners, ids = self.charuco.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            n = 0 if ids is None else len(ids)
            if n:
                cv2.aruco.drawDetectedCornersCharuco(frame, corners, ids, (0, 255, 0))
            color = (0, 255, 0) if n >= CaptureThresholds.min_corners else (0, 0, 255)
            cv2.putText(frame, f"corners {n}/{self.charuco.n_corners}  {self.status}", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, color, 3)
            self.preview.show(frame)
            time.sleep(0.05)

    def close(self):
        self._running = False
        self.preview.close()


def capture_sample(name, arm, camera, charuco, session, ref_sharpness, commanded=None):
    """Settle, verify arm state, grab frames, keep the sharpest, run layer-1 checks, record."""
    joints, t_still = arm.wait_settled()
    arm_reasons = []
    if commanded is not None:
        err = max(abs(a - b) for a, b in zip(joints, commanded))
        if err > REACH_TOL_DEG:
            arm_reasons.append(f"joint error {err:.3f} deg vs commanded")
    flange = arm.fk(joints)                     # pose from the measured joints
    controller_pose = arm.flange_pose()
    if np.linalg.norm(np.subtract(flange[:3], controller_pose[:3])) > 0.2:
        arm_reasons.append("FK(measured joints) disagrees with controller pose by > 0.2 mm")

    best = None
    for attempt in range(1, MAX_ATTEMPTS + 1):
        frames = camera.frames_after(t_still + 0.1, FRAMES_PER_CAPTURE)
        for frame, stamp in frames:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = charuco.detect(gray)
            ok, reasons, metrics = evaluate_capture(gray, corners, ids, charuco.object_points, ref_sharpness,
                                                    CaptureThresholds())
            score = (ok, metrics.get("corners", 0), metrics.get("sharpness", 0.0))
            if best is None or score > best[0]:
                best = (score, frame, stamp, ok, reasons, metrics)
        if best[3]:
            break
        t_still = time.monotonic()

    after = arm.joints()
    if max(abs(a - b) for a, b in zip(after, joints)) > 0.01:
        arm_reasons.append("arm moved during capture")

    _, frame, stamp, ok, reasons, metrics = best
    rec = {"name": name, "accepted": ok and not arm_reasons, "reasons": arm_reasons + reasons,
           "metrics": metrics, "attempts": attempt, "joints": joints, "flange_pose": flange,
           "controller_pose": controller_pose, "commanded_joints": commanded,
           "t_still": t_still, "frame_stamp": stamp}
    session.record(rec, frame)
    tag = "OK " if rec["accepted"] else "REJ"
    print(f"  [{tag}] {name}: corners {metrics.get('corners', 0):2d}  sharp {metrics.get('sharpness', 0):7.1f}"
          f"  {'; '.join(rec['reasons'])}")
    return rec


def teach_center(arm, live, camera, charuco, session):
    while True:
        arm.free_drive()
        live.status = "FREE-DRIVE"
        input("\nFREE-DRIVE: hand-guide the arm so the board fills most of the view, in focus\n"
              "(watch http://192.168.94.94:8080/). Hold the arm still and press Enter... ")
        arm.enable_position_mode()
        live.status = "holding"
        j5 = arm.joints()[4]
        if abs(j5) < MIN_CENTER_J5_DEG:
            print(f"J5 is {j5:.1f} deg; keep |J5| >= {MIN_CENTER_J5_DEG} so tilted poses don't flip the wrist. "
                  "Bend the wrist more and try again.")
            continue
        rec = capture_sample("c00", arm, camera, charuco, session, None)
        if rec["accepted"]:
            return rec
        print("Centre capture rejected; adjust and try again.")


def print_plan(planned):
    print(f"\n{'#':>3} {'tilt':>5} {'az':>5} {'roll':>5} {'dist':>5} {'x':>7} {'y':>7} {'z':>7} {'J5':>6} {'deg/s':>6}  status")
    for p in planned:
        x, y, z = p.flange_pose[:3]
        j5 = f"{p.joints[4]:6.1f}" if p.joints else "     -"
        sp = f"{p.speed_deg_s:6.2f}" if p.speed_deg_s else "     -"
        o = p.offset
        print(f"{p.index:3d} {o.tilt_deg:5.0f} {o.azimuth_deg:5.0f} {o.roll_deg:5.0f} {o.dist_scale:5.2f} "
              f"{x:7.1f} {y:7.1f} {z:7.1f} {j5} {sp}  {'ok' if p.ok else '; '.join(p.reasons)}")


def confirm(prompt):
    return input(prompt).strip().lower() == "yes"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pivot-mm", type=float, help="flange-to-board distance along tool Z (auto mode)")
    ap.add_argument("--manual", action="store_true", help="hand-guide every pose; no automatic motion")
    ap.add_argument("--dry-run", action="store_true", help="plan and print the poses, do not move")
    ap.add_argument("--center", help="session.json of an earlier run: reuse its taught centre pose")
    ap.add_argument("--min-z", type=float, help="lowest allowed flange z (mm); default centre z - 0.35*pivot")
    ap.add_argument("--n-poses", type=int, default=24)
    ap.add_argument("--out", default="data")
    ap.add_argument("--preview-port", type=int, default=8080)
    args = ap.parse_args()
    if not args.manual and not args.pivot_mm:
        ap.error("--pivot-mm is required unless --manual")

    spec = BoardSpec.load()
    charuco = Charuco(spec)
    camera = arm = live = None
    moving = False
    try:
        camera = Camera()
        live = LivePreview(camera, charuco, args.preview_port)
        arm = SafeArm()
        arm.assert_healthy()
        session = Session(args.out, args, spec, camera)
        print(f"Session: {session.dir}\nBoard: {spec}\nPreview: http://192.168.94.94:{args.preview_port}/")

        if args.manual:
            ref = teach_center(arm, live, camera, charuco, session)["metrics"]["sharpness"]
            i = 1
            while True:
                arm.free_drive()
                live.status = "FREE-DRIVE"
                if input(f"\nPose {i}: hand-guide, hold still, Enter to capture (q + Enter to finish): ").strip() == "q":
                    break
                arm.enable_position_mode()
                live.status = "holding"
                capture_sample(f"m{i:02d}", arm, camera, charuco, session, ref)
                i += 1
            arm.enable_position_mode()
        else:
            ref_rec, approach = None, None
            if args.center:
                center_joints = json.loads(Path(args.center).read_text())["center_joints"]
                center_pose = arm.fk(center_joints)
            else:
                ref_rec = teach_center(arm, live, camera, charuco, session)
                center_joints, center_pose = ref_rec["joints"], ref_rec["flange_pose"]
            T_center = pose_to_matrix(center_pose)
            session.meta.update(center_joints=center_joints, center_pose=center_pose)
            session.save_meta()

            min_z = args.min_z if args.min_z is not None else T_center[2, 3] - 0.35 * args.pivot_mm
            lim = Limits(JOINT_LIMITS, min_flange_z_mm=min_z)
            if args.center:
                cur = arm.joints()
                reasons, speed = check_path(arm.fk, cur, center_joints, lim)
                reasons += arm.check_joint_limits(center_joints)
                if reasons:
                    raise ArmError("cannot move to saved centre: " + "; ".join(reasons))
                approach = speed
                print(f"First move: current pose -> saved centre {np.round(center_joints, 1).tolist()} at {speed} "
                      f"deg/s (max joint change {max(abs(a - b) for a, b in zip(cur, center_joints)):.1f} deg).")
            planned = plan_poses(T_center, center_joints, args.pivot_mm, default_offsets(args.n_poses),
                                 arm.ik, arm.fk, lim)
            print_plan(planned)
            good = [p for p in planned if p.ok]
            print(f"\n{len(good)}/{len(planned)} poses pass all checks. Min flange z {min_z:.0f} mm, "
                  f"speeds <= {lim.max_joint_speed_deg_s} deg/s and ~{lim.max_tcp_speed_mm_s:.0f} mm/s. "
                  f"Each pose: centre -> pose -> centre.")
            session.meta["plan"] = [{"index": p.index, "offset": asdict(p.offset), "flange_pose": p.flange_pose,
                                     "joints": p.joints, "speed": p.speed_deg_s, "ok": p.ok, "reasons": p.reasons}
                                    for p in planned]
            session.save_meta()
            if len(good) < MIN_GOOD_POSES:
                print(f"WARNING: fewer than {MIN_GOOD_POSES} usable poses. Try a different centre pose or --pivot-mm.")
            if args.dry_run:
                print("Dry run: no motion.")
                return
            if not confirm("\nStand at the arm with the E-stop in reach. Type yes to start motion: "):
                print("Not confirmed; no motion.")
                return

            if approach:
                arm.enable_position_mode()
                moving = True
                arm.move_joints(center_joints, approach)
                moving = False
                ref_rec = capture_sample("c00", arm, camera, charuco, session, None, center_joints)
                if not ref_rec["accepted"]:
                    raise RuntimeError("centre capture rejected; has the board or camera moved?")
            ref = ref_rec["metrics"]["sharpness"]
            for p in good:
                print(f"Pose {p.index}: moving at {p.speed_deg_s} deg/s")
                moving = True
                arm.move_joints(p.joints, p.speed_deg_s)
                moving = False
                capture_sample(f"p{p.index:02d}", arm, camera, charuco, session, ref, p.joints)
                moving = True
                arm.move_joints(center_joints, p.speed_deg_s)
                moving = False

        print(f"\nDone: {session.accepted} accepted, {session.rejected} rejected. Saved in {session.dir}")
        print(f"Next: python3 -m calibration.solve {session.dir}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        if arm and moving:
            arm.stop()
            print("Arm stopped (state 4). Re-enable from UFACTORY Studio or rerun when safe.")
    except Exception:
        if arm and moving:
            arm.stop()
        raise
    finally:
        if arm:
            if arm.in_free_drive and not moving:
                try:
                    arm.enable_position_mode()   # never leave the arm in free-drive
                except Exception as e:
                    print(f"warning: could not return arm to position mode: {e}")
            arm.close()
        if live:
            live.close()
        if camera:
            camera.close()


if __name__ == "__main__":
    main()
