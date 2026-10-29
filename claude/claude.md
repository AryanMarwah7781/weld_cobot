# CLAUDE.md — T-Joint Weld Visual Inspection (UFACTORY Lite 6 + Arducam + Jetson)

## Project goal
Visually inspect a T-joint fillet weld with a camera on a robot arm. The system:
1. Plans camera viewpoints along the weld seam.
2. Scans continuously while recording video, with arm joint/TCP state recorded in sync.
3. Detects ROIs (possible defects) in the video.
4. Returns the arm to the exact pose where an ROI was seen and captures a clean, high-quality still.

## Hardware
- **Arm:** UFACTORY Lite 6. 6 DoF, 440 mm reach, 600 g payload, ±0.5 mm repeatability, 500 mm/s max TCP speed.
  - Joint limits: J1 ±360°, J2 ±150°, J3 -3.5°~300°, J4 ±360°, J5 ±124°, J6 ±360°.
  - Built-in control box with its own 24 V supply. The Jetson is NOT the arm's power source.
  - Communicates with the Jetson over Ethernet.
- **Camera:** Arducam mounted on the Lite 6 gripper, powered by the Jetson. Model, lens, and interface (USB/CSI) are TBD; see Open Questions.
- **Depth camera (added 2026-10-20):** Intel RealSense D405 (short range, about 7–50 cm, passive stereo with no IR projector). It is to be mounted together with the Arducam on one bracket; the bracket design is in progress. Today the Arducam just sits in the gripper. Python: `pyrealsense2` 2.59.1 has a cp310 aarch64 wheel on PyPI, so `pip3 install --user pyrealsense2` should work on the Jetson. Non-root access may also need Intel's udev rules.
  - `common/camera.find_v4l2_device("Arducam")` finds the Arducam by name, because /dev/videoN numbers shift once the D405 is plugged in.
  - D405 (SN 260322272991, FW 5.15.1.55) works with the pip wheel, with no udev rules needed. Colour and depth go up to 1280×720@30 (848×480 up to 90 fps). Colour comes from the stereo module itself. Depth unit is 0.1 mm.
  - **USB contention (measured 2026-10-20):** all four Type-A ports share one internal USB3 hub. D405 at 1280×720@30 delivers 30 fps alone, but only ~13 fps while the Arducam streams 1920×1200@30, and ~16 fps with the Arducam at 1920×1200@15. With the Arducam at 960×600 (even @60), the D405 holds 30 fps. The Arducam's resolution, not its data rate, starves the D405, which suggests per-frame-size USB bus reservation. For simultaneous streaming, run the Arducam at 960×600 or try one camera on the USB-C port (untested). `python3 -m tools.rs_diag` re-measures this.
  - `python3 -m tools.viewer` serves a live side-by-side view on http://192.168.94.94:8080/ for the iPad: Arducam, D405 colour, and D405 depth (tap to measure). A Capture button saves full-res stills to `data/compare/<ts>/`. `--fake` runs it without hardware.
- **Compute:** NVIDIA Jetson runs planning, recording, detection, and arm control.
- **Payload budget:** gripper + camera + bracket must stay under 600 g.

## Software stack
- Python 3, `xarm-python-sdk` (`from xarm.wrapper import XArmAPI`), OpenCV, NumPy, SciPy.
- Optional: ROS 2 + `xarm_ros2` + MoveIt 2 for collision-checked Cartesian paths.
- Verify every SDK call signature against the installed `xarm-python-sdk` version before using it. Do not assume API details from memory.

## Conventions
- **Units:** millimetres and degrees at the arm interface, matching the xArm SDK default with `is_radian=False`.
- **Orientation:** xArm roll/pitch/yaw is fixed-axis XYZ, R = Rz(yaw)·Ry(pitch)·Rx(roll). This equals SciPy `Rotation.from_euler('xyz', ...)` with lowercase (extrinsic) axes.
- **Time:** all timestamps use the Jetson's `time.monotonic()`. Never mix in the arm controller's clock or the video container's timestamps.
- **Frames:** robot base frame is the world frame. TCP is set from hand-eye calibration (see Calibration).

## Architecture (planned modules)
```
calibration/   hand-eye calibration (ChArUco + cv2.calibrateHandEye), TCP offset, workpiece registration
planning/      weld frame construction, viewpoint generation, IK/reachability + collision checks
recording/     robot state logger, video logger, latency calibration, frame↔pose alignment
detection/     ROI detection on scan video (method TBD)
revisit/       return-to-ROI motion, settle, close-up, best-still selection
data/          per-scan output folders (gitignored)
```

## 1. Calibration
- **Board:** `calibration/board/charuco_7x5_30mm_A4.pdf`, with 7×5 squares, 30.0 mm squares, 22.0 mm markers, DICT_5X5_100. Use the measured printed square size if it differs. The Jetson runs OpenCV 4.5.4, which uses the old aruco API (`cv2.aruco.CharucoBoard_create`, `interpolateCornersCharuco`). Do not use the 4.7+ `CharucoBoard`/`CharucoDetector` classes.
- **Hand-eye:** capture 15–20 ChArUco board images from varied arm poses and solve with `cv2.calibrateHandEye`. This gives the camera pose relative to the flange.
- **Tools (built):** run these on the Jetson from `~/weld_cobot`. Push code from the PC with `JETSON_PASSWORD=... python tools/jetson.py push`.
  - `python3 -m calibration.capture --pivot-mm <flange→board mm> [--dry-run | --manual | --center <session.json>]`: teach the centre pose in free-drive (|J5| ≥ 35°), plan 24 poses around it, type `yes`, then it captures. Each pose goes centre → pose → centre at ≤ 10°/s and ~16 mm/s. Every sample is saved with accept/reject reasons. Live preview: http://192.168.94.94:8080/.
  - `python3 -m calibration.solve data/calib_<ts> [--mount x,y,z]`: dedupe, intrinsics with outlier rejection, hand-eye (park/tsai/horaud/daniilidis + joint reprojection refinement), board-consistency outlier loop, PASS/WARN/FAIL report. Typing `yes` saves `calibration/results/current.json`.
  - `python3 -m tools.plan_check <pivot> ['[joints]']`: read-only pose planning on the real controller IK.
  - Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests` (on the Jetson; the system pytest clashes with a user-installed anyio plugin).
  - Lite 6 IK often returns a flipped wrist (J5 sign change). Mapping it back via (J4+180, −J5, J6+180) is ~2.3 mm wrong on this arm, so those poses are rejected.
- **TCP offset:** place the TCP on the camera's optical axis at the working (focus) distance, not at the lens. Commanding the TCP to a seam point then means "look at this point, in focus."
- **Workpiece registration:** hand-guide the arm to touch off the seam start, the seam end, and one point on the flange plate. Fiducials or vision-based seam detection can replace this later.

## 2. Trajectory / viewpoint planning
- **Weld frame**, defined at each seam point:
  - `x` = seam direction
  - `n_flange` = base plate normal
  - `n_web` = standing plate normal
  - bisector = normalize(n_flange + n_web)
- **Camera optical axis** = −bisector, looking into the corner at 45° to each plate.
- **Step along seam** = FOV_length_along_seam × (1 − overlap), with overlap 30–50%.
- **Passes:**
  1. Bisector (0°): weld face and overall shape.
  2. Tilted 15–20° toward the flange: flange-side toe.
  3. Tilted 15–20° toward the web: web-side toe.
- Add extra views at the seam start and end, where craters and start defects occur.
- Use roll about the optical axis as a free parameter to avoid joint limits and the wrist singularity (J5 ≈ 0).
- **Reachability is the binding constraint** at 440 mm reach. Place the workpiece about 200–300 mm from the base. Check IK at both seam ends before anything else.
- **Collision:** check the gripper body and fingers against both plates. They usually collide first in the corner.

## 3. Synchronized recording
- **Robot logger** (background thread):
  - Polls joint angles and TCP pose together at a target of ~100 Hz.
  - Stamps each sample with `time.monotonic()`.
  - Logs the actual achieved rate. It must be ≥ the video FPS.
- **Video logger** (background thread):
  - Writes the video plus a per-frame timestamp side file.
  - Keeps camera buffering minimal to avoid stale frames.
  - Detects and reports dropped frames (timestamp gaps).
- **Start/stop order:** start both loggers before the arm moves and stop them after it stops.
- **Latency correction:** subtract a measured camera latency from each frame timestamp.
  - To measure: hold the arm still, make a sharp move, and compare motion onset in the robot log against image change onset in the video.
  - Store the value used with every scan.
- **Alignment:** for each frame, linearly interpolate robot state at the corrected timestamp. Output a frame→(j1..j6, x, y, z, roll, pitch, yaw) table. Drop frames outside the robot log span.

## 4. Scan vs. revisit strategy
- **Scan:** continuous blended motion at ~30–50 mm/s. Quality only needs to be sufficient for ROI detection. Motion blur ≈ speed × exposure.
- **Revisit:** stationary, full-quality capture only at flagged ROIs.

## 5. Revisit procedure
1. Look up the flagged frame's joint angles in the alignment table.
2. Move to a clearance pose first, then to the recorded configuration, so the approach follows the path the scan already proved safe.
3. **Return in joint space**, using all six recorded joint angles. Do not send a Cartesian command, because IK may pick a different elbow/wrist branch and collide in the corner.
4. Wait ~200–400 ms to settle (the arm is lightweight and vibrates).
5. Optional: move closer along the tool Z axis (the optical axis, given the calibrated TCP) for magnification.
6. Optional: switch the camera to its maximum still resolution or a longer exposure.
7. Grab several frames and keep the sharpest (variance of the Laplacian).
8. Save the still with its frame index, joints, and pose.

## Per-scan data layout
Each scan gets its own folder, `data/scan_<timestamp>/`, containing:
- the video
- per-frame timestamps
- the raw robot log
- the alignment table
- the latency value used
- the planned poses
- ROI stills plus their metadata

Never overwrite raw logs. Alignment must be re-runnable with a new latency value.

## Safety rules (for Claude Code)
- **Never run code that moves the arm without explicit confirmation from me.** Write it, explain what it will do, and wait.
- Default speeds for new or untested motion code: ≤ 20 mm/s (or a slow joint speed), and ≤ 50 mm/s for scans.
- Every motion script must check reachability/IK for all poses before moving, and abort on any error code returned by the SDK.
- Prefer testing planning logic offline (math, plots, simulation) before any hardware run.
- Always include a clean shutdown path: stop motion, release the camera, disconnect the arm.
- Keep collision detection enabled on the arm.

## Validation checklist
- Hand-eye calibration reprojection error is acceptable, and a known point looks centered when the arm targets it.
- All planned poses are reachable, with margin from joint limits and no J5 ≈ 0.
- The robot logging rate is ≥ the video FPS, and dropped frames are reported.
- **Revisit accuracy test:** flag a known feature, revisit it, and confirm it is centered. A consistent offset along the scan direction means the latency estimate is wrong.

## Open questions (fill in)
- Arm IP address: 192.168.1.158. Firmware v2.8.0, SN LI1006. Wired directly to the Jetson's Ethernet port `enP8p1s0`. The Jetson uses NetworkManager profile `xarm-link`: static 192.168.1.10/24, no default route; internet stays on Wi-Fi. Jetson has xarm-python-sdk 1.18.4 (pip --user). get_servo_angle+get_position loop ran at ~1.2 kHz; verify whether those values are fresh or SDK-cached before relying on it for the robot logger. `get_servo_angle(is_real=True)` queries measured joint states from the controller each call.
  - Settings as read on 2026-10-02: TCP offset 0, payload 0.145 kg @ z=30 mm (needs updating to the real tool), collision sensitivity 3, self-collision check on.
  - **SDK gotcha:** right after `XArmAPI()` connects, report-driven properties (`state`, `mode`, `collision_sensitivity`, `teach_sensitivity`, ...) hold placeholder values (e.g. sensitivity 0, state 4) until the first report packet arrives about 0.2 s later. `SafeArm` waits 1 s after connecting.
- Arducam model, lens, and interface (USB UVC / CSI), plus supported resolutions and FPS: Arducam B0495 (AR0234 global shutter), USB3 UVC — must be on a USB3 port. Find it by name (`find_v4l2_device`); don't assume /dev/video0. YUYV only: 1920x1200 @ 50/30/15 fps, 960x600 @ 80/60/30/15 fps. Manual-focus lens (no focus control); exposure/gain/WB settable via V4L2.
- Plate thicknesses, seam length, and one-sided or two-sided fillet:
- Working distance / standoff and FOV at that distance:
- Lighting setup:
- ROI detection approach (classical CV vs. trained model):
- ROS 2/MoveIt or pure Python SDK:
- Jetson model and JetPack version: Jetson Orin Nano, IP 192.168.94.94. JetPack 6.2.x (L4T R36.4.7), Python 3.10, OpenCV 4.5.4 (aruco available), xarm-python-sdk 1.18.4.