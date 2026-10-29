"""Enable the arm (no arm motion) and drive the Lite 6 gripper.

    python3 -m tools.gripper close      # close and KEEP the close signal on (holds continuously)
    python3 -m tools.gripper open       # open, then de-energise after GRIP_TIME_S
    python3 -m tools.gripper stop       # de-energise the gripper (fingers no longer driven)
    python3 -m tools.gripper enable     # only enable the arm: motors on, position mode, ready
    python3 -m tools.gripper freedrive  # hand-guiding mode; stays in it after exit (gripper unchanged)

The arm itself is never commanded to move.
"""
import sys
import time

from common.arm import SafeArm

GRIP_TIME_S = 1.5


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else ""
    if action not in ("open", "close", "stop", "enable", "freedrive"):
        sys.exit(__doc__)
    with SafeArm() as arm:
        print("before:", "state", arm.arm.state, "mode", arm.arm.mode, "error", arm.arm.error_code)
        arm.enable_position_mode()
        print("enabled:", "state", arm.arm.state, "mode", arm.arm.mode, "joints", [round(a, 2) for a in arm.joints()])
        if action == "enable":
            return
        if action == "freedrive":
            arm.free_drive()
            time.sleep(0.5)
            arm.assert_healthy()
            print(f"FREE-DRIVE on: mode {arm.arm.mode}, state {arm.arm.state}. Support the arm while moving it.\n"
                  "To hold position again: python3 -m tools.gripper enable")
            return
        if action == "stop":
            arm._check("stop_lite6_gripper", arm.arm.stop_lite6_gripper())
        elif action == "close":
            arm._check("close_lite6_gripper", arm.arm.close_lite6_gripper())
            time.sleep(GRIP_TIME_S)
        else:
            arm._check("open_lite6_gripper", arm.arm.open_lite6_gripper())
            time.sleep(GRIP_TIME_S)
            arm._check("stop_lite6_gripper", arm.arm.stop_lite6_gripper())
        arm.assert_healthy()
        _, outs = arm.arm.get_tgpio_output_digital()
        print(f"gripper {action} done; tool outputs {outs}; error {arm.arm.error_code}, warn {arm.arm.warn_code}")


if __name__ == "__main__":
    main()
