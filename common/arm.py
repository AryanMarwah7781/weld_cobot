"""Safety-first wrapper around xarm-python-sdk 1.18.4 for the UFACTORY Lite 6.

Rules enforced here (see claude/claude.md "Safety rules"):
- every SDK return code is checked; any non-zero code or controller error raises ArmError
- joint speed is hard-capped at MAX_JOINT_SPEED_DEG_S
- collision detection must be enabled before any motion
- motion targets must be inside JOINT_LIMITS with margin
"""
import time

from xarm.wrapper import XArmAPI

ARM_IP = "192.168.1.158"
JOINT_LIMITS = [(-360, 360), (-150, 150), (-3.5, 300), (-360, 360), (-124, 124), (-360, 360)]
MAX_JOINT_SPEED_DEG_S = 10.0
MAX_JOINT_ACC_DEG_S2 = 50.0
REPORT_WAIT_S = 1.0


class ArmError(RuntimeError):
    pass


def _code(ret):
    return ret[0] if isinstance(ret, (tuple, list)) else ret


class SafeArm:
    def __init__(self, ip=ARM_IP):
        self.arm = XArmAPI(ip, is_radian=False)
        if not self.arm.connected:
            raise ArmError(f"could not connect to arm at {ip}")
        # Properties such as state and collision_sensitivity hold SDK placeholder values until
        # the first report packet arrives (~0.2 s after connect, measured). Don't trust them before.
        time.sleep(REPORT_WAIT_S)
        self.in_free_drive = False

    # ---- reads -------------------------------------------------------------
    def _check(self, what, ret):
        code = _code(ret)
        if code != 0:
            raise ArmError(f"{what} returned code {code} (arm error {self.arm.error_code}, warn {self.arm.warn_code})")
        return ret

    def assert_healthy(self):
        if self.arm.error_code != 0:
            raise ArmError(f"arm error code {self.arm.error_code}; clear it on the arm/UFACTORY Studio first")
        if self.arm.warn_code != 0:
            raise ArmError(f"arm warning code {self.arm.warn_code}")

    def joints(self, measured=True):
        """Six joint angles in degrees. measured=True asks the controller for encoder states."""
        _, a = self._check("get_servo_angle", self.arm.get_servo_angle(is_real=measured))
        return [float(v) for v in a[:6]]

    def flange_pose(self):
        _, p = self._check("get_position", self.arm.get_position())
        return [float(v) for v in p[:6]]

    def ik(self, pose, ref_joints):
        """Controller IK. Returns six joint angles or None if unreachable."""
        code, a = self.arm.get_inverse_kinematics(list(pose), ref_angles=list(ref_joints))
        return [float(v) for v in a[:6]] if code == 0 and a else None

    def fk(self, joints):
        _, p = self._check("get_forward_kinematics", self.arm.get_forward_kinematics(list(joints)))
        return [float(v) for v in p[:6]]

    # ---- modes -------------------------------------------------------------
    def enable_position_mode(self):
        """Motors on, position mode, ready state. Does not move the arm."""
        self.assert_healthy()
        if not self.arm.collision_sensitivity:
            raise ArmError("collision detection is disabled (sensitivity 0); enable it before any motion")
        self._check("motion_enable", self.arm.motion_enable(enable=True))
        self._check("set_mode(0)", self.arm.set_mode(0))
        self._check("set_state(0)", self.arm.set_state(0))
        self.in_free_drive = False
        time.sleep(0.5)
        self.assert_healthy()

    def free_drive(self):
        """Joint teaching mode: the arm can be pushed by hand."""
        self.assert_healthy()
        self._check("set_mode(2)", self.arm.set_mode(2))
        self._check("set_state(0)", self.arm.set_state(0))
        self.in_free_drive = True

    # ---- motion ------------------------------------------------------------
    def check_joint_limits(self, joints, margin=5.0):
        problems = []
        for i, (q, (lo, hi)) in enumerate(zip(joints, JOINT_LIMITS)):
            if not lo + margin <= q <= hi - margin:
                problems.append(f"J{i + 1}={q:.1f} outside [{lo + margin}, {hi - margin}]")
        return problems

    def move_joints(self, joints, speed_deg_s, acc_deg_s2=MAX_JOINT_ACC_DEG_S2):
        """Blocking joint-space move. Raises on any problem; never exceeds the speed caps."""
        if speed_deg_s <= 0 or speed_deg_s > MAX_JOINT_SPEED_DEG_S:
            raise ArmError(f"joint speed {speed_deg_s} outside (0, {MAX_JOINT_SPEED_DEG_S}]")
        acc = min(acc_deg_s2, MAX_JOINT_ACC_DEG_S2)
        problems = self.check_joint_limits(joints)
        if problems:
            raise ArmError("refusing move: " + "; ".join(problems))
        self.assert_healthy()
        self._check("set_servo_angle",
                    self.arm.set_servo_angle(angle=list(joints), speed=speed_deg_s, mvacc=acc, wait=True))
        self.assert_healthy()

    def wait_settled(self, settle_s=0.3, tol_deg=0.01, gap_s=0.1, timeout=5.0):
        """Wait settle_s, then until two measured readings gap_s apart agree within tol_deg.

        Returns (joints, monotonic time at which the arm was judged still)."""
        time.sleep(settle_s)
        deadline = time.monotonic() + timeout
        prev = self.joints()
        while True:
            time.sleep(gap_s)
            cur = self.joints()
            if max(abs(a - b) for a, b in zip(prev, cur)) < tol_deg:
                return cur, time.monotonic()
            if time.monotonic() > deadline:
                raise ArmError(f"arm did not settle within {timeout}s")
            prev = cur

    def stop(self):
        """Stop motion immediately (state 4). Safe to call at any time."""
        try:
            self.arm.set_state(4)
        except Exception:
            pass

    def close(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
