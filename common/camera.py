"""Arducam B0495 (USB3 UVC) capture with a background reader.

Every frame is stamped with time.monotonic() when read() returns, so callers can
ask for frames that started after some event (e.g. the arm settling) instead of
trusting the driver's buffer to be empty.
"""
import threading
import time
from collections import deque
from pathlib import Path

import cv2


def find_v4l2_device(name_part="Arducam"):
    """Index N of the /dev/videoN capture node whose name contains name_part.

    Each UVC camera exposes two nodes; the capture node has index 0. Device numbers
    shift when other cameras (e.g. the RealSense) are plugged in, so never hard-code them."""
    for dev in sorted(Path("/sys/class/video4linux").glob("video*"), key=lambda p: int(p.name[5:])):
        name = (dev / "name").read_text().strip()
        index = (dev / "index").read_text().strip() if (dev / "index").exists() else "0"
        if name_part.lower() in name.lower() and index == "0":
            return int(dev.name[5:])
    raise RuntimeError(f"no V4L2 camera named like '{name_part}' (is it plugged into a USB3 port?)")


class RateMeter:
    """Frames per second over a sliding window of arrival times (robust to bursty delivery)."""

    def __init__(self, window_s=2.0):
        self.window_s, self._stamps = window_s, deque()

    def tick(self, stamp):
        self._stamps.append(stamp)
        while self._stamps and stamp - self._stamps[0] > self.window_s:
            self._stamps.popleft()

    @property
    def fps(self):
        if len(self._stamps) < 2:
            return 0.0
        return (len(self._stamps) - 1) / (self._stamps[-1] - self._stamps[0])


class Camera:
    def __init__(self, device=None, width=1920, height=1200, fps=30):
        if device is None:
            device = find_v4l2_device("Arducam")
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera /dev/video{device} (is another process using it?)")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = self.cap.read()
        if not ok:
            self.cap.release()
            raise RuntimeError("camera opened but returned no frame")
        self.size = (frame.shape[1], frame.shape[0])
        if self.size != (width, height):
            self.cap.release()
            raise RuntimeError(f"camera gave {self.size}, wanted {(width, height)}; is it on a USB3 port?")
        self._cond = threading.Condition()
        self._frame, self._stamp, self._seq = frame, time.monotonic(), 0
        self.rate = RateMeter()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            ok, frame = self.cap.read()
            stamp = time.monotonic()
            if ok:
                self.rate.tick(stamp)
                with self._cond:
                    self._frame, self._stamp, self._seq = frame, stamp, self._seq + 1
                    self._cond.notify_all()

    @property
    def fps(self):
        return self.rate.fps

    def latest(self):
        with self._cond:
            return self._frame.copy(), self._stamp

    def frames_after(self, t0, n, timeout=5.0):
        """Return n consecutive frames whose read completed after t0 (monotonic)."""
        out, deadline, last_seq = [], time.monotonic() + timeout, -1
        with self._cond:
            while len(out) < n:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"got {len(out)}/{n} fresh frames in {timeout}s")
                self._cond.wait(remaining)
                if self._seq != last_seq and self._stamp > t0:
                    out.append((self._frame.copy(), self._stamp))
                    last_seq = self._seq
        return out

    def close(self):
        self._running = False
        self._thread.join(timeout=2)
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
