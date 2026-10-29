"""Intel RealSense D405 capture (colour + depth) with a background reader.

Frames are stamped with time.monotonic() on arrival. Depth is kept raw (uint16 units);
multiply by depth_scale for metres. Intrinsics come from the camera's factory calibration.
"""
import threading
import time

import numpy as np

from common.camera import RateMeter

# Tried in order; the D405 does not support every colour mode at every rate.
MODES = [(1280, 720, 30), (1280, 720, 15), (848, 480, 30)]


class RealSense:
    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe, profile, errors = rs.pipeline(), None, []
        for w, h, fps in MODES:
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            try:
                profile = self.pipe.start(cfg)
                self.mode = (w, h, fps)
                break
            except RuntimeError as e:
                errors.append(f"{w}x{h}@{fps}: {e}")
        if profile is None:
            raise RuntimeError("D405 would not start: " + " | ".join(errors))
        dev = profile.get_device()
        self.info = {k: dev.get_info(getattr(rs.camera_info, k)) for k in
                     ("name", "serial_number", "firmware_version", "usb_type_descriptor")
                     if dev.supports(getattr(rs.camera_info, k))}
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.intrinsics = {s: self._intr(profile, getattr(rs.stream, s)) for s in ("color", "depth")}
        self._lock = threading.Lock()
        self._color = self._depth = None
        self._stamp, self._seq = 0.0, 0
        self.rate = RateMeter()
        self._last_number = None
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 5
        while self._color is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._color is None:
            self.close()
            raise RuntimeError("D405 started but sent no frames")

    @staticmethod
    def _intr(profile, stream):
        i = profile.get_stream(stream).as_video_stream_profile().get_intrinsics()
        return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
                "model": str(i.model), "coeffs": list(i.coeffs)}

    def _run(self):
        while self._running:
            try:
                frames = self.pipe.wait_for_frames(2000)
            except RuntimeError:
                continue
            c, d = frames.get_color_frame(), frames.get_depth_frame()
            if not c or not d:
                continue
            # The syncer can hand over the same colour frame in more than one frameset; only
            # treat a frameset as new when the camera's colour frame counter has advanced.
            number = c.get_frame_number()
            if number == self._last_number:
                continue
            self._last_number = number
            color = np.asanyarray(c.get_data()).copy()
            depth = np.asanyarray(d.get_data()).copy()
            now = time.monotonic()
            self.rate.tick(now)
            with self._lock:
                self._color, self._depth = color, depth
                self._stamp, self._seq = now, self._seq + 1

    @property
    def fps(self):
        return self.rate.fps

    def latest(self):
        """(colour BGR, depth uint16, monotonic stamp, sequence number)."""
        with self._lock:
            return self._color, self._depth, self._stamp, self._seq

    def depth_mm(self, depth, x, y, half=4):
        """Median depth in mm of a small patch around pixel (x, y), ignoring holes. None if no data."""
        h, w = depth.shape
        patch = depth[max(0, y - half):min(h, y + half + 1), max(0, x - half):min(w, x + half + 1)]
        valid = patch[patch > 0]
        return float(np.median(valid)) * self.depth_scale * 1000.0 if valid.size else None

    def close(self):
        self._running = False
        self._thread.join(timeout=3)
        try:
            self.pipe.stop()
        except RuntimeError:
            pass
