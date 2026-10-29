"""D405 frame-delivery diagnostic: unique colour/depth frames per second, with and without the Arducam."""
import time

import pyrealsense2 as rs

from common.camera import Camera


def measure(seconds=5.0, w=1280, h=720, fps=30):
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    pipe.start(cfg)
    try:
        for _ in range(30):
            pipe.wait_for_frames(5000)
        colour, depth, sets, gaps = set(), set(), 0, 0
        last, t0 = None, time.monotonic()
        while time.monotonic() - t0 < seconds:
            fs = pipe.wait_for_frames(5000)
            sets += 1
            c, d = fs.get_color_frame(), fs.get_depth_frame()
            if c:
                n = c.get_frame_number()
                if last is not None and n > last + 1:
                    gaps += n - last - 1
                if n != last:
                    colour.add(n)
                last = n
            if d:
                depth.add(d.get_frame_number())
        el = time.monotonic() - t0
        return f"framesets {sets / el:5.1f}/s  unique colour {len(colour) / el:5.1f}/s  unique depth {len(depth) / el:5.1f}/s  colour frames skipped {gaps}"
    finally:
        pipe.stop()


mode = (1280, 720, 30)
for aw, ah, afps in ((960, 600, 30), (960, 600, 60)):
    with Camera(width=aw, height=ah, fps=afps) as cam:
        time.sleep(1)
        print(f"D405 {mode} + Arducam {aw}x{ah}@{afps}:", measure(5.0, *mode), f" (arducam {cam.fps:.1f} fps)")
