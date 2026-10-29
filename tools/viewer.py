"""Side-by-side live view of the Arducam and the RealSense D405, for an iPad or any browser.

    python3 -m tools.viewer                  # on the Jetson, from ~/weld_cobot
    then open http://192.168.94.94:8080/

    python3 -m tools.viewer --fake           # synthetic images, no hardware (for testing)

Tiles: Arducam colour, D405 colour, D405 depth (tap it to read the distance at that spot).
"Capture" saves full-resolution stills from both cameras to data/compare/<timestamp>/.

USB note (measured 2026-10-20): both cameras share the Jetson's internal USB3 hub. With the
Arducam streaming at 1920x1200 the D405 drops to ~13 fps (at 15 fps Arducam: ~16 fps), but with
the Arducam at 960x600 (even at 60 fps) the D405 holds 30 fps. So the Arducam previews at 960x600
and switches to 1920x1200 only for the Capture still (--arducam-live full keeps it at 1920x1200).
The arm is not used. Uses the cameras exclusively: don't run calibration.capture at the same time.
"""
import argparse
import json
import re
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

OUT = Path("data/compare")
STREAM_WIDTH = 960
RENDER_HZ = 10
SAFE_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")


def focus_score(bgr):
    """Variance of the Laplacian over the middle half of the image (higher = sharper)."""
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr[h // 4:3 * h // 4, w // 4:3 * w // 4], cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def colorize_depth(depth, scale, near_mm, far_mm):
    mm = depth.astype(np.float32) * (scale * 1000.0)
    norm = np.clip((mm - near_mm) / (far_mm - near_mm), 0, 1)
    vis = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_TURBO)
    vis[depth == 0] = 0  # holes (no depth) in black
    return vis


# ---- fake hardware for --fake ---------------------------------------------------------
class FakeArducam:
    size = (1920, 1200)

    def __init__(self):
        self.fps = 30.0

    def latest(self):
        t = time.monotonic()
        img = np.full((1200, 1920, 3), 60, np.uint8)
        for i in range(0, 1920, 120):
            cv2.line(img, (i + int(40 * np.sin(t)), 0), (i, 1199), (200, 200, 200), 3)
        cv2.putText(img, "FAKE ARDUCAM", (600, 600), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 6)
        return img, t

    def close(self):
        pass


class FakeRealSense:
    depth_scale = 0.0001
    mode = (1280, 720, 30)
    info = {"name": "Fake D405", "serial_number": "000", "firmware_version": "-", "usb_type_descriptor": "3.2"}
    intrinsics = {}

    def __init__(self):
        self.fps, self._seq = 30.0, 0
        yy, xx = np.mgrid[0:720, 0:1280]
        self._depth = (1500 + 1.5 * xx + 0.5 * yy).astype(np.uint16)   # a tilted plane, 150-360 mm
        self._depth[300:420, 600:700] = 0                              # a hole

    def latest(self):
        self._seq += 1
        color = np.full((720, 1280, 3), 90, np.uint8)
        cv2.putText(color, "FAKE D405", (380, 380), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 200, 0), 6)
        return color, self._depth, time.monotonic(), self._seq

    def depth_mm(self, depth, x, y, half=4):
        patch = depth[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
        valid = patch[patch > 0]
        return float(np.median(valid)) * self.depth_scale * 1000.0 if valid.size else None

    def close(self):
        pass


# ---- state shared between the render loop and HTTP handlers ----------------------------
class Hub:
    TILES = ("arducam", "d405_color", "d405_depth")

    def __init__(self, arducam, realsense, errors, near_mm, far_mm, open_arducam=None):
        """open_arducam(full: bool) -> camera; given when stills need a resolution switch."""
        self.arducam, self.rs, self.errors = arducam, realsense, errors
        self.near_mm, self.far_mm = near_mm, far_mm
        self.open_arducam = open_arducam
        self.cam_lock = threading.Lock()   # guards swapping self.arducam during a still capture
        self.capture_lock = threading.Lock()   # one capture at a time (each one switches Arducam modes)
        self.cond = threading.Condition()
        self.jpeg = {t: None for t in self.TILES}
        self.stats = {t: {} for t in self.TILES}
        self.running = True

    def available(self, tile):
        return (self.arducam if tile == "arducam" else self.rs) is not None or (
            tile == "arducam" and self.open_arducam is not None and "arducam" not in self.errors)

    def _encode(self, tile, img):
        h, w = img.shape[:2]
        small = cv2.resize(img, (STREAM_WIDTH, int(h * STREAM_WIDTH / w)), interpolation=cv2.INTER_AREA)
        sh, sw = small.shape[:2]
        cv2.drawMarker(small, (sw // 2, sh // 2), (0, 255, 0), cv2.MARKER_CROSS, 30, 1)
        ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.jpeg[tile] = buf.tobytes()

    def render_loop(self):
        while self.running:
            t0 = time.monotonic()
            with self.cam_lock:
                cam = self.arducam
                frame = cam.latest()[0] if cam else None
                fps = cam.fps if cam else 0.0
            if frame is not None:
                res = f"{frame.shape[1]}x{frame.shape[0]}" + (" live, stills 1920x1200" if self.open_arducam else "")
                self.stats["arducam"] = {"res": res, "fps": round(fps, 1), "focus": round(focus_score(frame), 1)}
                self._encode("arducam", frame)
            if self.rs:
                color, depth, _, _ = self.rs.latest()
                self.stats["d405_color"] = {"res": f"{color.shape[1]}x{color.shape[0]}", "fps": round(self.rs.fps, 1),
                                            "focus": round(focus_score(color), 1)}
                self._encode("d405_color", color)
                h, w = depth.shape
                center = self.rs.depth_mm(depth, w // 2, h // 2)
                self.stats["d405_depth"] = {"res": f"{w}x{h}", "fps": round(self.rs.fps, 1),
                                            "center_mm": None if center is None else round(center, 1),
                                            "holes_pct": round(100.0 * float(np.mean(depth == 0)), 1),
                                            "range_mm": [self.near_mm, self.far_mm]}
                self._encode("d405_depth", colorize_depth(depth, self.rs.depth_scale, self.near_mm, self.far_mm))
            with self.cond:
                self.cond.notify_all()
            time.sleep(max(0.0, 1.0 / RENDER_HZ - (time.monotonic() - t0)))

    def status(self):
        return {"tiles": {t: {"available": self.available(t), "stats": self.stats[t],
                              "error": self.errors.get("arducam" if t == "arducam" else "d405")}
                          for t in self.TILES},
                "d405_info": getattr(self.rs, "info", None)}

    def depth_at(self, fx, fy):
        if not self.rs:
            return None
        _, depth, _, _ = self.rs.latest()
        h, w = depth.shape
        x, y = int(np.clip(fx, 0, 1) * (w - 1)), int(np.clip(fy, 0, 1) * (h - 1))
        mm = self.rs.depth_mm(depth, x, y)
        return {"x": x, "y": y, "mm": None if mm is None else round(mm, 1)}

    def capture(self):
        with self.capture_lock:
            return self._capture()

    def _capture(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]   # ms, so captures never share a folder
        out = OUT / stamp
        out.mkdir(parents=True)
        meta = {"created": stamp, "files": {}}
        if self.rs:   # D405 first, while the Arducam is still in its light preview mode
            color, depth, t, _ = self.rs.latest()
            cv2.imwrite(str(out / "d405_color.png"), color)
            cv2.imwrite(str(out / "d405_depth_raw.png"), depth)   # 16-bit; mm = value * depth_scale * 1000
            cv2.imwrite(str(out / "d405_depth_vis.png"),
                        colorize_depth(depth, self.rs.depth_scale, self.near_mm, self.far_mm))
            meta["files"].update({"d405_color.png": {"stamp": t, "focus": focus_score(color)},
                                  "d405_depth_raw.png": {"stamp": t, "depth_scale_m": self.rs.depth_scale},
                                  "d405_depth_vis.png": {"range_mm": [self.near_mm, self.far_mm]}})
            meta.update(d405_info=self.rs.info, d405_mode=self.rs.mode, d405_intrinsics=self.rs.intrinsics)
        if self.arducam or self.open_arducam:
            img, t = self._arducam_still()
            cv2.imwrite(str(out / "arducam.png"), img)
            meta["files"]["arducam.png"] = {"stamp": t, "focus": focus_score(img), "size": img.shape[1::-1]}
        (out / "meta.json").write_text(json.dumps(meta, indent=2, default=list))
        return {"dir": stamp, "files": [{"name": n, "url": f"/files/{stamp}/{n}"}
                                        for n in sorted(meta["files"]) + ["meta.json"]]}


    def _arducam_still(self, n=5):
        """Full-resolution Arducam still: the sharpest of n fresh frames."""
        if not self.open_arducam:
            return self.arducam.latest()
        with self.cam_lock:
            if self.arducam:
                self.arducam.close()
            self.arducam = None
        try:
            full = self.open_arducam(True)
            try:
                frames = full.frames_after(time.monotonic() + 0.3, n)
            finally:
                full.close()
            return max(frames, key=lambda f: focus_score(f[0]))
        finally:
            try:
                cam = self.open_arducam(False)
                self.errors.pop("arducam", None)
            except Exception as e:
                cam = None
                self.errors["arducam"] = f"preview did not reopen: {e}"
            with self.cam_lock:
                self.arducam = cam


# ---- HTTP ----------------------------------------------------------------------------
def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            url = urlparse(self.path)
            parts = url.path.strip("/").split("/")
            if url.path == "/":
                self._send(200, PAGE.encode(), "text/html; charset=utf-8")
            elif url.path == "/status":
                self._json(hub.status())
            elif url.path == "/depth":
                q = parse_qs(url.query)
                self._json(hub.depth_at(float(q["x"][0]), float(q["y"][0])))
            elif parts[0] == "stream" and len(parts) == 2 and parts[1] in hub.TILES:
                self._stream(parts[1])
            elif parts[0] == "files" and len(parts) == 3 and all(SAFE_NAME.match(p) for p in parts[1:]):
                path = OUT / parts[1] / parts[2]
                if not path.is_file():
                    self._json({"error": "not found"}, 404)
                    return
                ctype = "application/json" if path.suffix == ".json" else "image/png"
                self._send(200, path.read_bytes(), ctype)
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            if self.path == "/capture":
                self._json(hub.capture())
            else:
                self._json({"error": "not found"}, 404)

        def _stream(self, tile):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while hub.running:
                    with hub.cond:
                        hub.cond.wait(1.0)
                    b = hub.jpeg[tile]
                    if b:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                         % len(b) + b + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


PAGE = """<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Camera Compare</title>
<style>
:root { --bg:#111418; --panel:#1b2027; --text:#e8ecf1; --muted:#9aa5b1; --accent:#3fb950; --warn:#d29922; --bad:#f85149; }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--text); font:15px/1.4 -apple-system, system-ui, sans-serif; }
header { display:flex; align-items:center; gap:12px; padding:12px 16px; position:sticky; top:0; background:var(--bg); z-index:2; }
h1 { font-size:18px; margin:0; flex:1; }
button { font:inherit; font-weight:600; border:0; border-radius:10px; padding:12px 20px; background:var(--accent); color:#04130a; }
button:disabled { opacity:.5; }
main { display:grid; gap:12px; padding:0 16px 16px; grid-template-columns:repeat(auto-fit, minmax(340px, 1fr)); }
.tile { background:var(--panel); border-radius:12px; overflow:hidden; }
.tile h2 { font-size:14px; margin:0; padding:10px 12px 0; }
.view { position:relative; }
.view img { display:block; width:100%; height:auto; background:#000; min-height:120px; }
.marker { position:absolute; width:18px; height:18px; margin:-9px 0 0 -9px; border:2px solid #fff; border-radius:50%; display:none; pointer-events:none; }
.stats { padding:8px 12px 12px; color:var(--muted); font-variant-numeric:tabular-nums; min-height:2.8em; }
.stats b { color:var(--text); font-weight:600; }
.err { color:var(--bad); }
#shots { padding:0 16px 24px; }
.shot { background:var(--panel); border-radius:12px; padding:12px; margin-top:12px; }
.shot a { color:#79c0ff; margin-right:14px; display:inline-block; padding:4px 0; }
.thumbs { display:flex; gap:8px; overflow-x:auto; margin-top:8px; }
.thumbs img { height:110px; border-radius:6px; }
#conn { width:10px; height:10px; border-radius:50%; background:var(--bad); }
</style></head>
<body>
<header><span id="conn" title="connection"></span><h1>Camera Compare</h1><button id="cap">Capture</button></header>
<main>
  <section class="tile"><h2>Arducam B0495</h2><div class="view"><img src="/stream/arducam" alt="Arducam live"></div><div class="stats" id="s-arducam"></div></section>
  <section class="tile"><h2>D405 colour</h2><div class="view"><img src="/stream/d405_color" alt="D405 colour live"></div><div class="stats" id="s-d405_color"></div></section>
  <section class="tile"><h2>D405 depth <span style="color:var(--muted);font-weight:400">(tap to measure)</span></h2><div class="view" id="depthview"><img id="depthimg" src="/stream/d405_depth" alt="D405 depth live"><div class="marker" id="marker"></div></div><div class="stats" id="s-d405_depth"></div></section>
</main>
<div id="shots"></div>
<script>
const fmt = (v, unit='') => v === null || v === undefined ? '–' : v + unit;
let tapped = null;
async function poll() {
  try {
    const s = await (await fetch('/status', {cache:'no-store'})).json();
    document.getElementById('conn').style.background = 'var(--accent)';
    for (const [name, t] of Object.entries(s.tiles)) {
      const el = document.getElementById('s-' + name);
      if (!t.available) { el.innerHTML = '<span class="err">Not connected: ' + (t.error || 'unavailable') + '</span>'; continue; }
      const st = t.stats; let h = '<b>' + fmt(st.res) + '</b> · ' + fmt(st.fps, ' fps');
      if ('focus' in st) h += ' · focus <b>' + fmt(st.focus) + '</b>';
      if ('center_mm' in st) h += ' · centre <b>' + fmt(st.center_mm, ' mm') + '</b> · holes ' + fmt(st.holes_pct, '%');
      if (name === 'd405_depth' && tapped) h += '<br>tapped: <b>' + fmt(tapped.mm, ' mm') + '</b> at (' + tapped.x + ', ' + tapped.y + ')';
      el.innerHTML = h;
    }
  } catch (e) { document.getElementById('conn').style.background = 'var(--bad)'; }
}
setInterval(poll, 1000); poll();

document.getElementById('depthimg').addEventListener('click', async (ev) => {
  const r = ev.target.getBoundingClientRect();
  const fx = (ev.clientX - r.left) / r.width, fy = (ev.clientY - r.top) / r.height;
  const m = document.getElementById('marker');
  m.style.left = (fx * 100) + '%'; m.style.top = (fy * 100) + '%'; m.style.display = 'block';
  tapped = await (await fetch('/depth?x=' + fx + '&y=' + fy)).json();
  poll();
});

for (const img of document.querySelectorAll('.view img')) {
  img.addEventListener('error', () => setTimeout(() => { img.src = img.src.split('?')[0] + '?r=' + Date.now(); }, 2000));
}

document.getElementById('cap').addEventListener('click', async (ev) => {
  const b = ev.target; b.disabled = true; b.textContent = 'Saving…';
  const ctl = new AbortController(); const timer = setTimeout(() => ctl.abort(), 20000);
  try {
    const resp = await fetch('/capture', {method:'POST', signal: ctl.signal});
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const r = await resp.json();
    const div = document.createElement('div'); div.className = 'shot';
    let links = '', thumbs = '';
    for (const f of r.files) {
      links += '<a href="' + f.url + '" target="_blank">' + f.name + '</a>';
      if (f.name.endsWith('.png') && !f.name.includes('raw')) thumbs += '<a href="' + f.url + '" target="_blank"><img src="' + f.url + '" alt="' + f.name + '"></a>';
    }
    div.innerHTML = '<b>' + r.dir + '</b><br>' + links + '<div class="thumbs">' + thumbs + '</div>';
    document.getElementById('shots').prepend(div);
  } catch (e) { alert(e.name === 'AbortError' ? 'Capture took over 20 s; check the Jetson.' : 'Capture failed: ' + e); }
  clearTimeout(timer); b.disabled = false; b.textContent = 'Capture';
});
</script>
</body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--near-mm", type=float, default=50.0, help="depth colour scale: red at this distance")
    ap.add_argument("--far-mm", type=float, default=400.0, help="depth colour scale: blue at this distance")
    ap.add_argument("--fake", action="store_true", help="synthetic cameras, for testing without hardware")
    ap.add_argument("--arducam-live", choices=("preview", "full"), default="preview",
                    help="preview: 960x600 live (keeps the D405 at 30 fps), 1920x1200 stills; full: 1920x1200 live")
    args = ap.parse_args()

    errors, arducam, realsense, open_arducam = {}, None, None, None
    if args.fake:
        arducam, realsense = FakeArducam(), FakeRealSense()
    else:
        from common.camera import Camera
        from common.realsense import RealSense

        def make_arducam(full):
            return Camera(width=1920, height=1200, fps=15) if full else Camera(width=960, height=600, fps=30)

        try:
            arducam = make_arducam(args.arducam_live == "full")
            if args.arducam_live == "preview":
                open_arducam = make_arducam
        except Exception as e:
            errors["arducam"] = str(e)
        try:
            realsense = RealSense()
        except Exception as e:
            errors["d405"] = f"{type(e).__name__}: {e}"
    for k, v in errors.items():
        print(f"{k}: NOT AVAILABLE ({v})")
    if realsense is not None:
        print(f"D405: {realsense.info} mode {realsense.mode} depth scale {realsense.depth_scale}")

    hub = Hub(arducam, realsense, errors, args.near_mm, args.far_mm, open_arducam)
    threading.Thread(target=hub.render_loop, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(hub))
    server.daemon_threads = True
    print(f"Open http://192.168.94.94:{args.port}/ (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hub.running = False
        server.server_close()
        for cam in (arducam, realsense):
            if cam is not None:
                cam.close()


if __name__ == "__main__":
    main()
