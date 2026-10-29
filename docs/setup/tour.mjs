// The inspection run as a pure function of time, shared by the build (poses, audit) and the page.
// Joint angles come from tables the build solves with rig.mjs: a grid over (seam position, tilt)
// for the passes, and knot lists for the moves along the tool axis.

export const SCAN_SPEED = 40;     // mm/s along the seam (claude.md: scans at ~30–50 mm/s)
export const FPS = 30;            // Arducam video rate during the scan
export const RAMP = 0.3;          // s to reach scan speed

const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const ease = (x) => { const t = clamp(x, 0, 1); return t * t * t * (10 + t * (6 * t - 15)); };   // min-jerk
const mix = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);

// Distance covered after `t` seconds of a move of length `len` with trapezoidal speed.
export function trapezoid(t, len, speed = SCAN_SPEED, ramp = RAMP) {
  const dur = len / speed + ramp;
  const x = clamp(t, 0, dur);
  const a = speed / ramp;
  if (x < ramp) return 0.5 * a * x * x;
  if (x > dur - ramp) return len - 0.5 * a * (dur - x) ** 2;
  return 0.5 * a * ramp * ramp + speed * (x - ramp);
}

export function makeTour(D) {
  const { grid } = D;
  const gridAt = (s, tilt) => {
    const fs = clamp((s - grid.s0) / grid.ds, 0, grid.ns - 1);
    const ft = clamp((tilt - grid.t0) / grid.dt, 0, grid.nt - 1);
    const i = Math.min(grid.ns - 2, Math.floor(fs));
    const j = Math.min(grid.nt - 2, Math.floor(ft));
    const u = fs - i;
    const w = ft - j;
    const q = (a, b) => grid.q.slice((b * grid.ns + a) * 6, (b * grid.ns + a) * 6 + 6);
    return mix(mix(q(i, j), q(i + 1, j), u), mix(q(i, j + 1), q(i + 1, j + 1), u), w);
  };
  const knotsAt = (knots, x) => {
    const f = clamp(x, 0, 1) * (knots.length - 1);
    const i = Math.min(knots.length - 2, Math.floor(f));
    return mix(knots[i], knots[i + 1], f - i);
  };
  const starts = [];
  let total = 0;
  for (const seg of D.segments) {
    starts.push(total);
    total += seg.dur;
  }
  const at = (time) => {
    const t = ((time % total) + total) % total;
    let n = starts.length - 1;
    while (n > 0 && starts[n] > t) n--;
    const seg = D.segments[n];
    const local = t - starts[n];
    const x = local / seg.dur;
    const out = { seg: n, kind: seg.kind, phase: seg.phase, local, s: null, tilt: null, frame: null };
    if (seg.kind === "scan") {
      const len = Math.abs(seg.to - seg.from);
      out.s = seg.from + Math.sign(seg.to - seg.from) * trapezoid(local, len, seg.speed ?? SCAN_SPEED);
      out.tilt = seg.tilt;
      out.frame = Math.floor(local * FPS);
      out.q = gridAt(out.s, out.tilt);
    } else if (seg.kind === "swing") {
      out.s = seg.s;
      out.tilt = seg.from + (seg.to - seg.from) * ease(x);
      out.q = gridAt(out.s, out.tilt);
    } else if (seg.kind === "hold") {
      out.s = seg.s;
      out.tilt = seg.tilt;
      out.q = gridAt(seg.s, seg.tilt);
    } else if (seg.kind === "knots") {
      out.q = knotsAt(seg.knots, ease(x));
      out.d = seg.d0 + (seg.d1 - seg.d0) * ease(x);
    } else if (seg.kind === "joint") {
      out.q = mix(seg.from, seg.to, ease(x));
    } else {
      out.q = seg.q.slice();
    }
    return out;
  };
  return { at, gridAt, duration: total, starts };
}
