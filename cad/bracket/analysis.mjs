// What each bracket does to the arm: run the inspection passes through the Lite 6 model for every
// design and both ways of placing the T-joint, and measure J5, joint-limit margins and how close the
// bracket and cameras come to the plates.
//   node cad/bracket/analysis.mjs        reads out/specs.json, writes out/analysis.json
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as M from "../../docs/setup/rig.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const specs = JSON.parse(readFileSync(join(HERE, "out", "specs.json"), "utf8"));
const RAD = Math.PI / 180;
const STANDOFF = specs.standoff;
const SEAM = [-68, 68];
const TILTS = [0, 18, -18];
const PASS = ["bisector", "flange toe (+18°)", "web toe (−18°)"];

// ---- the two ways to place the T-joint (claude.md: workpiece ~200–300 mm out) ----------------------
const LAYOUTS = {
  back: {
    title: "Web on the robot's side (camera looks back)",
    web: 170, sign: 1,
    boxes: [
      { name: "flange plate", min: [116, -80, 0], max: [290, 80, 6] },
      { name: "web", min: [164, -80, 6], max: [170, 80, 76] },
    ],
  },
  out: {
    title: "Web beyond the seam (camera looks outward)",
    web: 370, sign: -1,
    boxes: [
      { name: "flange plate", min: [250, -80, 0], max: [424, 80, 6] },
      { name: "web", min: [370, -80, 6], max: [376, 80, 76] },
    ],
  },
};
const BENCH = { name: "bench", min: [-1000, -1000, -50], max: [1000, 1000, 0] };

// TCP pose for a seam point: optical axis = −bisector tilted `tilt` toward the flange; camera row across the seam
function viewAt(layout, s, tilt, along = 0, roll = 0) {
  const L = LAYOUTS[layout];
  const el = (45 + tilt) * RAD;
  const b = [L.sign * Math.cos(el), 0, Math.sin(el)];
  const z = b.map((v) => -v);
  const across = [-z[2], 0, z[0]];                       // roll 0: the camera row across the seam
  const along0 = [0, 1, 0];
  const c = Math.cos(roll * RAD), sn = Math.sin(roll * RAD);
  const x = [0, 1, 2].map((i) => c * across[i] + sn * along0[i]);
  const y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
  const p = [L.web, s, 6];
  return { R: [0, 1, 2].map((r) => [x[r], y[r], z[r]]), t: [p[0] + z[0] * along, p[1] + z[1] * along, p[2] + z[2] * along] };
}

function toolOf(d) {
  const z = d.axis;
  const x = [1, 0, 0];
  const y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
  return { R: [0, 1, 2].map((r) => [x[r], y[r], z[r]]), t: d.tcp };
}

const apply = (T, p) => [0, 1, 2].map((r) => T.R[r][0] * p[0] + T.R[r][1] * p[1] + T.R[r][2] * p[2] + T.t[r]);
function boxDistance(p, b) {
  const d = [0, 1, 2].map((i) => Math.max(b.min[i] - p[i], 0, p[i] - b.max[i]));
  const outside = Math.hypot(...d);
  if (outside > 0) return outside;
  return -Math.min(...[0, 1, 2].map((i) => Math.min(p[i] - b.min[i], b.max[i] - p[i])));
}

// the robot's own wrist near the work: J6 module, J5 housing, far forearm (from the figure's model)
function wristPoints(q) {
  const F = M.frames(q);
  const pts = [];
  const ring = (o, a, r, s0, s1) => {
    const u = Math.abs(a[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const v1 = norm(cross(a, u));
    const v2 = cross(a, v1);
    for (let s = s0; s <= s1 + 1e-9; s += (s1 - s0) / 4)
      for (let k = 0; k < 16; k++) {
        const t = (k / 16) * 2 * Math.PI;
        pts.push([0, 1, 2].map((i) => o[i] + a[i] * s + r * (Math.cos(t) * v1[i] + Math.sin(t) * v2[i])));
      }
  };
  ring(F[5].t, M.col(F[5], 2), 30, -25.5, -7.5);
  ring(F[4].t, M.col(F[4], 2), 34, -30, 30);
  ring(F[3].t, M.col(F[3], 2), 30, -110, -40);
  return pts;
}
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a); return a.map((v) => v / l); };

const inLimits = (q) => q.every((v, i) => v >= M.JOINT_LIMITS[i][0] && v <= M.JOINT_LIMITS[i][1]);
const SEEDS = [[0, 50, 74, 0, 69, 0], [0, 30, 60, 180, -40, 0], [0, 40, 90, 180, 40, 0], [0, 20, 40, 0, 40, 0], [0, 10, 30, 0, 90, 0], [0, 60, 120, 0, 100, 180], [0, 30, 50, 0, -60, 0], [0, 50, 100, 0, 30, 0], [0, 36, 35, 0, -46, 0]];

function analyse(key, layout) {
  const runs = [0, 90, 180, 270].map((roll) => analyseRoll(key, layout, roll));
  const score = (x) => (!x.feasible ? -1e9 : (x.collides ? -1e6 : 0) + (x.j5min < 15 ? -1e4 : 0) + Math.min(x.clearance.d, 60) + 0.5 * Math.min(x.j5min, 40));
  runs.sort((a, b) => score(b) - score(a));
  return { ...runs[0], rolls: runs.map((x) => ({ roll: x.roll, feasible: x.feasible, collides: x.collides, j5min: x.j5min, clearance: x.clearance?.d })) };
}

function analyseRoll(key, layout, roll) {
  const d = specs.designs[key];
  const tool = toolOf(d);
  const cloud = d.cloud;
  // pick the branch that keeps |J5| largest with one sign across all passes
  let best = null;
  const seeds = SEEDS.flatMap((q) => [[q[0], q[1], q[2], q[3], q[4], q[5] + roll], [q[0], q[1], q[2], q[3] + 180, -q[4], q[5] + roll + 180], [q[0], q[1], q[2], q[3], q[4], q[5] - roll]]);
  for (const s0 of seeds) {
    const seed = M.ik(viewAt(layout, 0, 0, 0, roll), s0, tool);
    if (!seed) continue;
    const sign = Math.sign(seed[4]);
    const passes = [];
    let ok = true;
    for (const tilt of TILTS) {
      const mid = M.ik(viewAt(layout, 0, tilt, 0, roll), seed, tool);
      if (!mid) { ok = false; break; }
      const samples = [];
      for (const dir of [1, -1]) {
        let q = mid;
        for (let s = 0; Math.abs(s) <= SEAM[1] + 1e-9; s += dir * 4) {
          q = q && M.ik(viewAt(layout, s, tilt, 0, roll), q, tool);
          if (!q) { ok = false; break; }
          if (dir === 1 || s !== 0) samples.push({ s, q });
        }
      }
      samples.sort((a, b) => a.s - b.s);
      if (samples.some((x) => Math.sign(x.q[4]) !== sign || !inLimits(x.q))) ok = false;
      passes.push({ tilt, samples });
      if (!ok) break;
    }
    if (!ok) continue;
    const minJ5 = Math.min(...passes.flatMap((p) => p.samples.map((x) => Math.abs(x.q[4]))));
    if (!best || minJ5 > best.minJ5) best = { seed, passes, minJ5 };
  }
  if (!best) return { key, layout, roll, feasible: false };
  // the revisit's closer step (+40 along tool z) and the clearance pose (−60), at three points on the seam
  const extra = [];
  for (const s of [-68, 0, 68]) {
    const q0 = best.passes[0].samples.find((x) => x.s === s).q;
    for (const along of [40, -60]) {
      const q = M.ik(viewAt(layout, s, 0, along, roll), q0, tool);
      extra.push({ s, along, q: q && inLimits(q) ? q : null });
    }
  }
  // J4 and J6 turn ±360°: shift each trajectory by whole turns so it sits around zero
  const everyQ = [...best.passes.flatMap((p) => p.samples.map((x) => x.q)), ...extra.filter((e) => e.q).map((e) => e.q)];
  for (const j of [3, 5]) {
    const mean = everyQ.reduce((sum, q) => sum + q[j], 0) / everyQ.length;
    const shift = -360 * Math.round(mean / 360);
    if (shift) for (const q of everyQ) q[j] += shift;
  }
  const boxes = [...LAYOUTS[layout].boxes, BENCH];
  let clear = { d: Infinity }, wrist = { d: Infinity };
  let limits = Infinity;
  const allPoses = [...best.passes.flatMap((p) => p.samples.map((x) => ({ ...x, tilt: p.tilt }))), ...extra.filter((e) => e.q).map((e) => ({ ...e, tilt: 0 }))];
  for (const pose of allPoses) {
    const Fl = M.flangeOf(pose.q);
    for (const p of cloud) {
      const w = apply(Fl, p);
      for (const b of boxes) {
        const dist = boxDistance(w, b);
        if (dist < clear.d) clear = { d: dist, what: b.name, s: pose.s, tilt: pose.tilt, along: pose.along ?? 0 };
      }
    }
    for (const w of wristPoints(pose.q))
      for (const b of boxes) {
        const dist = boxDistance(w, b);
        if (dist < wrist.d) wrist = { d: dist, what: b.name, s: pose.s, tilt: pose.tilt, along: pose.along ?? 0 };
      }
    pose.q.forEach((v, i) => { const [lo, hi] = M.JOINT_LIMITS[i]; limits = Math.min(limits, v - lo, hi - v); });
  }
  const j5 = best.passes.map((p) => ({ tilt: p.tilt, s: p.samples.map((x) => x.s), j5: p.samples.map((x) => +x.q[4].toFixed(2)) }));
  // the pose for a picture: pass 1, middle of the seam; the flange and the wrist frames in world
  const qMid = best.passes[0].samples.find((x) => x.s === 0).q;
  const Fs = M.frames(qMid);
  const pose = { q: qMid, frames: Fs.map((T) => ({ R: T.R, t: T.t })) };
  const range = (i) => { const vals = allPoses.map((p) => p.q[i]); return [Math.min(...vals), Math.max(...vals)]; };
  return {
    key, layout, roll, feasible: true,
    j5min: Math.min(...allPoses.map((p) => Math.abs(p.q[4]))),
    j5max: Math.max(...allPoses.map((p) => Math.abs(p.q[4]))),
    j5, ranges: [0, 1, 2, 3, 4, 5].map(range),
    closerOk: extra.filter((e) => e.along === 40).every((e) => e.q),
    collides: clear.d < 0 || wrist.d < 0,
    pose,
    clearance: clear, wrist, limitMargin: limits,
  };
}

const out = { standoff: STANDOFF, boxes: Object.fromEntries(Object.entries(LAYOUTS).map(([k, v]) => [k, v.boxes])), layouts: Object.fromEntries(Object.entries(LAYOUTS).map(([k, v]) => [k, v.title])), passes: PASS, designs: {} };
for (const key of Object.keys(specs.designs)) {
  const d = specs.designs[key];
  const r = Math.hypot(d.tcp[0], d.tcp[1]);
  out.designs[key] = {
    title: d.title,
    tcp: d.tcp,
    axis: d.axis,
    tcpOffAxis: r,
    rollShiftPer10: 2 * r * Math.sin(5 * RAD),
    lensFromFlange: d.lens_front,
    back: analyse(key, "back"),
    out: analyse(key, "out"),
  };
  const b = out.designs[key].back;
  const o = out.designs[key].out;
  const f = (x) => (x.feasible ? `roll ${x.roll}° · ${x.collides ? "COLLIDES · " : ""}|J5| ${x.j5min.toFixed(0)}–${x.j5max.toFixed(0)}°, bracket ${x.clearance.d.toFixed(1)} mm from ${x.clearance.what}, wrist ${x.wrist.d.toFixed(1)} mm from ${x.wrist.what}, limit margin ${x.limitMargin.toFixed(0)}°, closer ${x.closerOk ? "ok" : "fails"}` : "no clean branch");
  console.log(`${key.padEnd(7)} TCP ${d.tcp.map((v) => v.toFixed(1)).join(", ")} (off axis ${r.toFixed(1)} mm)\n  back: ${f(b)}\n  out:  ${f(o)}`);
}
writeFileSync(join(HERE, "out", "analysis.json"), JSON.stringify(out));
