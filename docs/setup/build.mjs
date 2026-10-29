// The weld-inspection cell as an isometric figure (anatomy skill).
//   node build.mjs [--light] [--audit] [--quick] [--verify]
// Writes weld-cell.html (or weld-cell-light.html) next to this script.
import { readFileSync, writeFileSync } from "node:fs";
import { homedir } from "node:os";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import * as M from "./rig.mjs";
import { makeTour, trapezoid, SCAN_SPEED, RAMP, FPS } from "./tour.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const SKILL = process.env.ANATOMY_SKILL ?? join(homedir(), ".claude", "skills", "anatomy");
const load = (path) => import(pathToFileURL(join(SKILL, path)).href);
const k = await load("kit/iso-kit.mjs");
const G = await load("kit/lathe.mjs");
const A = await load("kit/audit.mjs");
const { tubePieces, tubeSvg, runsOf } = await load("kit/tube.mjs");
const { turning } = await load("kit/turn-build.mjs");
const TA = await load("kit/turn-audit.mjs");
const { turnScript } = await load("scripts/inline-kit.mjs");

const argv = process.argv.slice(2);
const theme = argv.includes("--light") ? "light" : "dark";
const WIDTH = 720;
const HEIGHT = 520;
const AZIMUTH = 78;
const ELEVATION = 30;

// ---- layout (mm, robot base frame, z up) -----------------------------------------------------
const BENCH = { x: -250, y: -300, w: 780, d: 600, r: 10 };
const BENCH_T = 24;
const W = M.WORK;
const FLANGE_PLATE = { x: W.x - W.plate - W.back, y: W.y0, w: W.plate + W.back + W.flangeD, d: W.y1 - W.y0, r: 1.5 };
const WEB_PLATE = { x: W.x - W.plate, y: W.y0, w: W.plate, d: W.y1 - W.y0, r: 0.8 };
const BASE_R = 60;
const BASE_TOP = 96;
const JETSON = { x: -232, y: 60, w: 79, d: 100, r: 3 };           // carrier board, ports on its +x edge
const BOX = { x: -236, y: -268, w: 214, d: 108, r: 4 };           // Lite 6 control box, panel on +y
const BOX_H = 66;
const ETH_X = -100;                                              // control box panel sockets
const ROBOT_X = -40;
const ESTOP = { x: -236, y: -110, w: 64, d: 64, r: 6 };
const BOARD = { x: -10, y: 128, w: 7 * 31.79, d: 5 * 31.79, r: 0.6 };   // the ChArUco board (board.json)
const BOARD_T = 4;
const CHARUCO = JSON.parse(readFileSync(join(HERE, "charuco.json"), "utf8"));
const ROI_S = 30;                                                 // a pore on the weld at y = +30

export const P = k.fitProjection([...k.boxCorners(BENCH, -BENCH_T - 10, 0), [0, 0, 470], [150, 0, 430], [190, -40, 420]], WIDTH, HEIGHT, { pad: 24, azimuth: AZIMUTH, elevation: ELEVATION });
const V = G.viewOf(P);
const at = (p) => k.iso(p, P);
const S = (paths, style) => k.solidSvg(paths, style);
const L = (d, style) => k.lineSvg(d, style);
const D = (points, style) => k.dotsSvg(points, style);
const { add3, sub3, mul3, dot3, cross3, unit3, len3 } = G;
const frameOf = (o, a, u) => {
  const axis = unit3(a);
  const uu = unit3(sub3(u, mul3(axis, dot3(u, axis))));
  return { o, a: axis, u: uu, v: cross3(axis, uu) };
};
const VERT = (x, y, z = 0) => G.frameOf([x, y, z], [0, 0, 1], [1, 0, 0], [0, 1, 0]);
const circle = (cx, cy, r, steps = 40) => Array.from({ length: steps }, (_, j) => [cx + r * Math.cos((j / steps) * 2 * Math.PI), cy + r * Math.sin((j / steps) * 2 * Math.PI)]);
const hull = (pts) => G.hullOf(pts);
const rect = (cx, cy, w, h) => [[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2], [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]];
const byDepth = (points) => [...points].sort((a, b) => k.depthOf([a[0], a[1], 0], P) - k.depthOf([b[0], b[1], 0], P));

export const R = A.recorder(P);
const put = (item) => R.put({ layer: "back", ...item, shapes: (item.shapes ?? []).map((shape, index) => R.solid(Object.assign(shape, { name: shape.name ?? (index ? `${item.name}#${index + 1}` : item.name) }))) });
const slab = (name, plan, z, h, { tone = "mid", steps = 4, bevel = 0.5, inner = "", extra = "", lit = false, layer = "back", bias = 0, ground = false } = {}) =>
  put({ name, layer, bias, at: [plan.x + plan.w / 2, plan.y + plan.d / 2, z], svg: S(k.slabOf(plan, z, h, P, steps, bevel), { tone, inner, lit }) + extra, shapes: [A.slab(plan, z, h, ground ? { ground: true } : {}, steps)] });

// ---- the bench: an optical breadboard on four feet --------------------------------------------
function bench() {
  for (const [x, y] of k.corners(BENCH, 46)) {
    const svg = S(k.cylinder(x, y, 15, -BENCH_T - 10, 10, P, 32), { tone: "lo" });
    put({ name: `stand.foot.${x}.${y}`, at: [x, y, -BENCH_T - 10], bias: -40, svg, shapes: [A.cylinder(x, y, 15, -BENCH_T - 10, 10, { ground: true })] });
  }
  const holes = [];
  for (let x = BENCH.x + 20; x <= BENCH.x + BENCH.w - 20; x += 25) for (let y = BENCH.y + 25; y <= BENCH.y + BENCH.d - 25; y += 25) holes.push(at([x, y, 0]));
  const svg = `<path class="iso-halo" d="${k.haloOf(BENCH, -BENCH_T - 10, BENCH_T, P)}"/>` + S(k.slabOf(BENCH, -BENCH_T, BENCH_T, P, 8, 1.4), { tone: "mid", inner: L(k.sideSeam(BENCH, -BENCH_T + 6, P, 8), { tone: "faint" }) }) + L(k.planOutline(k.insetPlan(BENCH, 8), 0, P), { tone: "faint" }) + D(holes, { size: 0.4, tone: "lo" });
  put({ name: "stand.bench", at: [BENCH.x, BENCH.y, -BENCH_T], bias: -60, svg, shapes: [A.slab(BENCH, -BENCH_T, BENCH_T, { ground: true })] });
}

// ---- the robot base (static: J1 turns the turret above it) -----------------------------------------
const BASE_PORT = { a: (270 * Math.PI) / 180, z: 22 };                // robot cable socket on the base
function robotBase() {
  const bolts = byDepth([45, 135, 225, 315].map((a) => [64 * Math.cos((a * Math.PI) / 180), 64 * Math.sin((a * Math.PI) / 180)]));
  let svg = `<path class="iso-halo" d="${k.haloOf({ x: -74, y: -74, w: 148, d: 148, r: 74 }, 0, 4, P)}"/>` + S(k.cylinder(0, 0, 74, 0, 8, P, 72, 0.6), { tone: "lo" });
  svg += L(bolts.map(([x, y]) => k.ring(x, y, 4.6, 8, P, 18)).join(""), { tone: "lo" }) + D(bolts.map(([x, y]) => at([x, y, 8])), { size: 0.6 });
  svg += S(k.cylinder(0, 0, BASE_R, 8, BASE_TOP - 8, P, 72), { tone: "mid", inner: L(k.sideArc(0, 0, BASE_R, 20, P, 48) + k.sideArc(0, 0, BASE_R, BASE_TOP - 6, P, 48), { tone: "faint" }) });
  put({ name: "stand.base", at: [0, 0, 8], svg, shapes: [A.cylinder(0, 0, 74, 0, 8), A.cylinder(0, 0, BASE_R, 8, BASE_TOP - 8)] });
  // the robot-cable socket: a boss on the base wall
  const n = [Math.cos(BASE_PORT.a), Math.sin(BASE_PORT.a), 0];
  const Fp = frameOf(add3(mul3(n, BASE_R - 0.45), [0, 0, BASE_PORT.z]), n, [0, 0, 1]);
  put({ name: "stand.base.socket", at: add3(Fp.o, mul3(n, 6)), svg: S(G.disc(0, 9, 13, Fp, P), { tone: "lo", inner: L(G.arcOf(Fp, 3, 13, P), { tone: "faint" }) }), shapes: [A.disc(Fp, 0, 9, 13)] });
}

// ---- the workpiece: flange plate, web plate and the fillet weld ------------------------------------
const BEAD = (() => {
  const c = -0.94;
  const R0 = Math.hypot(W.leg - c, c);
  const a0 = Math.atan2(0 - c, W.leg - c);
  const a1 = Math.atan2(W.leg - c, 0 - c);
  const arc = Array.from({ length: 15 }, (_, j) => {
    const a = a0 + ((a1 - a0) * j) / 14;
    return [c + R0 * Math.cos(a), c + R0 * Math.sin(a)];
  });
  return { polygon: [[0, 0], ...arc], c, R0 };
})();
const BEAD_F = { o: [W.x, 0, W.plate], a: [0, 1, 0], u: [1, 0, 0], v: [0, 0, 1] };
const BEAD_S = [M.SEAM.from - 6, M.SEAM.to + 6];
const onBead = (s, angle) => add3(BEAD_F.o, [BEAD.c + BEAD.R0 * Math.cos(angle), s, BEAD.c + BEAD.R0 * Math.sin(angle)]);
const CROWN = Math.PI / 4;
function workpiece() {
  const clampAt = [W.y0 + 22, W.y1 - 22];
  slab("work.flange", FLANGE_PLATE, 0, W.plate, { steps: 3, bevel: 0.5 });
  slab("work.web", WEB_PLATE, W.plate, W.webH, { steps: 2, bevel: 0.4 });
  const [s0, s1] = BEAD_S;
  const ripples = [];
  for (let y = s0 + 2.4; y < s1 - 1; y += 3.2) {
    const pts = Array.from({ length: 11 }, (_, j) => {
      const a = 0.08 + (Math.PI / 2 - 0.16) * (j / 10);
      return at(add3(onBead(y - 1.3 * Math.sin(Math.PI * (j / 10)), a), [0, 0, 0]));
    });
    ripples.push(k.pathOf(pts));
  }
  // the pore the detector will flag: a small dark crater on the bead's crown
  const pore = Array.from({ length: 18 }, (_, j) => {
    const t = (j / 18) * 2 * Math.PI;
    return at(onBead(ROI_S + 1.5 * Math.cos(t), CROWN + 0.18 * Math.sin(t)));
  });
  const bead = G.prismOf(BEAD_F, BEAD.polygon, s0, s1, P);
  put({ name: "work.bead", at: [W.x + 3, (s0 + s1) / 2, W.plate + 3], svg: S(bead, { tone: "hi", crease: "lo", inner: L(ripples.join(""), { tone: "faint" }) }) + `<path class="wc-pore" d="${k.pathOf(pore, true)}"/>`, shapes: [A.prism(BEAD_F, BEAD.polygon, s0, s1)] });
  // two strap clamps hold the flange plate to the breadboard
  for (const y of clampAt) {
    const block = { x: W.x + W.flangeD + 8, y: y - 9, w: 18, d: 18, r: 1.5 };
    slab(`work.clamp.block.${y}`, block, 0, W.plate, { steps: 2, bevel: 0.3, tone: "lo" });
    const bar = { x: W.x + W.flangeD - 26, y: y - 7, w: 52, d: 14, r: 2 };
    slab(`work.clamp.bar.${y}`, bar, W.plate, 7, { steps: 3, bevel: 0.4, tone: "mid" });
    const sx = W.x + W.flangeD + 2;
    put({ name: `work.clamp.stud.${y}`, at: [sx, y, W.plate + 11.5], bias: 1, svg: S(k.cylinder(sx, y, 3, W.plate + 11.5, 4, P, 20), { tone: "mid", crease: "none" }), shapes: [A.cylinder(sx, y, 3, W.plate + 11.5, 4)] });
    const hex = Array.from({ length: 6 }, (_, j) => [sx + 5.6 * Math.cos(((2 * j + 1) * Math.PI) / 6), y + 5.6 * Math.sin(((2 * j + 1) * Math.PI) / 6)]);
    put({ name: `work.clamp.nut.${y}`, at: [sx, y, W.plate + 7], bias: 0.5, svg: S(k.extrude(hex, W.plate + 7, 4.5, P, { bevel: 0.3 }), { tone: "hi" }), shapes: [A.extruded(hex, W.plate + 7, 4.5)] });
  }
}

// ---- the Jetson Orin Nano developer kit (ports along its −y edge, toward the control box) ---------
const J_TOP = 10.6;
const JPORT = {};
function jetson() {
  const J = JETSON;
  for (const [x, y] of byDepth(k.corners(J, 4))) put({ name: `jet.standoff.${x}.${y}`, at: [x, y, 0], svg: S(k.cylinder(x, y, 2.6, 0, 9, P, 16), { tone: "lo", crease: "none" }), shapes: [A.cylinder(x, y, 2.6, 0, 9)] });
  const header = [];
  for (let i = 0; i < 20; i++) for (const dx of [0, 2.54]) header.push(at([J.x + 5 + dx, J.y + 22 + i * 2.54, J_TOP]));
  slab("jet.board", J, 9, 1.6, { tone: "lo", steps: 3, bevel: 0.3, extra: D(header, { size: 0.35, tone: "mid" }) });
  const sink = { x: J.x + 8, y: J.y + 20, w: 46, d: 62, r: 3 };
  const fins = [];
  for (let x = sink.x + 4; x < sink.x + sink.w - 2; x += 4) fins.push(k.lineOnTop([x, sink.y], [x, sink.y + sink.d], J_TOP + 12, P));
  slab("jet.heatsink", sink, J_TOP, 12, { tone: "mid", steps: 3, bevel: 0.4, extra: L(fins.join(""), { tone: "lo" }) });
  const fan = { x: sink.x + 5, y: sink.y + 13, w: 36, d: 36, r: 3 };
  const fc = [fan.x + 18, fan.y + 18];
  const blades = Array.from({ length: 7 }, (_, j) => {
    const a = (j / 7) * 2 * Math.PI;
    return k.onTop([[fc[0] + 5 * Math.cos(a), fc[1] + 5 * Math.sin(a)], [fc[0] + 11 * Math.cos(a + 0.35), fc[1] + 11 * Math.sin(a + 0.35)], [fc[0] + 15.6 * Math.cos(a + 0.8), fc[1] + 15.6 * Math.sin(a + 0.8)]], J_TOP + 20, P);
  });
  const fanScrews = k.corners(fan, 3.2);
  slab("jet.fan", fan, J_TOP + 12, 8, { tone: "hi", steps: 3, bevel: 0.5, extra: L(k.ring(fc[0], fc[1], 16.2, J_TOP + 20, P, 56) + k.ring(fc[0], fc[1], 5, J_TOP + 20, P, 24), { tone: "lo" }) + L(blades.join(""), { tone: "faint" }) + D(fanScrews.map(([x, y]) => at([x, y, J_TOP + 20])), { size: 0.45 }) });
  // the port row on the +x edge, back to front: RJ45, DisplayPort, USB-C, two stacked pairs of USB-A
  // (all four share one internal USB3 hub), DC jack
  const ex = J.x + J.w;
  const port = (name, y0, w, h, depth = 14) => {
    const plan = { x: ex - depth + 1.5, y: y0, w: depth, d: w, r: 0.6 };
    const slot = k.pathOf([at([ex + 1.5, y0 + 1.6, J_TOP + 1.6]), at([ex + 1.5, y0 + w - 1.6, J_TOP + 1.6]), at([ex + 1.5, y0 + w - 1.6, J_TOP + h - 1.6]), at([ex + 1.5, y0 + 1.6, J_TOP + h - 1.6])], true);
    slab(`jet.port.${name}`, plan, J_TOP, h, { tone: "mid", steps: 2, bevel: 0.25, inner: L(slot, { tone: "lo" }) });
    return { y: y0 + w / 2, face: ex + 1.5, z: J_TOP + h / 2, w, h };
  };
  JPORT.eth = port("eth", J.y + 2, 16, 13.5, 18);
  JPORT.dp = port("dp", J.y + 21, 16, 8, 10);
  JPORT.usbc = port("usbc", J.y + 40, 9, 4, 8);
  JPORT.usbA = port("usba.0", J.y + 52, 14, 15);
  JPORT.usbB = port("usba.1", J.y + 69, 14, 15);
  JPORT.dc = port("dc", J.y + 87, 9, 11, 12);
}

// ---- the Lite 6 control box (its own 24 V supply; panel on +y) ------------------------------------
const BPORT = {};
function controlBox() {
  for (const [x, y] of byDepth(k.corners(BOX, 14))) put({ name: `box.foot.${x}.${y}`, at: [x, y, 0], svg: S(k.cylinder(x, y, 7, 0, 5, P, 20), { tone: "lo" }), shapes: [A.cylinder(x, y, 7, 0, 5)] });
  const top = 5 + BOX_H;
  const vents = [];
  for (let x = BOX.x + 26; x < BOX.x + BOX.w - 64; x += 12) vents.push(k.planOutline({ x, y: BOX.y + 22, w: 6, d: 50, r: 3 }, top, P, 4));
  const plate = { x: BOX.x + BOX.w - 54, y: BOX.y + 24, w: 40, d: 28, r: 1.5 };
  const rules = [[8, 26], [14, 30], [20, 20]].map(([dy, len]) => k.lineOnTop([plate.x + 6, plate.y + dy], [plate.x + 6 + len, plate.y + dy], top + 0.6, P));
  const extra = L(vents.join(""), { tone: "lo" }) + S(k.slabOf(plate, top, 0.6, P, 3), { tone: "hi", crease: "none" }) + L(rules.join(""), { tone: "lo", free: true }) + D([[plate.x + 3, plate.y + 3], [plate.x + plate.w - 3, plate.y + plate.d - 3]].map(([x, y]) => at([x, y, top + 0.6])), { size: 0.45 });
  slab("box.body", BOX, 5, BOX_H, { tone: "mid", steps: 6, bevel: 1.1, inner: L(k.sideSeam(BOX, top - 8, P, 6), { tone: "faint" }), extra });
  const face = BOX.y + BOX.d;
  const zc = 5 + BOX_H / 2;
  const boss = (name, x, r, len = 8) => {
    const Fb = frameOf([x, face, zc], [0, 1, 0], [0, 0, 1]);
    put({ name: `box.${name}`, at: [x, face + len / 2, zc], svg: S(G.disc(0, len, r, Fb, P), { tone: "lo", inner: L(G.arcOf(Fb, len * 0.4, r, P), { tone: "faint" }) }) + L(G.circleOf(Fb, len, r * 0.55, P, 28), { tone: "lo" }), shapes: [A.disc(Fb, 0, len, r)] });
    return { o: [x, face + len, zc], r };
  };
  BPORT.estop = boss("socket.estop", ESTOP.x + ESTOP.w / 2, 8, 8);
  const ex = ETH_X;
  const eth = { x: ex - 8, y: face - 0.4, w: 16, d: 13.4, r: 0.6 };
  slab("box.eth", eth, zc - 7, 14, { tone: "mid", steps: 2, bevel: 0.25, inner: L(k.pathOf([at([eth.x + 2, face + 13, zc - 5]), at([eth.x + 14, face + 13, zc - 5]), at([eth.x + 14, face + 13, zc + 5]), at([eth.x + 2, face + 13, zc + 5])], true), { tone: "lo" }) });
  BPORT.eth = { o: [ex, face + 13, zc] };
  const sw = { x: BOX.x + 76, y: face - 0.4, w: 14, d: 5.4, r: 1 };
  slab("box.switch", sw, zc - 9, 18, { tone: "hi", steps: 2, bevel: 0.3 });
  BPORT.robot = boss("socket.robot", ROBOT_X, 13, 10);
  const leds = [0, 1, 2].map((i) => [BOX.x + 156 + i * 9, face, zc + 10]);
  const ledPlate = { x: BOX.x + 150, y: face - 0.4, w: 30, d: 1.4, r: 0.4 };
  slab("box.leds", ledPlate, zc + 5, 10, { tone: "hi", steps: 2, bevel: 0, extra: D(leds.map((p) => at([p[0], face + 1, p[2]])), { size: 0.9, tone: "mid" }) });
}

// ---- the emergency stop, beside the control box ---------------------------------------------------
const EPORT = {};
function estop() {
  const E = ESTOP;
  const c = [E.x + E.w / 2, E.y + E.d / 2];
  const screws = byDepth(k.corners(E, 6));
  slab("estop.box", E, 0, 46, { tone: "hi", steps: 5, bevel: 0.9, inner: L(k.sideSeam(E, 8, P, 5), { tone: "faint" }), extra: L(screws.map(([x, y]) => k.ring(x, y, 2, 46, P, 14)).join(""), { tone: "lo" }) + D(screws.map(([x, y]) => at([x, y, 46])), { size: 0.5 }) });
  put({ name: "estop.collar", at: [c[0], c[1], 46], svg: S(k.cylinder(c[0], c[1], 16, 46, 5, P, 40, 0.4), { tone: "mid" }), shapes: [A.cylinder(c[0], c[1], 16, 46, 5)] });
  put({ name: "estop.stem", at: [c[0], c[1], 51], svg: S(k.cylinder(c[0], c[1], 9, 51, 7, P, 32), { tone: "lo", crease: "none" }), shapes: [A.cylinder(c[0], c[1], 9, 51, 7)] });
  const Fm = VERT(c[0], c[1], 0);
  const profile = [[58, 21], [62, 21, 1], [64.5, 18, 1], [66, 12, 1], [66.6, 0]];
  put({ name: "estop.mushroom", at: [c[0], c[1], 62], svg: S(G.solidOf(profile, Fm, P), { tone: "hi", crease: "lo", inner: L(k.knurl(c[0], c[1], 21, 58.4, 61.6, 44, P, 0.35), { tone: "lo" }) }), shapes: [A.solid(Fm, profile)] });
  const Fg = frameOf([c[0], E.y, 5 + BOX_H / 2], [0, -1, 0], [0, 0, 1]);
  put({ name: "estop.gland", at: [c[0], E.y - 3.5, 5 + BOX_H / 2], svg: S(G.disc(0, 7, 5.5, Fg, P), { tone: "lo" }), shapes: [A.disc(Fg, 0, 7, 5.5)] });
  EPORT.out = { o: [c[0], E.y - 7, 5 + BOX_H / 2] };
}

// ---- the tablet running the live viewer (tools/viewer.py on :8080) ----------------------------------
const TAB = (() => {
  const c = [436, -238, 5 + 60 * Math.cos((16 * Math.PI) / 180) + 7 * Math.sin((16 * Math.PI) / 180)];
  const back = (16 * Math.PI) / 180;                         // leans back 16°
  const a = (AZIMUTH * Math.PI) / 180;
  const across = [Math.sin(a), -Math.cos(a), 0];
  const flat = [Math.cos(a), Math.sin(a), 0];
  const n = add3(mul3(flat, Math.cos(back)), [0, 0, Math.sin(back)]);
  const up = unit3(cross3(across, n));
  return { c, n, u: across, v: up, w: 172, h: 120, t: 7, flat };
})();
function tablet() {
  const { c, n, u, v, w, h, t, flat } = TAB;
  const Ft = { o: c, a: n, u, v };
  const body = G.prismOf(Ft, rect(0, 0, w, h), -t, 0, P);
  const corner = (x, y, s = 0.2) => at(add3(add3(add3(c, mul3(u, x)), mul3(v, y)), mul3(n, s)));
  const glass = k.pathOf([corner(-w / 2 + 6, -h / 2 + 6), corner(w / 2 - 6, -h / 2 + 6), corner(w / 2 - 6, h / 2 - 6), corner(-w / 2 + 6, h / 2 - 6)], true);
  put({ name: "tab.body", layer: "back", at: c, svg: S(body, { tone: "mid", crease: "lo" }) + `<path class="wc-glass" d="${glass}"/><g class="wc-screen" transform="matrix(${screenMatrix().join(" ")})">${screenSvg()}</g>`, shapes: [A.prism(Ft, rect(0, 0, w, h), -t, 0)] });
  // the stand: a base on the bench and a leg propping the tablet's back
  const bottom = add3(c, mul3(v, -h / 2));
  const base = { x: bottom[0] - 74, y: bottom[1] - 44, w: 148, d: 52, r: 6 };
  slab("tab.stand", base, 0, 5, { tone: "lo", steps: 4, bevel: 0.4, layer: "back" });
}

// the viewer page on the tablet's glass: three live tiles, as tools/viewer.py lays them out
// (local mm on the glass, y down; the moving weld images are filled in by the page)
const TILE = { w: 48, h: 30, y: -32, xs: [-52, 0, 52] };
function screenSvg() {
  const r = (x, y, w, h, cls, rx = 0.8) => `<rect class="${cls}" x="${x}" y="${y}" width="${w}" height="${h}" rx="${rx}"/>`;
  const t = (x, y, text, size = 3, tone = "mid", anchor = "start") => `<text class="iso-face-text" data-tone="${tone}" x="${x}" y="${y}" font-size="${size}" text-anchor="${anchor}">${text}</text>`;
  const clips = TILE.xs.map((x, i) => `<clipPath id="wc-tile${i}"><rect x="${x - TILE.w / 2}" y="${TILE.y}" width="${TILE.w}" height="${TILE.h}"/></clipPath>`).join("");
  let svg = `<defs>${clips}</defs>` + r(-80, -54, 160, 108, "wc-page", 1.5) + r(-80, -54, 160, 9, "wc-bar", 1.5) + t(-76, -48, "192.168.94.94:8080", 3.6, "hi");
  const names = ["Arducam · 960×600", "D405 colour", "D405 depth · tap to measure"];
  TILE.xs.forEach((x, i) => {
    svg += r(x - TILE.w / 2, TILE.y, TILE.w, TILE.h, "wc-tile", 0.4);
    if (i < 2) {
      svg += `<g clip-path="url(#wc-tile${i})"><g class="wc-scroll" data-tile="${i}"></g></g>`;
    } else {
      const bands = Array.from({ length: 7 }, (_, j) => r(x - TILE.w / 2 + j * (TILE.w / 7), TILE.y, TILE.w / 7 + 0.05, TILE.h, `wc-depth${j}`, 0)).join("");
      svg += `<g clip-path="url(#wc-tile${i})">${bands}<rect class="wc-hole" x="${x - 4}" y="${TILE.y + 9}" width="7" height="9"/></g>`;
    }
    svg += `<rect class="wc-frame" x="${x - TILE.w / 2}" y="${TILE.y}" width="${TILE.w}" height="${TILE.h}"/>` + t(x - TILE.w / 2, TILE.y + TILE.h + 4.4, names[i], 2.7, "mid");
  });
  svg += r(-14, 14, 28, 9, "wc-button", 2) + t(0, 20, "Capture", 3.4, "hi", "middle");
  svg += t(-76, 34, "stills → data/compare/", 2.8, "lo") + t(76, 34, "D405 30 fps", 2.8, "lo", "end");
  return svg;
}

function screenMatrix() {
  const { c, n, u, v } = TAB;
  const o = add3(c, mul3(n, 0.3));
  const p0 = at(o);
  const px = at(add3(o, u));
  const py = at(add3(o, mul3(v, -1)));
  return [px[0] - p0[0], px[1] - p0[1], py[0] - p0[0], py[1] - p0[1], p0[0], p0[1]].map((x) => +x.toFixed(4));
}

// ---- the ChArUco calibration board (7 × 5, 31.79 mm squares, DICT_5X5_100) ---------------------
function charucoBoard() {
  const sq = 31.79;
  const top = BOARD_T;
  const cellAt = (c, r) => ({ x: BOARD.x + c * sq, y: BOARD.y + r * sq });
  const quad = (x0, y0, x1, y1) => k.pathOf([at([x0, y0, top]), at([x1, y0, top]), at([x1, y1, top]), at([x0, y1, top])], true);
  const ink = [];
  for (const [c, r] of CHARUCO.black) {
    const p = cellAt(c, r);
    ink.push(quad(p.x, p.y, p.x + sq, p.y + sq));
  }
  const m = 23.31;
  const cell = m / 7;
  for (const { c, r, bits } of CHARUCO.markers) {
    const p = cellAt(c, r);
    const ox = p.x + (sq - m) / 2;
    const oy = p.y + (sq - m) / 2;
    for (let i = 0; i < 7; i++) for (let j = 0; j < 7; j++) if (bits[i][j]) ink.push(quad(ox + j * cell, oy + i * cell, ox + (j + 1) * cell, oy + (i + 1) * cell));
  }
  slab("board", BOARD, 0, BOARD_T, { tone: "hi", steps: 2, bevel: 0.3, extra: `<path class="wc-ink" d="${ink.join("")}"/>` + L(k.planOutline(BOARD, top, P, 2), { tone: "lo" }) });
}

// ---- bench cables ------------------------------------------------------------------------------
const CABLES = [];
function cable(name, waypoints, r, { bend = 8, gaps = [], free = [false, false], tone = "hi", bias = 0, layer = "back", bundle } = {}) {
  const points = G.fillet(waypoints, Math.max(bend * r, 6), 10, { P, tube: r });
  const total = G.pathLength(points);
  const pieces = tubePieces(points, r, P, { maxLength: 40, gaps });
  for (const [index, piece] of pieces.entries()) R.put({ name: `${name}#${index}`, layer, at: piece.mid, bias, svg: tubeSvg(piece, { tone }), route: name, chunk: index, shapes: [R.solid(A.tube(piece.points, piece.radii, { name: `${name}#${index}`, owner: name, limp: true, bundle, touch: ["stand.bench"] }))] });
  R.route(name, points, r, { owner: name, limp: true, gaps, free, bundle, touch: ["stand.bench"] });
  CABLES.push({ name, r, total });
  return points;
}
function plug(name, o, axis, w, h, len, owner) {
  const Fp = frameOf(o, axis, [0, 0, 1]);
  put({ name, at: G.pointOf(Fp, len / 2, 0, 0), svg: S(G.prismOf(Fp, rect(0, 0, w, h), 0, len, P), { tone: "lo", crease: "faint" }), shapes: [A.prism(Fp, rect(0, 0, w, h), 0, len)] });
  return G.pointOf(Fp, len, 0, 0);
}
// where the camera cables, dressed down the arm's far side, come off the robot onto the bench
const USB_BOX = { a: (150 * Math.PI) / 180, z0: 18, z1: 60 };
function usbJunction() {
  const n = [Math.cos(USB_BOX.a), Math.sin(USB_BOX.a), 0];
  const F = { o: add3(mul3(n, BASE_R - 0.5), [0, 0, (USB_BOX.z0 + USB_BOX.z1) / 2]), a: n, u: [0, 0, 1], v: cross3(n, [0, 0, 1]) };
  const poly = rect(0, 0, USB_BOX.z1 - USB_BOX.z0, 30);
  const screws = [[-14, -10], [14, 10]];
  put({ name: "stand.base.usb", at: G.pointOf(F, 7, 0, 0), svg: S(G.prismOf(F, poly, 0, 14, P), { tone: "hi", crease: "lo" }) + D(screws.map(([a, b]) => at(G.pointOf(F, 14, a, b))), { size: 0.5 }), shapes: [A.prism(F, poly, 0, 14)] });
  return { F, s1: 14 };
}
// a cable clip screwed to the bench edge, where a supply cable leaves the bench
function edgeClip(name, x, y, d) {
  const plan = { x: x - 8, y, w: 16, d, r: 1.2 };
  slab(name, plan, 0, 9, { tone: "lo", steps: 2, bevel: 0.3, extra: D([at([x, y + d / 2, 9])], { size: 0.5 }) });
}
function benchCables() {
  const ramp = (p, d, len, z) => [p[0] + d[0] * len, p[1] + d[1] * len, z];
  // robot cable: control box → robot base socket, climbing to it across the screen
  const rc = 4.5;
  const n = [Math.cos(BASE_PORT.a), Math.sin(BASE_PORT.a), 0];
  const baseEnd = add3(mul3(n, BASE_R + 8.55), [0, 0, BASE_PORT.z]);
  const shellA = plug("cable.robot.shell.box", BPORT.robot.o, [0, 1, 0], 18, 18, 14, "cable.robot");
  const shellB = plug("cable.robot.shell.base", baseEnd, n, 18, 18, 14, "cable.robot");
  cable("cable.robot", [shellA, [shellA[0], shellA[1] + 16, shellA[2]], [shellB[0] - 8, shellB[1] - 30, BASE_PORT.z + 4], ramp(shellB, n, 16, BASE_PORT.z), shellB], rc, { bend: 5 });
  // Ethernet: out of the Jetson's RJ45, round and straight back to the control box
  const e1 = plug("cable.eth.shell.jet", [JPORT.eth.face, JPORT.eth.y, JPORT.eth.z], [1, 0, 0], 11, 12, 16, "cable.eth");
  const e2 = plug("cable.eth.shell.box", BPORT.eth.o, [0, 1, 0], 12, 11, 16, "cable.eth");
  cable("cable.eth", [e1, [e1[0] + 8, e1[1], e1[2]], [e2[0], e1[1], 2.4], [e2[0], e2[1] + 36, 2.4], [e2[0], e2[1] + 14, e2[2]], e2], 2.4, { bend: 6 });
  // E-stop: straight back into the control box
  const s1 = plug("cable.estop.shell.box", BPORT.estop.o, [0, 1, 0], 13, 13, 12, "cable.estop");
  cable("cable.estop", [EPORT.out.o, s1], 2.6, { bend: 3 });
  // control box mains: off the back edge
  edgeClip("stand.clip.mains", BOX.x + 30, BENCH.y + 1, 8);
  cable("cable.mains", [[BOX.x + 30, BOX.y, 25], [BOX.x + 30, BOX.y - 6, 25], [BOX.x + 30, BOX.y - 10, 3.5], [BOX.x + 30, BENCH.y + 9, 3.5]], 3.5, { bend: 2 });
  // Jetson 19 V barrel: out to the side and off the front edge
  const d1 = plug("cable.dc.shell", [JPORT.dc.face, JPORT.dc.y, JPORT.dc.z], [1, 0, 0], 9, 9, 12, "cable.dc");
  edgeClip("stand.clip.dc", d1[0] + 30, BENCH.y + BENCH.d - 9, 8);
  cable("cable.dc", [d1, [d1[0] + 10, d1[1], d1[2]], [d1[0] + 26, d1[1], 1.9], [d1[0] + 30, d1[1] + 40, 1.9], [d1[0] + 30, BENCH.y + BENCH.d - 9, 1.9]], 1.9, { bend: 8 });
  // the two camera cables: from the junction on the robot base to two USB-A ports
  const jb = usbJunction();
  [JPORT.usbA, JPORT.usbB].forEach((p, i) => {
    const shell = plug(`cable.usb${i}.shell.jet`, [p.face, p.y, p.z], [1, 0, 0], 7, 11, 14, `cable.usb${i}`);
    const off = i * 12 - 6;
    const s = G.pointOf(jb.F, jb.s1, -12, off);
    cable(`cable.usb${i}`, [s, add3(s, mul3(jb.F.a, 10)), [s[0] + jb.F.a[0] * 22, s[1] + jb.F.a[1] * 22, 2.2], [shell[0] + (i ? 54 : 32), shell[1], 2.2], [shell[0] + 14, shell[1], shell[2]], shell], 2.2, { bend: 6, bundle: "usb" });
  });
}

const q0 = M.ik(M.viewAt(0, 0), [0, 50, 74, 0, 69, 0]);

// ---- the run: joint tables solved here, played by tour.mjs -------------------------------------
const GRID = { s0: M.SEAM.from, ds: 4, ns: Math.round((M.SEAM.to - M.SEAM.from) / 4) + 1, t0: -18, dt: 3, nt: 13 };
GRID.q = (() => {
  const rows = new Map();
  let centre = q0;
  const order = [6, 7, 8, 9, 10, 11, 12, 5, 4, 3, 2, 1, 0];
  const solved = [];
  for (const j of order) {
    const tilt = GRID.t0 + j * GRID.dt;
    const prev = j > 6 ? rows.get(j - 1) : j < 6 ? rows.get(j + 1) : null;
    const seed = prev ? prev[(GRID.ns - 1) / 2] : q0;
    const mid = (GRID.ns - 1) / 2;
    const row = new Array(GRID.ns);
    row[mid] = M.ik(M.viewAt(0, tilt), seed);
    for (let i = mid + 1; i < GRID.ns; i++) row[i] = M.ik(M.viewAt(GRID.s0 + i * GRID.ds, tilt), row[i - 1]);
    for (let i = mid - 1; i >= 0; i--) row[i] = M.ik(M.viewAt(GRID.s0 + i * GRID.ds, tilt), row[i + 1]);
    if (row.some((q) => !q)) throw new Error(`IK failed on the tilt ${tilt} row`);
    rows.set(j, row);
    solved.push(row);
    void centre;
  }
  const flat = [];
  for (let j = 0; j < GRID.nt; j++) for (const q of rows.get(j)) flat.push(...q.map((v) => +v.toFixed(4)));
  return flat;
})();
const draft = makeTour({ grid: GRID, segments: [] });
const viewOf = (s, tilt) => M.viewAt(s, tilt);
const along = (s, tilt, ds, seed) => {
  const knots = [];
  let q = seed;
  const [d0, d1] = ds;
  const n = Math.max(2, Math.round(Math.abs(d1 - d0) / 5) + 1);
  for (let i = 0; i < n; i++) {
    const d = d0 + ((d1 - d0) * i) / (n - 1);
    q = M.ik(M.alongTool(viewOf(s, tilt), d), q);
    if (!q) throw new Error(`IK failed ${d} mm along the tool`);
    knots.push(q.map((v) => +v.toFixed(4)));
  }
  return knots;
};
const SPAN = M.SEAM.to - M.SEAM.from;
const SCAN = SPAN / SCAN_SPEED + RAMP;
const ROI_T = (() => {
  let lo = 0;
  let hi = SCAN;
  for (let i = 0; i < 50; i++) {
    const mid = (lo + hi) / 2;
    if (M.SEAM.from + trapezoid(mid, SPAN) < ROI_S) lo = mid;
    else hi = mid;
  }
  return lo;
})();
const ROI_FRAME = Math.floor(ROI_T * FPS);
const qRoi = draft.gridAt(M.SEAM.from + trapezoid(ROI_T, SPAN), 0).map((v) => +v.toFixed(4));   // the logged joints at that frame
const qEnd = draft.gridAt(M.SEAM.to, -18);
const roiPose = M.tcpOf(qRoi);
const alongFrom = (T0, ds, seed) => {
  const knots = [];
  let q = seed;
  const n = Math.max(2, Math.round(Math.abs(ds[1] - ds[0]) / 5) + 1);
  for (let i = 0; i < n; i++) {
    const d = ds[0] + ((ds[1] - ds[0]) * i) / (n - 1);
    q = M.ik(M.alongTool(T0, d), q);
    if (!q) throw new Error(`IK failed ${d} mm along the tool`);
    knots.push(q.map((v) => +v.toFixed(4)));
  }
  return knots;
};
const qRoiClear = alongFrom(roiPose, [0, -60], qRoi).at(-1);
// lift off at tilt 0: backing away at the pass-3 tilt would fold the wrist to J5 ≈ 118°, into the forearm
const qClearEnd = alongFrom(M.viewAt(M.SEAM.to, 0), [0, -60], draft.gridAt(M.SEAM.to, 0)).at(-1);
const closer = alongFrom(roiPose, [0, 40], qRoi);
const leave = alongFrom(roiPose, [40, -70], closer.at(-1));
const qStart = draft.gridAt(M.SEAM.from, 0);
const SEGMENTS = [
  { kind: "hold", dur: 1.4, s: M.SEAM.from, tilt: 0, phase: "ready" },
  { kind: "scan", dur: SCAN, from: M.SEAM.from, to: M.SEAM.to, tilt: 0, pass: 1, phase: "scan" },
  { kind: "swing", dur: 1.8, s: M.SEAM.to, from: 0, to: 18, pass: 2, phase: "swing" },
  { kind: "scan", dur: SCAN, from: M.SEAM.to, to: M.SEAM.from, tilt: 18, pass: 2, phase: "scan" },
  { kind: "swing", dur: 3.2, s: M.SEAM.from, from: 18, to: -18, pass: 3, phase: "swing" },
  { kind: "scan", dur: SCAN, from: M.SEAM.from, to: M.SEAM.to, tilt: -18, pass: 3, phase: "scan" },
  { kind: "pause", dur: 2.0, q: qEnd, phase: "detect" },
  { kind: "joint", dur: 2.0, from: qEnd, to: qClearEnd, phase: "clear" },
  { kind: "joint", dur: 2.4, from: qClearEnd, to: qRoiClear, phase: "revisit" },
  { kind: "joint", dur: 1.4, from: qRoiClear, to: qRoi, phase: "revisit" },
  { kind: "pause", dur: 0.4, q: qRoi, phase: "settle" },
  { kind: "knots", dur: 1.2, knots: closer, d0: 0, d1: 40, phase: "closer" },
  { kind: "pause", dur: 1.6, q: closer.at(-1), phase: "still" },
  { kind: "knots", dur: 1.6, knots: leave, d0: 40, d1: -70, phase: "leave" },
  { kind: "joint", dur: 2.6, from: leave.at(-1), to: qStart, phase: "return" },
];
const TOUR = makeTour({ grid: GRID, segments: SEGMENTS });
const valuesOf = (q) => ({ j1: q[0] - q0[0], j2: q[1] - q0[1], j3: q[2] - q0[2], j4: q[3] - q0[3], j5: q[4] - q0[4], j6: q[5] - q0[5] });
export const follow = (values) => values;
// The engine fixes cross-link separating planes from 24 samples of poses(), so poses() is the run's
// 24 key states: every segment's ends (joints move monotonically within a segment) and the
// middles of the longest moves. The orbit audit then checks every frame of the run.
const KEY_TIMES = (() => {
  const times = [...TOUR.starts];
  const long = SEGMENTS.map((seg, i) => [seg.dur, i]).sort((x, y) => y[0] - x[0]).slice(0, 24 - times.length);
  for (const [, i] of long) times.push(TOUR.starts[i] + SEGMENTS[i].dur / 2);
  return times.sort((x, y) => x - y);
})();
export const poses = () => KEY_TIMES.map((t) => valuesOf(TOUR.at(t).q));
const RANGES = (() => {
  const lo = [1e9, 1e9, 1e9, 1e9, 1e9, 1e9];
  const hi = lo.map((v) => -v);
  const take = (q) => valuesOf(q) && Object.values(valuesOf(q)).forEach((v, i) => { lo[i] = Math.min(lo[i], v); hi[i] = Math.max(hi[i], v); });
  for (let t = 0; t < TOUR.duration; t += 1 / 60) take(TOUR.at(t).q);
  for (let i = 0; i < GRID.q.length; i += 6) take(GRID.q.slice(i, i + 6));
  return lo.map((v, i) => [Math.floor(v - 1), Math.ceil(hi[i] + 1)]);
})();

// ---- the arm (live: six nested joint groups) --------------------------------------------------------
export const T = turning(P, { recorder: R });
const F = M.frames(q0);
const O = F.map((f) => f.t);
const Z = F.map((f) => M.col(f, 2));
const X = F.map((f) => M.col(f, 0));
const Y = F.map((f) => M.col(f, 1));
const groups = [];
groups[0] = T.group("j1", { hinge: { point: [0, 0, 0], axis: [0, 0, 1], range: RANGES[0] } });   // J1 turns ±13° here; a hinge carries its real range
for (let i = 1; i < 6; i++) groups[i] = T.group(`j${i + 1}`, { parent: groups[i - 1], hinge: { point: O[i], axis: Z[i], range: RANGES[i] } });
const arm = T.layer("arm");
T.stack(["back", "arm", "front"]);
const [j1, j2, j3, j4, j5, j6] = groups;
const LAT = mul3(Z[1], -1);                    // the upper arm sits on the far side of the arm plane
const P4 = add3(O[2], mul3(X[2], 87));          // the forearm axis's foot at the elbow

const round = (name, group, Fr, s0, s1, r, extra = {}) => T.round({ name, group, layer: arm, F: Fr, s0, s1, r, tone: "hi", crease: "lo", ...extra });
const prism = (name, group, Fr, polygon, s0, s1, extra = {}) => T.prism({ name, group, layer: arm, F: Fr, polygon, s0, s1, tone: "hi", crease: "lo", ...extra });
const bolts = (s, r, count, phase = 0.5) => [{ s, r, count, phase, size: 0.5, tone: "mid", fade: [0.04, 0.42] }];

function armParts() {
  // J1: the turret on the base, and the shoulder housing on the turret
  round("arm.turret", j1, VERT(0, 0), BASE_TOP, O[1][2] - 46, 54, { details: { seams: [BASE_TOP + 5] } });
  const shoulder = frameOf(O[1], LAT, [0, 0, 1]);
  round("arm.shoulder", j1, shoulder, -56, 28, 46, { details: { seams: [-50, 22] } });
  round("arm.shoulder.cap", j1, shoulder, -60, -56, 36, { tone: "mid", details: { bolts: bolts(-60, 29, 6) } });
  // J2: knuckle and the upper arm, a flat link on the far side of the arm plane
  round("arm.j2.knuckle", j2, shoulder, 28, 34, 44, { tone: "mid" });
  const upper = frameOf(O[1], LAT, sub3(O[2], O[1]));
  const UL = len3(sub3(O[2], O[1]));
  prism("arm.upper", j2, upper, hull([...circle(0, 0, 44, 48), ...circle(UL, 0, 40, 48)]), 34, 70, { bevel: 1.2 });
  round("arm.upper.cap2", j2, shoulder, 70, 73, 32, { tone: "mid", details: { bolts: bolts(73, 25, 6) } });
  const elbowOut = frameOf(O[2], LAT, [0, 0, 1]);
  round("arm.upper.cap3", j2, elbowOut, 70, 73, 30, { tone: "mid", details: { bolts: bolts(73, 23, 6) } });
  // J3: knuckle, the elbow casting (the joint round and the forearm's seat) and the forearm root
  round("arm.j3.knuckle", j3, elbowOut, 30, 34, 40, { tone: "mid" });
  const elbow = frameOf(O[2], LAT, X[2]);
  const w = dot3(Z[3], elbow.v) > 0 ? 1 : -1;     // the forearm runs along +w·v
  const dShape = hull([...circle(0, 0, 40, 48), ...circle(109, -32 * w, 8, 16), ...circle(109, 32 * w, 8, 16), [57, 40 * w], [0, 40 * w]]);
  prism("arm.elbow", j3, elbow, dShape, -34, 30, { bevel: 1 });
  round("arm.elbow.cap", j3, frameOf(O[2], mul3(LAT, -1), [0, 0, 1]), 34, 37, 32, { tone: "mid", details: { bolts: bolts(37, 25, 6) } });
  const fore = frameOf(P4, Z[3], LAT);
  round("arm.forearm", j3, fore, 40, 108, 30, { details: { seams: [46] } });
  // J4: the roll joint and the far forearm
  round("arm.j4.ring", j4, fore, 108, 112, 31.5, { tone: "mid" });
  const reach = len3(sub3(O[3], P4));
  round("arm.forearm2", j4, fore, 112, reach - 40, 30, { details: { seams: [reach - 46] } });
  // J5 housing: a D on the forearm's end, its round on the wrist axis
  const wrist = frameOf(O[4], Z[4], mul3(Z[3], -1));
  prism("arm.j5.housing", j4, wrist, hull([...circle(0, 0, 34, 48), ...circle(34, -24, 6, 12), ...circle(34, 24, 6, 12)]), -30, 30, { bevel: 0.8 });
  // J5: the fork round the housing, the J6 module between its plates
  const fork = frameOf(O[4], Z[4], Z[5]);
  const plate = hull([...circle(0, 0, 34, 48), ...circle(26, 0, 26, 40)]);
  prism("arm.fork+", j5, fork, plate, 30, 36, { bevel: 0.6 });
  prism("arm.fork-", j5, fork, plate, -36, -30, { bevel: 0.6 });
  round("arm.fork.cap+", j5, frameOf(O[4], Z[4], Z[5]), 36, 39, 24, { tone: "mid", details: { bolts: bolts(39, 18, 4) } });
  round("arm.fork.cap-", j5, frameOf(O[4], mul3(Z[4], -1), Z[5]), 36, 39, 24, { tone: "mid", details: { bolts: bolts(39, 18, 4) } });
  const tool = frameOf(O[5], Z[5], X[5]);
  const back = (s) => s - 61.5;                    // s along tool z, measured from the wrist centre
  round("arm.j6", j5, tool, back(36), back(54), 30, { details: { seams: [back(40)] } });
  // J6: the Ø75 tool flange
  round("arm.flange", j6, tool, back(54), 0, 37.5, { tone: "mid" });
}

function bracket() {
  const B = M.BRACKET;
  const tool = frameOf(O[5], Z[5], X[5]);
  const g = j6;
  const at6 = (x, y) => add3(add3(O[5], mul3(X[5], x)), mul3(Y[5], y));
  const F6 = (x, y) => ({ o: at6(x, y), a: Z[5], u: X[5], v: Y[5] });
  round("cam.adapter", g, tool, 0, B.adapter.t, B.adapter.r, { tone: "mid", details: { seams: [2] } });
  const p0 = B.plate.at;
  const p1 = p0 + B.plate.t;
  const screws = [[-50, -20], [50, -20], [-50, 20], [50, 20]].map(([x, y]) => ({ at: [p1, x, y], normal: Z[5], size: 0.5, tone: "mid" }));
  prism("cam.plate", g, tool, rect(0, 0, B.plate.w, B.plate.h), p0, p1, { round: 3, steps: 3, bevel: 0.6, tone: "mid", details: { dots: screws } });
  // D405: 42 × 42 × 23, back face on the plate; two imagers 18 apart
  const d = B.d405;
  const lenses = [-9, 9].map((x) => ({ at: [p1 + d.d, d.x + x, 6], normal: Z[5], r: 3.4, tone: "lo" }));
  prism("cam.d405", g, tool, rect(d.x, 0, d.w, d.h), p1, p1 + d.d, { round: 4, steps: 4, bevel: 0.8, tone: "hi", details: { rings: lenses, dots: [-9, 9].map((x) => ({ at: [p1 + d.d, d.x + x, 6], normal: Z[5], size: 0.9, tone: "lo" })) } });
  // Arducam B0495: rear board on standoffs, front board, lens holder, M12 lens
  const a = B.arducam;
  const r0 = p1 + a.standoff;
  const corners = [[-14, -14], [14, -14], [-14, 14], [14, 14]];
  for (const [x, y] of corners) round(`cam.standoff.${x}.${y}`, g, F6(a.x + x, y), p1, r0, 1.6, { tone: "mid" });
  prism("cam.ard.rear", g, tool, rect(a.x, 0, a.board, a.board), r0, r0 + 1.6, { round: 2.5, steps: 3, tone: "mid" });
  for (const [x, y] of corners) round(`cam.post.${x}.${y}`, g, F6(a.x + x, y), r0 + 1.6, r0 + a.stack - 1.6, 1.4, { tone: "lo" });
  prism("cam.ard.front", g, tool, rect(a.x, 0, a.board, a.board), r0 + a.stack - 1.6, r0 + a.stack, { round: 2.5, steps: 3, tone: "mid" });
  // USB: a right-angle USB-C plug between the plate and the Arducam's rear board, and the D405's plug on its side
  prism("cam.ard.usb", g, tool, [[a.x - 4.5, 4], [a.x + 4.5, 4], [a.x + 4.5, 27], [a.x - 4.5, 27]], p1, r0 - 1.5, { round: 1, steps: 2, tone: "lo", bevel: 0.3 });
  prism("cam.d405.usb", g, tool, [[d.x - d.w / 2 - 13, -4.5], [d.x - d.w / 2, -4.5], [d.x - d.w / 2, 4.5], [d.x - d.w / 2 - 13, 4.5]], p1 + 8, p1 + 15, { round: 1, steps: 2, tone: "lo", bevel: 0.3 });
  const front = r0 + a.stack;
  prism("cam.ard.holder", g, tool, rect(a.x, 0, a.holder, a.holder), front, front + 8, { round: 1.5, steps: 2, tone: "mid", bevel: 0.4 });
  round("cam.ard.lens", g, F6(a.x, 0), front + 8, front + a.lens - 2.5, a.lensR, { tone: "hi", details: { seams: [front + 12] } });
  round("cam.ard.ring", g, F6(a.x, 0), front + a.lens - 2.5, front + a.lens, a.lensR + 0.8, { tone: "hi", details: { ribs: { s0: front + a.lens - 2.3, s1: front + a.lens - 0.2, count: 36, seams: true, fade: [0.1, 0.55], tone: "lo" }, rings: [{ at: [front + a.lens, 0, 0], normal: Z[5], r: 4.2, tone: "lo" }] } });
}

bench();
robotBase();
workpiece();
jetson();
controlBox();
estop();
charucoBoard();
tablet();
benchCables();
armParts();
bracket();

const out = T.build({ statics: R.items, poses, verify: argv.includes("--verify") });
A.settle(R, P);
const layerOf = (name) => R.items.filter((item) => item.layer === name).sort((a, b) => a.key - b.key).map((item) => item.svg);
if (argv.includes("--audit")) {
  A.auditOrExit(R, P);
  TA.orbitOrExit(T, R, P, { sweep: { at: (t) => valuesOf(TOUR.at(t).q), from: 0, to: TOUR.duration, frame: 1 / 30, unit: " s" }, poses: poses(), quick: argv.includes("--quick"), only: argv.find((a) => a.startsWith("--only="))?.slice(7).split(",") ?? null });
}

// ---- the page ----------------------------------------------------------------------------------
const roiRing = (() => {
  const pts = Array.from({ length: 40 }, (_, j) => {
    const t = (j / 40) * 2 * Math.PI;
    return at(onBead(ROI_S + 5.5 * Math.cos(t), CROWN + 0.62 * Math.sin(t)));
  });
  return k.pathOf(pts, true);
})();
const overlay = `<g class="wc-overlay"><path class="wc-roi" d="${roiRing}"/><path class="wc-sight" d=""/><circle class="wc-aim" r="1.6" cx="-9" cy="-9"/></g>`;
const LABEL = "A UFACTORY Lite 6 arm on an optical breadboard holds an Arducam and a RealSense D405 on one bracket over a T-joint fillet weld, wired to a Jetson Orin Nano beside the arm's control box and emergency stop, with a ChArUco calibration board on the bench.";
const backSvg = k.figureSvg({ width: WIDTH, height: HEIGHT, label: LABEL, body: [...layerOf("back"), overlay] });
const staticSvg = (list) => `<svg xmlns="http://www.w3.org/2000/svg" class="iso-svg iso-layer" viewBox="0 0 ${WIDTH} ${HEIGHT}" aria-hidden="true">${list.join("")}</svg>`;
const stage = `<div class="iso-stage iso-turn wc-stage" tabindex="0" role="slider" aria-label="Weld inspection cell: drag along the weld or use the arrow keys to move the camera, 1 2 3 choose the pass, R revisits the flagged spot, Space resumes the run" aria-valuemin="${M.SEAM.from}" aria-valuemax="${M.SEAM.to}" aria-valuenow="${M.SEAM.from}">${backSvg}${out.layerSvg("arm", { width: WIDTH, height: HEIGHT })}${staticSvg(layerOf("front"))}<div class="wc-live" aria-live="polite"></div></div>`;
const DATA = {
  turn: out.data, q0, grid: GRID, tile: TILE, segments: SEGMENTS, roi: { s: ROI_S, frame: ROI_FRAME, pass: 1 },
  tcp: M.TCP.t, standoff: M.STANDOFF, lensFront: M.LENS_FRONT, seam: M.SEAM, work: W, bead: { c: BEAD.c, R0: BEAD.R0, s: BEAD_S },
  P: { origin: P.origin, scale: P.scale, azimuth: P.azimuth, elevation: P.elevation, width: WIDTH }, dh: M.DH,
};
const TOUR_SRC = readFileSync(join(HERE, "tour.mjs"), "utf8").replace(/^export /gm, "");
const RIG_SRC = readFileSync(join(HERE, "rig.mjs"), "utf8").replace(/^export /gm, "").replace(/^import .*$/gm, "");
const LIVE = readFileSync(join(HERE, "live.js"), "utf8").replace("__DATA__", JSON.stringify(DATA));
const CSS = out.css + "\n" + readFileSync(join(HERE, "cell.css"), "utf8");
const body = k.plateHtml({
  fig: "Fig 1",
  title: "Weld inspection cell",
  hint: "Drag along the weld · 1 2 3 passes · R revisit",
  readout: "ready",
  keys: [
    { mark: "raised", label: "UFACTORY Lite 6 on the breadboard: 6 joints, 440 mm reach" },
    { mark: "edge", label: "Bracket on the Ø75 flange: Arducam B0495 and RealSense D405" },
    { mark: "lit", label: "The Arducam's line of sight; the TCP sits on it 120 mm out" },
    { mark: "flat", label: "Jetson Orin Nano: plans, records, detects, drives the arm over Ethernet" },
    { mark: "dim", label: "Control box (its own 24 V supply) and the E-stop" },
    { mark: "dotted", label: "ChArUco 7 × 5 board for hand-eye; the tablet shows the :8080 viewer" },
  ],
  caption: `Three passes scan the weld at ${SCAN_SPEED} mm/s while the robot log (100 Hz) and the video (30 fps) are stamped on the Jetson's one clock: along the bisector, then tilted 18° toward the flange and 18° toward the web. A flagged frame's six logged joint angles bring the camera back in joint space; it settles, steps 40 mm closer along its axis and keeps the sharpest of five stills. Both cameras share the Jetson's USB3 hub, so the Arducam streams at 960 × 600 to let the D405 hold 30 fps. The web stands on the robot's side so J5 stays between 48° and 93°, clear of the wrist singularity. The seam, plates and 120 mm standoff are example values the project hasn't fixed yet.`,
  body: `<style>${CSS}</style>${stage}`,
});
const page = k.pageHtml({ title: "Weld inspection cell", theme, body, script: turnScript() + RIG_SRC + TOUR_SRC + LIVE, width: 780 });
const name = `weld-cell${theme === "light" ? "-light" : ""}.html`;
writeFileSync(join(HERE, name), page);
console.log(`wrote ${name} · ${(page.length / 1024).toFixed(0)} KB · ${out.report.parts} parts · ${R.items.length} static items · ${CABLES.length} bench cables · tour ${TOUR.duration.toFixed(1)} s · ROI frame ${ROI_FRAME} · build ${out.report.seconds.toFixed(1)} s`);
