// The inspection cell as a pure model, shared by the build and the page.
// Units: mm and degrees, robot base frame = world frame, z up (claude/claude.md "Conventions").

// UFACTORY Lite 6, modified D-H: [alpha(i-1), a(i-1), d(i), theta offset(i)].
// Shoulder 243.3 above the base, upper arm 200, elbow offset 87, forearm 227.6, wrist to flange 61.5.
export const DH = [
  [0, 0, 243.3, 0],
  [-90, 0, 0, -90],
  [180, 200, 0, -90],
  [90, 87, 227.6, 0],
  [90, 0, 0, 0],
  [-90, 0, 61.5, 0],
];
export const JOINT_LIMITS = [[-360, 360], [-150, 150], [-3.5, 300], [-360, 360], [-124, 124], [-360, 360]];

const RAD = Math.PI / 180;
const mat = (R, t) => ({ R, t });
const mul = (A, B) => ({
  R: [0, 1, 2].map((r) => [0, 1, 2].map((c) => A.R[r][0] * B.R[0][c] + A.R[r][1] * B.R[1][c] + A.R[r][2] * B.R[2][c])),
  t: [0, 1, 2].map((r) => A.R[r][0] * B.t[0] + A.R[r][1] * B.t[1] + A.R[r][2] * B.t[2] + A.t[r]),
});
const rotX = (deg) => { const c = Math.cos(deg * RAD), s = Math.sin(deg * RAD); return [[1, 0, 0], [0, c, -s], [0, s, c]]; };
const rotZ = (deg) => { const c = Math.cos(deg * RAD), s = Math.sin(deg * RAD); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; };
const I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
export const apply = (T, p) => [0, 1, 2].map((r) => T.R[r][0] * p[0] + T.R[r][1] * p[1] + T.R[r][2] * p[2] + T.t[r]);
export const col = (T, c) => [T.R[0][c], T.R[1][c], T.R[2][c]];

// Joint frames (origin on each joint axis, z along it) and the flange, for six joint angles.
export function frames(q) {
  let T = mat(I3, [0, 0, 0]);
  const out = [];
  for (let i = 0; i < 6; i++) {
    const [alpha, a, d, off] = DH[i];
    T = mul(T, mat(rotX(alpha), [0, 0, 0]));
    T = mul(T, mat(I3, [a, 0, 0]));
    T = mul(T, mat(rotZ(q[i] + off), [0, 0, 0]));
    T = mul(T, mat(I3, [0, 0, d]));
    out.push(T);
  }
  return out;
}
export const flangeOf = (q) => frames(q)[5];

// Fixed-axis XYZ roll/pitch/yaw of a rotation (R = Rz(yaw)·Ry(pitch)·Rx(roll)), as the xArm SDK reports.
export function rpyOf(R) {
  const pitch = Math.asin(-Math.max(-1, Math.min(1, R[2][0])));
  return [Math.atan2(R[2][1], R[2][2]) / RAD, pitch / RAD, Math.atan2(R[1][0], R[0][0]) / RAD];
}

// ---- the camera bracket ------------------------------------------------------------------
// An adapter disc on the Ø75 flange, a camera plate across it, the Arducam B0495 on one side and
// the D405 on the other, both looking along tool +z (docs/hardware/mounting-reference.md).
export const BRACKET = {
  adapter: { r: 37.5, t: 8 },                       // on the flange face, Ø75
  plate: { w: 112, h: 52, t: 5, at: 8 },            // camera plate, its back face 8 from the flange
  // 34 × 34 boards on 12 mm standoffs, so a right-angle USB-C plug fits behind the rear board;
  // the stack is 20.2 deep and the M12 lens (Ø14) reaches 20 ahead of the front board
  arducam: { x: 28, board: 34, standoff: 12, stack: 20.2, lens: 20, lensR: 7, holder: 20 },
  d405: { x: -30, w: 42, h: 42, d: 23, baseline: 18 },
};
// The TCP sits on the Arducam's optical axis at the working distance (claude.md, "TCP offset").
// The working distance is an open question in the project; 120 mm is assumed for this figure.
export const STANDOFF = 120;
export const LENS_FRONT = BRACKET.plate.at + BRACKET.plate.t + BRACKET.arducam.standoff + BRACKET.arducam.stack + BRACKET.arducam.lens;
export const TCP = mat(I3, [BRACKET.arducam.x, 0, LENS_FRONT + STANDOFF]);
export const tcpOf = (q) => mul(flangeOf(q), TCP);

// ---- the workpiece: a T-joint with one fillet weld --------------------------------------
// The seam runs along y in front of the robot. The web stands on the robot's side of it and the
// weld faces outward, so the arm reaches over the joint and looks back into the corner: that keeps
// J5 between about 48° and 93° on every pass, well clear of the wrist singularity at J5 = 0
// (a camera looking outward lines its axis up with the forearm on the steep pass).
// Plate sizes and seam length are open questions in the project; these are example values.
export const WORK = {
  x: 170,             // the web's weld-side face at x = 170, facing +x
  y0: -80, y1: 80,    // both plates run from y0 to y1; so does the weld
  plate: 6,           // both plates 6 thick
  flangeD: 120,       // the flange plate reaches 120 outward from the web face
  back: 48,           // and 48 past the web, toward the robot
  webH: 70,           // the web stands 70 above the flange plate
  leg: 6,             // fillet leg
};
export const SEAM = { from: WORK.y0 + 12, to: WORK.y1 - 12 };   // inspected stretch, craters included
// Weld frame at a seam point: y along the seam, n_flange up, n_web toward the weld side (+x).
export const N_FLANGE = [0, 0, 1];
export const N_WEB = [1, 0, 0];
export const BISECTOR = [Math.SQRT1_2, 0, Math.SQRT1_2];
export const seamPoint = (s) => [WORK.x, s, WORK.plate];

// Camera orientation for a pass: optical axis = −bisector, tilted `tilt` degrees about the seam
// (+ toward the flange, so it looks down more steeply; − toward the web). The Arducam/D405 row
// runs across the seam, so the two cameras look at the same stretch of weld.
export function bisectorAt(tilt = 0) {
  const t = tilt * RAD;
  return [BISECTOR[0] * Math.cos(t) - BISECTOR[2] * Math.sin(t), 0, BISECTOR[0] * Math.sin(t) + BISECTOR[2] * Math.cos(t)];
}
export function viewAt(s, tilt = 0) {
  const b = bisectorAt(tilt);
  const z = b.map((v) => -v);                        // tool z: looking into the corner
  const xAxis = [-z[2], 0, z[0]];                    // across the seam, outward and down
  const y = [z[1] * xAxis[2] - z[2] * xAxis[1], z[2] * xAxis[0] - z[0] * xAxis[2], z[0] * xAxis[1] - z[1] * xAxis[0]];
  const R = [0, 1, 2].map((r) => [xAxis[r], y[r], z[r]]);
  return mat(R, seamPoint(s));
}

// ---- inverse kinematics ------------------------------------------------------------------
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function errorOf(T, goal) {
  const dp = sub(goal.t, T.t);
  const w = [0, 0, 0];
  for (let c = 0; c < 3; c++) {
    const x = cross(col(T, c), col(goal, c));
    w[0] += x[0] / 2; w[1] += x[1] / 2; w[2] += x[2] / 2;
  }
  return [...dp, ...w.map((v) => v * 200)];
}
function solve6(A, b) {
  const n = 6, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}
// Damped least squares from a seed, so neighbouring poses stay on the seed's elbow and wrist branch.
export function ik(goal, seed, tool = TCP) {
  let q = seed.slice();
  for (let it = 0; it < 200; it++) {
    const T = mul(flangeOf(q), tool);
    const e = errorOf(T, goal);
    // the cross-product error is also zero for a half-turn, so check the axes really agree
    const aligned = [0, 1, 2].every((c) => col(T, c).reduce((s, v, i) => s + v * col(goal, c)[i], 0) > 0.999);
    if (Math.hypot(...e.slice(0, 3)) < 1e-4 && Math.hypot(...e.slice(3)) < 1e-4) return aligned ? q : null;
    const J = [];
    const h = 1e-4;
    for (let j = 0; j < 6; j++) {
      const qq = q.slice();
      qq[j] += h;
      const e2 = errorOf(mul(flangeOf(qq), tool), goal);
      J.push(e.map((v, i) => (v - e2[i]) / h));
    }
    const Jt = J;                                    // J[j][i] = d e_i / d q_j (negated)
    const A = [0, 1, 2, 3, 4, 5].map((a) => [0, 1, 2, 3, 4, 5].map((b) => Jt[a].reduce((s, v, i) => s + v * Jt[b][i], 0) + (a === b ? 0.01 : 0)));
    const g = [0, 1, 2, 3, 4, 5].map((a) => Jt[a].reduce((s, v, i) => s + v * e[i], 0));
    const dq = solve6(A, g);
    q = q.map((v, i) => v + Math.max(-5, Math.min(5, dq[i])));
  }
  return null;
}

// Pose along tool z: positive moves toward the work (claude.md, revisit step 5).
export const alongTool = (T, d) => mat(T.R, [T.t[0] + T.R[0][2] * d, T.t[1] + T.R[1][2] * d, T.t[2] + T.R[2][2] * d]);
