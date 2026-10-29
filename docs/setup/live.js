const DATA = __DATA__;
const stage = document.querySelector(".wc-stage");
const readout = document.querySelector("[data-readout]");
const announcer = stage.querySelector(".wc-live");
const sight = stage.querySelector(".wc-sight");
const aim = stage.querySelector(".wc-aim");
const roi = stage.querySelector(".wc-roi");
const still = matchMedia("(prefers-reduced-motion: reduce)");
const tour = makeTour(DATA);
const q0 = DATA.q0;
const WK = DATA.work;
const RADS = Math.PI / 180;
const cam = (() => {
  const a = DATA.P.azimuth * RADS;
  const e = DATA.P.elevation * RADS;
  return { sa: Math.sin(a), ca: Math.cos(a), se: Math.sin(e), ce: Math.cos(e), k: DATA.P.scale * Math.sqrt(1.6) };
})();
const iso = ([x, y, z]) => [DATA.P.origin[0] + (x * cam.sa - y * cam.ca) * cam.k, DATA.P.origin[1] + ((x * cam.ca + y * cam.sa) * cam.se - z * cam.ce) * cam.k];
const valuesOf = (q) => ({ j1: q[0] - q0[0], j2: q[1] - q0[1], j3: q[2] - q0[2], j4: q[3] - q0[3], j5: q[4] - q0[4], j6: q[5] - q0[5] });
const PASSES = ["along the bisector", "tilted 18° to the flange", "tilted 18° to the web"];
const TILTS = [0, 18, -18];

let mode = still.matches ? "hand" : "tour";
let clock = 0;
let frame = 0;
let last = 0;
let visible = false;
let idle = 0;
let wait = 1.4;                      // the tour starts after this long in view
const hand = { s: 0, tilt: 0, ts: 0, tt: 0 };
let blend = null;                    // { from: q, t } while one source hands over to the next
let drawnQ = q0.slice();
let roiOn = 0;
let lastPhase = "";
let hooked = false;                  // a tool set the joints through window.__isoTurn: keep them until the reader acts

const seamClamp = (s) => Math.max(DATA.seam.from, Math.min(DATA.seam.to, s));
const startBlend = () => { blend = { from: drawnQ.slice(), t: 0 }; };
const takeOver = () => {
  hooked = false;
  if (mode === "tour") {
    const st = tour.at(clock);
    hand.s = hand.ts = st.s ?? DATA.roi.s;
    hand.tilt = hand.tt = st.tilt ?? 0;
    startBlend();
  }
  mode = "hand";
  clearTimeout(idle);
  idle = setTimeout(() => { if (!still.matches) resume(); }, 9000);
};
const resume = () => {
  hooked = false;
  mode = "tour";
  clock = 0;
  wait = 0;
  startBlend();
  run();
};
const revisit = () => {
  hooked = false;
  mode = "tour";
  clock = tour.starts[DATA.segments.findIndex((seg) => seg.phase === "detect")];
  wait = 0;
  startBlend();
  clearTimeout(idle);
  run();
};

// where the Arducam's optical axis meets the work: the bead, the flange plate or the web
function aimOf(q) {
  const Fl = frames(q)[5];
  const o = apply(Fl, [DATA.tcp[0], 0, DATA.lensFront]);
  const d = col(Fl, 2);
  const hits = [];
  const inY = (p) => p[1] >= WK.y0 && p[1] <= WK.y1;
  const { c, R0 } = DATA.bead;
  const u0 = o[0] - WK.x - c;
  const v0 = o[2] - WK.plate - c;
  const A = d[0] * d[0] + d[2] * d[2];
  const B = 2 * (u0 * d[0] + v0 * d[2]);
  const C = u0 * u0 + v0 * v0 - R0 * R0;
  const disc = B * B - 4 * A * C;
  if (disc > 0) {
    const t = (-B - Math.sqrt(disc)) / (2 * A);
    const p = [o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t];
    if (t > 0 && p[0] >= WK.x - 0.01 && p[2] >= WK.plate - 0.01 && p[1] >= DATA.bead.s[0] && p[1] <= DATA.bead.s[1]) hits.push([t, p]);
  }
  if (d[2] < 0) {
    const t = (WK.plate - o[2]) / d[2];
    const p = [o[0] + d[0] * t, o[1] + d[1] * t, WK.plate];
    if (t > 0 && p[0] >= WK.x && p[0] <= WK.x + WK.flangeD && inY(p)) hits.push([t, p]);
  }
  if (d[0] < 0) {
    const t = (WK.x - o[0]) / d[0];
    const p = [WK.x, o[1] + d[1] * t, o[2] + d[2] * t];
    if (t > 0 && p[2] >= WK.plate && p[2] <= WK.plate + WK.webH && inY(p)) hits.push([t, p]);
  }
  hits.sort((a, b) => a[0] - b[0]);
  return hits.length ? { from: o, to: hits[0][1], dist: hits[0][0] } : null;
}

const fmt = (v) => `${v >= 0 ? "+" : "−"}${Math.abs(v).toFixed(0)}`;
function textOf(st) {
  if (mode === "hand") return `by hand · pass ${TILTS.indexOf(hand.tt) + 1} · y ${fmt(hand.s)} mm · J5 ${drawnQ[4].toFixed(0)}°`;
  switch (st.phase) {
    case "ready": return "ready · robot log 100 Hz · video 30 fps";
    case "scan": return `pass ${DATA.segments[st.seg].pass} of 3 · y ${fmt(st.s)} mm · frame ${st.frame}`;
    case "swing": return `to pass ${DATA.segments[st.seg].pass} · tilt ${fmt(st.tilt)}°`;
    case "detect": return `ROI · pass ${DATA.roi.pass}, frame ${DATA.roi.frame}`;
    case "clear": return "clearance · 60 mm off the seam, in joint space";
    case "revisit": return `revisit · joint space, frame ${DATA.roi.frame}'s angles`;
    case "settle": return "settling 0.4 s (the arm is light and rings)";
    case "closer": return `closer · ${st.d.toFixed(0)} mm along tool z`;
    case "still": return "still · sharpest of 5 frames";
    default: return "back to the start";
  }
}
const SAY = { ready: "Ready to scan", scan: "Scanning", detect: "Region of interest found", revisit: "Returning to the flagged pose", still: "Taking the still", return: "Run finished" };

function draw(st, dt) {
  let q = mode === "hand" ? tour.gridAt(hand.s, hand.tilt) : st.q;
  if (blend) {
    blend.t += dt / 0.7;
    q = blend.from.map((v, i) => v + (q[i] - v) * ease(blend.t));
    if (blend.t >= 1) blend = null;
  }
  drawnQ = q;
  controller.set(valuesOf(q));
  const hit = aimOf(q);
  if (hit) {
    const a = iso(hit.from);
    const b = iso(hit.to);
    sight.setAttribute("d", `M${a[0].toFixed(2)} ${a[1].toFixed(2)}L${b[0].toFixed(2)} ${b[1].toFixed(2)}`);
    aim.setAttribute("cx", b[0].toFixed(2));
    aim.setAttribute("cy", b[1].toFixed(2));
    let r = 1.6;
    if (mode === "tour" && st.phase === "still") for (let i = 0; i < 5; i++) r += 1.4 * burst(st.local - 0.25 - i * 0.2, 0.04, 0.08);
    aim.setAttribute("r", r.toFixed(2));
    showTiles(hit.to[1]);
  }
  sight.style.opacity = aim.style.opacity = hit ? "" : "0";
  const want = mode === "tour" ? (["detect", "clear", "revisit", "settle", "closer", "still", "leave"].includes(st.phase) ? 1 : 0) : roiOn > 0.5 ? 1 : 0;
  roiOn += (want - roiOn) * (still.matches ? 1 : 1 - Math.exp(-dt / 0.25));
  roi.style.opacity = roiOn.toFixed(3);
  for (const t of TILES) t.box.style.opacity = roiOn.toFixed(3);
  const text = textOf(st);
  if (readout.textContent !== text) readout.textContent = text;
  if (mode === "tour" && st.phase !== lastPhase) {
    lastPhase = st.phase;
    if (SAY[st.phase]) announcer.textContent = SAY[st.phase];
  }
}
function burst(age, rise, fall) {
  if (age <= 0) return 0;
  if (age < rise) { const x = age / rise; return x * x * (3 - 2 * x); }
  return Math.exp(-(age - rise) / fall);
}

// the viewer's camera tiles: the weld as each camera sees it, scrolling as the camera moves
const TILES = [...stage.querySelectorAll(".wc-scroll")].map((g) => {
  const i = Number(g.dataset.tile);
  const T0 = DATA.tile;
  const xc = T0.xs[i] + (i ? 10 : 0);              // the D405 sits 58 mm across the seam from the Arducam
  const k = i ? 0.18 : 0.19;                       // tile mm per mm of weld along the seam
  const bw = i ? 2 : 2.6;
  const cy = T0.y + T0.h / 2;
  const Y = (yw) => (cy + yw * k).toFixed(2);
  const [b0, b1] = DATA.bead.s;
  let m = `<rect class="wc-web" x="${xc - 60}" y="-300" width="${(60 - bw / 2).toFixed(2)}" height="600"/>`;
  m += `<rect class="wc-bead" x="${(xc - bw / 2).toFixed(2)}" y="${Y(b0)}" width="${bw}" height="${((b1 - b0) * k).toFixed(2)}" rx="${(bw / 2).toFixed(2)}"/>`;
  let d = "";
  for (let yw = b0 + 3; yw < b1 - 2; yw += 6.4) d += `M${(xc - bw / 2 + 0.2).toFixed(2)} ${Y(yw)}Q${xc.toFixed(2)} ${(cy + (yw - 1.6) * k).toFixed(2)} ${(xc + bw / 2 - 0.2).toFixed(2)} ${Y(yw)}`;
  m += `<path class="wc-ripple" d="${d}"/><circle class="wc-pit" cx="${xc.toFixed(2)}" cy="${Y(DATA.roi.s)}" r="${i ? 0.35 : 0.45}"/>`;
  m += `<rect class="wc-box" x="${(xc - 2.6).toFixed(2)}" y="${(cy + DATA.roi.s * k - 2.6).toFixed(2)}" width="5.2" height="5.2"/>`;
  g.innerHTML = m;
  return { g, k, box: g.querySelector(".wc-box") };
});
let seen = 0;
const showTiles = (y) => {
  if (Math.abs(y - seen) < 0.01 && TILES.length && TILES[0].done) return;
  seen = y;
  for (const t of TILES) {
    t.g.setAttribute("transform", `translate(0 ${(-y * t.k).toFixed(3)})`);
    t.done = true;
  }
};

const controller = TURN.mount(stage, DATA.turn, { onHold: () => { hooked = true; clearTimeout(idle); } });
function tick(now) {
  if (hooked) { frame = 0; return; }
  const dt = last ? Math.min((now - last) / 1000, 1 / 30) : 1 / 60;
  last = now;
  if (mode === "tour") {
    if (wait > 0) wait -= dt;
    else clock += dt;
  } else {
    const f = still.matches ? 1 : 1 - Math.exp(-dt / 0.2);
    hand.s += (hand.ts - hand.s) * f;
    hand.tilt += (hand.tt - hand.tilt) * f;
    if (Math.abs(hand.ts - hand.s) < 0.01) hand.s = hand.ts;
    if (Math.abs(hand.tt - hand.tilt) < 0.01) hand.tilt = hand.tt;
  }
  const st = tour.at(clock);
  draw(st, dt);
  const settled = mode === "hand" && hand.s === hand.ts && hand.tilt === hand.tt && !blend && Math.abs(roiOn - Math.round(roiOn)) < 1e-3;
  frame = visible && !settled ? requestAnimationFrame(tick) : 0;
}
const run = () => {
  if (!frame && visible) {
    last = 0;
    frame = requestAnimationFrame(tick);
  }
};
new IntersectionObserver((entries) => {
  visible = entries[entries.length - 1].isIntersecting;
  if (visible) run();
  else if (frame) {
    cancelAnimationFrame(frame);
    frame = 0;
  }
}, { rootMargin: "120px 0px" }).observe(stage);

// ---- input ------------------------------------------------------------------------------------
const seamDir = (() => {
  const a = iso([WK.x, 0, WK.plate]);
  const b = iso([WK.x, 1, WK.plate]);
  return [b[0] - a[0], b[1] - a[1]];
})();
const describe = () => {
  stage.setAttribute("aria-valuenow", hand.ts.toFixed(0));
  stage.setAttribute("aria-valuetext", `camera over the weld at y ${hand.ts.toFixed(0)} mm, pass ${TILTS.indexOf(hand.tt) + 1}, ${PASSES[TILTS.indexOf(hand.tt)]}`);
};
let drag = null;
stage.addEventListener("pointerdown", (event) => {
  takeOver();
  drag = { x: event.clientX, y: event.clientY, s: hand.ts, id: event.pointerId, captured: false };
  run();
});
stage.addEventListener("pointermove", (event) => {
  if (!drag || event.pointerId !== drag.id) return;
  const box = stage.getBoundingClientRect();
  const scale = DATA.P.width / box.width;
  const dx = (event.clientX - drag.x) * scale;
  const dy = (event.clientY - drag.y) * scale;
  if (!drag.captured && Math.hypot(dx, dy) > 3) {
    drag.captured = true;
    stage.setPointerCapture(event.pointerId);
  }
  if (!drag.captured) return;
  const len2 = seamDir[0] ** 2 + seamDir[1] ** 2;
  hand.ts = seamClamp(drag.s + (dx * seamDir[0] + dy * seamDir[1]) / len2);
  takeOver();
  describe();
  run();
});
const release = (event) => {
  if (!drag || event.pointerId !== drag.id) return;
  drag = null;
  try { stage.releasePointerCapture(event.pointerId); } catch {}
};
stage.addEventListener("pointerup", release);
stage.addEventListener("pointercancel", release);
stage.addEventListener("keydown", (event) => {
  const key = event.key;
  const step = event.shiftKey ? 20 : 4;
  if (key === " " || key === "Enter") {
    event.preventDefault();
    resume();
    return;
  }
  if (key === "r" || key === "R") {
    event.preventDefault();
    revisit();
    return;
  }
  let handled = true;
  if (key === "ArrowUp" || key === "ArrowRight") { takeOver(); hand.ts = seamClamp(Math.round(hand.ts) - step); }
  else if (key === "ArrowDown" || key === "ArrowLeft") { takeOver(); hand.ts = seamClamp(Math.round(hand.ts) + step); }
  else if (key === "Home") { takeOver(); hand.ts = DATA.seam.to; }
  else if (key === "End") { takeOver(); hand.ts = DATA.seam.from; }
  else if (key === "1" || key === "2" || key === "3") { takeOver(); hand.tt = TILTS[Number(key) - 1]; }
  else handled = false;
  if (!handled) return;
  event.preventDefault();
  describe();
  run();
});
stage.addEventListener("focus", () => { announcer.textContent = mode === "tour" ? "Running the inspection" : "Moving by hand"; });
if (still.matches) {
  hand.s = hand.ts = 0;
  hand.tilt = hand.tt = 0;
}
describe();
draw(tour.at(0), 0);
