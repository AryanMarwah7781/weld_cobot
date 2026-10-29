// Builds cad/bracket/index.html: the three bracket designs, what each does on the arm, and how to print them.
//   node cad/bracket/page.mjs        (after bracket.py, analysis.mjs and render.py)
import { readFileSync, writeFileSync, existsSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const read = (p) => JSON.parse(readFileSync(join(HERE, "out", p), "utf8"));
const specs = read("specs.json");
const analysis = read("analysis.json");
const viewer = read("viewer.json");

const PRINT_FILL = 0.8;            // printed mass ≈ 80% of solid at 5 walls, 5 top/bottom, 40% gyroid
const D405_G = 58;                 // datasheet nominal
const ARD_G = 30;                  // not listed in the B0495 datasheet: an assumption, weigh yours
const KEYS = ["twin", "inline", "angled"];
const TITLES = { twin: "Twin", inline: "Inline", angled: "Angled 45°" };
// What each design does. Every number comes from the computed design (n) and the twin baseline (t).
const r0 = (v) => Math.round(v);
// how far the camera turns about its own axis between looking back and looking outward
const relRoll = (n) => (((n.out.roll - n.back.roll) % 360) + 360) % 360;
const COPY = {
  twin: {
    line: "Both cameras side by side on a flat plate, looking straight out of the flange.",
    does: (n) => [
      `Shortest lever: the lens sits ${r0(n.lens[2])} mm out from the flange.`,
      "The Arducam sits outboard, so a plain straight USB-C cable runs back past the flange.",
      "Suits the layout the setup figure uses, with the web on the robot's side.",
      `With the weld facing outward it fails: at roll 0° it hits the bench, and rolled 90° it drives J5 down to ${r0(n.out.j5min)}°, the singularity.`,
    ],
    verdict: "Print this one first",
  },
  inline: {
    line: "The Arducam on the J6 axis, on a tower; the D405 on a deck beside it.",
    does: (n, t) => [
      "TCP on the J6 axis: turning J6 spins the image about the spot it inspects without moving it, so roll is free in planning.",
      `Costs ${r0(n.lens[2] - t.lens[2])} mm more reach: the lens sits ${r0(n.lens[2])} mm out from the flange and J5 runs to ${r0(n.back.j5max)}°, ${r0(124 - n.back.j5max)}° from its 124° limit.`,
      "The USB-C plug drops through the deck into the tower and leaves through its open side.",
      `With the weld facing outward it fails: J5 falls to ${r0(n.out.j5min)}°, at the singularity.`,
    ],
    verdict: "Only if free J6 roll matters most",
  },
  angled: {
    line: "Both cameras on a 45° deck, so the camera axis is pitched 45° off the flange axis.",
    does: (n) => [
      `The only design that works with the weld facing either way: J5 stays ${r0(n.back.j5min)}–${r0(n.back.j5max)}° looking back and ${r0(n.out.j5min)}–${r0(n.out.j5max)}° looking outward.`,
      `To look outward, turn the camera ${relRoll(n)}° about its own axis from where it sits looking back; the analysis picks that roll.`,
      `The aim point sits ${r0(n.offAxis)} mm off the J6 axis, so a J6 roll swings it ${r0(n.roll10)} mm per 10°; plan with the full IK.`,
      "The heaviest head. Its hollow wedge carries the cables out the open back.",
    ],
    verdict: "If the weld must face outward",
  },
};

// ---- numbers for the page ------------------------------------------------------------------------
const mat = (m) => m;
const applyM = (m, p) => [0, 1, 2].map((r) => m[r][0] * p[0] + m[r][1] * p[1] + m[r][2] * p[2] + m[r][3]);
const designs = KEYS.map((key, i) => {
  const s = specs.designs[key];
  const a = analysis.designs[key];
  const headG = s.head_mass_solid * PRINT_FILL;
  const adapterG = specs.parts.adapter.mass_solid * PRINT_FILL;
  const ringG = specs.parts["centring-ring"].mass_solid * PRINT_FILL;
  const printedG = headG + adapterG + ringG;
  const d405c = applyM(mat(s.d405), [0, 0, 11.5]);
  const ardc = applyM(mat(s.ard), [0, 0, 16]);
  const total = printedG + D405_G + ARD_G;
  const com = [0, 1, 2].map((j) => (s.printed_com[j] * printedG + d405c[j] * D405_G + ardc[j] * ARD_G) / total);
  const lay = (L) => {
    const r = a[L];
    return { feasible: r.feasible, collides: r.collides, roll: r.roll, j5min: r.j5min, j5max: r.j5max, clear: r.clearance?.d, clearWhat: r.clearance?.what, wrist: r.wrist?.d, margin: r.limitMargin, j5: r.j5, rolls: r.rolls };
  };
  return {
    key, i, title: TITLES[key], ...COPY[key],
    headG, printedG, total, com, height: s.print_height,
    tcp: s.tcp, offAxis: a.tcpOffAxis, roll10: a.rollShiftPer10, lens: s.lens_front, lensDist: s.lens_front[2],
    back: lay("back"), out: lay("out"),
    files: { stl: `out/print/head-${key}.stl`, step: `out/step/assembly-${key}.step`, fcstd: `out/fcstd/bracket-${key}.FCStd` },
  };
});
const okOut = (d) => d.out.feasible && !d.out.collides && d.out.j5min >= 15;

// ---- data for the 3D viewer: indexed meshes, packed ------------------------------------------------
const pack = (m) => ({ v: Buffer.from(new Float32Array(m.v).buffer).toString("base64"), f: Buffer.from(new Uint32Array(m.f).buffer).toString("base64") });
const VIEW = {
  adapter: pack(viewer.adapter), ring: pack(viewer.ring), d405: pack(viewer.d405),
  heads: Object.fromEntries(KEYS.map((k) => [k, { head: pack(viewer.heads[k].head), ardMesh: pack(viewer.heads[k].ardMesh), plugMesh: pack(viewer.heads[k].plugMesh), d405: viewer.heads[k].d405, ard: viewer.heads[k].ard, lens: specs.designs[k].lens_front, tcp: specs.designs[k].tcp }])),
};
const CHART = {
  passes: analysis.passes,
  layouts: [["back", "Web on the robot's side · camera looks back"], ["out", "Web beyond the seam · camera looks outward"]],
  designs: designs.map((d) => ({ key: d.key, title: d.title, back: d.back, out: d.out })),
  titles: TITLES,
};

const f0 = (v) => (Math.abs(v) < 0.05 ? "0" : v.toFixed(0));
const f1 = (v) => v.toFixed(1);
const img = (name) => (existsSync(join(HERE, "out", "img", `${name}.png`)) ? `out/img/${name}.png` : `out/img/${name}-quick.png`);
const esc = (s) => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;");

const card = (d) => `
<article class="design" id="d-${d.key}">
  <header><span class="key" data-s="${d.i + 1}"></span><h3>${d.title}</h3><span class="verdict">${d.verdict}</span></header>
  <img src="${img(`${d.key}-hero`)}" alt="${d.title} bracket with both cameras fitted on the Lite 6 flange" loading="lazy">
  <p class="line">${d.line}</p>
  <dl class="stats">
    <div><dt>J5 range, weld facing the robot</dt><dd>${f0(d.back.j5min)}–${f0(d.back.j5max)}°</dd></div>
    <div><dt>Weld facing outward</dt><dd>${okOut(d) ? `works (J5 ${f0(d.out.j5min)}–${f0(d.out.j5max)}°)` : d.out.collides ? "collides" : `J5 ${f0(d.out.j5min)}°: singular`}</dd></div>
    <div><dt>Aim point off the J6 axis</dt><dd>${f0(d.offAxis)} mm</dd></div>
    <div><dt>Printed head / whole bracket</dt><dd>${f0(d.headG)} / ${f0(d.printedG)} g</dd></div>
  </dl>
  <ul class="does">${d.does(d, designs[0]).map((t) => `<li>${t}</li>`).join("")}</ul>
  <div class="actions"><button type="button" class="view-btn" data-design="${d.key}">View in 3D</button><a href="${d.files.stl}" download>Head STL</a><a href="${d.files.step}" download>Assembly STEP</a><a href="${d.files.fcstd}" download>FreeCAD</a></div>
</article>`;

const row = (label, fn, cls = "") => `<tr${cls ? ` class="${cls}"` : ""}><th scope="row">${label}</th>${designs.map((d) => `<td>${fn(d)}</td>`).join("")}</tr>`;
const yes = (t) => `<span class="ok">✓</span> ${t}`;
const no = (t) => `<span class="bad">✕</span> ${t}`;

const html = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lite 6 camera brackets</title>
<style>
:root{
  color-scheme:light;
  --surface:#fcfcfb;--surface-2:#f3f2ee;--surface-3:#e9e8e3;--line:#dddcd6;--grid:#e6e5e0;
  --text:#0b0b0b;--text-2:#52514e;--muted:#7c7b76;
  --s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;
  --good:#0ca30c;--critical:#d03b3b;--accent:#0b0b0b;
  --zone:#f4e3df;--zone-ink:#a23a2c;
  --font:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  --mono:ui-monospace,"Cascadia Mono","SF Mono",Consolas,monospace;
}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])){color-scheme:dark;
  --surface:#1a1a19;--surface-2:#222220;--surface-3:#2b2b29;--line:#3a3a37;--grid:#2f2f2c;
  --text:#f0efec;--text-2:#c3c2b7;--muted:#8d8c86;
  --s1:#3987e5;--s2:#d95926;--s3:#199e70;--zone:#3a2422;--zone-ink:#f0a598;--accent:#f0efec}}
:root[data-theme="dark"]{color-scheme:dark;
  --surface:#1a1a19;--surface-2:#222220;--surface-3:#2b2b29;--line:#3a3a37;--grid:#2f2f2c;
  --text:#f0efec;--text-2:#c3c2b7;--muted:#8d8c86;
  --s1:#3987e5;--s2:#d95926;--s3:#199e70;--zone:#3a2422;--zone-ink:#f0a598;--accent:#f0efec}
*{box-sizing:border-box}
html{background:var(--surface)}
body{margin:0;background:var(--surface);color:var(--text);font:16px/1.55 var(--font);-webkit-font-smoothing:antialiased}
main{max-width:1180px;margin:0 auto;padding:56px 24px 96px}
h1{font-size:clamp(30px,4.4vw,46px);line-height:1.08;letter-spacing:-.02em;margin:0 0 14px;font-weight:650}
h2{font-size:24px;letter-spacing:-.01em;margin:72px 0 6px;font-weight:620}
h3{font-size:19px;margin:0;font-weight:620}
p{margin:0 0 12px}
.lede{font-size:18px;color:var(--text-2);max-width:760px}
.sub{color:var(--text-2);max-width:760px;margin-bottom:22px}
.eyebrow{font:500 12px/1 var(--mono);letter-spacing:.12em;text-transform:uppercase;color:var(--muted);margin-bottom:18px}
.bottom-line{margin:30px 0 0;padding:20px 22px;border:1px solid var(--line);border-radius:14px;background:var(--surface-2);display:grid;gap:8px;max-width:880px}
.bottom-line strong{font-weight:620}
.key{display:inline-block;width:10px;height:10px;border-radius:50%;flex:none}
.key[data-s="1"]{background:var(--s1)}.key[data-s="2"]{background:var(--s2)}.key[data-s="3"]{background:var(--s3)}
.designs{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:18px;margin-top:26px}
.design{border:1px solid var(--line);border-radius:16px;padding:18px;display:flex;flex-direction:column;gap:12px;background:var(--surface)}
.design header{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.verdict{margin-left:auto;font:500 12px/1 var(--font);color:var(--text-2);background:var(--surface-3);padding:6px 9px;border-radius:999px}
.design img{width:100%;aspect-ratio:7/5;object-fit:cover;border-radius:10px;background:radial-gradient(circle at 50% 40%,var(--surface-2),var(--surface-3))}
.line{color:var(--text-2);margin:0}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:10px 14px;margin:0}
.stats div{border-top:1px solid var(--line);padding-top:8px}
.stats dt{font-size:12px;color:var(--muted);line-height:1.3}
.stats dd{margin:2px 0 0;font-weight:600;font-size:15px}
.does{margin:0;padding-left:18px;color:var(--text-2);font-size:14.5px;display:grid;gap:6px}
.actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:auto;padding-top:6px}
.actions a,.actions button,.seg button,.toggles label{font:500 13px/1 var(--font);color:var(--text);background:var(--surface);border:1px solid var(--line);border-radius:999px;padding:9px 12px;text-decoration:none;cursor:pointer}
.actions button{background:var(--accent);color:var(--surface);border-color:var(--accent)}
.actions a:hover,.seg button:hover{background:var(--surface-2)}
:focus-visible{outline:2px solid var(--s1);outline-offset:2px}
.viewer{margin-top:22px;border:1px solid var(--line);border-radius:16px;overflow:hidden;background:var(--surface-2)}
.viewer-bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;padding:12px 14px;border-bottom:1px solid var(--line);background:var(--surface)}
.seg{display:flex;gap:6px}
.seg button[aria-pressed="true"]{background:var(--accent);color:var(--surface);border-color:var(--accent)}
.seg button .key{margin-right:6px;vertical-align:-1px}
.toggles{display:flex;gap:8px;margin-left:auto;flex-wrap:wrap}
.toggles input{margin:0 6px 0 0;vertical-align:-2px;accent-color:var(--text)}
#stage{position:relative;height:min(62vh,560px);touch-action:none;cursor:grab}
#stage canvas{display:block;width:100%;height:100%}
#stage .hint{position:absolute;left:14px;bottom:12px;font:500 12px/1.3 var(--font);color:var(--muted);pointer-events:none}
#stage .readout{position:absolute;right:14px;top:12px;font:12.5px/1.5 var(--mono);color:var(--text-2);background:color-mix(in srgb,var(--surface) 86%,transparent);border:1px solid var(--line);border-radius:10px;padding:8px 10px;max-width:300px}
#stage .fallback{position:absolute;inset:0;display:none;align-items:center;justify-content:center;color:var(--text-2);padding:24px;text-align:center}
.legend3d{display:flex;flex-wrap:wrap;gap:14px;padding:10px 14px;border-top:1px solid var(--line);background:var(--surface);font-size:13px;color:var(--text-2)}
.legend3d span::before{content:"";display:inline-block;width:10px;height:10px;border-radius:3px;margin-right:6px;vertical-align:-1px;background:var(--c)}
.context{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px;margin-top:22px}
.context figure{margin:0;border:1px solid var(--line);border-radius:16px;overflow:hidden;background:var(--surface)}
.context img{width:100%;display:block;aspect-ratio:16/10;object-fit:cover;background:var(--surface-2)}
.context figcaption{padding:12px 14px;font-size:14px;color:var(--text-2)}
.context figcaption b{color:var(--text);font-weight:600}
.charts{margin-top:18px}
.chart-legend{display:flex;gap:18px;flex-wrap:wrap;font-size:13.5px;color:var(--text-2);margin:8px 0 6px}
.chart-legend span{display:inline-flex;align-items:center;gap:7px}
.chart-legend i{display:inline-block;width:18px;height:2px;border-radius:2px;background:var(--c)}
.chart-legend .zone{width:14px;height:10px;background:var(--zone);border-radius:2px}
.row-title{font-weight:600;margin:20px 0 4px;font-size:15px}
.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
.panel{position:relative;border:1px solid var(--line);border-radius:12px;background:var(--surface);padding:10px 10px 4px}
.panel h4{margin:0 0 2px;font-size:13px;font-weight:600;color:var(--text-2)}
.panel svg{width:100%;height:auto;display:block;overflow:visible}
.panel .tip{position:absolute;pointer-events:none;background:var(--surface);border:1px solid var(--line);border-radius:8px;padding:6px 8px;font:12px/1.45 var(--mono);color:var(--text);box-shadow:0 6px 18px rgba(0,0,0,.08);display:none;white-space:nowrap;z-index:2}
.axis text{font:11px var(--mono);fill:var(--muted)}
.axis line,.grid line{stroke:var(--grid);stroke-width:1}
.series{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
.zone-band{fill:var(--zone)}
.zone-label{font:600 10.5px var(--font);fill:var(--zone-ink)}
.limit-line{stroke:var(--zone-ink);stroke-width:1}
details{margin-top:12px}
summary{cursor:pointer;color:var(--text-2);font-size:14px}
table{border-collapse:collapse;width:100%;font-size:14px;margin-top:10px}
th,td{border-bottom:1px solid var(--line);padding:10px 10px;text-align:left;vertical-align:top}
thead th{font-size:12.5px;color:var(--muted);font-weight:600}
td{font-variant-numeric:tabular-nums}
tbody th{font-weight:500;color:var(--text-2);width:30%}
.ok{color:var(--good);font-weight:700}.bad{color:var(--critical);font-weight:700}
.compare th .key{margin-right:6px;vertical-align:0}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:26px;margin-top:18px}
.box{border:1px solid var(--line);border-radius:16px;padding:18px 20px;background:var(--surface)}
.box h3{margin-bottom:10px;font-size:17px}
.box ol,.box ul{margin:0;padding-left:20px;display:grid;gap:7px;color:var(--text-2);font-size:15px}
.box li b{color:var(--text);font-weight:600}
code,.mono{font-family:var(--mono);font-size:.92em}
.files{display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:8px;margin-top:14px}
.files a{display:flex;justify-content:space-between;gap:10px;border:1px solid var(--line);border-radius:10px;padding:10px 12px;color:var(--text);text-decoration:none;font-size:14px;background:var(--surface)}
.files a:hover{background:var(--surface-2)}
.files a span{color:var(--muted);font:12px var(--mono)}
.note{font-size:13.5px;color:var(--muted);margin-top:14px;max-width:880px}
@media (max-width:980px){.designs{grid-template-columns:1fr}.context,.cols{grid-template-columns:1fr}.panels{grid-template-columns:1fr}}
@media (max-width:560px){main{padding:36px 16px 72px}.toggles{margin-left:0}.stats{grid-template-columns:1fr}}
</style>
</head>
<body>
<main>
  <div class="eyebrow">weld_cobot · camera bracket</div>
  <h1>Three ways to put both cameras on the Lite 6</h1>
  <p class="lede">Printable brackets for the Arducam B0495 and the RealSense D405 on the Lite 6 tool flange, gripper off. All three share one flange adapter; only the camera head differs. Below is what each head does to the arm on the weld-inspection passes, and how to print and fit it.</p>
  <div class="bottom-line">
    <div><strong>Print Twin first.</strong> It is the simplest and lightest, holds the cameras closest to the flange, and keeps J5 at ${f0(designs[0].back.j5min)}–${f0(designs[0].back.j5max)}° with the T-joint placed web-toward-the-robot (the layout the setup figure settled on).</div>
    <div><strong>Switch to Angled 45°</strong> if the weld has to face away from the robot. It is the only head that stays clear of the J5 singularity both ways.</div>
    <div><strong>Inline</strong> puts the TCP on the J6 axis, so roll is free, at the cost of ${f0(designs[1].lens[2] - designs[0].lens[2])} mm more reach and J5 within ${f0(124 - designs[1].back.j5max)}° of its limit.</div>
  </div>

  <h2>The designs</h2>
  <p class="sub">Every head bolts to the same adapter with 4 × M3 into heat-set inserts, so you can print more than one and swap. Numbers come from the FreeCAD models and the Lite 6 kinematic model used for the setup figure.</p>
  <div class="designs">${designs.map(card).join("")}</div>

  <h2 id="viewer">Turn them over</h2>
  <p class="sub">The printed parts are orange and graphite; the cameras, robot flange and cable are stand-ins for fit. The amber line is the Arducam's line of sight to the TCP, ${specs.standoff} mm out, inside its 82° × 66° field of view.</p>
  <div class="viewer">
    <div class="viewer-bar">
      <div class="seg" role="group" aria-label="Design">${designs.map((d) => `<button type="button" data-design="${d.key}" aria-pressed="${d.key === "twin"}"><span class="key" data-s="${d.i + 1}"></span>${d.title}</button>`).join("")}</div>
      <div class="toggles">
        <label><input type="checkbox" id="t-cams" checked>Cameras</label>
        <label><input type="checkbox" id="t-fov" checked>Line of sight</label>
        <label><input type="checkbox" id="t-explode">Exploded</label>
      </div>
    </div>
    <div id="stage" aria-label="3D view of the selected bracket. Drag to turn, scroll or pinch to zoom.">
      <div class="readout" id="readout"></div>
      <div class="hint">Drag to turn · scroll to zoom · double-click to reset</div>
      <div class="fallback" id="fallback">WebGL isn't available here. The renders above show each design; the STL and STEP files open in any CAD or slicer.</div>
    </div>
    <div class="legend3d"><span style="--c:#e2752f">Printed head</span><span style="--c:#3a3d42">Adapter and centring ring (shared)</span><span style="--c:#6a6f77">RealSense D405</span><span style="--c:#2f7a50">Arducam B0495</span><span style="--c:#d8d8d4">Robot flange</span><span style="--c:#ffb347">Line of sight</span></div>
  </div>

  <h2>On the arm</h2>
  <p class="sub">Each head on the Lite 6 wrist, in the pose the arm takes halfway along pass 1 (bisector, ${specs.standoff} mm standoff). The plates are the example T-joint from the setup figure.</p>
  <div class="context">
    ${designs.map((d) => `<figure><img src="${img(`${d.key}-context-back`)}" alt="${d.title} on the Lite 6 wrist over the T-joint, weld facing the robot" loading="lazy"><figcaption><b>${d.title}</b>, weld facing the robot · J5 ${f0(d.back.j5min)}–${f0(d.back.j5max)}° over the three passes · bracket and cameras stay ${f0(d.back.clear)} mm from the plates.</figcaption></figure>`).join("")}
    <figure><img src="${img("angled-context-out")}" alt="Angled 45° on the Lite 6 wrist over the T-joint, weld facing outward" loading="lazy"><figcaption><b>Angled 45°</b>, weld facing outward, camera turned ${relRoll(designs[2])}° from looking back · J5 ${f0(designs[2].out.j5min)}–${f0(designs[2].out.j5max)}° · the setup neither Twin nor Inline can reach without the wrist singularity.</figcaption></figure>
  </div>

  <h2>J5 along the weld</h2>
  <p class="sub">The wrist angle J5 at every point along the seam (horizontal axis, mm from its middle) on each of the three passes. Near 0° the wrist is singular: J4 and J6 swing wildly and the arm loses accuracy. The project keeps |J5| ≥ 10°; the limit is 124°.</p>
  <div class="charts" id="charts">
    <div class="chart-legend">${designs.map((d) => `<span style="--c:var(--s${d.i + 1})"><i></i>${d.title}</span>`).join("")}<span><i class="zone"></i>|J5| under 15°: near the singularity</span></div>
  </div>
  <details><summary>Show the numbers as a table</summary><div id="chart-table"></div></details>

  <h2>Side by side</h2>
  <table class="compare">
    <thead><tr><th></th>${designs.map((d) => `<th scope="col"><span class="key" data-s="${d.i + 1}"></span>${d.title}</th>`).join("")}</tr></thead>
    <tbody>
      ${row("Weld facing the robot", (d) => yes(`J5 ${f0(d.back.j5min)}–${f0(d.back.j5max)}°, ${f0(d.back.margin)}° from any joint limit`))}
      ${row("Weld facing outward", (d) => (okOut(d) ? yes(`J5 ${f0(d.out.j5min)}–${f0(d.out.j5max)}°, camera turned ${relRoll(d)}° from looking back`) : d.out.rolls.every((r) => !r.feasible || r.collides || r.j5min < 15) ? no(d.out.rolls.some((r) => r.collides) && d.key === "twin" ? "hits the bench at roll 0°; J5 9° at roll 90/270°" : `J5 drops to ${f0(d.out.j5min)}°`) : no("—")))}
      ${row("Closest approach to the plates", (d) => `${f0(d.back.clear)} mm (lens, on the web-side pass)`)}
      ${row("Aim point off the J6 axis", (d) => `${f0(d.offAxis)} mm · a 10° J6 roll moves it ${f1(d.roll10)} mm`)}
      ${row("Lens front, out from the flange face", (d) => `${f0(d.lensDist)} mm`)}
      ${row("TCP in the flange frame (x, y, z)", (d) => `<span class="mono">${d.tcp.map(f1).join(", ")}</span> mm`)}
      ${row("Printed mass (head + adapter + ring)", (d) => `≈ ${f0(d.printedG)} g`)}
      ${row("Payload for the controller", (d) => `${(d.total / 1000).toFixed(3)} kg at <span class="mono">${d.com.map(f0).join(", ")}</span> mm`)}
      ${row("Head print height", (d) => `${f0(d.height)} mm, no supports`)}
    </tbody>
  </table>
  <p class="note">Payload adds the D405 (58 g, datasheet) and the Arducam (assumed 30 g; its datasheet doesn't list a weight) to the printed parts at about 80% of solid density. The controller still holds the old setting of 0.145 kg at z = 30 mm, so update it for whichever head you fit.</p>

  <h2>Print and fit</h2>
  <div class="cols">
    <div class="box">
      <h3>Printing</h3>
      <ul>
        <li><b>Material:</b> PETG, or ASA if the cell gets warm. PLA creeps under the screw clamp.</li>
        <li><b>Settings:</b> 0.4 mm nozzle, 0.2 mm layers, 5 walls, 5 top and 5 bottom layers, 40% gyroid.</li>
        <li><b>Orientation:</b> the STLs come oriented as they should print, back face on the bed. No part needs supports: nothing overhangs past 45°.</li>
        <li><b>Two spots to watch:</b> Inline's deck is a 39 mm bridge between its walls, and Angled's deck underside is exactly 45°. Both print clean in PETG with bridging on. If your slicer adds supports inside them, block those.</li>
        <li><b>Per bracket:</b> one adapter, one centring ring, one head. The adapter and ring serve every head.</li>
        <li><b>Check the fit first:</b> print the ring alone and try it in the flange's Ø31.5 recess. If it's tight, scale it 99.5% in XY.</li>
      </ul>
    </div>
    <div class="box">
      <h3>Hardware, per bracket</h3>
      <ul>
        <li>4 × <b>M6 × 12</b> socket-head screws, adapter to flange, on the 50 mm bolt circle. The Lite 6 manual's End Flange section gives 6 × M6 plus the Ø6 locating hole, and the gripper fits with M6 bolts; ignore the "(M5×6)" in its spec summary table.</li>
        <li>1 × <b>Ø6 × 16 steel dowel pin</b>, pressed into the adapter; 6 mm enters the flange's locating hole. Printed holes come out a little small, so open it with a 6 mm drill by hand if the pin won't go.</li>
        <li>4 × <b>M3 heat-set inserts</b> (M3 × 5.7) and 4 × <b>M3 × 8</b> socket-head screws, head to adapter.</li>
        <li>4 × <b>M2 × 6 thread-forming screws</b>, Arducam to its posts (Ø1.75 pilot holes).</li>
        <li>1 × <b>1/4"-20 × 3/8"</b> screw for the D405's tripod hole, 8.35 mm from its back face (Intel D400 datasheet, Figure 10-13). The camera allows 5 mm of thread; the ledge is 5 mm thick.</li>
        <li>A straight USB-C cable for the Arducam and a Micro-B cable for the D405, tied down the arm's far side.</li>
      </ul>
    </div>
    <div class="box">
      <h3>Fitting it, in this order</h3>
      <ol>
        <li>Press the dowel into the adapter from the back and push the centring ring into its pocket. Melt the four M3 inserts into the front.</li>
        <li>Take the gripper off. Seat the adapter, ring in the flange recess and dowel in the Ø6 hole, and bolt it down with the four M6 screws.</li>
        <li>Bolt the head on with the four M3 screws. The hex key reaches each one through its access hole; do this <b>before</b> the cameras, which cover some holes.</li>
        <li>Fit the D405 on its ledge with the 1/4"-20 screw from underneath, its back flat against the head.</li>
        <li>Screw the Arducam to its four posts with its USB-C edge, the bottom of the board, over the slot: outboard on Twin, toward the tower's open side on Inline, up the slope on Angled. The plug goes straight in through the slot.</li>
        <li>Refocus the Arducam lens for the working distance; it ships focused at 2 m to infinity.</li>
        <li>Set the payload in the controller, then run <code>calibration.capture</code> and <code>calibration.solve</code> for the new hand-eye transform.</li>
      </ol>
    </div>
    <div class="box">
      <h3>Questions to answer before printing</h3>
      <ol>
        <li><b>What screw fits the Arducam's four mounting holes: M2, M2.5 or M3?</b> Measure one hole: about 2.2 mm means M2, 2.7 mm M2.5, 3.2 mm M3. The posts' pilot holes are sized for M2.</li>
        <li><b>What does the Arducam weigh with its lens?</b> The payload figures assume 30 g.</li>
        <li><b>What working distance will you use?</b> The 120 mm standoff is still an open question. The TCP moves with it, and the hand-eye calibration measures the real one anyway.</li>
      </ol>
      <p class="note" style="margin-top:12px">Reach numbers come from an ideal model of the Lite 6. Check the passes with <code>tools/plan_check.py</code> on the controller's own IK before running them.</p>
    </div>
  </div>

  <h2>Files</h2>
  <p class="sub">Print STLs are oriented for the bed. STEP and FreeCAD files open in any CAD tool; every FreeCAD document has each part as its own object, with the cameras as stand-ins.</p>
  <div class="files">
    <a href="out/print/adapter.stl" download>Flange adapter · STL <span>all designs</span></a>
    <a href="out/print/centring-ring.stl" download>Centring ring · STL <span>all designs</span></a>
    ${designs.map((d) => `<a href="out/print/head-${d.key}.stl" download>${d.title} head · STL <span>print</span></a><a href="out/step/assembly-${d.key}.step" download>${d.title} assembly · STEP <span>CAD</span></a><a href="out/fcstd/bracket-${d.key}.FCStd" download>${d.title} · FreeCAD <span>.FCStd</span></a>`).join("")}
  </div>
  <p class="note">Every print STL was checked in FreeCAD: watertight, no non-manifold edges, no self-intersections, flat on the bed. The largest, Twin's head, is 140 × 75 × 30 mm.</p>
  <p class="note">Rebuild everything from <code>cad/bracket/</code>: <code>bracket.py</code> (FreeCAD, the models), <code>analysis.mjs</code> (Node, the arm model), <code>render.py</code> (Blender) and <code>page.mjs</code> (this page). Each script's header has its command line.</p>
</main>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
const VIEW = ${JSON.stringify(VIEW)};
const CHART = ${JSON.stringify(CHART)};
${readFileSync(join(HERE, "page.js"), "utf8")}
</script>
</body>
</html>`;

writeFileSync(join(HERE, "index.html"), html);
console.log(`wrote index.html · ${(html.length / 1024).toFixed(0)} KB`);
for (const d of designs) console.log(`${d.key}: head ${f0(d.headG)} g, bracket ${f0(d.printedG)} g, payload ${(d.total / 1000).toFixed(3)} kg at ${d.com.map(f1).join(", ")}`);
