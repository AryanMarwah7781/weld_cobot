// Page behaviour: the 3D viewer and the J5 charts. Inlined into index.html by page.mjs.
(() => {
  const still = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const NAMES = CHART.titles;

  // ---------------------------------------------------------------- 3D viewer
  const stage = document.getElementById("stage");
  const readout = document.getElementById("readout");
  const fail = () => { document.getElementById("fallback").style.display = "flex"; readout.style.display = "none"; };
  let renderer = null;
  try {
    if (window.THREE) renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  } catch (e) { renderer = null; }
  let select = () => {};
  if (!renderer) fail();
  else {
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    stage.prepend(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(32, 1, 1, 6000);
    camera.up.set(0, 0, 1);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x9a9a9a, 0.85));
    const key = new THREE.DirectionalLight(0xffffff, 0.75);
    key.position.set(320, -420, 620);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.3);
    fill.position.set(-380, 300, 160);
    scene.add(fill);

    const bytes = (b64) => Uint8Array.from(atob(b64), (c) => c.charCodeAt(0)).buffer;
    const geoCache = new Map();
    const geometry = (m) => {
      if (geoCache.has(m)) return geoCache.get(m);
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(bytes(m.v)), 3));
      g.setIndex(new THREE.BufferAttribute(new Uint32Array(bytes(m.f)), 1));
      const flat = g.toNonIndexed();
      flat.computeVertexNormals();
      const out = { solid: flat, edges: new THREE.EdgesGeometry(g, 28) };
      geoCache.set(m, out);
      return out;
    };
    const mesh = (m, color, { metal = 0, rough = 0.6, edge = 0x1b1b1b, edgeOpacity = 0.32 } = {}) => {
      const { solid, edges } = geometry(m);
      const group = new THREE.Group();
      group.add(new THREE.Mesh(solid, new THREE.MeshStandardMaterial({ color, metalness: metal, roughness: rough, flatShading: true })));
      group.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: edge, transparent: true, opacity: edgeOpacity })));
      return group;
    };
    const m4 = (m) => new THREE.Matrix4().set(m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], 0, 0, 0, 1);
    const ORANGE = 0xe2752f, GRAPHITE = 0x3a3d42;

    // the robot's flange and J6 housing, the shared adapter and ring
    const robot = new THREE.Group();
    const cylZ = (r, h, z, color) => {
      const c = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 72), new THREE.MeshStandardMaterial({ color, roughness: 0.45 }));
      c.rotation.x = Math.PI / 2;
      c.position.z = z;
      return c;
    };
    robot.add(cylZ(37.5, 10, -5, 0xd8d8d4), cylZ(31, 46, -33, 0xcfcfca));
    scene.add(robot);
    const adapter = mesh(VIEW.adapter, GRAPHITE, { rough: 0.55, edge: 0x000000, edgeOpacity: 0.5 });
    const ring = mesh(VIEW.ring, GRAPHITE, { rough: 0.55, edge: 0x000000, edgeOpacity: 0.5 });
    scene.add(adapter, ring);

    // per design: head, cameras, line of sight
    const designs = {};
    for (const [k, d] of Object.entries(VIEW.heads)) {
      const head = mesh(d.head, ORANGE, { rough: 0.5 });
      const cams = new THREE.Group();
      const d405 = mesh(VIEW.d405, 0x6a6f77, { metal: 0.15, rough: 0.45, edge: 0x000000, edgeOpacity: 0.55 });
      d405.applyMatrix4(m4(d.d405));
      const ardM = m4(d.ard);
      const ard = mesh(d.ardMesh, 0x2f7a50, { rough: 0.45 });
      ard.applyMatrix4(ardM);
      const plug = mesh(d.plugMesh, 0x161616, { rough: 0.7 });
      plug.applyMatrix4(ardM);
      // where each part is mounted; the exploded view adds its offset to this
      for (const part of [d405, ard, plug]) part.userData.home = part.position.clone();
      cams.add(d405, ard, plug);
      const sight = new THREE.Group();
      const lens = new THREE.Vector3(...d.lens);
      const tcp = new THREE.Vector3(...d.tcp);
      const dir = tcp.clone().sub(lens);
      const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 0.7, dir.length(), 12), new THREE.MeshBasicMaterial({ color: 0xffa53a }));
      rod.position.copy(lens.clone().add(tcp).multiplyScalar(0.5));
      rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
      const dot = new THREE.Mesh(new THREE.SphereGeometry(2.6, 24, 16), new THREE.MeshBasicMaterial({ color: 0xffa53a }));
      dot.position.copy(tcp);
      // field of view: 82° × 66°, apex at the lens, base at the working distance
      const R = new THREE.Matrix4().extractRotation(ardM);
      const ex = new THREE.Vector3(1, 0, 0).applyMatrix4(R), ey = new THREE.Vector3(0, 1, 0).applyMatrix4(R);
      const L = dir.length();
      const hx = L * Math.tan((41 * Math.PI) / 180), hy = L * Math.tan((33 * Math.PI) / 180);
      const corner = (sx, sy) => tcp.clone().add(ex.clone().multiplyScalar(sx * hx)).add(ey.clone().multiplyScalar(sy * hy));
      const c = [corner(1, 1), corner(-1, 1), corner(-1, -1), corner(1, -1)];
      const pos = [];
      for (let i = 0; i < 4; i++) pos.push(...lens.toArray(), ...c[i].toArray(), ...c[(i + 1) % 4].toArray());
      const fg = new THREE.BufferGeometry();
      fg.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
      fg.computeVertexNormals();
      const frustum = new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ color: 0xffa53a, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false }));
      const rimPts = [...c, c[0]].flatMap((p) => p.toArray());
      const rim = new THREE.Line(new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(rimPts, 3)), new THREE.LineBasicMaterial({ color: 0xffa53a, transparent: true, opacity: 0.7 }));
      sight.add(rod, dot, frustum, rim);
      const all = new THREE.Group();
      all.add(head, cams, sight);
      all.visible = false;
      scene.add(all);
      designs[k] = { all, head, cams, d405, ard, plug, sight, d, ardAxis: new THREE.Vector3(d.ard[0][2], d.ard[1][2], d.ard[2][2]), d405Axis: new THREE.Vector3(d.d405[0][2], d.d405[1][2], d.d405[2][2]) };
    }

    // orbit
    const orbit = { theta: -0.95, phi: 0.55, dist: 600, target: new THREE.Vector3(0, 0, 95) };
    const home = { ...orbit, target: orbit.target.clone() };
    let current = "twin", explode = 0, explodeTo = 0, frame = 0;
    const place = () => {
      const { theta, phi, dist, target } = orbit;
      camera.position.set(target.x + dist * Math.cos(phi) * Math.cos(theta), target.y + dist * Math.cos(phi) * Math.sin(theta), target.z + dist * Math.sin(phi));
      camera.lookAt(target);
    };
    const size = () => {
      const w = stage.clientWidth, h = stage.clientHeight;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    const applyExplode = () => {
      const e = explode;
      adapter.position.z = -26 * e;
      ring.position.z = -46 * e;
      robot.position.z = -70 * e;
      for (const D of Object.values(designs)) {
        D.d405.position.copy(D.d405.userData.home).addScaledVector(D.d405Axis, 34 * e);
        D.ard.position.copy(D.ard.userData.home).addScaledVector(D.ardAxis, 34 * e);
        D.plug.position.copy(D.plug.userData.home).addScaledVector(D.ardAxis, -18 * e);
        D.sight.position.copy(D.ardAxis.clone().multiplyScalar(34 * e));
      }
    };
    const draw = () => {
      place();
      applyExplode();
      renderer.render(scene, camera);
    };
    const tick = () => {
      frame = 0;
      if (explode !== explodeTo) {
        explode += (explodeTo - explode) * 0.18;
        if (Math.abs(explodeTo - explode) < 0.002) explode = explodeTo;
        frame = requestAnimationFrame(tick);
      }
      draw();
    };
    const kick = () => { if (!frame) frame = requestAnimationFrame(tick); };
    const describe = (k) => {
      const d = VIEW.heads[k];
      const off = Math.hypot(d.tcp[0], d.tcp[1]);
      readout.innerHTML = `<b>${NAMES[k]}</b><br>TCP ${d.tcp.map((v) => v.toFixed(0)).join(", ")} mm<br>${off < 0.5 ? "aim point on the J6 axis" : `aim point ${off.toFixed(0)} mm off the J6 axis`}<br>lens ${d.lens[2].toFixed(0)} mm out from the flange`;
    };
    select = (k) => {
      current = k;
      for (const [name, D] of Object.entries(designs)) D.all.visible = name === k;
      document.querySelectorAll(".seg button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.design === k)));
      describe(k);
      kick();
    };
    document.getElementById("t-cams").addEventListener("change", (e) => { for (const D of Object.values(designs)) D.cams.visible = e.target.checked; kick(); });
    document.getElementById("t-fov").addEventListener("change", (e) => { for (const D of Object.values(designs)) D.sight.visible = e.target.checked; kick(); });
    document.getElementById("t-explode").addEventListener("change", (e) => { explodeTo = e.target.checked ? 1 : 0; if (still) explode = explodeTo; kick(); });
    document.querySelectorAll(".seg button").forEach((b) => b.addEventListener("click", () => select(b.dataset.design)));

    const pointers = new Map();
    let pinch = 0;
    stage.addEventListener("pointerdown", (e) => { stage.setPointerCapture(e.pointerId); pointers.set(e.pointerId, [e.clientX, e.clientY]); stage.style.cursor = "grabbing"; });
    stage.addEventListener("pointermove", (e) => {
      if (!pointers.has(e.pointerId)) return;
      const [px, py] = pointers.get(e.pointerId);
      pointers.set(e.pointerId, [e.clientX, e.clientY]);
      if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) orbit.dist = Math.min(2200, Math.max(220, orbit.dist * (pinch / d)));
        pinch = d;
      } else {
        orbit.theta -= (e.clientX - px) * 0.008;
        orbit.phi = Math.min(1.45, Math.max(-1.2, orbit.phi + (e.clientY - py) * 0.008));
      }
      kick();
    });
    const up = (e) => { pointers.delete(e.pointerId); pinch = 0; stage.style.cursor = ""; };
    stage.addEventListener("pointerup", up);
    stage.addEventListener("pointercancel", up);
    stage.addEventListener("wheel", (e) => { e.preventDefault(); orbit.dist = Math.min(2200, Math.max(220, orbit.dist * Math.exp(e.deltaY * 0.0012))); kick(); }, { passive: false });
    stage.addEventListener("dblclick", () => { Object.assign(orbit, { ...home, target: home.target.clone() }); kick(); });
    new ResizeObserver(() => { size(); kick(); }).observe(stage);
    size();
    select("twin");
  }
  document.querySelectorAll(".view-btn").forEach((b) => b.addEventListener("click", () => {
    select(b.dataset.design);
    document.getElementById("viewer").scrollIntoView({ behavior: still ? "auto" : "smooth" });
  }));

  // ---------------------------------------------------------------- J5 charts
  const root = document.getElementById("charts");
  const W = 320, H = 196, ML = 34, MR = 10, MT = 10, MB = 28;
  const X = (s) => ML + ((s + 68) / 136) * (W - ML - MR);
  const Y = (v) => MT + (1 - v / 130) * (H - MT - MB);
  const NS = "http://www.w3.org/2000/svg";
  const el = (tag, attrs, parent) => { const n = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v); if (parent) parent.appendChild(n); return n; };
  const series = CHART.designs.map((d, i) => ({ ...d, slot: i + 1 }));
  for (const [layout, title] of CHART.layouts) {
    const head = document.createElement("div");
    head.className = "row-title";
    head.textContent = title;
    root.appendChild(head);
    if (layout === "out") {
      const note = document.createElement("p");
      note.className = "note";
      note.style.margin = "0 0 8px";
      note.textContent = `Each head at its best camera roll, counted from where it sits looking back: ${series.map((s) => `${s.title} ${(((s.out.roll - s.back.roll) % 360) + 360) % 360}°`).join(", ")}. Twin at 0° would hit the bench.`;
      root.appendChild(note);
    }
    const panels = document.createElement("div");
    panels.className = "panels";
    root.appendChild(panels);
    CHART.passes.forEach((passName, p) => {
      const panel = document.createElement("div");
      panel.className = "panel";
      panel.innerHTML = `<h4>Pass ${p + 1} · ${passName}</h4>`;
      const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, role: "img", "aria-label": `J5 along the seam, ${title}, pass ${p + 1}: ${series.map((s) => { const v = s[layout].j5?.[p]?.j5.map(Math.abs) ?? []; return `${s.title} ${Math.min(...v).toFixed(0)} to ${Math.max(...v).toFixed(0)} degrees`; }).join("; ")}` }, null);
      panel.appendChild(svg);
      el("rect", { class: "zone-band", x: ML, y: Y(15), width: W - ML - MR, height: Y(0) - Y(15) }, svg);
      const grid = el("g", { class: "grid" }, svg);
      for (const v of [0, 30, 60, 90, 120]) el("line", { x1: ML, x2: W - MR, y1: Y(v), y2: Y(v) }, grid);
      el("line", { class: "limit-line", x1: ML, x2: W - MR, y1: Y(124), y2: Y(124) }, svg);
      const lim = el("text", { class: "zone-label", x: W - MR, y: Y(124) - 3, "text-anchor": "end" }, svg);
      lim.textContent = "limit 124°";
      const zl = el("text", { class: "zone-label", x: ML + 4, y: Y(2.5) }, svg);
      zl.textContent = "singular";
      const axis = el("g", { class: "axis" }, svg);
      for (const v of [0, 30, 60, 90, 120]) { const t = el("text", { x: ML - 6, y: Y(v) + 3.5, "text-anchor": "end" }, axis); t.textContent = `${v}°`; }
      for (const s of [-60, -30, 0, 30, 60]) { const t = el("text", { x: X(s), y: H - 10, "text-anchor": "middle" }, axis); t.textContent = s === 0 ? "0" : `${s > 0 ? "+" : "−"}${Math.abs(s)}`; }
      const xl = el("text", { x: W - MR, y: H - 10, "text-anchor": "end" }, axis);
      xl.textContent = "";
      for (const s of series) {
        const pass = s[layout].j5?.[p];
        if (!pass) continue;
        const d = pass.s.map((sv, i) => `${i ? "L" : "M"}${X(sv).toFixed(1)} ${Y(Math.abs(pass.j5[i])).toFixed(1)}`).join("");
        el("path", { class: "series", d, stroke: `var(--s${s.slot})` }, svg);
      }
      const cross = el("line", { x1: 0, x2: 0, y1: MT, y2: H - MB, stroke: "var(--muted)", "stroke-width": 1, visibility: "hidden" }, svg);
      const dots = series.map((s) => el("circle", { r: 4, fill: `var(--s${s.slot})`, stroke: "var(--surface)", "stroke-width": 2, visibility: "hidden" }, svg));
      const hit = el("rect", { x: ML, y: MT, width: W - ML - MR, height: H - MT - MB, fill: "transparent" }, svg);
      const tip = document.createElement("div");
      tip.className = "tip";
      panel.appendChild(tip);
      const show = (e) => {
        const box = svg.getBoundingClientRect();
        const vx = ((e.clientX - box.left) / box.width) * W;
        const sv = Math.max(-68, Math.min(68, Math.round((((vx - ML) / (W - ML - MR)) * 136 - 68) / 4) * 4));
        cross.setAttribute("x1", X(sv)); cross.setAttribute("x2", X(sv)); cross.setAttribute("visibility", "visible");
        const lines = [`seam y ${sv > 0 ? "+" : ""}${sv} mm`];
        series.forEach((s, i) => {
          const pass = s[layout].j5?.[p];
          const k = pass ? pass.s.indexOf(sv) : -1;
          if (k < 0) { dots[i].setAttribute("visibility", "hidden"); return; }
          const v = Math.abs(pass.j5[k]);
          dots[i].setAttribute("cx", X(sv)); dots[i].setAttribute("cy", Y(v)); dots[i].setAttribute("visibility", "visible");
          lines.push(`<span class="key" data-s="${s.slot}"></span> ${s.title} ${v.toFixed(1)}°`);
        });
        tip.innerHTML = lines.join("<br>");
        tip.style.display = "block";
        const px = (X(sv) / W) * box.width;
        tip.style.left = `${Math.min(px + 14, box.width - tip.offsetWidth + 10)}px`;
        tip.style.top = `${28}px`;
      };
      const hide = () => { tip.style.display = "none"; cross.setAttribute("visibility", "hidden"); dots.forEach((d) => d.setAttribute("visibility", "hidden")); };
      hit.addEventListener("pointermove", show);
      hit.addEventListener("pointerleave", hide);
      panels.appendChild(panel);
    });
  }
  // the table view
  const rows = [];
  for (const [layout, title] of CHART.layouts)
    CHART.passes.forEach((passName, p) => {
      const cells = series.map((s) => { const v = s[layout].j5?.[p]?.j5.map(Math.abs); return v ? `${Math.min(...v).toFixed(0)}–${Math.max(...v).toFixed(0)}°` : "—"; });
      rows.push(`<tr><th scope="row">${title.split(" · ")[0]} · pass ${p + 1} (${passName})</th>${cells.map((c) => `<td>${c}</td>`).join("")}</tr>`);
    });
  document.getElementById("chart-table").innerHTML = `<table><thead><tr><th>|J5| range</th>${series.map((s) => `<th scope="col">${s.title}</th>`).join("")}</tr></thead><tbody>${rows.join("")}</tbody></table>`;
})();
