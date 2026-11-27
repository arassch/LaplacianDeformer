import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const view = $('view'), statusEl = $('status'), modeEl = $('mode'), rectEl = $('rect');

// ---------- renderer + one shared camera ----------
const renderer = new THREE.WebGLRenderer({ canvas: view, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x2b2d31);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 1000);
const controls = new OrbitControls(camera, view);

const meshMat = new THREE.MeshStandardMaterial({ color: 0xffffcc, roughness: 0.65, metalness: 0, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
const tintMat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.65, metalness: 0, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
const heatMat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.7, metalness: 0, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
// Zebra / reflection lines: stripes of the reflected view direction. Creases and kinks show up as broken stripes.
const zebraMat = new THREE.ShaderMaterial({
  side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
  uniforms: { freq: { value: 14.0 } },
  vertexShader: `varying vec3 vN; varying vec3 vP;
    void main(){ vec4 mv = modelViewMatrix * vec4(position, 1.0); vP = mv.xyz; vN = normalMatrix * normal; gl_Position = projectionMatrix * mv; }`,
  fragmentShader: `uniform float freq; varying vec3 vN; varying vec3 vP;
    void main(){ vec3 n = normalize(vN); if (!gl_FrontFacing) n = -n;
      vec3 r = reflect(normalize(vP), n);
      float s = smoothstep(-0.12, 0.12, sin(r.y * freq));
      vec3 c = mix(vec3(0.07, 0.07, 0.09), vec3(0.98, 0.97, 0.90), s);
      c *= 0.6 + 0.4 * max(dot(n, normalize(vec3(0.3, 0.6, 1.0))), 0.0);
      gl_FragColor = vec4(c, 1.0); }`,
});
const SHADING = ['shaded', 'displacement', 'zebra'];
let shading = 'shaded';
function turbo(x) {
  x = Math.min(1, Math.max(0, x));
  const r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
  const g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
  const b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
  return [Math.min(1, Math.max(0, r)), Math.min(1, Math.max(0, g)), Math.min(1, Math.max(0, b))];
}
const edgeMat = new THREE.LineBasicMaterial({ color: 0x1b1b1b, transparent: true, opacity: 0.55 });

function makeScene() {
  const s = new THREE.Scene();
  s.add(new THREE.HemisphereLight(0xffffff, 0x404050, 1.6));
  const key = new THREE.DirectionalLight(0xffffff, 1.6); key.position.set(3, 5, 4); s.add(key);
  const rim = new THREE.DirectionalLight(0xbfd4ff, 0.6); rim.position.set(-4, 2, -3); s.add(rim);
  return s;
}

// ---------- state ----------
// Two views, each with its own copy of Sara's Deformer (separate WASM instances) and its own Ks/Kb.
const views = [0, 1].map((i) => ({ i, M: null, scene: makeScene(), geom: null, mesh: null, edges: null, selPts: null, fixPts: null }));
let compare = true;
let N = 0, objText = null, objName = '';
const loadedTexts = {};          // meshes loaded this session, so demos can switch back to them
let selection = [], fixed = [];
let moveMode = false, undoStack = [], showEdges = true, showMarkers = true;

const active = () => (compare ? views : [views[0]]);
const call = (v, n, r, t, a) => v.M.ccall(n, r, t, a);
const positions = (v) => new Float32Array(v.M.HEAPF32.buffer, call(v, 'ld_positions', 'number', [], []), 3 * N);
const restPositions = (v) => new Float32Array(v.M.HEAPF32.buffer, call(v, 'ld_restPositions', 'number', [], []), 3 * N);
const setStatus = (t) => { statusEl.textContent = t; };
const kVal = (i, k) => Math.max(0, +$(k + i).value || 0);

// ---------- layout ----------
function vpRect(i) {               // CSS-pixel viewport of view i inside the canvas
  const r = view.getBoundingClientRect();
  if (!compare) return { x: 0, y: 0, w: r.width, h: r.height };
  const w = r.width / 2;
  return { x: i * w, y: 0, w, h: r.height };
}
function viewAt(x) { return compare && x >= vpRect(1).x ? 1 : 0; }

function resize() {
  const r = view.parentElement.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  const vp = vpRect(0);
  camera.aspect = vp.w / Math.max(1, vp.h); camera.updateProjectionMatrix();
  document.body.classList.toggle('compare', compare);
  render();
}
new ResizeObserver(resize).observe(view.parentElement);

function draw() {
  const r = view.getBoundingClientRect();
  renderer.setScissorTest(true);
  for (const v of active()) {
    const p = vpRect(v.i);
    renderer.setViewport(p.x, r.height - p.y - p.h, p.w, p.h);
    renderer.setScissor(p.x, r.height - p.y - p.h, p.w, p.h);
    renderer.render(v.scene, camera);
  }
  renderer.setScissorTest(false);
}
let pending = false;
function render() { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; draw(); }); }
controls.addEventListener('change', render);

function updateLabels() {
  for (const i of [0, 1]) $('lab' + i).textContent = `Ks ${kVal(i, 'ks')} · Kb ${kVal(i, 'kb')}`;
}

// ---------- loading ----------
function buildView(v, idx, nf) {
  if (v.mesh) { v.scene.remove(v.mesh, v.edges, v.selPts, v.fixPts); v.geom.dispose(); }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions(v)), 3));
  g.setIndex(idx); g.computeVertexNormals();
  g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3 * N), 3));
  v.geom = g;
  v.base = new Float32Array(g.attributes.position.array);     // shape as loaded: displacement is measured from here
  v.mesh = new THREE.Mesh(g, materialFor());
  const eg = new THREE.BufferGeometry();
  eg.setAttribute('position', g.attributes.position);
  const seen = new Set(), ei = [];
  for (let t = 0; t < idx.length; t += 3) for (let k = 0; k < 3; k++) {
    const a = idx[t + k], b = idx[t + (k + 1) % 3], key = a < b ? a * 4194304 + b : b * 4194304 + a;
    if (!seen.has(key)) { seen.add(key); ei.push(a, b); }
  }
  eg.setIndex(ei);
  v.edges = new THREE.LineSegments(eg, edgeMat); v.edges.visible = showEdges;
  const size = N > 5000 ? 4 : 10;
  const pm = (c) => new THREE.PointsMaterial({ color: c, size, sizeAttenuation: false });
  v.selPts = new THREE.Points(new THREE.BufferGeometry(), pm(0xff2a2a)); v.selPts.renderOrder = 3; v.selPts.frustumCulled = false;
  v.fixPts = new THREE.Points(new THREE.BufferGeometry(), pm(0xb3b3b3)); v.fixPts.renderOrder = 2; v.fixPts.frustumCulled = false;
  v.selPts.visible = v.fixPts.visible = showMarkers;
  v.scene.add(v.mesh, v.edges, v.fixPts, v.selPts);
}

const tinted = () => N > 5000;     // big meshes: show fixed/handle as surface tint instead of thousands of dots
function materialFor() { return shading === 'zebra' ? zebraMat : shading === 'displacement' ? heatMat : tinted() ? tintMat : meshMat; }
function updateTint() {
  if (shading !== 'shaded' || !tinted()) return;
  const fx = new Set(fixed), sl = new Set(selection);
  for (const v of views) { if (!v.geom) continue; const c = v.geom.attributes.color.array;
    for (let k = 0; k < N; k++) {
      const col = !showMarkers ? [1, 1, 0.8] : sl.has(k) ? [0.95, 0.2, 0.17] : fx.has(k) ? [0.62, 0.64, 0.68] : [1, 1, 0.8];
      c[3 * k] = col[0]; c[3 * k + 1] = col[1]; c[3 * k + 2] = col[2]; }
    v.geom.attributes.color.needsUpdate = true; }
}
// Colour by distance from the loaded shape, on one scale shared by both views.
function updateColors() {
  if (shading !== 'displacement') return;
  const vs = active().filter((v) => v.geom);
  const mags = vs.map((v) => { const p = v.geom.attributes.position.array, m = new Float32Array(N);
    for (let k = 0; k < N; k++) m[k] = Math.hypot(p[3 * k] - v.base[3 * k], p[3 * k + 1] - v.base[3 * k + 1], p[3 * k + 2] - v.base[3 * k + 2]); return m; });
  let mx = 0; for (const m of mags) for (let k = 0; k < N; k++) if (m[k] > mx) mx = m[k];
  vs.forEach((v, j) => { const c = v.geom.attributes.color.array, m = mags[j];
    for (let k = 0; k < N; k++) { const t = turbo(mx > 0 ? m[k] / mx : 0); c[3 * k] = t[0]; c[3 * k + 1] = t[1]; c[3 * k + 2] = t[2]; }
    v.geom.attributes.color.needsUpdate = true; });
  $('legMax').textContent = mx > 0 ? mx.toPrecision(3) : '0';
}
function cycleShading() {
  shading = SHADING[(SHADING.indexOf(shading) + 1) % SHADING.length];
  for (const v of views) if (v.mesh) v.mesh.material = materialFor();
  document.body.dataset.shading = shading;
  $('bShade').querySelector('span').textContent = { shaded: 'Shaded', displacement: 'Displacement', zebra: 'Zebra' }[shading];
  updateColors(); refreshPoints();
}

function loadOBJ(text, name) {
  if (!text || !text.trim()) { setStatus(`${name} is empty (0 bytes). If it lives in Dropbox, it may not have finished downloading yet.`); return false; }
  const t0 = performance.now();
  let nf = 0, idx = null;
  for (const v of views) {
    v.M.FS.writeFile('/mesh.obj', text);
    const n = call(v, 'ld_load', 'number', ['string'], ['/mesh.obj']);
    if (!n) { setStatus(`Could not read ${name} as an OBJ triangle mesh.`); return false; }
    N = n;
    call(v, 'ld_setKs', null, ['number'], [kVal(v.i, 'ks')]);
    call(v, 'ld_setKb', null, ['number'], [kVal(v.i, 'kb')]);
    if (!idx) {
      nf = call(v, 'ld_numFaces', 'number', [], []);
      idx = Array.from(new Int32Array(v.M.HEAP32.buffer, call(v, 'ld_faces', 'number', [], []), 3 * nf));
    }
  }
  objText = text; objName = name; loadedTexts[name] = text;
  selection = []; fixed = []; undoStack = []; moveMode = false;
  showEdges = nf <= 20000;
  for (const v of views) buildView(v, idx, nf);
  frame(); updateColors(); refreshPoints();
  const ms = performance.now() - t0;
  setStatus(`${name}: ${N} vertices, ${nf} triangles (set-up ${ms.toFixed(0)} ms)` + (N > 20000 ? ' · big mesh: each new drag pauses a few seconds to factorise' : '') + (showEdges ? '' : ' · wireframe hidden, press E to show'));
  return true;
}

function frame(dir) {
  const g = views[0].geom; g.computeBoundingSphere();
  const s = g.boundingSphere, vp = vpRect(0);
  camera.aspect = vp.w / Math.max(1, vp.h);
  const vf = THREE.MathUtils.degToRad(camera.fov) / 2, hf = Math.atan(Math.tan(vf) * camera.aspect);
  const d = s.radius / Math.sin(Math.min(vf, hf)) * 1.02;
  controls.target.copy(s.center);
  camera.position.copy(s.center).add((dir || new THREE.Vector3(0.35, 0.75, 1)).clone().normalize().multiplyScalar(d));
  camera.near = d / 100; camera.far = d * 100; camera.updateProjectionMatrix();
  controls.update(); render();
}

function refreshGeometry(v) {
  v.geom.attributes.position.array.set(positions(v));
  v.geom.attributes.position.needsUpdate = true;
  v.geom.computeVertexNormals();
}
function refreshPoints() {
  for (const v of views) {
    if (!v.geom) continue;
    const p = v.geom.attributes.position.array;
    const pts = (ids) => { const a = new Float32Array(ids.length * 3); ids.forEach((id, i) => a.set(p.subarray(3 * id, 3 * id + 3), 3 * i)); return new THREE.BufferAttribute(a, 3); };
    v.selPts.geometry.setAttribute('position', pts(tinted() ? [] : selection));     // big meshes: no dots in any mode
    v.fixPts.geometry.setAttribute('position', pts(tinted() ? [] : fixed));
  }
  updateTint(); updateMode(); render();
}

// ---------- picking (in the view under the mouse) ----------
const tmp = new THREE.Vector3();
function toLocal(p, i) { tmp.set(p[0], p[1], p[2]).project(camera); const vp = vpRect(i); return [(tmp.x + 1) / 2 * vp.w, (1 - tmp.y) / 2 * vp.h, tmp.z]; }
function selectRect(i, x0, y0, x1, y1, add) {
  const [ax, bx] = [Math.min(x0, x1), Math.max(x0, x1)], [ay, by] = [Math.min(y0, y1), Math.max(y0, y1)];
  const p = views[i].geom.attributes.position.array, sel = new Set(selection);
  let any = false;
  for (let k = 0; k < N; k++) {
    const [sx, sy, sz] = toLocal(p.subarray(3 * k, 3 * k + 3), i);
    if (sz < 1 && sx >= ax && sx <= bx && sy >= ay && sy <= by) { any = true; if (add) sel.add(k); else sel.delete(k); }
  }
  selection = any ? [...sel] : [];
  refreshPoints();
}
function nearestVertex(i, x, y, maxPx) {
  const p = views[i].geom.attributes.position.array;
  let best = -1, bd = maxPx * maxPx;
  for (let k = 0; k < N; k++) {
    const [sx, sy, sz] = toLocal(p.subarray(3 * k, 3 * k + 3), i);
    const d = (sx - x) ** 2 + (sy - y) ** 2;
    if (sz < 1 && d < bd) { bd = d; best = k; }
  }
  return best;
}

// ---------- the manipulation, reproduced in every view ----------
// The drag is turned into one world-space displacement per handle (measured in the view you drag in,
// exactly like DeformViewer: project rest position, add the mouse offset, unproject at the same depth).
// The same displacements are then applied to the handles in every view; fixed vertices get zero.
let drag = null, lastMove = null, busy = false, lastSolveMs = 0;

function solveTo(dx, dy) {
  const src = views[drag.view], vp = vpRect(drag.view);
  const rest = restPositions(src), disp = new Float32Array(3 * selection.length);
  selection.forEach((id, k) => {
    tmp.set(rest[3 * id], rest[3 * id + 1], rest[3 * id + 2]).project(camera);
    tmp.x += 2 * dx / vp.w; tmp.y -= 2 * dy / vp.h;
    tmp.unproject(camera);
    disp[3 * k] = tmp.x - rest[3 * id]; disp[3 * k + 1] = tmp.y - rest[3 * id + 1]; disp[3 * k + 2] = tmp.z - rest[3 * id + 2];
  });
  const selSet = new Set(selection), fx = fixed.filter((id) => !selSet.has(id));
  const times = [];
  for (const v of active()) {
    const r = restPositions(v), targets = new Float32Array(3 * selection.length);
    selection.forEach((id, k) => { for (let a = 0; a < 3; a++) targets[3 * k + a] = r[3 * id + a] + disp[3 * k + a]; });
    const M = v.M, base = M._malloc(4 * (selection.length + fx.length + targets.length));
    const hP = base, fP = hP + 4 * selection.length, tP = fP + 4 * fx.length;
    M.HEAP32.set(selection, hP >> 2); M.HEAP32.set(fx, fP >> 2); M.HEAPF32.set(targets, tP >> 2);
    const t0 = performance.now();
    call(v, 'ld_deform', null, ['number', 'number', 'number', 'number', 'number'], [hP, tP, selection.length, fP, fx.length]);
    times.push(performance.now() - t0);
    M._free(base);
    refreshGeometry(v);
  }
  lastSolveMs = Math.max(...times);
  updateColors();
  refreshPoints();
}
function liveSolve(dx, dy) {
  if (busy) { lastMove = [dx, dy]; return; }
  busy = true;
  try {
    solveTo(dx, dy);
    while (lastMove) { const m = lastMove; lastMove = null; solveTo(m[0], m[1]); }
    draw();
  } catch (err) { setStatus('Deform error: ' + (err && err.message || err)); }
  finally { busy = false; }
}

function localXY(e) { const r = view.getBoundingClientRect(); const x = e.clientX - r.left, y = e.clientY - r.top; const i = drag ? drag.view : viewAt(x); return [i, x - vpRect(i).x, y - vpRect(i).y]; }

view.addEventListener('pointerdown', () => { const a = document.activeElement; if (a && a.tagName === 'INPUT') a.blur(); }, true);
view.addEventListener('mousedown', (e) => { if (e.shiftKey || e.ctrlKey || e.metaKey) e.preventDefault(); });
view.addEventListener('pointerdown', (e) => {
  if (!N || e.button !== 0) return;
  const [i, x, y] = localXY(e);
  if (e.shiftKey || e.ctrlKey || e.metaKey) {
    drag = { kind: 'select', view: i, x0: x, y0: y, add: e.shiftKey };
    const vp = vpRect(i);
    Object.assign(rectEl.style, { display: 'block', left: vp.x + x + 'px', top: y + 'px', width: '0px', height: '0px' });
  } else if (moveMode) {
    if (!selection.length) {
      const id = nearestVertex(i, x, y, 14);
      if (id < 0) { setStatus('Nothing to move: Shift-drag around vertices to pick a handle, or press on a vertex.'); e.preventDefault(); e.stopImmediatePropagation(); return; }
      selection = [id]; refreshPoints();
    }
    if (!fixed.some((k) => !selection.includes(k)))
      setStatus('No fixed vertices, so the whole mesh just translates. Shift-drag some vertices and press S to fix them.');
    drag = { kind: 'move', view: i, x0: x, y0: y };
    undoStack.push(views.map((v) => (v.M ? new Float32Array(positions(v)) : null))); if (undoStack.length > 50) undoStack.shift();
  } else return;
  controls.enabled = false;
  view.setPointerCapture(e.pointerId);
  e.preventDefault(); e.stopImmediatePropagation();
}, true);

view.addEventListener('pointermove', (e) => {
  if (!drag) return;
  const [, x, y] = localXY(e);
  if (drag.kind === 'select') {
    const vp = vpRect(drag.view);
    Object.assign(rectEl.style, { left: vp.x + Math.min(x, drag.x0) + 'px', top: Math.min(y, drag.y0) + 'px', width: Math.abs(x - drag.x0) + 'px', height: Math.abs(y - drag.y0) + 'px' });
  } else liveSolve(x - drag.x0, y - drag.y0);
});

function endDrag(e) {
  if (!drag) return;
  const [, x, y] = localXY(e);
  if (drag.kind === 'select') { rectEl.style.display = 'none'; selectRect(drag.view, drag.x0, drag.y0, x, y, drag.add); }
  else {
    liveSolve(x - drag.x0, y - drag.y0);
    for (const v of active()) call(v, 'ld_commit', null, [], []);      // release: shape becomes the rest pose, as in DeformViewer
    if (fixed.some((k) => !selection.includes(k))) setStatus(`${objName}: ${N} vertices · last solve ${lastSolveMs.toFixed(0)} ms` + (compare ? ' per view' : ''));
  }
  drag = null; controls.enabled = true;
}
view.addEventListener('pointerup', endDrag);
view.addEventListener('pointercancel', endDrag);

// ---------- commands ----------
function updateMode() {
  modeEl.textContent = (moveMode ? 'Move mode: drag to deform the red handle' : 'View mode: drag to orbit') + ` · ${selection.length} handle · ${fixed.length} fixed`;
  modeEl.classList.toggle('on', moveMode);
  $('bMove').setAttribute('aria-pressed', moveMode);
  $('bCompare').setAttribute('aria-pressed', compare);
}
function setFixed() { if (!selection.length) { setStatus('Select vertices first (Shift-drag), then press S to fix them.'); return; } fixed = selection.slice(); selection = []; refreshPoints(); }
function unfix() {
  if (selection.length) { const sel = new Set(selection), before = fixed.length; fixed = fixed.filter((i) => !sel.has(i)); setStatus(`Unfixed ${before - fixed.length} of the selected vertices.`); }
  else { setStatus(`Cleared all ${fixed.length} fixed vertices.`); fixed = []; }
  refreshPoints();
}
function toggleMove() { moveMode = !moveMode; updateMode(); }
function undo() {
  const s = undoStack.pop(); if (!s) return;
  views.forEach((v, k) => { if (!s[k]) return; const p = v.M._malloc(4 * s[k].length); v.M.HEAPF32.set(s[k], p >> 2); call(v, 'ld_setShape', null, ['number'], [p]); v.M._free(p); refreshGeometry(v); });
  updateColors(); refreshPoints();
}
function reset() {
  if (!objText) return;
  const keepSel = selection, keepFix = fixed;
  if (!loadOBJ(objText, objName)) return;
  if (objName === 'plane.obj' && !keepFix.length) defaultSetup();
  else { selection = keepSel; fixed = keepFix; refreshPoints(); }       // reset the shape, keep the set-up
}
// Make both views the same shape again (copy the left view's current shape into the right one).
function syncShapes() {
  const s = new Float32Array(positions(views[0])), v = views[1], p = v.M._malloc(4 * s.length);
  v.M.HEAPF32.set(s, p >> 2); call(v, 'ld_setShape', null, ['number'], [p]); v.M._free(p); refreshGeometry(v); updateColors(); refreshPoints();
}
function toggleCompare() {
  compare = !compare;
  if (compare) syncShapes();              // the right view missed drags made in single view
  resize(); updateMode();
}
function toggleUI() { document.body.classList.toggle('clean'); resize(); }
function toggleMarkers() { showMarkers = !showMarkers; for (const v of views) if (v.selPts) { v.selPts.visible = v.fixPts.visible = showMarkers; } $('bMarkers').setAttribute('aria-pressed', !showMarkers); refreshPoints(); }
function toggleEdges() { showEdges = !showEdges; for (const v of views) if (v.edges) v.edges.visible = showEdges; render(); }

addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' && e.target.type !== 'file') {
    if (e.key === 'Escape' || e.key === 'Enter') { e.target.dispatchEvent(new Event('change')); e.target.blur(); e.preventDefault(); }
    return;
  }
  const k = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && k === 'z') { undo(); e.preventDefault(); return; }
  const dm = /^Digit([123])$/.exec(e.code) || /^Numpad([123])$/.exec(e.code);
  if (dm && !e.ctrlKey && !e.metaKey && !e.altKey) { if (e.shiftKey) saveDemo(+dm[1]); else applyDemo(+dm[1]); e.preventDefault(); return; }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (k === 'w') toggleMove();
  else if (k === 's') setFixed();
  else if (k === 'u') unfix();
  else if (k === 'f') frame(demo && DEMOS[demo].view);
  else if (k === 'h') toggleUI();
  else if (k === 'r') reset();
  else if (k === 'e') toggleEdges();
  else if (k === 'c') toggleCompare();
  else if (k === 'v') cycleShading();
  else if (k === 'm') toggleMarkers();
  else return;
  e.preventDefault();
});

$('bFix').onclick = setFixed; $('bUnfix').onclick = unfix; $('bMove').onclick = toggleMove; $('bUndo').onclick = undo;
$('bReset').onclick = reset; $('bHide').onclick = toggleUI; $('bFrame').onclick = () => frame(); $('bEdges').onclick = toggleEdges;
$('bCompare').onclick = toggleCompare;
$('bShade').onclick = cycleShading;
$('bMarkers').onclick = toggleMarkers;
for (const i of [0, 1]) for (const k of ['ks', 'kb']) $(k + i).addEventListener('change', (e) => {
  const val = Math.max(0, +e.target.value || 0); e.target.value = val;
  if (views[i].M && N) call(views[i], k === 'ks' ? 'ld_setKs' : 'ld_setKb', null, ['number'], [val]);
  if (!kVal(i, 'ks') && !kVal(i, 'kb')) setStatus('Ks and Kb are both 0 in one view: the system has no solution. Set one of them above 0.');
  updateLabels();
});

function readFile(f) { if (!f) return; f.text().then((t) => loadOBJ(t, f.name)); }
$('file').addEventListener('change', (e) => { readFile(e.target.files[0]); e.target.value = ''; e.target.blur(); });
const stage = $('stage');
stage.addEventListener('dragover', (e) => { e.preventDefault(); stage.classList.add('drop'); });
stage.addEventListener('dragleave', () => stage.classList.remove('drop'));
stage.addEventListener('drop', (e) => { e.preventDefault(); stage.classList.remove('drop'); readFile(e.dataTransfer.files[0]); });

// starting set-up on the plane: left edge fixed, near corner as the handle
function defaultSetup() {
  const p = views[0].geom.attributes.position.array;
  fixed = []; selection = [];
  for (let i = 0; i < N; i++) { if (p[3 * i] < -1.5) fixed.push(i); if (p[3 * i] > 1.5 && p[3 * i + 2] > 1.5) selection.push(i); }
  refreshPoints();
}

// ---------- demos (keys 1-3; Shift+1-3 saves the current set-up into that slot) ----------
const DEMOS = {
  1: { title: 'Plane, one edge fixed', mesh: 'plane.obj', text: () => window.DEFAULT_OBJ,
       setup(p) { const f = [], h = []; for (let i = 0; i < N; i++) { if (p[3 * i] < -1.5) f.push(i); if (p[3 * i] > 1.5 && p[3 * i + 2] > 1.5) h.push(i); } return { f, h }; } },
  2: { title: 'Plane, all edges fixed', mesh: 'plane.obj', text: () => window.DEFAULT_OBJ,
       setup(p) { const f = [], h = []; for (let i = 0; i < N; i++) { const x = p[3 * i], z = p[3 * i + 2];
         if (Math.abs(x) > 1.5 || Math.abs(z) > 1.5) f.push(i); if (Math.abs(x) < 0.01 && Math.abs(z) < 0.01) h.push(i); } return { f, h }; } },
  3: { title: 'Kitten, body and lower head fixed', mesh: 'kitten.obj', text: () => window.KITTEN_OBJ || loadedTexts['kitten.obj'], view: new THREE.Vector3(0.45, 0.25, 1),
       setup(p) { const f = [], h = [], c = [0.023, 0.431, 0.027];
         for (let i = 0; i < N; i++) { const y = p[3 * i + 1];
           if (y < 0.20) f.push(i);
           if (Math.hypot(p[3 * i] - c[0], y - c[1], p[3 * i + 2] - c[2]) < 0.04) h.push(i); } return { f, h }; } },
};
let demo = 0;
const saved = {};
try { for (const n of [1, 2, 3]) { const t = localStorage.getItem('ld-demo-' + n); if (t) saved[n] = JSON.parse(t); } } catch (e) {}

function restoreShape() {           // back to the shape as loaded, in both views, without reloading the file
  for (const v of views) { const p = v.M._malloc(4 * v.base.length); v.M.HEAPF32.set(v.base, p >> 2); call(v, 'ld_setShape', null, ['number'], [p]); v.M._free(p); refreshGeometry(v); }
  undoStack = [];
}
function applyDemo(n) {
  const d = DEMOS[n];
  if (objName === d.mesh && N) restoreShape();
  else {
    const text = d.text();
    if (!text) { setStatus(`Demo ${n} needs ${d.mesh}: load it once with Load OBJ (or build with KITTEN=path/to/kitten.obj), then press ${n}.`); return; }
    setStatus(`Loading ${d.mesh}…`); if (!loadOBJ(text, d.mesh)) return;
  }
  demo = n;
  const sv = saved[n] && saved[n].mesh === d.mesh && saved[n].N === N ? saved[n] : null;
  if (sv) { fixed = sv.fixed.slice(); selection = sv.selection.slice(); }
  else { const r = d.setup(views[0].base); fixed = r.f; selection = r.h; }
  if (sv && sv.cam) { camera.position.fromArray(sv.cam.pos); controls.target.fromArray(sv.cam.target); camera.updateProjectionMatrix(); controls.update(); }
  else frame(d.view);
  moveMode = true;
  updateColors(); refreshPoints();
  setStatus(`Demo ${n}: ${d.title}${sv ? ' (your saved set-up)' : ''} · drag to deform · ${N} vertices`);
}
function saveDemo(n) {
  if (!N) return;
  const rec = { mesh: objName, N, fixed, selection, cam: { pos: camera.position.toArray(), target: controls.target.toArray() } };
  saved[n] = rec; demo = n;
  try { localStorage.setItem('ld-demo-' + n, JSON.stringify(rec)); } catch (e) {}
  setStatus(`Saved the current set-up (${fixed.length} fixed, ${selection.length} handle, camera) as demo ${n}.`);
}

// ---------- boot ----------
updateLabels(); resize();
Promise.all(views.map(() => window.createDeformer({ print: () => {}, printErr: () => {} }))).then((mods) => {
  mods.forEach((m, k) => { views[k].M = m; });
  applyDemo(1);
}).catch((err) => setStatus('Could not start the deformer: ' + err));
