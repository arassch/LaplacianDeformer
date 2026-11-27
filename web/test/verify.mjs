// Checks that the sparse cached solve used by the page (ld_deform) gives the same result as the
// original dense Deformer::deform (ld_deformOriginal), on plane.obj and on a generated icosphere.
// Run after ./build.sh:  node test/verify.mjs
import fs from 'fs';
import { createRequire } from 'module';
const require = createRequire(import.meta.url);
const here = new URL('..', import.meta.url).pathname;
const createDeformer = require(here + 'build/deformer.js');
process.setMaxListeners(0);   // one listener per WASM instance

function icosphere(sub) {
  const t = (1 + 5 ** 0.5) / 2;
  let V = [[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t], [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]]
    .map((v) => { const l = Math.hypot(...v); return v.map((c) => c / l); });
  let F = [[0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8], [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1]];
  for (let s = 0; s < sub; s++) {
    const cache = new Map(), NF = [];
    const mid = (a, b) => { const k = Math.min(a, b) + '_' + Math.max(a, b);
      if (!cache.has(k)) { const m = V[a].map((x, i) => (x + V[b][i]) / 2), l = Math.hypot(...m); V.push(m.map((x) => x / l)); cache.set(k, V.length - 1); }
      return cache.get(k); };
    for (const [a, b, c] of F) { const ab = mid(a, b), bc = mid(b, c), ca = mid(c, a); NF.push([a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]); }
    F = NF;
  }
  return V.map((v) => 'v ' + v.join(' ')).join('\n') + '\n' + F.map((f) => 'f ' + f.map((i) => i + 1).join(' ')).join('\n') + '\n';
}

async function run(objText, fn, ks, kb, pick) {
  const M = await createDeformer({ print: () => {}, printErr: () => {} });
  M.FS.writeFile('/m.obj', objText);
  const n = M.ccall('ld_load', 'number', ['string'], ['/m.obj']);
  M.ccall('ld_setKs', null, ['number'], [ks]); M.ccall('ld_setKb', null, ['number'], [kb]);
  const rc = (i, a) => M.ccall('ld_restCoord', 'number', ['number', 'number'], [i, a]);
  const { h, f } = pick(n, rc);
  const tg = h.flatMap((i) => [rc(i, 0), rc(i, 1) + 0.8, rc(i, 2) + 0.3]);
  const p = M._malloc(4 * (h.length + f.length + tg.length)), fP = p + 4 * h.length, tP = fP + 4 * f.length;
  M.HEAP32.set(h, p >> 2); M.HEAP32.set(f, fP >> 2); M.HEAPF32.set(tg, tP >> 2);
  M.ccall(fn, null, ['number', 'number', 'number', 'number', 'number'], [p, tP, h.length, fP, f.length]);
  return Float32Array.from(new Float32Array(M.HEAPF32.buffer, M.ccall('ld_positions', 'number', [], []), 3 * n));
}

const cases = [
  ['plane.obj', fs.readFileSync(here + 'plane.obj', 'utf8'), (n, rc) => { const h = [], f = []; for (let i = 0; i < n; i++) { if (rc(i, 0) < -1.5) f.push(i); if (rc(i, 0) > 1.5 && rc(i, 2) > 1.5) h.push(i); } return { h, f }; }],
  ['icosphere', icosphere(2), (n, rc) => { const h = [], f = []; for (let i = 0; i < n; i++) { if (rc(i, 1) < -0.8) f.push(i); if (rc(i, 1) > 0.95) h.push(i); } return { h, f }; }],
];
let ok = true;
for (const [name, text, pick] of cases) for (const [ks, kb] of [[1, 0], [0, 1], [1, 1]]) {
  const a = await run(text, 'ld_deformOriginal', ks, kb, pick), b = await run(text, 'ld_deform', ks, kb, pick);
  let md = 0; for (let i = 0; i < a.length; i++) md = Math.max(md, Math.abs(a[i] - b[i]));
  const pass = md < 1e-4; ok &&= pass;
  console.log(`${pass ? 'ok  ' : 'FAIL'} ${name} Ks=${ks} Kb=${kb}  max |original - fast| = ${md.toExponential(2)}`);
}
process.exit(ok ? 0 : 1);
