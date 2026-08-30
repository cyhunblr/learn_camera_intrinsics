#!/usr/bin/env node
// Print reference values for scripts/check_parity.sh, using the viewer's own
// maths lifted straight out of web/index.html. See that script.
"use strict";
const fs = require("fs");
const path = require("path");

const html = fs.readFileSync(path.join(__dirname, "..", "web", "index.html"), "utf8");
const js = html.match(/<script>([\s\S]*?)\/\* ===+\n   Scene/)[1]
  .replace(/^"use strict";/, "");

// Indirect eval runs in global scope, where `require` is not in scope, so hand
// the input over on globalThis.
globalThis.__stdin = fs.readFileSync(0, "utf8");

const dump = `
const f = (v) => Number.isNaN(v) ? "nan" : v.toFixed(9);
for (const line of globalThis.__stdin.split("\\n")) {
  if (!line.trim()) continue;
  const n = line.trim().split(/\\s+/).map(Number);
  const [fx, fy, cx, cy, k1, k2, p1, p2, k3, w, h] = n;
  const K = makeK(fx, fy, cx, cy), D = [k1, k2, p1, p2, k3];
  console.log("case " + line.trim());
  for (const P of [[0.9, -0.4, 3.0], [-1.2, 0.7, 5.0], [2.0, 1.5, 2.2]]) {
    const q = project(P, K, D);
    console.log("  project " + f(q[0]) + " " + f(q[1]));
  }
  console.log("  fov " + fovDeg(K, w, h).map(f).join(" "));
  console.log("  rmax " + f(maxDistortedRadius(D)));
  for (const uv of [[0, 0], [w / 2, 0], [w, h], [w * 1.5, h * 1.5]]) {
    const nrm = toNormalized(uv[0], uv[1], K);
    const g = undistort(nrm[0], nrm[1], D);
    console.log("  undistort " + f(g[0]) + " " + f(g[1]));
  }
  for (const a of [0.0, 0.5, 1.0]) {
    const r = {};
    let Kn = null;
    try { Kn = optimalNewCameraMatrix(K, D, w, h, a, r); } catch (e) { }
    if (Kn === null) { console.log("  newK unavailable"); continue; }
    console.log("  newK " + [Kn.fx, Kn.fy, Kn.cx, Kn.cy].map(f).join(" ") +
                " " + [r.x, r.y, r.w, r.h].join(" "));
  }
}
`;

(0, eval)(js + dump);
