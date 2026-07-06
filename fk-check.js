// ponytail: self-check for delta robot FK derivation (round-trip vs. IK, not a full test suite)
const D2R = Math.PI / 180;

function deltaIK(x, y, z, Rb, Rp, Lb, Lf) {
  const Rdiff = Rb - Rp;
  const thetas = [];
  for (let i = 0; i < 3; i++) {
    const g = i * 120 * D2R;
    const sg = Math.sin(g), cg = Math.cos(g);
    const Wx = x * sg - y * cg - Rdiff;
    const Wy = x * cg + y * sg;
    const Wz = z;
    const a = 2 * Lb * Wz;
    const b = -2 * Lb * Wx;
    const K = Wx * Wx + Wy * Wy + Wz * Wz + Lb * Lb - Lf * Lf;
    const c = -K;
    const disc = a * a + b * b - c * c;
    if (disc < 0) { thetas.push(null); continue; }
    const sq = Math.sqrt(disc);
    const t1 = (a + sq) / (b + c);
    const t2 = (a - sq) / (b + c);
    const th1 = 2 * Math.atan(t1);
    const th2 = 2 * Math.atan(t2);
    thetas.push(Math.abs(th1) < Math.abs(th2) ? th1 : th2);
  }
  return thetas;
}

function deltaFK(thetas, Rb, Rp, Lb, Lf) {
  const Rdiff = Rb - Rp;
  const C = [];
  for (let i = 0; i < 3; i++) {
    const g = i * 120 * D2R;
    const k = Rdiff + Lb * Math.cos(thetas[i]);
    const h = -Lb * Math.sin(thetas[i]);
    const sg = Math.sin(g), cg = Math.cos(g);
    C.push([k * sg, -k * cg, h]);
  }
  const [C0, C1, C2] = C;
  const sq = p => p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
  const S0 = sq(C0);
  function planeEq(Ci) {
    const Si = sq(Ci);
    return [2 * (Ci[0] - C0[0]), 2 * (Ci[1] - C0[1]), 2 * (Ci[2] - C0[2]), Si - S0];
  }
  const [A1, B1, D1, E1] = planeEq(C1);
  const [A2, B2, D2, E2] = planeEq(C2);
  const det = A1 * B2 - A2 * B1;
  const nx = (E1 * B2 - E2 * B1) / det;
  const mx = (-D1 * B2 + D2 * B1) / det;
  const ny = (A1 * E2 - A2 * E1) / det;
  const my = (-A1 * D2 + A2 * D1) / det;
  const px = mx, qx = nx - C0[0];
  const py = my, qy = ny - C0[1];
  const Aq = px * px + py * py + 1;
  const Bq = 2 * px * qx + 2 * py * qy - 2 * C0[2];
  const Cq = qx * qx + qy * qy + C0[2] * C0[2] - Lf * Lf;
  const disc = Bq * Bq - 4 * Aq * Cq;
  if (disc < 0) return null;
  const sqd = Math.sqrt(disc);
  const z1 = (-Bq + sqd) / (2 * Aq);
  const z2 = (-Bq - sqd) / (2 * Aq);
  const z = Math.min(z1, z2); // physical: further below the base
  const x = mx * z + nx;
  const y = my * z + ny;
  return [x, y, z];
}

const Rb = 100, Rp = 40, Lb = 200, Lf = 400;
const testPoints = [
  [0, 0, -400],
  [50, 30, -420],
  [-80, 60, -380],
  [100, -100, -450],
  [0, 0, -300],
];

let allOk = true;
for (const [x, y, z] of testPoints) {
  const thetas = deltaIK(x, y, z, Rb, Rp, Lb, Lf);
  if (thetas.includes(null)) { console.log('UNREACHABLE', x, y, z); continue; }
  const p = deltaFK(thetas, Rb, Rp, Lb, Lf);
  const err = Math.hypot(p[0] - x, p[1] - y, p[2] - z);
  const ok = err < 1e-6;
  allOk = allOk && ok;
  console.log(`P=(${x},${y},${z}) -> theta(deg)=${thetas.map(t => (t / D2R).toFixed(2))} -> FK=(${p.map(v => v.toFixed(3))}) err=${err.toExponential(2)} ${ok ? 'OK' : 'FAIL'}`);
}
console.log(allOk ? 'ALL CHECKS PASSED' : 'CHECKS FAILED');
process.exit(allOk ? 0 : 1);
