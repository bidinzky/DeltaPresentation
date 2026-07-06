// ponytail: self-check for delta robot IK derivation (round-trip validation, not a full test suite)
const D2R = Math.PI / 180;

function deltaIK(x, y, z, Rb, Rp, Lb, Lf) {
  const Rdiff = Rb - Rp;
  const thetas = [];
  for (let i = 0; i < 3; i++) {
    const g = i * 120 * D2R; // gamma_i
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
    // two branches; pick the one matching a physically hanging delta (elbow swings down/out)
    const t1 = (a + sq) / (b + c);
    const t2 = (a - sq) / (b + c);
    const th1 = 2 * Math.atan(t1);
    const th2 = 2 * Math.atan(t2);
    if (process.env.DEBUG_BRANCHES) console.log('  branch i=' + i, (th1 / D2R).toFixed(2), (th2 / D2R).toFixed(2));
    // choose branch with smaller |theta| (elbow bends outward/down, not folded past vertical)
    thetas.push(Math.abs(th1) < Math.abs(th2) ? th1 : th2);
  }
  return thetas;
}

function forwardCheck(x, y, z, Rb, Rp, Lb, Lf, thetas) {
  const errs = [];
  for (let i = 0; i < 3; i++) {
    const g = i * 120 * D2R;
    const r = [Math.sin(g), -Math.cos(g), 0];
    const B = [Rb * r[0], Rb * r[1], 0];
    const th = thetas[i];
    const E = [B[0] + Lb * Math.cos(th) * r[0], B[1] + Lb * Math.cos(th) * r[1], -Lb * Math.sin(th)];
    const Pw = [x + Rp * r[0], y + Rp * r[1], z];
    const dist = Math.hypot(Pw[0] - E[0], Pw[1] - E[1], Pw[2] - E[2]);
    errs.push(Math.abs(dist - Lf));
  }
  return errs;
}

// test geometry (radius-based params, mm)
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
  const errs = forwardCheck(x, y, z, Rb, Rp, Lb, Lf, thetas);
  const maxErr = Math.max(...errs);
  const ok = maxErr < 1e-6;
  allOk = allOk && ok;
  console.log(`P=(${x},${y},${z}) theta(deg)=${thetas.map(t => (t / D2R).toFixed(2))} maxForearmErr=${maxErr.toExponential(2)} ${ok ? 'OK' : 'FAIL'}`);
}
console.log(allOk ? 'ALL CHECKS PASSED' : 'CHECKS FAILED');
process.exit(allOk ? 0 : 1);
