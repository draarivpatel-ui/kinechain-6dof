import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const K = require('../src/robotics.js');

const EPS = 1e-9;
const close = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} !~= ${b}`);
const vecClose = (a, b, tol = 1e-9) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tol)); };
const matClose = (A, B, tol = 1e-9) => A.forEach((r, i) => vecClose(r, B[i], tol));

// ---------- scalar helpers ----------
test('clamp bounds values', () => {
  assert.equal(K.clamp(5, 0, 3), 3);
  assert.equal(K.clamp(-5, 0, 3), 0);
  assert.equal(K.clamp(1.5, 0, 3), 1.5);
});

test('lerp interpolates', () => {
  close(K.lerp(0, 10, 0.5), 5);
  close(K.lerp(2, 8, 0), 2);
  close(K.lerp(2, 8, 1), 8);
});

test('approx with tolerance', () => {
  assert.ok(K.approx(1, 1 + 1e-10, 1e-9));
  assert.ok(!K.approx(1, 1.1, 1e-9));
});

test('mean and sum', () => {
  close(K.sum([1, 2, 3, 4]), 10);
  close(K.mean([2, 4, 6]), 4);
});

test('round honours decimals', () => {
  close(K.round(1.23456, 3), 1.235);
  assert.equal(K.round(2.5, 0), 3);
});

test('degree/radian constants and conversion', () => {
  close(K.DEG, 180 / Math.PI);
  close(K.RAD, Math.PI / 180);
  close(K.toDeg(Math.PI), 180);
  close(K.toRad(90), Math.PI / 2);
});

test('zeros builds finite vector', () => {
  vecClose(K.zeros(4), [0, 0, 0, 0]);
});

// ---------- vectors ----------
test('vector arithmetic', () => {
  vecClose(K.vecAdd([1, 2, 3], [4, 5, 6]), [5, 7, 9]);
  vecClose(K.vecSub([4, 5, 6], [1, 2, 3]), [3, 3, 3]);
  vecClose(K.vecScale([1, 2, 3], 2), [2, 4, 6]);
});

test('dot and cross', () => {
  close(K.dot([1, 2, 3], [4, 5, 6]), 32);
  vecClose(K.cross([1, 0, 0], [0, 1, 0]), [0, 0, 1]);
});

test('norm and normalize', () => {
  close(K.norm([3, 4]), 5);
  vecClose(K.normalize([0, 3, 0]), [0, 1, 0]);
  vecClose(K.normalize([0, 0, 0], [1, 0, 0]), [1, 0, 0]);
});

test('angleBetween basic cases', () => {
  close(K.angleBetween([1, 0, 0], [0, 1, 0]), Math.PI / 2);
  close(K.angleBetween([1, 0, 0], [1, 0, 0]), 0);
  close(K.angleBetween([1, 0, 0], [-1, 0, 0]), Math.PI);
  assert.equal(K.angleBetween([0, 0, 0], [1, 0, 0]), 0);
});

// ---------- matrices ----------
test('matrix identity and zeros', () => {
  const I = K.matIdentity(3);
  matClose(I, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
  matClose(K.matZeros(2, 2), [[0, 0], [0, 0]]);
});

test('matMul associativity and identity', () => {
  const A = [[1, 2], [3, 4]];
  const B = [[5, 6], [7, 8]];
  matClose(K.matMul(A, B), [[19, 22], [43, 50]]);
  matClose(K.matMul(A, K.matIdentity(2)), A);
});

test('matVec applies matrix to vector', () => {
  vecClose(K.matVec([[1, 2], [3, 4]], [1, 1]), [3, 7]);
});

test('matTranspose swaps indices', () => {
  matClose(K.matTranspose([[1, 2, 3], [4, 5, 6]]), [[1, 4], [2, 5], [3, 6]]);
});

test('matDet of known matrices', () => {
  close(K.matDet([[1, 2], [3, 4]]), -2);
  close(K.matDet(K.matIdentity(3)), 1);
  close(K.matDet([[2, 0, 0], [0, 2, 0], [0, 0, 2]]), 8);
});

test('matInverse recovers identity product', () => {
  const A = [[4, 3], [3, 2]];
  const inv = K.matInverse(A);
  matClose(K.matMul(A, inv), K.matIdentity(2), 1e-12);
  assert.equal(K.matInverse([[1, 2], [2, 4]]), null);
});

test('matTrace sums diagonal', () => {
  close(K.matTrace([[1, 2], [3, 4]]), 5);
});

test('solveLinear solves Ax=b', () => {
  const x = K.solveLinear([[2, 0], [0, 2]], [4, 6]);
  vecClose(x, [2, 3]);
  assert.equal(K.solveLinear([[1, 2], [2, 4]], [1, 1]), null);
});

test('eigSymmetric returns eigenpairs', () => {
  const r = K.eigSymmetric([[2, 0], [0, 1]]);
  assert.equal(r.values.length, 2);
  const srt = r.values.slice().sort((a, b) => a - b);
  close(srt[0], 1, 1e-9);
  close(srt[1], 2, 1e-9);
});

test('matrixRank detects deficient matrix', () => {
  assert.equal(K.matrixRank([[1, 0], [0, 1]]), 2);
  assert.equal(K.matrixRank([[1, 1], [1, 1]]), 1);
});

test('singularValues sorted ascending and nonneg', () => {
  const s = K.singularValues([[3, 0], [0, 2], [0, 0]]);
  assert.equal(s.length, 2);
  for (let i = 1; i < s.length; i++) assert.ok(s[i] >= s[i - 1]);
  close(s[1], 3, 1e-9);
});

test('conditionNumber is huge at a known singularity', () => {
  // PUMA-560 with all joints at zero sits on a wrist singularity.
  assert.ok(K.conditionNumber(K.PUMA_560, [0, 0, 0, 0, 0, 0]) > 1e6);
  // A well posed pose has a moderate condition number.
  const c = K.conditionNumber(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]);
  assert.ok(c > 1 && c < 100);
});

// ---------- rotations ----------
function axisRot(axis, angle) { return K.quatToMat(K.quatFromAxisAngle(axis, angle)); }

test('axis rotations are orthonormal', () => {
  for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1]]) {
    const R = axisRot(axis, 0.7);
    matClose(K.matMul(K.matTranspose(R), R), K.matIdentity(3), 1e-12);
    close(K.matDet(R), 1, 1e-12);
  }
});

test('rotZ by 90 degrees maps x to y', () => {
  const v = K.matVec(axisRot([0, 0, 1], Math.PI / 2), [1, 0, 0]);
  vecClose(v, [0, 1, 0], 1e-12);
});

test('euler roundtrip mid-range', () => {
  const e = K.matToEuler(K.eulerToMat(0.3, -0.5, 1.1));
  close(e.roll, 0.3, 1e-9);
  close(e.pitch, -0.5, 1e-9);
  close(e.yaw, 1.1, 1e-9);
});

test('euler gimbal branches reconstruct the matrix', () => {
  for (const p of [Math.PI / 2, -Math.PI / 2]) {
    const M = K.eulerToMat(0.7, p, -0.4);
    const e = K.matToEuler(M);
    const M2 = K.eulerToMat(e.roll, e.pitch, e.yaw);
    matClose(M, M2, 1e-9);
    assert.equal(e.gimbal, true);
  }
});

test('eulerFromDeg converts a degree triple to radians', () => {
  const e = K.eulerFromDeg([30, -45, 90]);
  close(e.roll, Math.PI / 6, 1e-9);
  close(e.pitch, -Math.PI / 4, 1e-9);
  close(e.yaw, Math.PI / 2, 1e-9);
});

// ---------- quaternions ----------
test('quat identity and normalize', () => {
  vecClose(K.quatIdentity(), [1, 0, 0, 0]);
  vecClose(K.quatNormalize([2, 0, 0, 0]), [1, 0, 0, 0]);
});

test('quatMul composes rotations', () => {
  const qx = K.quatFromAxisAngle([1, 0, 0], Math.PI / 2);
  const qy = K.quatFromAxisAngle([0, 1, 0], Math.PI / 2);
  const q = K.quatMul(qy, qx);
  const v = K.quatRotate(q, [1, 0, 0]);
  const m = K.quatToMat(q);
  vecClose(v, K.matVec(m, [1, 0, 0]), 1e-12);
});

test('quat matrix roundtrip', () => {
  const q = K.quatFromAxisAngle(K.normalize([1, 1, 0]), 0.9);
  const M = K.quatToMat(q);
  const q2 = K.quatFromMat(M);
  const M2 = K.quatToMat(q2);
  matClose(M, M2, 1e-12);
});

test('quatToAxisAngle recovers angle', () => {
  const q = K.quatFromAxisAngle([0, 0, 1], 0.8);
  const aa = K.quatToAxisAngle(q);
  close(aa.angle, 0.8, 1e-9);
  vecClose(aa.axis, [0, 0, 1], 1e-9);
});

test('quatSlerp endpoints and midpoint', () => {
  const a = K.quatIdentity();
  const b = K.quatFromAxisAngle([0, 0, 1], Math.PI / 2);
  const mid = K.quatSlerp(a, b, 0.5);
  const v = K.quatRotate(mid, [1, 0, 0]);
  vecClose(v, [Math.SQRT1_2, Math.SQRT1_2, 0], 1e-9);
  vecClose(K.quatSlerp(a, b, 0), a, 1e-12);
});

test('quatConj inverts rotation', () => {
  const q = K.quatFromAxisAngle([0, 1, 0], 0.5);
  const qi = K.quatConj(q);
  const I = K.quatMul(q, qi);
  vecClose(I, K.quatIdentity(), 1e-12);
});

test('quatDot and euler bridge', () => {
  const q = K.eulerToQuat(0.2, -0.3, 0.4);
  close(K.quatDot(q, q), 1, 1e-12);
  const e = K.quatToEuler(q);
  close(e.roll, 0.2, 1e-9);
  close(e.pitch, -0.3, 1e-9);
  close(e.yaw, 0.4, 1e-9);
});

// ---------- poses ----------
test('pose identity and mul', () => {
  const I = K.poseIdentity();
  const p = K.pose(axisRot([0, 0, 1], 0.5), [1, 2, 3]);
  const r = K.poseMul(I, p);
  vecClose(r.t, p.t);
  matClose(r.R, p.R);
});

test('poseInv undoes poseMul', () => {
  const p = K.pose(K.eulerToMat(0.2, 0.4, -0.1), [0.3, -0.1, 0.2]);
  const r = K.poseMul(p, K.poseInv(p));
  vecClose(r.t, [0, 0, 0], 1e-12);
  matClose(r.R, K.matIdentity(3), 1e-12);
});

test('poseApply transforms a point', () => {
  const p = K.pose(axisRot([0, 0, 1], Math.PI / 2), [1, 0, 0]);
  vecClose(K.poseApply(p, [1, 0, 0]), [1, 1, 0], 1e-12);
});

test('poseInterp midpoint rotation', () => {
  const a = K.poseIdentity();
  const b = K.pose(axisRot([0, 0, 1], Math.PI / 2), [0, 0, 2]);
  const m = K.poseInterp(a, b, 0.5);
  vecClose(m.t, [0, 0, 1], 1e-9);
  vecClose(K.poseApply(m, [1, 0, 0]), [Math.SQRT1_2, Math.SQRT1_2, 1], 1e-9);
});

test('poseDistance is translation length plus rotation angle', () => {
  const a = K.poseIdentity();
  const b = K.pose(axisRot([0, 0, 1], 0.4), [1, 0, 0]);
  close(K.poseDistance(a, b), 1.4, 1e-9);
});

test('homogeneous conversion roundtrip', () => {
  const p = K.pose(K.eulerToMat(0.1, 0.2, 0.3), [1, 2, 3]);
  const H = K.poseToHomogeneous(p);
  assert.equal(H.length, 4);
  const back = K.poseFromHomogeneous(H);
  vecClose(back.t, p.t, 1e-12);
  matClose(back.R, p.R, 1e-12);
});

test('pose quaternion bridge', () => {
  const p = K.pose(K.eulerToMat(-0.2, 0.5, 0.7), [0.1, 0.2, 0.3]);
  const back = K.poseFromQuat(K.poseToQuat(p), p.t);
  vecClose(back.t, p.t, 1e-12);
  matClose(back.R, p.R, 1e-12);
});

// ---------- DH and robot structure ----------
test('dhMatrix applies standard DH transform', () => {
  // theta=0 alpha=0 gives pure Tx(a) Tz(d)
  const A = K.dhMatrix(0.5, 0, 0.2, 0);
  vecClose(A.t, [0.5, 0, 0.2], 1e-12);
  matClose(A.R, K.matIdentity(3), 1e-12);
});

test('dhMatrix rotates about z by theta', () => {
  const A = K.dhMatrix(0, 0, 0, Math.PI / 2);
  vecClose(K.matVec(A.R, [1, 0, 0]), [0, 1, 0], 1e-12);
});

test('modifiedDHMatrix differs from standard on nonzero alpha', () => {
  const std = K.dhMatrix(0.3, 0.4, 0.1, 0.2);
  const mod = K.modifiedDHMatrix(0.3, 0.4, 0.1, 0.2);
  let diff = 0;
  for (let i = 0; i < 3; i++) diff = Math.max(diff, Math.abs(std.t[i] - mod.t[i]));
  assert.ok(diff > 1e-3);
});

test('dhParams folds prismatic q into d', () => {
  const params = K.dhParams(K.SCARA, [0.1, 0.2, 0.05, 0.3]);
  close(params[2].d, K.SCARA.links[2].d + 0.05);
  close(params[0].theta, K.SCARA.links[0].theta + 0.1);
});

test('jointAxis returns unit z axes and origins', () => {
  const ax = K.jointAxis(K.PUMA_560, [0.3, 0.4, -0.5, 0.2, 0.1, 0.6], 0);
  close(K.norm(ax.z), 1, 1e-12);
  assert.equal(ax.o.length, 3);
});

test('createRobot rejects malformed tables', () => {
  assert.throws(() => K.createRobot({ name: 'x', links: [] }));
  assert.throws(() => K.createRobot({ name: 'x', links: [{ a: 0, alpha: 0, d: 0, lo: 1, hi: -1 }] }));
});

test('getRobot by name and object passthrough', () => {
  assert.equal(K.getRobot('PUMA-560'), K.PUMA_560);
  assert.equal(K.getRobot(K.UR5E), K.UR5E);
  assert.throws(() => K.getRobot('nope'), /unknown robot/);
});

test('getPreset finds presets and misses unknown ids', () => {
  assert.ok(K.getPreset('ready'));
  assert.equal(K.getPreset('does-not-exist'), null);
});

test('toolPoint composes end pose', () => {
  const e = K.endPose(K.PUMA_560, [0, 0, 0, 0, 0, 0]);
  const tp = K.toolPoint(K.PUMA_560, [0, 0, 0, 0, 0, 0]);
  vecClose(tp, e.t, 1e-12);
});

test('linkFrames matches forwardKinematics frames', () => {
  const q = [0.2, 0.3, -0.4, 0.5, 0.1, -0.2];
  const a = K.linkFrames(K.PUMA_560, q);
  const b = K.forwardKinematics(K.PUMA_560, q).frames;
  assert.equal(a.length, b.length - 1);
  for (let i = 0; i < a.length; i++) vecClose(a[i].translation, b[i].t, 1e-12);
});

test('robot names and structure', () => {
  assert.equal(K.PUMA_560.dof, 6);
  assert.equal(K.SCARA.dof, 4);
  assert.equal(K.SCARA.links[2].type, 'prismatic');
  assert.equal(Object.keys(K.ROBOTS).length, 3);
});

// ---------- misc utilities ----------
test('num formats fixed precision without negative zero', () => {
  assert.equal(K.num(-0), '0');
  assert.equal(K.num(1.23456789), '1.234568');
});

test('xmlEscape escapes the five characters', () => {
  assert.equal(K.xmlEscape('a<b>&"\'c'), 'a&lt;b&gt;&amp;&quot;&apos;c');
});

test('segmentDuration grows with distance and shrinks with limits', () => {
  const short = K.segmentDuration(0, 0.1, 2, 4);
  const long = K.segmentDuration(0, 1.0, 2, 4);
  assert.ok(long > short);
  const slow = K.segmentDuration(0, 0.1, 0.5, 1);
  assert.ok(slow > short);
});

test('segmentJerkCost is finite and positive', () => {
  const c = K.segmentJerkCost(1, 0, 1, 0, 0, 0, 0);
  assert.ok(Number.isFinite(c) && c > 0);
});

test('quinticBlend endpoints and symmetric midpoint', () => {
  close(K.quinticBlend(0), 0);
  close(K.quinticBlend(1), 1);
  close(K.quinticBlend(0.5), 0.5);
});

test('quintic blend derivatives vanish at both ends', () => {
  close(K.quinticVelocityBlend(0), 0, 1e-12);
  close(K.quinticVelocityBlend(1), 0, 1e-12);
  close(K.quinticAccelBlend(0), 0, 1e-12);
  close(K.quinticAccelBlend(1), 0, 1e-12);
});

test('quinticSimple is the blend at t over T', () => {
  close(K.quinticSimple(1, 2), 0.5);
  close(K.quinticSimple(0, 2), 0);
  close(K.quinticSimple(2, 2), 1);
});

test('endpointVelocityNumerical approximates Jacobian velocity', () => {
  const q = [0.3, 0.4, -0.5, 0.2, 0.6, -0.1];
  const dq = [0.1, -0.2, 0.05, 0.3, -0.1, 0.2];
  const v = K.endpointVelocityNumerical(K.PUMA_560, q, dq);
  const J = K.jacobian(K.PUMA_560, q);
  const expected = K.matVec(J, dq).slice(0, 3);
  vecClose(v, expected, 1e-3);
});

test('robotReport bundles the analysis', () => {
  const rep = K.robotReport(K.PUMA_560, [0.3, 0.4, -0.5, 0.2, 0.6, -0.1]);
  assert.equal(rep.robot, 'PUMA-560');
  assert.equal(rep.dof, 6);
  assert.ok(rep.jacobian);
  assert.ok(typeof rep.manipulability === 'number');
  assert.ok(rep.frames.length === 7);
});
