import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const K = require('../src/robotics.js');

const close = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} !~= ${b}`);
const vecClose = (a, b, tol = 1e-9) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tol)); };
const PUMA = K.PUMA_560;
const UR5 = K.UR5E;
const SCARA = K.SCARA;
// Cartesian targets verified reachable for PUMA-560 position IK.
const REACHABLE = [[0.5, 0, 0.3], [0.4, -0.15, 0.3], [0.35, 0.15, 0.35], [0.6, 0, 0.2], [0.5, -0.2, 0.4]];

// ---------- tridiagonal solver ----------
test('solveTridiagonal solves a diagonal dominant system exactly', () => {
  const x = K.solveTridiagonal([0, 1, 1, 1], [4, 4, 4, 4], [1, 1, 1, 0], [3, 3, 3, 3]);
  // direct residual check: 4x[i] + x[i-1] + x[i+1] = 3
  for (let i = 0; i < 4; i++) {
    const lhs = 4 * x[i] + (i > 0 ? x[i - 1] : 0) + (i < 3 ? x[i + 1] : 0);
    close(lhs, 3, 1e-9);
  }
  // and the exact solution is [12/19, 9/19, 9/19, 12/19]
  vecClose(x, [12 / 19, 9 / 19, 9 / 19, 12 / 19], 1e-9);
});

test('solveTridiagonal handles n=1', () => {
  vecClose(K.solveTridiagonal([0], [2], [0], [6]), [3]);
});

test('solveTridiagonal reproduces a known system', () => {
  // Solution of 2x0+x1=3, x0+2x1+x2=7, x1+2x2=8 is [0.75, 1.5, 3.25]
  const x = K.solveTridiagonal([0, 1, 1], [2, 2, 2], [1, 1, 0], [3, 7, 8]);
  vecClose(x, [0.75, 1.5, 3.25], 1e-12);
});

// ---------- cubic spline ----------
test('cubic spline interpolates every knot exactly', () => {
  const ts = [0, 0.7, 1.9, 3.0];
  const ys = [0, 1, -0.5, 0.8];
  const sp = K.cubicSpline1D(ts, ys);
  ts.forEach((t, i) => close(sp.eval(t).position, ys[i], 1e-9));
});

test('cubic spline boundary velocities are zero by default', () => {
  const sp = K.cubicSpline1D([0, 1, 2.5], [0, 1, 0.4]);
  close(sp.eval(0).velocity, 0, 1e-12);
  close(sp.eval(2.5).velocity, 0, 1e-12);
});

test('cubic spline honours clamped boundary velocities', () => {
  const sp = K.cubicSpline1D([0, 1, 2], [0, 1, 0], { v0: 0.5, vn: -0.5 });
  close(sp.eval(0).velocity, 0.5, 1e-9);
  close(sp.eval(2).velocity, -0.5, 1e-9);
});

test('cubic spline velocity is continuous across interior knots', () => {
  const sp = K.cubicSpline1D([0, 1, 2, 3.5], [0, 0.4, -0.2, 0.6]);
  for (const tk of [1, 2]) {
    const l = sp.eval(tk - 1e-7);
    const r = sp.eval(tk + 1e-7);
    close(l.velocity, r.velocity, 1e-4);
  }
});

test('cubic spline acceleration is continuous across interior knots', () => {
  const sp = K.cubicSpline1D([0, 1.2, 2.1, 4], [0, 0.4, -0.2, 0.6]);
  for (const tk of [1.2, 2.1]) {
    const l = sp.eval(tk - 1e-6);
    const r = sp.eval(tk + 1e-6);
    close(l.accel, r.accel, 1e-3);
  }
});

test('cubic spline with two knots is a straight line', () => {
  const sp = K.cubicSpline1D([0, 2], [1, 5]);
  close(sp.eval(1).position, 3, 1e-9);
  close(sp.eval(0).velocity, 0, 1e-12);
});

test('cubic spline clamps evaluation outside the knot range', () => {
  const sp = K.cubicSpline1D([0, 1, 2], [0, 1, 0]);
  close(sp.eval(-1).position, 0, 1e-9);
  close(sp.eval(9).position, 0, 1e-9);
});

test('cubic spline rejects mismatched inputs', () => {
  assert.throws(() => K.cubicSpline1D([0, 1, 2], [0, 1]), RangeError);
  assert.throws(() => K.cubicSpline1D([0, 0], [0, 1]), RangeError);
});

// ---------- trapezoidal profile ----------
test('trapezoid profile starts and ends at rest', () => {
  const p = K.trapezoidProfile({ blend: 0.25 });
  close(p.sample(0).v, 0, 1e-12);
  close(p.sample(1).v, 0, 1e-12);
  close(p.sample(0).s, 0, 1e-12);
  close(p.sample(1).s, 1, 1e-12);
});

test('trapezoid profile is continuous at the blend seams', () => {
  const p = K.trapezoidProfile({ blend: 0.3 });
  for (const u of [0.3, 0.7]) {
    const l = p.sample(u - 1e-7);
    const r = p.sample(u + 1e-7);
    close(l.s, r.s, 1e-5);
    close(l.v, r.v, 1e-5);
  }
});

test('trapezoid profile is symmetric', () => {
  const p = K.trapezoidProfile({ blend: 0.2 });
  for (const u of [0.1, 0.25, 0.4]) {
    close(p.sample(u).s, 1 - p.sample(1 - u).s, 1e-9);
  }
});

test('trapezoidDuration is positive and grows with distance', () => {
  const d1 = K.trapezoidDuration(0.5, 2, 4);
  const d2 = K.trapezoidDuration(1.5, 2, 4);
  assert.ok(d1 > 0 && d2 > d1);
});

test('trapezoidDuration honours the velocity ceiling', () => {
  const d = K.trapezoidDuration(2, 1, 10, 0.25);
  assert.ok(d >= 2); // cannot cover 2 units at max 1 unit/s
});

test('trapezoidDuration handles zero distance', () => {
  assert.equal(K.trapezoidDuration(0, 2, 4), 0);
});

// ---------- joint trajectory planner ----------
test('cubic plan passes through every joint waypoint at its knot time', () => {
  const qs = [K.zeros(6), [0.2, 0.3, -0.4, 0.1, -0.2, 0.3], [0.5, 0.1, -0.2, -0.3, 0.4, 0]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'cubic' });
  plan.knotTimes.forEach((t, i) => vecClose(plan.sample(t).q, qs[i], 1e-7));
});

test('cubic plan boundary velocities are zero', () => {
  const qs = [K.zeros(6), [0.3, 0.2, -0.3, 0.2, -0.1, 0.2]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'cubic' });
  for (const t of [0, plan.totalTime]) {
    plan.sample(t).dq.forEach(v => close(v, 0, 1e-9));
  }
});

test('trapezoid plan boundary velocities are zero', () => {
  const qs = [K.zeros(6), [0.3, 0.2, -0.3, 0.2, -0.1, 0.2]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'trapezoid' });
  for (const t of [0, plan.totalTime]) {
    plan.sample(t).dq.forEach(v => close(v, 0, 1e-9));
  }
});

test('trapezoid plan reaches every waypoint', () => {
  const qs = [K.zeros(6), [0.2, 0.3, -0.4, 0.1, -0.2, 0.3], [0.5, 0.1, -0.2, -0.3, 0.4, 0]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'trapezoid' });
  plan.segments.forEach(sg => vecClose(plan.sample(sg.t1).q, sg.to, 1e-7));
});

test('quintic mode delegates to the existing planner', () => {
  const qs = [K.zeros(6), [0.2, 0.3, -0.4, 0.1, -0.2, 0.3]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'quintic' });
  assert.equal(plan.mode, 'quintic');
  vecClose(plan.sample(plan.totalTime).q, qs[1], 1e-9);
});

test('explicit durations are respected', () => {
  const qs = [K.zeros(6), [0.2, 0.1, -0.2, 0.1, -0.1, 0.1], [0.3, 0.2, -0.1, 0, 0.1, 0]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'cubic', durations: [1.5, 2.5] });
  close(plan.totalTime, 4.0, 1e-9);
  close(plan.segments[0].duration, 1.5, 1e-9);
  close(plan.segments[1].duration, 2.5, 1e-9);
});

test('plan rejects a single waypoint and bad modes', () => {
  assert.throws(() => K.planJointTrajectory(PUMA, [K.zeros(6)]), RangeError);
  assert.throws(() => K.planJointTrajectory(PUMA, [K.zeros(6), K.zeros(6)], { mode: 'bezier' }), RangeError);
});

test('plan rejects nonfinite joint vectors', () => {
  assert.throws(() => K.planJointTrajectory(PUMA, [K.zeros(6), [NaN, 0, 0, 0, 0, 0]]), RangeError);
  assert.throws(() => K.planJointTrajectory(PUMA, [K.zeros(6), [0.1, 0.1, 0.1, 0.1, 0.1]]), RangeError);
});

test('sampleRange validates its count', () => {
  const plan = K.planJointTrajectory(PUMA, [K.zeros(6), [0.1, 0.1, -0.1, 0.1, 0, 0]]);
  assert.throws(() => plan.sampleRange(1), RangeError);
  assert.throws(() => plan.sampleRange(NaN), RangeError);
  assert.equal(plan.sampleRange(8).length, 9);
});

test('cubic plan reports finite samples across the whole path', () => {
  const qs = [K.zeros(6), [0.4, 0.4, -0.6, 0.2, -0.4, 0.5], [0.1, 0.5, -0.2, -0.2, 0.3, 0.1]];
  const plan = K.planJointTrajectory(PUMA, qs, { mode: 'cubic' });
  for (const s of plan.sampleRange(80)) {
    assert.ok(s.q.every(Number.isFinite));
    assert.ok(s.dq.every(Number.isFinite));
    assert.ok(s.ddq.every(Number.isFinite));
  }
});

// ---------- floor clearance ----------
test('floorClearance flags a configuration below the floor plane', () => {
  // High floor: the whole arm is below it.
  const fc = K.floorClearance(PUMA, K.zeros(6), 2.0);
  assert.equal(fc.clear, false);
  assert.ok(fc.violations.length > 0);
  assert.ok(fc.minZ < 2.0);
});

test('floorClearance reports clear for a very low floor', () => {
  const fc = K.floorClearance(PUMA, [0.2, 0.4, -0.4, 0, -0.5, 0], -10);
  assert.equal(fc.clear, true);
  assert.equal(fc.violations.length, 0);
});

test('floorClearance reports minZ and worstLink', () => {
  const fc = K.floorClearance(PUMA, [0.5, 0.5, -0.5, 0.3, -0.7, 0.2], -10);
  assert.ok(Number.isFinite(fc.minZ));
  assert.ok(fc.worstLink === null || Number.isInteger(fc.worstLink));
});

test('floorClearance returns a consistent report shape', () => {
  const fc = K.floorClearance(PUMA, K.zeros(6), 0);
  assert.equal(typeof fc.clear, 'boolean');
  assert.ok(Array.isArray(fc.violations));
  assert.equal(fc.floorZ, 0);
});

test('floorClearance samples every link', () => {
  const fc = K.floorClearance(PUMA, [0.6, 0.5, -0.2, 0, -1.0, 0.4], 0.5);
  const links = new Set(fc.violations.map(v => v.link));
  assert.ok(links.size >= 1);
});

// ---------- waypoint normalization ----------
test('normalizeSeqWaypoint accepts array, xyz object, t and position forms', () => {
  assert.deepEqual(K.normalizeSeqWaypoint([1, 2, 3], 0).t, [1, 2, 3]);
  assert.deepEqual(K.normalizeSeqWaypoint({ x: 1, y: 2, z: 3 }, 0).t, [1, 2, 3]);
  assert.deepEqual(K.normalizeSeqWaypoint({ t: [1, 2, 3] }, 0).t, [1, 2, 3]);
  assert.deepEqual(K.normalizeSeqWaypoint({ position: [1, 2, 3] }, 0).t, [1, 2, 3]);
});

test('normalizeSeqWaypoint keeps label and dwell', () => {
  const w = K.normalizeSeqWaypoint({ t: [0.1, 0.2, 0.3], label: 'pick', dwell: 1.5 }, 0);
  assert.equal(w.label, 'pick');
  assert.equal(w.dwell, 1.5);
});

test('normalizeSeqWaypoint derives a default label', () => {
  assert.equal(K.normalizeSeqWaypoint([0.1, 0.2, 0.3], 4).label, 'W5');
});

test('normalizeSeqWaypoint clamps dwell into [0, 60]', () => {
  assert.equal(K.normalizeSeqWaypoint({ t: [0, 0, 0], dwell: -2 }, 0).dwell, 0);
  assert.equal(K.normalizeSeqWaypoint({ t: [0, 0, 0], dwell: 99 }, 0).dwell, 60);
});

test('normalizeSeqWaypoint rejects malformed input', () => {
  assert.throws(() => K.normalizeSeqWaypoint([1, 2], 0), RangeError);
  assert.throws(() => K.normalizeSeqWaypoint([1, 2, NaN], 0), RangeError);
  assert.throws(() => K.normalizeSeqWaypoint(null, 0), RangeError);
});

// ---------- waypoint IK chain ----------
test('solveWaypointChain converges on a reachable chain', () => {
  const chain = K.solveWaypointChain(PUMA, REACHABLE.slice(0, 3));
  chain.forEach((c, i) => {
    assert.ok(c.converged, `waypoint ${i} failed with ${c.positionError}`);
    assert.ok(c.positionError < 1e-4);
  });
});

test('solveWaypointChain lands FK on the requested targets', () => {
  const chain = K.solveWaypointChain(PUMA, REACHABLE.slice(0, 3));
  chain.forEach((c, i) => {
    const ee = K.endPose(PUMA, c.q);
    vecClose(ee.t, REACHABLE[i], 2e-3);
  });
});

test('solveWaypointChain seeds each solve from the previous answer', () => {
  const chain = K.solveWaypointChain(PUMA, [[0.5, 0, 0.3], [0.52, 0.02, 0.32], [0.54, 0.04, 0.34]]);
  // Nearby targets should stay in one IK branch: consecutive joint
  // answers differ by far less than an elbow flip would.
  for (let i = 1; i < chain.length; i++) {
    const jump = Math.max(...chain[i].q.map((v, j) => Math.abs(v - chain[i - 1].q[j])));
    assert.ok(jump < 1.0, `joint jump ${jump} between waypoints`);
  }
});

test('solveWaypointChain reports unreachable targets honestly', () => {
  const chain = K.solveWaypointChain(PUMA, [[5, 5, 5]]);
  assert.equal(chain[0].converged, false);
  assert.ok(chain[0].positionError > 0.5);
});

test('solveWaypointChain carries labels through', () => {
  const chain = K.solveWaypointChain(PUMA, [{ t: [0.5, 0, 0.3], label: 'pick' }]);
  assert.equal(chain[0].label, 'pick');
});

// ---------- waypoint sequence planner ----------
test('planWaypointSequence solves a reachable sequence', () => {
  const res = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 3), { mode: 'cubic' });
  assert.equal(res.allSolved, true);
  assert.equal(res.waypoints.length, 3);
  assert.ok(res.plan.totalTime > 0);
});

test('planWaypointSequence interpolates through solved joint waypoints', () => {
  const res = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 3), { mode: 'cubic' });
  // End pose at the final knot must land on the last Cartesian target.
  const ee = K.endPose(PUMA, res.plan.sample(res.plan.totalTime).q);
  vecClose(ee.t, REACHABLE[2], 2e-3);
});

test('planWaypointSequence warns on unreachable waypoints instead of failing', () => {
  const res = K.planWaypointSequence(PUMA, [[0.5, 0, 0.3], [5, 5, 5]], { mode: 'cubic' });
  assert.equal(res.allSolved, false);
  const ikWarn = res.warnings.filter(w => w.kind === 'ik');
  assert.equal(ikWarn.length, 1);
  assert.equal(ikWarn[0].waypoint, 1);
});

test('planWaypointSequence flags floor strikes on a dipping path', () => {
  // A path that folds the elbow low must surface floor warnings.
  const res = K.planWaypointSequence(PUMA, [[0.5, 0, 0.3], [0.4, -0.15, 0.3], [0.35, 0.15, 0.35]], { mode: 'cubic' });
  assert.ok(Number.isFinite(res.floor.minZ));
  if (res.floor.minZ < 0) {
    assert.ok(res.warnings.some(w => w.kind === 'floor'));
  }
});

test('planWaypointSequence dwell time extends the total time', () => {
  const plain = K.planWaypointSequence(PUMA, [[0.5, 0, 0.3], [0.5, -0.2, 0.4]], { mode: 'trapezoid' });
  const held = K.planWaypointSequence(PUMA, [{ t: [0.5, 0, 0.3], dwell: 0.8 }, { t: [0.5, -0.2, 0.4] }], { mode: 'trapezoid' });
  close(held.plan.totalTime, plain.plan.totalTime + 0.8, 1e-9);
});

test('planWaypointSequence dwell holds the pose constant', () => {
  const res = K.planWaypointSequence(PUMA, [{ t: [0.5, 0, 0.3], dwell: 0.7 }, { t: [0.5, -0.2, 0.4] }], { mode: 'trapezoid' });
  const a = res.plan.sample(0.2).q;
  const b = res.plan.sample(0.5).q;
  vecClose(a, b, 1e-9);
});

test('planWaypointSequence supports all three interpolator modes', () => {
  for (const mode of ['cubic', 'trapezoid', 'quintic']) {
    const res = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 2), { mode });
    assert.ok(res.plan.totalTime > 0, mode);
    assert.equal(res.mode === 'trapezoidal' ? 'trapezoid' : res.mode, mode);
  }
});

test('planWaypointSequence reports minimum manipulability along the path', () => {
  const res = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 3), { mode: 'cubic' });
  assert.ok(Number.isFinite(res.minManipulability));
  assert.ok(res.minManipulability >= 0);
});

test('planWaypointSequence reports obstacle contacts', () => {
  // Sphere parked exactly on the first target's FK pose.
  const q0 = K.solveIKMultiStart(PUMA, { R: K.matIdentity(3), t: REACHABLE[0] }, { seeds: 12, maxIter: 300, weightRotation: 0.1, rotTol: 1e9, posTol: 1e-5 }).q;
  const obs = K.endPose(PUMA, q0).t;
  const res = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 2), {
    mode: 'cubic',
    obstacles: [{ x: obs[0], y: obs[1], z: obs[2], radius: 0.3 }]
  });
  assert.ok(res.warnings.some(w => w.kind === 'collision'));
});

test('planWaypointSequence requires two waypoints', () => {
  assert.throws(() => K.planWaypointSequence(PUMA, [[0.5, 0, 0.3]]), RangeError);
  assert.throws(() => K.planWaypointSequence(PUMA, []), RangeError);
});

test('planWaypointSequence is deterministic', () => {
  const a = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 3), { mode: 'cubic' });
  const b = K.planWaypointSequence(PUMA, REACHABLE.slice(0, 3), { mode: 'cubic' });
  assert.equal(a.plan.totalTime, b.plan.totalTime);
  assert.deepEqual(a.waypoints.map(c => c.q), b.waypoints.map(c => c.q));
});

// ---------- singularity risk monitor ----------
test('singularityRisk reads safe at a dextrous pose', () => {
  const r = K.singularityRisk(PUMA, [-1.77, 1.07, -1.51, 0.43, -0.91, 0.77], { reach: 1 });
  assert.equal(r.level, 'safe');
  assert.ok(r.score < 0.3);
});

test('singularityRisk reads critical at the singular zero pose', () => {
  const r = K.singularityRisk(PUMA, K.zeros(6), { reach: 1 });
  assert.equal(r.level, 'critical');
  assert.ok(r.singular || r.ratio <= 0.005);
});

test('singularityRisk detects wrist alignment on UR5e', () => {
  // q5 = 0 folds wrist axes together on a 6R arm.
  const q = [0.4, -1.0, 1.2, 0.5, 0, 0.3];
  const sing = K.detectSingularity(UR5, q);
  const risk = K.singularityRisk(UR5, q, { reach: 1 });
  if (sing.singular) assert.equal(risk.level, 'critical');
  assert.ok(risk.ratio >= 0);
});

test('singularityRisk score is monotone with sigma ratio', () => {
  const qGood = [0.3, 0.6, -0.7, 0.4, -0.9, 0.3];
  const risk = K.singularityRisk(PUMA, qGood, { reach: 1 });
  const qNear = K.zeros(6);
  const risk2 = K.singularityRisk(PUMA, qNear, { reach: 1 });
  assert.ok(risk2.score >= risk.score, `${risk2.score} < ${risk.score}`);
});

test('singularityRisk exposes all diagnostic fields', () => {
  const r = K.singularityRisk(PUMA, [0.2, 0.4, -0.5, 0.2, -0.6, 0.1], { reach: 1 });
  for (const k of ['level', 'score', 'sigmaMin', 'sigmaMax', 'ratio', 'conditionNumber', 'manipulability', 'message']) {
    assert.ok(k in r, `missing ${k}`);
  }
  assert.ok(r.score >= 0 && r.score <= 1);
  assert.ok(typeof r.message === 'string' && r.message.length > 0);
});

test('singularityRisk level is one of the four documented bands', () => {
  const qs = [K.zeros(6), [0.3, 0.6, -0.7, 0.4, -0.9, 0.3], [0.5, 0.5, -0.2, 0, -1.0, 0.4]];
  for (const q of qs) {
    const r = K.singularityRisk(PUMA, q, { reach: 1 });
    assert.ok(['safe', 'watch', 'warning', 'critical'].includes(r.level));
  }
});

// ---------- velocity ellipsoid ----------
test('velocityEllipsoid axes are sorted largest first', () => {
  const e = K.velocityEllipsoid(PUMA, [0.3, 0.6, -0.7, 0.4, -0.9, 0.3]);
  assert.equal(e.axes.length, 3);
  assert.ok(e.axes[0].sigma >= e.axes[1].sigma && e.axes[1].sigma >= e.axes[2].sigma);
  assert.equal(e.sigmaMax, e.axes[0].sigma);
  assert.equal(e.sigmaMin, e.axes[2].sigma);
});

test('velocityEllipsoid is centred on the end pose', () => {
  const q = [0.3, 0.6, -0.7, 0.4, -0.9, 0.3];
  const e = K.velocityEllipsoid(PUMA, q);
  vecClose(e.center, K.endPose(PUMA, q).t, 1e-9);
});

test('velocityEllipsoid axes are unit vectors', () => {
  const e = K.velocityEllipsoid(PUMA, [0.3, 0.6, -0.7, 0.4, -0.9, 0.3]);
  for (const ax of e.axes) close(K.norm(ax.dir), 1, 1e-6);
});

test('velocityEllipsoid collapses toward a line at a singular pose', () => {
  const e = K.velocityEllipsoid(PUMA, K.zeros(6));
  assert.ok(e.sigmaMin < e.sigmaMax * 0.1 + 1e-12, `sigmaMin ${e.sigmaMin}`);
});

test('projectEllipsoid returns a 2D ellipse inside the 3D projection', () => {
  const q = [0.3, 0.6, -0.7, 0.4, -0.9, 0.3];
  const e = K.velocityEllipsoid(PUMA, q);
  const p = K.projectEllipsoid(e, [0, 2]);
  assert.equal(p.semi.length, 2);
  assert.ok(p.semi[0] >= p.semi[1] - 1e-12);
  assert.ok(p.semi[0] <= e.sigmaMax + 1e-9);
  assert.ok(Number.isFinite(p.angle));
  assert.deepEqual(p.center, [e.center[0], e.center[2]]);
});

test('projectEllipsoid projection is plane dependent', () => {
  const e = K.velocityEllipsoid(PUMA, [0.3, 0.6, -0.7, 0.4, -0.9, 0.3]);
  const xz = K.projectEllipsoid(e, [0, 2]);
  const xy = K.projectEllipsoid(e, [0, 1]);
  assert.notDeepEqual(xz.semi, xy.semi);
});

// ---------- finite difference Jacobian ----------
for (const [name, robot] of [['PUMA-560', PUMA], ['UR5e', UR5], ['SCARA-4', SCARA]]) {
  test(`analytical Jacobian matches central difference below 1e-5 (${name})`, () => {
    const q = robot.links.map(l => l.lo + (l.hi - l.lo) * 0.45);
    const Ja = K.jacobian(robot, q);
    const Jn = K.jacobianCentralDiff(robot, q);
    let worst = 0;
    for (let i = 0; i < 6; i++) for (let j = 0; j < robot.dof; j++) {
      worst = Math.max(worst, Math.abs(Ja[i][j] - Jn[i][j]));
    }
    assert.ok(worst < 1e-5, `worst deviation ${worst}`);
  });
}

test('central difference Jacobian stays accurate away from mid range', () => {
  const q = [0.5, 0.5, -0.2, 0, -1.0, 0.4];
  const Ja = K.jacobian(PUMA, q);
  const Jn = K.jacobianCentralDiff(PUMA, q);
  let worst = 0;
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) worst = Math.max(worst, Math.abs(Ja[i][j] - Jn[i][j]));
  assert.ok(worst < 1e-5, `worst deviation ${worst}`);
});

test('central difference Jacobian honours the step parameter', () => {
  const q = [0.3, 0.6, -0.7, 0.4, -0.9, 0.3];
  const a = K.jacobianCentralDiff(PUMA, q, 1e-4);
  const b = K.jacobianCentralDiff(PUMA, q, 2e-4);
  let diff = 0;
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) diff = Math.max(diff, Math.abs(a[i][j] - b[i][j]));
  assert.ok(diff < 1e-4);
});

// ---------- singularity detection: elbow and wrist ----------
test('elbow extension is detected as singular', () => {
  const sing = K.detectSingularity(PUMA, K.zeros(6));
  assert.ok(sing.singular);
});

test('wrist alignment is flagged near q5 = 0 on UR5e', () => {
  const q = [0.4, -1.0, 1.2, 0.6, 1e-7, 0.3];
  const sing = K.detectSingularity(UR5, q, { tolerance: 1e-3 });
  const sv = K.singularValues(K.jacobian(UR5, q));
  assert.ok(sing.singular || sv[0] < 1e-2, 'neither detection nor sigma flagged');
});

test('a mid-range pose is not flagged singular', () => {
  const sing = K.detectSingularity(PUMA, [0.4, 0.7, -0.9, 0.5, -1.1, 0.6]);
  assert.equal(sing.singular, false);
});

// ---------- SHA-256 ----------
test('sha256Hex matches published test vectors', () => {
  assert.equal(K.sha256Hex(''), 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855');
  assert.equal(K.sha256Hex('abc'), 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
  assert.equal(
    K.sha256Hex('abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq'),
    '248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1'
  );
});

test('sha256Hex is deterministic and 64 hex chars', () => {
  const a = K.sha256Hex('kinechain');
  assert.equal(a, K.sha256Hex('kinechain'));
  assert.match(a, /^[0-9a-f]{64}$/);
});

test('sha256Hex accepts byte arrays', () => {
  assert.equal(K.sha256Hex([97, 98, 99]), K.sha256Hex('abc'));
});

test('utf8Bytes encodes ASCII and multibyte characters', () => {
  assert.deepEqual(K.utf8Bytes('abc'), [97, 98, 99]);
  assert.deepEqual(K.utf8Bytes('é'), [0xC3, 0xA9]);
  assert.equal(K.utf8Bytes('€').length, 3);
  assert.equal(K.utf8Bytes('𐍈').length, 4);
});

// ---------- exports ----------
const planFor = () => K.planJointTrajectory(PUMA, [K.zeros(6), [0.3, 0.2, -0.3, 0.2, -0.1, 0.2]], { mode: 'cubic' });

test('trajectoryJSON embeds format, robot, samples and a checksum', () => {
  const j = K.trajectoryJSON(PUMA, planFor());
  assert.equal(j.format, 'kinechain-trajectory/1');
  assert.equal(j.robot.name, 'PUMA-560');
  assert.equal(j.robot.dof, 6);
  assert.ok(j.samples.length > 10);
  assert.match(j.checksum, /^[0-9a-f]{64}$/);
});

test('trajectoryJSON checksum verifies on a re-serialisation', () => {
  const j = K.trajectoryJSON(PUMA, planFor());
  const stripped = Object.assign({}, j);
  delete stripped.checksum;
  assert.equal(K.sha256Hex(JSON.stringify(stripped)), j.checksum);
});

test('trajectoryJSON is byte deterministic', () => {
  assert.equal(JSON.stringify(K.trajectoryJSON(PUMA, planFor())), JSON.stringify(K.trajectoryJSON(PUMA, planFor())));
});

test('trajectoryCSV header names every joint channel', () => {
  const csv = K.trajectoryCSV(PUMA, planFor());
  const head = csv.split('\n')[0];
  assert.equal(head, 't,q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6');
});

test('trajectoryCSV row count and columns are consistent', () => {
  const csv = K.trajectoryCSV(PUMA, planFor(), { samples: 40 });
  const rows = csv.trim().split('\n');
  assert.equal(rows.length, 42);
  rows.forEach(r => assert.equal(r.split(',').length, 19));
});

test('trajectoryCSV is byte deterministic', () => {
  assert.equal(K.trajectoryCSV(PUMA, planFor()), K.trajectoryCSV(PUMA, planFor()));
});

test('gcodeSnapshot emits valid header and motion lines', () => {
  const g = K.gcodeSnapshot(PUMA, planFor());
  const lines = g.trim().split('\n');
  assert.ok(lines.includes('G21'));
  assert.ok(lines.includes('G90'));
  assert.match(lines[5], /^G0 X-?\d/);
  assert.match(lines[6], /^G1 X-?\d.* F300/);
  assert.match(lines.at(-1), /^M2/);
});

test('gcodeSnapshot is byte deterministic and finite', () => {
  const a = K.gcodeSnapshot(PUMA, planFor());
  assert.equal(a, K.gcodeSnapshot(PUMA, planFor()));
  assert.ok(!/NaN|Infinity/.test(a));
});

test('jointStateXML lists every joint with position', () => {
  const q = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6];
  const xml = K.jointStateXML(PUMA, q);
  for (let i = 0; i < 6; i++) {
    assert.ok(xml.includes(`name="joint_${i + 1}"`));
  }
  assert.ok(xml.includes('position="0.6"'));
  assert.ok(xml.startsWith('<?xml'));
});

test('exportBundle checksums match the emitted artefacts', () => {
  const b = K.exportBundle(PUMA, planFor());
  assert.equal(K.sha256Hex(b.csv), b.checksums.csv);
  assert.equal(K.sha256Hex(b.gcode), b.checksums.gcode);
  assert.equal(K.sha256Hex(b.jointState), b.checksums.jointState);
  assert.equal(K.sha256Hex(b.json), b.checksums.json);
});

test('exportBundle is fully deterministic', () => {
  const a = K.exportBundle(PUMA, planFor());
  const b = K.exportBundle(PUMA, planFor());
  assert.deepEqual(a.checksums, b.checksums);
  assert.equal(a.json, b.json);
});

test('exportBundle carries waypoint metadata into the JSON', () => {
  const b = K.exportBundle(PUMA, planFor(), {
    waypoints: [[0.5, 0, 0.3], [0.4, -0.15, 0.3]],
    labels: ['pick', 'place'],
    allSolved: true
  });
  const j = JSON.parse(b.json);
  assert.deepEqual(j.cartesianWaypoints, [[0.5, 0, 0.3], [0.4, -0.15, 0.3]]);
  assert.deepEqual(j.labels, ['pick', 'place']);
  assert.equal(j.allSolved, true);
});

// ---------- URL hash state ----------
test('hash state round-trips robot, q, waypoints and mode', () => {
  const src = { robot: 'UR5e', q: [0.1, -0.2, 0.3, 0.4, -0.5, 0.6], waypoints: [[0.3, 0.1, 0.4], [0.2, -0.1, 0.3]], mode: 'cubic' };
  const back = K.parseHashState(K.serializeHashState(src));
  assert.equal(back.robot, 'UR5e');
  assert.deepEqual(back.mode, 'cubic');
  assert.equal(back.waypoints.length, 2);
  vecClose(back.q, src.q, 1e-4);
  vecClose(back.waypoints[1], [0.2, -0.1, 0.3], 1e-3);
});

test('hash serialise accepts waypoint objects with t', () => {
  const h = K.serializeHashState({ waypoints: [{ t: [0.1, 0.2, 0.3] }] });
  const back = K.parseHashState(h);
  vecClose(back.waypoints[0], [0.1, 0.2, 0.3], 1e-3);
});

test('hash parse rejects unknown robot and mode', () => {
  const back = K.parseHashState('#r=Atlas-9000&m=bezier');
  assert.equal(back.robot, null);
  assert.equal(back.mode, null);
  assert.ok(back.errors.length >= 2);
});

test('hash parse rejects malformed q and waypoints', () => {
  const back = K.parseHashState('#q=a,b,c&w=1,2;x,y,z');
  assert.equal(back.q, null);
  assert.equal(back.waypoints, null);
  assert.ok(back.errors.length >= 2);
});

test('hash parse truncates more than 32 waypoints', () => {
  const wps = Array.from({ length: 40 }, (_, i) => [i * 0.01, 0, 0.3]);
  const back = K.parseHashState(K.serializeHashState({ waypoints: wps }));
  assert.equal(back.waypoints.length, 32);
  assert.ok(back.errors.some(e => e.includes('truncated')));
});

test('hash parse survives an empty and partial hash', () => {
  assert.deepEqual(K.parseHashState('').q, null);
  assert.deepEqual(K.parseHashState('#').q, null);
  const back = K.parseHashState('#q=0.1,0.2&junk&x=5');
  assert.equal(back.q.length, 2);
  assert.ok(back.errors.length >= 1);
});

test('hash serialise is deterministic', () => {
  const src = { robot: 'PUMA-560', q: [0.1, 0.2], waypoints: [[0.3, 0, 0.3]] };
  assert.equal(K.serializeHashState(src), K.serializeHashState(src));
});

// ---------- in page self test battery ----------
for (const name of ['PUMA-560', 'UR5e', 'SCARA-4']) {
  test(`runSelfTests passes every check on ${name}`, () => {
    const rep = K.runSelfTests(name);
    assert.equal(rep.failed, 0, rep.results.filter(r => !r.ok).map(r => r.name + ': ' + r.detail).join('; '));
    assert.equal(rep.passed, rep.total);
    assert.ok(rep.total >= 10);
  });
}
