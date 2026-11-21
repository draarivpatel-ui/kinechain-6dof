import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const K = require('../src/robotics.js');

const close = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} !~= ${b}`);
const vecClose = (a, b, tol = 1e-9) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tol)); };
const matClose = (A, B, tol = 1e-9) => A.forEach((r, i) => vecClose(r, B[i], tol));

function seeded(seed) { let s = seed; return () => { s = (s * 1103515245 + 12345) % 2147483648; return s / 2147483648; }; }
function inLimits(robot) { const r = seeded(20261121); return () => robot.links.map(l => l.lo + r() * (l.hi - l.lo)); }
const ALL_ROBOTS = [K.PUMA_560, K.UR5E, K.SCARA];

// ---------- FK rotation orthonormality ----------
for (const robot of ALL_ROBOTS) {
  test(`${robot.name}: every FK frame is a proper rotation (R^T R = I)`, () => {
    const rand = inLimits(robot);
    for (let t = 0; t < 60; t++) {
      const q = rand();
      const fk = K.forwardKinematics(robot, q);
      for (const frame of fk.frames.concat([fk.end])) {
        const R = frame.R;
        matClose(K.matMul(K.matTranspose(R), R), K.matIdentity(3), 1e-10);
      }
    }
  });

  test(`${robot.name}: every FK frame has det(R) = +1`, () => {
    const rand = inLimits(robot);
    for (let t = 0; t < 60; t++) {
      const fk = K.forwardKinematics(robot, rand());
      for (const frame of fk.frames.concat([fk.end])) {
        close(K.matDet(frame.R), 1, 1e-10);
      }
    }
  });

  test(`${robot.name}: FK equals chained dhMatrix products`, () => {
    const q = inLimits(robot)();
    const fk = K.forwardKinematics(robot, q);
    let cur = robot.base;
    for (let i = 0; i < robot.dof; i++) {
      const p = K.dhParams(robot, q)[i];
      cur = K.poseMul(cur, K.dhMatrix(p.a, p.alpha, p.d, p.theta));
      vecClose(cur.t, fk.frames[i + 1].t, 1e-12);
      matClose(cur.R, fk.frames[i + 1].R, 1e-12);
    }
  });

  test(`${robot.name}: end pose equals last frame times tool`, () => {
    const q = inLimits(robot)();
    const fk = K.forwardKinematics(robot, q);
    const manual = K.poseMul(fk.frames[robot.dof], robot.tool);
    vecClose(manual.t, fk.end.t, 1e-12);
    matClose(manual.R, fk.end.R, 1e-12);
  });

  test(`${robot.name}: finite outputs for extreme joint values`, () => {
    for (const sign of [0, 1]) {
      const q = robot.links.map(l => sign ? l.hi : l.lo);
      const e = K.endPose(robot, q);
      for (let i = 0; i < 3; i++) assert.ok(Number.isFinite(e.t[i]));
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) assert.ok(Number.isFinite(e.R[i][j]));
    }
  });

  test(`${robot.name}: Jacobian matches a numeric derivative`, () => {
    const q = inLimits(robot)();
    const J = K.jacobian(robot, q);
    const dq = robot.links.map(() => 0.3);
    const vNum = K.endpointVelocityNumerical(robot, q, dq);
    const vJac = K.matVec(J, dq).slice(0, 3);
    vecClose(vNum, vJac, 2e-3);
  });

  test(`${robot.name}: Jacobian has 6 rows and dof columns`, () => {
    const J = K.jacobian(robot, inLimits(robot)());
    assert.equal(J.length, 6);
    for (const row of J) assert.equal(row.length, robot.dof);
  });

  test(`${robot.name}: singular values sorted, nonnegative, right count`, () => {
    const s = K.singularValues(K.jacobian(robot, inLimits(robot)()).map(r => r.slice()));
    assert.equal(s.length, robot.dof);
    for (const v of s) assert.ok(v >= -1e-12);
    for (let i = 1; i < s.length; i++) assert.ok(s[i] >= s[i - 1]);
  });
}

// ---------- manipulability and ellipsoid ----------
test('manipulability equals the product of singular values', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const s = K.singularValues(K.jacobian(K.PUMA_560, q).map(r => r.slice()));
  const prod = s.reduce((a, v) => a * v, 1);
  close(K.manipulability(K.PUMA_560, q), prod, 1e-6);
});

test('ellipsoidAxes returns axes consistent with the Jacobian', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const e = K.ellipsoidAxes(K.PUMA_560, q);
  assert.equal(e.axes.length, 3);
  assert.equal(e.basis.length, 3);
  for (const b of e.basis) {
    assert.equal(b.axis.length, 3);
    assert.ok(b.length > 0 && b.length <= 1.0001);
    assert.ok(b.sigma >= 0);
  }
  for (const a of e.axes) assert.ok(a >= 0);
  assert.ok(e.cond >= 1);
  const s = K.singularValues(K.jacobian(K.PUMA_560, q).map(r => r.slice()));
  vecClose(e.axes.slice().sort((a, b) => a - b), s.slice(-3), 1e-9);
});

test('manipulabilityProfile stays inside joint limits (regression)', () => {
  const prof = K.manipulabilityProfile(K.PUMA_560, 8);
  assert.equal(prof.length, 8);
  for (const p of prof) {
    assert.ok(K.jointWithinLimits(K.PUMA_560, p.q));
    assert.ok(Number.isFinite(p.w));
  }
});

test('manipulabilityProfile interpolation covers both corners', () => {
  const prof = K.manipulabilityProfile(K.SCARA, 2);
  vecClose(prof[0].q, K.SCARA.links.map(l => l.lo));
  vecClose(prof[1].q, K.SCARA.links.map(l => l.hi));
});

// ---------- singularity detection ----------
test('detectSingularity: PUMA zero pose is wrist singular', () => {
  const s = K.detectSingularity(K.PUMA_560, [0, 0, 0, 0, 0, 0]);
  assert.equal(s.singular, true);
  assert.equal(s.type, 'wrist');
  assert.ok(s.sigmaMin < 1e-5);
});

test('detectSingularity: mid pose is regular', () => {
  const s = K.detectSingularity(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]);
  assert.equal(s.singular, false);
  assert.equal(s.type, 'none');
});

test('detectSingularity: SCARA fully extended is boundary', () => {
  const s = K.detectSingularity(K.SCARA, [0, 0, 0, 0]);
  assert.equal(s.singular, true);
  assert.equal(s.type, 'boundary');
});

test('detectSingularity: SCARA bent pose is regular (regression)', () => {
  // Old SCARA table made joints 1 and 2 coaxial, so sigma_min was always 0.
  const s = K.detectSingularity(K.SCARA, [0.9, 1.5, 0.05, 0.3]);
  assert.equal(s.singular, false);
});

test('SCARA joint axes are no longer coaxial (regression)', () => {
  const q = [0.9, 1.5, 0.05, 0.3];
  const J = K.jacobian(K.SCARA, q);
  const s = K.singularValues(J.map(r => r.slice()));
  assert.ok(s[0] > 0.02, `sigma_min ${s[0]} should be nonzero for a real SCARA`);
});

// ---------- IK ----------
test('solveIK converges on an FK-generated target', () => {
  const qstar = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const res = K.solveIK(K.PUMA_560, [0, 0, 0, 0, 0, 0], K.endPose(K.PUMA_560, qstar), { maxIter: 400 });
  assert.equal(res.converged, true);
  assert.ok(res.error < 1e-5);
  assert.ok(res.positionError < 1e-5);
  assert.ok(res.rotationError < 1e-4);
});

test('solveIK honours rotation weighting option (regression weightRot)', () => {
  const qstar = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const t = K.endPose(K.PUMA_560, qstar);
  for (const w of [0.5, 1, 2]) {
    const res = K.solveIK(K.PUMA_560, [0, 0, 0, 0, 0, 0], t, { weightRotation: w, maxIter: 400 });
    assert.equal(res.converged, true, `weightRotation ${w} should still converge`);
  }
});

test('solveIK sweeps FK targets for all three robots', () => {
  for (const robot of ALL_ROBOTS) {
    const rand = inLimits(robot);
    let fails = 0;
    for (let i = 0; i < 10; i++) {
      const qs = rand().map((v, j) => robot.links[j].lo + (v - robot.links[j].lo) * 0.6);
      const res = K.solveIKMultiStart(robot, K.endPose(robot, qs), { maxIter: 400 });
      if (!res.converged) fails++;
    }
    assert.ok(fails <= 3, `${robot.name} failed ${fails}/10 interior targets`);
  }
});

test('solveIK result stays inside joint limits', () => {
  const res = K.solveIK(K.PUMA_560, [0, 0, 0, 0, 0, 0], K.endPose(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]), {});
  assert.ok(res.withinLimits);
  assert.ok(K.jointWithinLimits(K.PUMA_560, res.q));
});

test('solveIK does not converge on a far unreachable target', () => {
  const res = K.solveIK(K.PUMA_560, [0, 0, 0, 0, 0, 0], { R: K.matIdentity(3), t: [5, 5, 5] }, { maxIter: 60 });
  assert.equal(res.converged, false);
  assert.ok(res.error > 0.1);
});

test('solveIK relaxes rotation only when rotTol allows (regression)', () => {
  // A huge rotTol must not fake convergence: position still has to land.
  const res = K.solveIKMultiStart(K.PUMA_560, { R: K.matIdentity(3), t: [0.3, -0.1, 0.45] }, {
    seeds: 12, maxIter: 600, weightRotation: 0, rotTol: 1e9
  });
  // position-only solve either converges with tiny pos error or fails honestly
  if (res.converged) assert.ok(res.positionError < 1e-5);
  else assert.ok(res.positionError > 1e-6);
});

test('solveIKMultiStart accepts a numeric seed count (regression)', () => {
  const res = K.solveIKMultiStart(K.PUMA_560, K.endPose(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]), { seeds: 4, maxIter: 300 });
  assert.equal(res.converged, true);
});

test('solveIKMultiStart accepts explicit seed arrays', () => {
  const seeds = [K.zeros(6), [0.2, 0.4, -0.3, 0.1, -0.2, 0.1]];
  const res = K.solveIKMultiStart(K.PUMA_560, K.endPose(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]), { seeds: seeds, maxIter: 300 });
  assert.equal(res.converged, true);
});

test('twistError is zero at the target pose', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const e = K.twistError(K.PUMA_560, q, K.endPose(K.PUMA_560, q));
  for (const v of e) close(v, 0, 1e-9);
});

test('IK with obstacle reports penalty and still solves', () => {
  const qstar = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const res = K.solveIK(K.PUMA_560, [0, 0, 0, 0, 0, 0], K.endPose(K.PUMA_560, qstar), {
    maxIter: 400, obstacles: [{ x: 0, y: 0, z: -0.3, radius: 0.05, margin: 0.05 }]
  });
  assert.equal(res.converged, true);
  assert.ok(Number.isFinite(res.obstaclePenalty));
});

// ---------- joint limits ----------
test('clampToLimits respects every link', () => {
  const q = K.clampToLimits(K.PUMA_560, [9, -9, 9, -9, 9, -9]);
  assert.ok(K.jointWithinLimits(K.PUMA_560, q));
});

test('jointViolations lists out-of-range joints', () => {
  const v = K.jointViolations(K.PUMA_560, [99, 0, 0, 0, 0, -99]);
  assert.ok(v.length >= 2);
  assert.ok(v.some(x => x.joint === 0 || x.index === 0));
});

test('limitUtilization is within [0,1] for legal q', () => {
  const u = K.limitUtilization(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]);
  for (const x of u) assert.ok(x >= 0 && x <= 1);
});

test('reachable returns a boolean target check', () => {
  assert.equal(K.reachable(K.PUMA_560, { t: [0.3, 0.1, 0.4] }), true);
  assert.equal(K.reachable(K.PUMA_560, { t: [30, 10, 40] }), false);
});

// ---------- collisions ----------
test('sphereDistance and spheresCollide agree (regression)', () => {
  close(K.sphereDistance([0, 0, 0], { x: 0.15, y: 0, z: 0 }), 0.15, 1e-12);
  assert.equal(K.spheresCollide({ x: 0, y: 0, z: 0, radius: 0.1 }, { x: 0.15, y: 0, z: 0, radius: 0.1 }), true);
  assert.equal(K.spheresCollide({ x: 0, y: 0, z: 0, radius: 0.1 }, { x: 0.5, y: 0, z: 0, radius: 0.1 }), false);
});

test('segmentDistance is the point to segment distance', () => {
  close(K.segmentDistance([0.5, 0.2, 0], [0, 0, 0], [1, 0, 0]), 0.2, 1e-12);
  close(K.segmentDistance([3, 0, 0], [0, 0, 0], [1, 0, 0]), 2, 1e-12);
});

test('checkCollisions detects a placed sphere on a link', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const fk = K.forwardKinematics(K.PUMA_560, q);
  const mid = fk.frames[2].t.map((v, i) => (v + fk.frames[3].t[i]) / 2);
  const r = K.checkCollisions(K.PUMA_560, q, [{ x: mid[0], y: mid[1], z: mid[2], radius: 0.05 }], {});
  assert.equal(r.collided, true);
  assert.ok(r.collisions.length >= 1);
});

test('checkCollisions links filter names link segments (regression)', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const fk = K.forwardKinematics(K.PUMA_560, q);
  const mid = fk.frames[2].t.map((v, i) => (v + fk.frames[3].t[i]) / 2);
  const obs = [{ x: mid[0], y: mid[1], z: mid[2], radius: 0.05 }];
  // The collision lives on link segment 2 (between frames 2 and 3);
  // filtering to link 0 must report clear.
  const r0 = K.checkCollisions(K.PUMA_560, q, obs, { links: [0] });
  assert.equal(r0.collided, false);
  const r2 = K.checkCollisions(K.PUMA_560, q, obs, { links: [2] });
  assert.equal(r2.collided, true);
});

test('checkCollisions disabled obstacles are skipped', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const fk = K.forwardKinematics(K.PUMA_560, q);
  const mid = fk.frames[2].t.map((v, i) => (v + fk.frames[3].t[i]) / 2);
  const r = K.checkCollisions(K.PUMA_560, q, [{ x: mid[0], y: mid[1], z: mid[2], radius: 0.05, enabled: false }], {});
  assert.equal(r.collided, false);
});

test('obstaclePotential is positive inside the margin', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const p = K.obstaclePotential(K.PUMA_560, q, [{ x: 0.3, y: 0.1, z: 0.4, radius: 0.5, margin: 0.2 }]);
  assert.ok(p > 0);
  const far = K.obstaclePotential(K.PUMA_560, q, [{ x: 10, y: 10, z: 10, radius: 0.1, margin: 0.05 }]);
  close(far, 0, 1e-12);
});

test('clearanceProfile returns checkCollisions minClearance', () => {
  const q = [0.4, 0.9, -0.7, 0.3, -0.8, 0.5];
  const obs = [{ x: 0.3, y: 0.1, z: 0.4, radius: 0.1 }];
  close(K.clearanceProfile(K.PUMA_560, q, obs), K.checkCollisions(K.PUMA_560, q, obs, {}).minClearance, 1e-12);
});

test('linkSamplePoints returns finite samples', () => {
  const pts = K.linkSamplePoints(K.PUMA_560, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5], 4);
  assert.ok(pts.length > 0);
  for (const p of pts) {
    assert.ok(typeof p.link === 'number');
    for (const v of p.point) assert.ok(Number.isFinite(v));
  }
});

// ---------- trajectories ----------
test('quinticSegment honours all four boundary conditions (regression)', () => {
  const T = 2;
  for (const [v0, v1, a0, a1] of [[0.3, 0, 0, 0], [0, 0.7, 0, 0], [0.2, -0.4, 0.5, -0.2]]) {
    const s0 = K.quinticSegment(0, T, 0, 1, v0, v1, a0, a1);
    const s1 = K.quinticSegment(T, T, 0, 1, v0, v1, a0, a1);
    close(s0.position, 0); close(s0.velocity, v0, 1e-9); close(s0.accel, a0, 1e-9);
    close(s1.position, 1); close(s1.velocity, v1, 1e-9); close(s1.accel, a1, 1e-9);
  }
});

test('quinticSegment midpoint is finite', () => {
  const s = K.quinticSegment(1, 2, 0, 1, 0.2, 0.3, 0, 0);
  assert.ok(Number.isFinite(s.position) && Number.isFinite(s.velocity) && Number.isFinite(s.accel));
});

test('planTrajectory endpoints land on the waypoints', () => {
  const plan = K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0, 0, 0, 0] }, { q: [0.3, 0.5, -0.4, 0.2, -0.3, 0.1] }]);
  vecClose(plan.sample(0).q, [0, 0, 0, 0, 0, 0], 1e-9);
  vecClose(plan.sample(plan.totalTime).q, [0.3, 0.5, -0.4, 0.2, -0.3, 0.1], 1e-9);
});

test('planTrajectory seam continuity carries waypoint velocity (regression)', () => {
  const plan = K.planTrajectory(K.PUMA_560, [
    { q: [0, 0, 0, 0, 0, 0] },
    { q: [0.5, 0.5, 0, 0, 0, 0], v: [0.5, 0, 0, 0, 0, 0] },
    { q: [0.9, 0.8, 0, 0, 0, 0] }
  ]);
  const seam = plan.segments[0].t1;
  const pre = plan.sample(seam - 1e-4);
  const post = plan.sample(seam + 1e-4);
  close(pre.dq[0], 0.5, 1e-3);
  close(post.dq[0], 0.5, 1e-3);
});

test('planTrajectory segment velocity at waypoint matches spec exactly', () => {
  const plan = K.planTrajectory(K.PUMA_560, [
    { q: [0, 0, 0, 0, 0, 0] },
    { q: [0.5, 0.5, 0, 0, 0, 0], v: [0.5, 0.25, 0, 0, 0, 0] },
    { q: [0.9, 0.8, 0, 0, 0, 0] }
  ]);
  const at = plan.sample(plan.segments[0].t1);
  close(at.dq[0], 0.5, 1e-9);
  close(at.dq[1], 0.25, 1e-9);
});

test('planTrajectory rejects malformed waypoints', () => {
  assert.throws(() => K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0, 0, 0, 0] }]), /two waypoints/);
  assert.throws(() => K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0] }, { q: [0, 0, 0, 0, 0, 0] }]), /6 joint/);
  assert.throws(() => K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0, 0, 0, 0] }, { q: [NaN, 0, 0, 0, 0, 0] }]), /nonfinite/);
});

test('planTrajectory accepts raw joint arrays', () => {
  const plan = K.planTrajectory(K.PUMA_560, [[0, 0, 0, 0, 0, 0], [0.2, 0.3, -0.2, 0.1, -0.1, 0.05]]);
  assert.ok(plan.totalTime > 0);
  vecClose(plan.sample(plan.totalTime).q, [0.2, 0.3, -0.2, 0.1, -0.1, 0.05], 1e-9);
});

test('planTrajectory validate option measures the real path (regression)', () => {
  const good = K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0, 0, 0, 0] }, { q: [0.3, 0.5, -0.4, 0.2, -0.3, 0.1] }], { validate: true });
  assert.equal(good.validity.inLimits, true);
  assert.ok(good.validity.maxJerk > 0);
  assert.ok(good.validity.sampleCount >= 8);
  const bad = K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0, 0, 0, 0] }, { q: [99, 0, 0, 0, 0, 0] }], { validate: true });
  assert.equal(bad.validity.inLimits, false);
  assert.ok(bad.validity.violations.length > 0);
});

test('sampleRange covers the full duration', () => {
  const plan = K.planTrajectory(K.PUMA_560, [{ q: [0, 0, 0, 0, 0, 0] }, { q: [0.3, 0.5, -0.4, 0.2, -0.3, 0.1] }]);
  const samples = plan.sampleRange(10);
  assert.equal(samples.length, 11);
  close(samples[0].t, 0);
  close(samples[samples.length - 1].t, plan.totalTime);
});

test('buildPickPlace assembles durations (regression)', () => {
  const built = K.buildPickPlace(K.PUMA_560, [
    { name: 'pick', q: [0.4, 0.9, -0.7, 0.3, -0.8, 0.5], duration: 1.5 },
    { name: 'place', q: [0, 0, 0, 0, 0, 0], duration: 2 }
  ]);
  assert.ok(built.plan.totalTime > 0);
  close(built.totalTime, 1.5 + 2 + 2.0, 1e-9);
  assert.equal(built.moves.length, 2);
});

test('buildPickPlace honours homeDuration and returnDuration', () => {
  const built = K.buildPickPlace(K.PUMA_560, [
    { name: 'pick', q: [0.4, 0.9, -0.7, 0.3, -0.8, 0.5], duration: 1.5 }
  ], { homeDuration: 3.0, returnDuration: 4.0 });
  close(built.totalTime, 3.0 + 4.0, 1e-9);
});

test('buildPickPlace empty segments still yields a valid plan', () => {
  const built = K.buildPickPlace(K.PUMA_560, []);
  assert.ok(Number.isFinite(built.totalTime));
  assert.ok(built.plan.totalTime > 0);
});

// ---------- URDF export ----------
test('exportURDF output validates itself (regression)', () => {
  const xml = K.exportURDF(K.PUMA_560);
  const v = K.validateURDF(xml);
  assert.equal(v.valid, true);
  assert.equal(v.links, 9);
  assert.equal(v.joints, 8);
  assert.equal(v.revolute, 6);
  assert.equal(v.fixed, 2);
});

test('exportURDF validates for UR5e and SCARA', () => {
  for (const robot of [K.UR5E, K.SCARA]) {
    const v = K.validateURDF(K.exportURDF(robot));
    assert.equal(v.valid, true, robot.name);
  }
});

test('URDF joint origins reproduce the FK chain (regression)', () => {
  const robot = K.UR5E;
  const xml = K.exportURDF(robot);
  const joints = [];
  const re = /<joint\s+name="([^"]+)"\s+type="(?:revolute|continuous|prismatic)">[\s\S]*?<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\/>/g;
  let m;
  while ((m = re.exec(xml))) joints.push({ xyz: m[2].split(' ').map(Number), rpy: m[3].split(' ').map(Number) });
  assert.equal(joints.length, 6);
  const tool = /<joint\s+name="tool_joint"[^>]*>[\s\S]*?<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\/>/.exec(xml);
  const base = /<joint\s+name="world_to_base"[^>]*>[\s\S]*?<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\/>/.exec(xml);
  const q = [0.4, -1.2, 1.0, -1.2, 1.5708, 0.6];
  let T = { R: K.eulerToMat(+base[2].split(' ')[0], +base[2].split(' ')[1], +base[2].split(' ')[2]), t: base[1].split(' ').map(Number) };
  for (let i = 0; i < 6; i++) {
    const J = joints[i];
    T = K.poseMul(T, { R: K.eulerToMat(J.rpy[0], J.rpy[1], J.rpy[2]), t: J.xyz });
    T = K.poseMul(T, { R: K.eulerToMat(0, 0, q[i]), t: [0, 0, 0] });
  }
  T = K.poseMul(T, { R: K.eulerToMat(+tool[2].split(' ')[0], +tool[2].split(' ')[1], +tool[2].split(' ')[2]), t: tool[1].split(' ').map(Number) });
  const want = K.endPose(robot, q);
  vecClose(T.t, want.t, 1e-5);
});

test('extractDHRows returns one row per link (regression)', () => {
  const rows = K.extractDHRows(K.exportURDF(K.PUMA_560));
  assert.equal(rows.length, 6);
  assert.equal(rows[0].name, 'shoulder');
  for (const r of rows) {
    assert.ok(Number.isFinite(r.a) && Number.isFinite(r.alpha) && Number.isFinite(r.d) && Number.isFinite(r.theta));
    assert.ok(r.name.length > 0 && r.name.length < 40);
  }
});

test('xmlEscape and num survive hostile names', () => {
  const robot = K.createRobot({ name: 'x', links: [{ name: 'a<b>&"c', a: 0.1, alpha: 0, d: 0, lo: -1, hi: 1 }] });
  const xml = K.exportURDF(robot);
  assert.ok(!xml.includes('a<b>&"c'));
  assert.ok(xml.includes('a&lt;b&gt;&amp;&quot;c'));
});

// ---------- reporting ----------
test('robotReport fields for all robots', () => {
  for (const robot of ALL_ROBOTS) {
    const rep = K.robotReport(robot, K.zeros(robot.dof));
    assert.equal(rep.dof, robot.dof);
    assert.ok(Array.isArray(rep.frames));
    assert.ok(typeof rep.manipulability === 'number');
    assert.ok(typeof rep.singular === 'boolean');
    assert.ok(rep.limits && typeof rep.limits.within === 'boolean');
  }
});

test('jacobianDot matches finite difference of jacobian', () => {
  const q = [0.3, 0.4, -0.5, 0.2, 0.6, -0.1];
  const dq = [0.1, -0.2, 0.05, 0.3, -0.1, 0.2];
  const Jd = K.jacobianDot(K.PUMA_560, q, dq);
  const h = 1e-5;
  const J2 = K.jacobian(K.PUMA_560, q.map((v, i) => v + dq[i] * h));
  const J1 = K.jacobian(K.PUMA_560, q);
  let maxErr = 0;
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) maxErr = Math.max(maxErr, Math.abs(Jd[i][j] - (J2[i][j] - J1[i][j]) / h));
  assert.ok(maxErr < 0.05, `jacobianDot deviation ${maxErr}`);
});

test('approx and matEquals comparisons', () => {
  assert.ok(K.approx(1.000000001, 1, 1e-6));
  assert.ok(K.matEquals(K.matIdentity(3), K.matIdentity(3)));
  assert.ok(!K.matEquals(K.matIdentity(3), K.matScale(K.matIdentity(3), 2)));
});

test('matClone deep-copies', () => {
  const A = [[1, 2], [3, 4]];
  const B = K.matClone(A);
  B[0][0] = 99;
  assert.equal(A[0][0], 1);
});

// ---------- all-finite sweep ----------
test('all public numeric outputs stay finite across poses', () => {
  for (const robot of ALL_ROBOTS) {
    const rand = inLimits(robot);
    for (let i = 0; i < 12; i++) {
      const q = rand();
      const fk = K.forwardKinematics(robot, q);
      const J = K.jacobian(robot, q);
      const sv = K.singularValues(J.map(r => r.slice()));
      const s = K.detectSingularity(robot, q);
      const m = K.manipulability(robot, q);
      assert.ok(Number.isFinite(m));
      for (const v of sv) assert.ok(Number.isFinite(v));
      assert.ok(Number.isFinite(s.sigmaMin));
      for (const f of fk.frames) for (const v of f.t) assert.ok(Number.isFinite(v));
    }
  }
});
