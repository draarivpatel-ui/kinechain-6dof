import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const K = require('../src/robotics.js');
const PUMA = K.PUMA_560;

test('trajectory validation flags negative velocity and acceleration excursions', () => {
  const q0 = K.zeros(6);
  const q1 = [-1.5, 0.4, -1.0, 0.2, -0.3, 0.1];
  const plan = K.planTrajectory(PUMA, [q0, q1], { durations: [0.03], validate: true });
  assert.equal(plan.validity.inVelocityLimits, false);
  assert.equal(plan.validity.inAccelLimits, false);
  assert.ok(plan.validity.violations.some(v => v.kind === 'velocity' && v.value < 0), 'negative velocity breach missing');
  assert.ok(plan.validity.violations.some(v => v.kind === 'acceleration' && v.value < 0), 'negative acceleration breach missing');
});

test('sampleRange requires an integer count in both planners', () => {
  const waypoints = [K.zeros(6), [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]];
  const quintic = K.planTrajectory(PUMA, waypoints);
  const cubic = K.planJointTrajectory(PUMA, waypoints, { mode: 'cubic' });
  assert.throws(() => quintic.sampleRange(2.5), RangeError);
  assert.throws(() => cubic.sampleRange(2.5), RangeError);
});

test('manipulabilityProfile rejects malformed sample counts', () => {
  for (const count of [NaN, 0, -2, 2.5]) {
    assert.throws(() => K.manipulabilityProfile(PUMA, count), RangeError);
  }
});

test('linkSamplePoints clamps hostile sample counts instead of dropping or looping forever', () => {
  const nanPoints = K.linkSamplePoints(PUMA, K.zeros(6), NaN);
  assert.ok(nanPoints.length > 0);
  assert.ok(nanPoints.every(p => p.point.every(Number.isFinite)));
});

test('jointViolations reports nonfinite joint values', () => {
  const bad = K.jointViolations(PUMA, [NaN, 0, 0, 0, 0, 0]);
  assert.equal(bad.length, 1);
  assert.equal(bad[0].index, 0);
  assert.equal(bad[0].side, 'nonfinite');
});

test('cubicSpline1D rejects nonfinite boundary velocities', () => {
  assert.throws(() => K.cubicSpline1D([0, 1], [0, 1], { v0: NaN }), RangeError);
  assert.throws(() => K.cubicSpline1D([0, 1], [0, 1], { vn: Infinity }), RangeError);
});

test('collision inputs reject nonfinite coordinates and negative radii or margins', () => {
  const q = K.zeros(6);
  assert.throws(() => K.checkCollisions(PUMA, q, [{ x: NaN, y: 0, z: 0, radius: 0.1 }]), RangeError);
  assert.throws(() => K.checkCollisions(PUMA, q, [{ x: 0, y: 0, z: 0, radius: -0.1 }]), RangeError);
  assert.throws(() => K.checkCollisions(PUMA, q, [{ x: 0, y: 0, z: 0, radius: 0.1 }], { clearance: -0.01 }), RangeError);
  assert.throws(() => K.obstaclePotential(PUMA, q, [{ x: 0, y: 0, z: 0, radius: 0.1, margin: -0.2 }]), RangeError);
});

test('createRobot rejects negative velocity and effort while preserving explicit zero', () => {
  const base = { name: 'x', links: [{ name: 'l', a: 0.1, lo: -1, hi: 1 }] };
  assert.throws(() => K.createRobot({ ...base, links: [{ ...base.links[0], velocity: -1 }] }), RangeError);
  assert.throws(() => K.createRobot({ ...base, links: [{ ...base.links[0], effort: -1 }] }), RangeError);
  const zero = K.createRobot({ ...base, links: [{ ...base.links[0], velocity: 0, effort: 0 }] });
  assert.equal(zero.links[0].velocity, 0);
});

test('URDF inertial output uses the required izz attribute', () => {
  const xml = K.exportURDF(PUMA);
  assert.match(xml, /izz="/);
  assert.doesNotMatch(xml, /zzz="/);
});

test('only a full two pi both sided range exports as a continuous joint', () => {
  const robot = K.createRobot({
    name: 'one-sided',
    links: [{ name: 'l', a: 0.1, alpha: 0, d: 0, theta: 0, lo: -2 * Math.PI, hi: 0 }]
  });
  const xml = K.exportURDF(robot);
  assert.match(xml, /type="revolute"/);
  assert.match(xml, /lower="/);
  assert.doesNotMatch(xml, /type="continuous"/);
});

test('exportURDF refuses comment-breaking double hyphen link names', () => {
  const robot = K.createRobot({
    name: 'x',
    links: [{ name: 'bad--link', a: 0.1, alpha: 0, d: 0, theta: 0, lo: -1, hi: 1 }]
  });
  assert.throws(() => K.exportURDF(robot), /double hyphen/);
});

test('extractDHRows survives a colon in a link name', () => {
  const robot = K.createRobot({
    name: 'x',
    links: [{ name: 'forearm:left', a: 0.1, alpha: 0, d: 0, theta: 0, lo: -1, hi: 1 }]
  });
  const rows = K.extractDHRows(K.exportURDF(robot));
  assert.equal(rows.length, 1);
  assert.equal(rows[0].name, 'forearm:left');
});

test('solveIK rejects malformed targets and solver controls', () => {
  const target = K.endPose(PUMA, [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]);
  assert.throws(() => K.solveIK(PUMA, K.zeros(6), null), RangeError);
  assert.throws(() => K.solveIK(PUMA, K.zeros(6), { R: K.matIdentity(3) }), RangeError);
  for (const options of [
    { maxIter: 0 }, { maxIter: 2.5 }, { posTol: -1 }, { rotTol: -1 },
    { damping: 0 }, { damping: NaN }, { obstacleWeight: -0.1 },
    { stepScale: 0 }, { stepScale: 1.2 }, { maxStep: NaN }, { maxStep: -0.2 }
  ]) {
    assert.throws(() => K.solveIK(PUMA, K.zeros(6), target, options), RangeError, JSON.stringify(options));
  }
});

test('trajectory planners reject nonpositive velocity and acceleration scales', () => {
  const waypoints = [K.zeros(6), [0.4, 0.9, -0.7, 0.3, -0.8, 0.5]];
  assert.throws(() => K.planTrajectory(PUMA, waypoints, { velocityScale: 0 }), RangeError);
  assert.throws(() => K.planTrajectory(PUMA, waypoints, { accelScale: -1 }), RangeError);
  assert.throws(() => K.planJointTrajectory(PUMA, waypoints, { velocityScale: NaN }), RangeError);
  assert.throws(() => K.planJointTrajectory(PUMA, waypoints, { accelScale: Infinity }), RangeError);
});

test('floorClearance rejects a nonfinite floor plane', () => {
  assert.throws(() => K.floorClearance(PUMA, K.zeros(6), NaN), RangeError);
  assert.throws(() => K.floorClearance(PUMA, K.zeros(6), Infinity), RangeError);
});

test('solveWaypointChain normalizes or rejects malformed direct input', () => {
  assert.throws(() => K.solveWaypointChain(PUMA, [{ x: 0.4, y: 0 }]), RangeError);
  assert.throws(() => K.solveWaypointChain(PUMA, [null]), RangeError);
});

test('parseHashState records malformed percent escapes instead of throwing', () => {
  const parsed = K.parseHashState('#r=PUMA-560&q=%E0%A4%A');
  assert.equal(parsed.robot, 'PUMA-560');
  assert.ok(parsed.errors.some(e => /decode|escape|malformed/i.test(e)));
});

test('hash state round-trips optional waypoint dwell values', () => {
  const src = { waypoints: [{ t: [0.3, 0.2, 0.4], dwell: 1.25 }, { t: [0.2, -0.1, 0.3], dwell: 0 }] };
  const encoded = K.serializeHashState(src);
  assert.match(encoded, /0\.3,0\.2,0\.4,1\.25/);
  const back = K.parseHashState(encoded);
  assert.equal(back.waypoints.length, 2);
  assert.deepEqual(back.dwells, [1.25, 0]);
});
