/* KineChain 6DOF
 * Pure JavaScript spatial robotics and motion planning core.
 *
 *   Denavit-Hartenberg SE(3) forward kinematics for PUMA-560, a
 *   UR5e class 6-DOF arm and a SCARA 4-DOF arm, plus quaternion and
 *   Euler conversions, the geometric Jacobian, Yoshikawa manipulability,
 *   a singularity detector, damped least squares inverse kinematics with
 *   joint limits and sphere obstacle avoidance, a quintic minimum jerk
 *   trajectory planner, and a ROS URDF XML exporter.
 *
 * Zero dependencies. The same file runs under Node via require() and in a
 * browser through the global KineChain object.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.KineChain = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  // ============================================================
  // Module 1: scalar and small vector helpers
  // ============================================================
  const RAD = Math.PI / 180;
  const DEG = 180 / Math.PI;

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  function lerp(a, b, t) { return a + (b - a) * t; }

  function round(v, p) { const f = Math.pow(10, p || 0); return Math.round(v * f) / f; }

  function toRad(deg) { return deg * RAD; }
  function toDeg(rad) { return rad * DEG; }
  function approx(a, b, tol) { return Math.abs(a - b) <= (tol === undefined ? 1e-9 : tol); }

  function sum(a) { let s = 0; for (let i = 0; i < a.length; i++) s += a[i]; return s; }
  function mean(a) { return a.length ? sum(a) / a.length : 0; }

  function dot(a, b) { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; }

  function cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]
    ];
  }

  function vecAdd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function vecSub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function vecScale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }

  function norm(a) {
    let s = 0;
    for (let i = 0; i < a.length; i++) s += a[i] * a[i];
    return Math.sqrt(s);
  }

  function normalize(a, fallback) {
    const n = norm(a);
    if (n < 1e-12) return fallback || [0, 0, 1];
    return [a[0] / n, a[1] / n, a[2] / n];
  }

  // Smallest angle between two vectors, in radians.
  function angleBetween(a, b) {
    const na = norm(a), nb = norm(b);
    if (na < 1e-12 || nb < 1e-12) return 0;
    const c = clamp(dot(a, b) / (na * nb), -1, 1);
    return Math.acos(c);
  }

  // ============================================================
  // Module 2: dense matrices
  // A matrix is an array of row arrays.
  // ============================================================
  function matZeros(r, c) {
    const m = new Array(r);
    for (let i = 0; i < r; i++) m[i] = new Array(c).fill(0);
    return m;
  }

  function matIdentity(n) {
    const m = matZeros(n, n);
    for (let i = 0; i < n; i++) m[i][i] = 1;
    return m;
  }

  function matAdd(a, b) {
    const m = matZeros(a.length, a[0].length);
    for (let i = 0; i < a.length; i++) for (let j = 0; j < a[0].length; j++) m[i][j] = a[i][j] + b[i][j];
    return m;
  }

  function matSub(a, b) {
    const m = matZeros(a.length, a[0].length);
    for (let i = 0; i < a.length; i++) for (let j = 0; j < a[0].length; j++) m[i][j] = a[i][j] - b[i][j];
    return m;
  }

  function matScale(a, s) {
    const m = matZeros(a.length, a[0].length);
    for (let i = 0; i < a.length; i++) for (let j = 0; j < a[0].length; j++) m[i][j] = a[i][j] * s;
    return m;
  }

  function matTranspose(a) {
    const m = matZeros(a[0].length, a.length);
    for (let i = 0; i < a.length; i++) for (let j = 0; j < a[0].length; j++) m[j][i] = a[i][j];
    return m;
  }

  function matMul(a, b) {
    const rows = a.length, inner = b.length, cols = b[0].length;
    const m = matZeros(rows, cols);
    for (let i = 0; i < rows; i++) {
      const ai = a[i];
      for (let k = 0; k < inner; k++) {
        const aik = ai[k];
        if (aik === 0) continue;
        const bk = b[k];
        for (let j = 0; j < cols; j++) m[i][j] += aik * bk[j];
      }
    }
    return m;
  }

  function matVec(m, v) {
    const out = new Array(m.length).fill(0);
    for (let i = 0; i < m.length; i++) {
      let s = 0;
      const row = m[i];
      for (let k = 0; k < v.length; k++) s += row[k] * v[k];
      out[i] = s;
    }
    return out;
  }

  function matTrace(m) { let t = 0; for (let i = 0; i < m.length; i++) t += m[i][i]; return t; }

  function matEquals(a, b, tol) {
    const t = tol === undefined ? 1e-9 : tol;
    if (a.length !== b.length || a[0].length !== b[0].length) return false;
    for (let i = 0; i < a.length; i++) for (let j = 0; j < a[0].length; j++) {
      if (Math.abs(a[i][j] - b[i][j]) > t) return false;
    }
    return true;
  }

  function matClone(a) { return a.map(r => r.slice()); }

  // Determinant through LU factorization with partial pivoting.
  function matDet(a) {
    const n = a.length;
    const m = matClone(a);
    let det = 1;
    for (let k = 0; k < n; k++) {
      let piv = k;
      for (let i = k + 1; i < n; i++) if (Math.abs(m[i][k]) > Math.abs(m[piv][k])) piv = i;
      if (Math.abs(m[piv][k]) < 1e-300) return 0;
      if (piv !== k) { const t = m[piv]; m[piv] = m[k]; m[k] = t; det = -det; }
      det *= m[k][k];
      for (let i = k + 1; i < n; i++) {
        const f = m[i][k] / m[k][k];
        m[i][k] = 0;
        for (let j = k + 1; j < n; j++) m[i][j] -= f * m[k][j];
      }
    }
    return det;
  }

  // Solve A x = b, or return null for a singular system.
  function solveLinear(A, b) {
    const n = A.length;
    const m = matClone(A);
    const x = b.slice();
    for (let k = 0; k < n; k++) {
      let piv = k;
      for (let i = k + 1; i < n; i++) if (Math.abs(m[i][k]) > Math.abs(m[piv][k])) piv = i;
      if (Math.abs(m[piv][k]) < 1e-14) return null;
      if (piv !== k) { const t = m[piv]; m[piv] = m[k]; m[k] = t; const tv = x[piv]; x[piv] = x[k]; x[k] = tv; }
      const mk = m[k];
      for (let i = k + 1; i < n; i++) {
        const f = m[i][k] / mk[k];
        if (f === 0) continue;
        for (let j = k; j < n; j++) m[i][j] -= f * mk[j];
        x[i] -= f * x[k];
      }
    }
    for (let i = n - 1; i >= 0; i--) {
      let s = x[i];
      for (let j = i + 1; j < n; j++) s -= m[i][j] * x[j];
      x[i] = s / m[i][i];
    }
    return x;
  }

  // Matrix inverse by Gauss-Jordan with partial pivoting.
  function matInverse(a) {
    const n = a.length;
    const m = matClone(a);
    const inv = matIdentity(n);
    for (let k = 0; k < n; k++) {
      let piv = k;
      for (let i = k + 1; i < n; i++) if (Math.abs(m[i][k]) > Math.abs(m[piv][k])) piv = i;
      if (Math.abs(m[piv][k]) < 1e-14) return null;
      if (piv !== k) {
        const t = m[piv]; m[piv] = m[k]; m[k] = t;
        const ti = inv[piv]; inv[piv] = inv[k]; inv[k] = ti;
      }
      const d = m[k][k];
      for (let j = 0; j < n; j++) { m[k][j] /= d; inv[k][j] /= d; }
      for (let i = 0; i < n; i++) {
        if (i === k) continue;
        const f = m[i][k];
        if (f === 0) continue;
        for (let j = 0; j < n; j++) { m[i][j] -= f * m[k][j]; inv[i][j] -= f * inv[k][j]; }
      }
    }
    return inv;
  }

  // ============================================================
  // Module 3: eigen decomposition of a symmetric matrix
  // Cyclic Jacobi rotations. Returns ascending eigenvalues and the
  // matching eigenvectors as columns.
  // ============================================================
  function eigSymmetric(A) {
    const n = A.length;
    const a = matClone(A);
    const v = matIdentity(n);
    for (let sweep = 0; sweep < 100; sweep++) {
      let off = 0;
      for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
      if (off < 1e-28) break;
      for (let p = 0; p < n - 1; p++) {
        for (let q = p + 1; q < n; q++) {
          const apq = a[p][q];
          if (Math.abs(apq) < 1e-300) continue;
          const app = a[p][p], aqq = a[q][q];
          const tau = (aqq - app) / (2 * apq);
          const t = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
          const c = 1 / Math.sqrt(1 + t * t);
          const s = t * c;
          for (let k = 0; k < n; k++) {
            const akp = a[k][p], akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
          }
          for (let k = 0; k < n; k++) {
            const apk = a[p][k], aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
          }
          for (let k = 0; k < n; k++) {
            const vkp = v[k][p], vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
          }
        }
      }
    }
    const order = [];
    for (let i = 0; i < n; i++) order.push(i);
    order.sort((x, y) => a[x][x] - a[y][y]);
    const values = [];
    const vectors = matZeros(n, n);
    for (let i = 0; i < n; i++) {
      values.push(a[order[i]][order[i]]);
      for (let k = 0; k < n; k++) vectors[k][i] = v[k][order[i]];
    }
    return { values: values, vectors: vectors };
  }

  // Singular values of an m by n matrix, ascending. Only the complete set
  // of the smaller dimension is produced.
  function singularValues(M) {
    const m = M.length, n = M[0].length;
    const G = m <= n ? matMul(M, matTranspose(M)) : matMul(matTranspose(M), M);
    const ev = eigSymmetric(G).values.slice().map(v => Math.max(0, v));
    ev.sort((x, y) => x - y);
    return ev.map(v => Math.sqrt(v));
  }

  function matrixRank(M, tol) {
    const sv = singularValues(M);
    const t = tol === undefined ? 1e-9 : tol;
    const smax = sv.length ? sv[sv.length - 1] : 0;
    let r = 0;
    for (let i = 0; i < sv.length; i++) if (sv[i] > Math.max(t, smax * 1e-12)) r++;
    return r;
  }

  // ============================================================
  // Module 4: quaternions (w, x, y, z) and Euler angles
  // ============================================================
  function quatIdentity() { return [1, 0, 0, 0]; }

  function quatNormalize(q) {
    const n = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-15) return quatIdentity();
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
  }

  function quatMul(a, b) {
    return [
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    ];
  }

  function quatConj(q) { return [q[0], -q[1], -q[2], -q[3]]; }

  function quatDot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]; }

  function quatFromAxisAngle(axis, angle) {
    const a = normalize(axis);
    const h = angle / 2;
    const s = Math.sin(h);
    return [Math.cos(h), a[0] * s, a[1] * s, a[2] * s];
  }

  function quatToAxisAngle(q) {
    const n = quatNormalize(q);
    const w = clamp(n[0], -1, 1);
    const angle = 2 * Math.acos(Math.abs(w));
    const s = Math.sqrt(Math.max(0, 1 - w * w));
    if (s < 1e-12) return { axis: [1, 0, 0], angle: angle };
    const sign = n[0] < 0 ? -1 : 1;
    return { axis: normalize([n[1] * sign / s, n[2] * sign / s, n[3] * sign / s]), angle: angle };
  }

  function quatRotate(q, v) {
    const n = quatNormalize(q);
    const t = cross([n[1], n[2], n[3]], v);
    const tt = cross([n[1], n[2], n[3]], t);
    return [
      v[0] + 2 * (n[0] * t[0] + tt[0]),
      v[1] + 2 * (n[0] * t[1] + tt[1]),
      v[2] + 2 * (n[0] * t[2] + tt[2])
    ];
  }

  function quatFromMat(R) {
    const m00 = R[0][0], m01 = R[0][1], m02 = R[0][2];
    const m10 = R[1][0], m11 = R[1][1], m12 = R[1][2];
    const m20 = R[2][0], m21 = R[2][1], m22 = R[2][2];
    const tr = m00 + m11 + m22;
    if (tr > 0) {
      const s = Math.sqrt(tr + 1) * 2;
      return quatNormalize([0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]);
    }
    if (m00 > m11 && m00 > m22) {
      const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
      return quatNormalize([(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]);
    }
    if (m11 > m22) {
      const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
      return quatNormalize([(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]);
    }
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    return quatNormalize([(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]);
  }

  function quatToMat(q) {
    const n = quatNormalize(q);
    const w = n[0], x = n[1], y = n[2], z = n[3];
    return [
      [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
    ];
  }

  function quatSlerp(a, b, t) {
    const qa = quatNormalize(a);
    let qb = quatNormalize(b);
    let d = quatDot(qa, qb);
    if (d < 0) { qb = quatScale(qb, -1); d = -d; }
    if (d > 0.999999) return quatNormalize(vecLerp4(qa, qb, t));
    const theta = Math.acos(clamp(d, -1, 1));
    const s = Math.sin(theta);
    const wa = Math.sin((1 - t) * theta) / s;
    const wb = Math.sin(t * theta) / s;
    return quatNormalize([
      wa * qa[0] + wb * qb[0],
      wa * qa[1] + wb * qb[1],
      wa * qa[2] + wb * qb[2],
      wa * qa[3] + wb * qb[3]
    ]);
  }

  function quatScale(q, s) { return [q[0] * s, q[1] * s, q[2] * s, q[3] * s]; }
  function vecLerp4(a, b, t) { return [lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t), lerp(a[3], b[3], t)]; }

  // Roll, pitch, yaw convention: R = Rz(yaw) Ry(pitch) Rx(roll).
  function eulerToMat(roll, pitch, yaw) {
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr]
    ];
  }

  function eulerToQuat(roll, pitch, yaw) { return quatFromMat(eulerToMat(roll, pitch, yaw)); }

  function matToEuler(R) {
    const pitch = Math.asin(clamp(-R[2][0], -1, 1));
    if (Math.abs(Math.cos(pitch)) < 1e-9) {
      const roll = 0;
      const yaw = Math.atan2(-R[0][1], R[1][1]);
      return { roll: roll, pitch: pitch, yaw: yaw, gimbal: true };
    }
    const roll = Math.atan2(R[2][1], R[2][2]);
    const yaw = Math.atan2(R[1][0], R[0][0]);
    return { roll: roll, pitch: pitch, yaw: yaw, gimbal: false };
  }

  function quatToEuler(q) { return matToEuler(quatToMat(q)); }

  function eulerFromDeg(rpy) {
    return {
      roll: toRad(rpy[0] || 0),
      pitch: toRad(rpy[1] || 0),
      yaw: toRad(rpy[2] || 0)
    };
  }

  // ============================================================
  // Module 5: rigid transforms in SE(3)
  // A pose is { R: 3x3, t: [x, y, z] } with t in world coordinates.
  // ============================================================
  function pose(R, t) {
    return {
      R: R ? matClone(R) : matIdentity(3),
      t: t ? t.slice() : [0, 0, 0]
    };
  }

  function poseIdentity() { return pose(matIdentity(3), [0, 0, 0]); }

  function poseMul(a, b) {
    return {
      R: matMul(a.R, b.R),
      t: vecAdd(matVec(a.R, b.t), a.t)
    };
  }

  function poseInv(a) {
    const Rt = matTranspose(a.R);
    return { R: Rt, t: matVec(Rt, [-a.t[0], -a.t[1], -a.t[2]]) };
  }

  function poseApply(a, p) { return vecAdd(matVec(a.R, p), a.t); }

  function poseFromQuat(q, t) { return pose(quatToMat(q), t); }

  function poseToQuat(a) { return quatFromMat(a.R); }

  function poseToHomogeneous(a) {
    return [
      [a.R[0][0], a.R[0][1], a.R[0][2], a.t[0]],
      [a.R[1][0], a.R[1][1], a.R[1][2], a.t[1]],
      [a.R[2][0], a.R[2][1], a.R[2][2], a.t[2]],
      [0, 0, 0, 1]
    ];
  }

  function poseFromHomogeneous(H) {
    return {
      R: [[H[0][0], H[0][1], H[0][2]], [H[1][0], H[1][1], H[1][2]], [H[2][0], H[2][1], H[2][2]]],
      t: [H[0][3], H[1][3], H[2][3]]
    };
  }

  function poseInterp(a, b, s) {
    const q = quatSlerp(poseToQuat(a), poseToQuat(b), s);
    return poseFromQuat(q, [lerp(a.t[0], b.t[0], s), lerp(a.t[1], b.t[1], s), lerp(a.t[2], b.t[2], s)]);
  }

  // Geodesic distance: translation length plus the rotation angle.
  function poseDistance(a, b) {
    const rel = poseMul(poseInv(a), b);
    const ang = quatToAxisAngle(quatFromMat(rel.R)).angle;
    return norm(rel.t) + ang;
  }

  // ============================================================
  // Module 6: Denavit-Hartenberg kinematics
  // Standard (Craig) convention:
  //   A_i = Rz(theta_i) Tz(d_i) Tx(a_{i-1}) Rx(alpha_{i-1})
  // ============================================================
  function dhMatrix(a, alpha, d, theta) {
    const ct = Math.cos(theta), st = Math.sin(theta);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);
    return pose([
      [ct, -st * ca, st * sa],
      [st, ct * ca, -ct * sa],
      [0, sa, ca]
    ], [a * ct, a * st, d]);
  }

  // Modified DH convention:
  //   A_i = Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(theta_i) Tz(d_i)
  function modifiedDHMatrix(a, alpha, d, theta) {
    const ct = Math.cos(theta), st = Math.sin(theta);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);
    return pose([
      [ct, -st, 0],
      [st * ca, ct * ca, -sa],
      [st * sa, ct * sa, ca]
    ], [a, -sa * d, ca * d]);
  }

  function dhParams(robot, q) {
    if (!q || q.length !== robot.dof) throw new RangeError('expected ' + robot.dof + ' joint values for ' + (robot.name || 'robot'));
    return robot.links.map((link, i) => ({
      a: link.a,
      alpha: link.alpha,
      d: link.type === 'prismatic' ? link.d + q[i] : link.d,
      theta: link.type === 'prismatic' ? link.theta : link.theta + q[i],
      index: i
    }));
  }

  // Build a robot from a declaration. Each link carries its standard DH
  // row plus metadata used by the Jacobian, the URDF exporter and the
  // viewport renderer.
  function createRobot(spec) {
    const o = spec || {};
    if (!o.links || !o.links.length) throw new RangeError('a robot needs at least one link');
    const links = o.links.map((l, i) => {
      const type = l.type || 'revolute';
      if (type !== 'revolute' && type !== 'prismatic') throw new RangeError('link ' + i + ' has unknown type "' + type + '"');
      const lo = l.lo === undefined ? -Math.PI : l.lo;
      const hi = l.hi === undefined ? Math.PI : l.hi;
      for (const key of ['a', 'alpha', 'd', 'theta', 'lo', 'hi']) {
        const v = key === 'lo' ? lo : key === 'hi' ? hi : (l[key] === undefined ? 0 : l[key]);
        if (!Number.isFinite(v)) throw new RangeError('link ' + i + ' has nonfinite ' + key);
      }
      if (!(hi > lo)) throw new RangeError('link ' + i + ' limits are empty or inverted');
      return {
        name: l.name || ('link' + (i + 1)),
        type: type,
        alpha: l.alpha || 0,
        a: l.a || 0,
        d: l.d || 0,
        theta: l.theta || 0,
        lo: lo,
        hi: hi,
        velocity: l.velocity === undefined ? 3.0 : l.velocity,
        effort: l.effort === undefined ? 100 : l.effort,
        mass: l.mass === undefined ? 2.0 : l.mass,
        radius: l.radius === undefined ? 0.05 : l.radius,
        length: l.length === undefined ? 0.15 : l.length,
        color: l.color || ['#3987e5', '#d95926', '#199e70', '#eda100', '#8b5cf6', '#ef476f', '#06b6d4'][i % 7]
      };
    });
    for (const [i, l] of links.entries()) {
      for (const key of ['velocity', 'effort', 'mass', 'radius', 'length']) {
        if (!Number.isFinite(l[key])) throw new RangeError('link ' + i + ' has nonfinite ' + key);
      }
      if (l.radius < 0 || l.length < 0 || l.mass < 0) {
        throw new RangeError('link ' + i + ' has a negative physical dimension');
      }
    }
    const checkPose = (p, label) => {
      if (!p) return poseIdentity();
      for (const v of (p.t || [])) if (!Number.isFinite(v)) throw new RangeError(label + ' translation is nonfinite');
      for (const row of (p.R || [])) for (const v of (row || [])) if (!Number.isFinite(v)) throw new RangeError(label + ' rotation is nonfinite');
      return pose(p.R, p.t);
    };
    const robot = {
      name: o.name || 'robot',
      dof: links.length,
      links: links,
      base: checkPose(o.base, 'base'),
      tool: checkPose(o.tool, 'tool'),
      description: o.description || '',
      units: o.units || 'm'
    };
    return robot;
  }

  // Full forward kinematics. frames[0] is the base, frames[i+1] sits at
  // the end of link i, and end includes the tool offset.
  function forwardKinematics(robot, q) {
    const n = robot.dof;
    if (!q || q.length !== n) throw new RangeError('expected ' + n + ' joint values for ' + robot.name);
    for (const v of q) if (!Number.isFinite(v)) throw new RangeError('forwardKinematics joint values must be finite');
    const frames = [pose(robot.base.R.map(row => row.slice()), robot.base.t.slice())];
    let cur = frames[0];
    for (let i = 0; i < n; i++) {
      const link = robot.links[i];
      const p = dhParams(robot, q)[i];
      cur = poseMul(cur, dhMatrix(p.a, p.alpha, p.d, p.theta));
      frames.push(cur);
    }
    return { frames: frames, end: poseMul(frames[n], robot.tool), joints: frames.slice(0, n) };
  }

  // Per-frame data for rendering and for tool-center diagnostics.
  function linkFrames(robot, q) {
    const fk = forwardKinematics(robot, q);
    const out = [];
    for (let i = 0; i < robot.dof; i++) {
      const f = fk.frames[i];
      const nxt = fk.frames[i + 1];
      out.push({
        index: i,
        name: robot.links[i].name,
        type: robot.links[i].type,
        origin: f,
        translation: f.t,
        quaternion: poseToQuat(f),
        child: nxt,
        axis: [
          nxt.t[0] - f.t[0],
          nxt.t[1] - f.t[1],
          nxt.t[2] - f.t[2]
        ]
      });
    }
    return out;
  }

  function endPose(robot, q) { return forwardKinematics(robot, q).end; }

  function toolPoint(robot, q) { return endPose(robot, q).t; }

  // Distance from the tool origin to the target, the standard convergence
  // measure for an iterative solver.
  function poseError(robot, q, target) {
    const e = endPose(robot, q);
    const rel = poseMul(poseInv(target), e);
    return norm(rel.t);
  }

  // Six-element error in the base frame: the translation the tool must make
  // plus the axis-angle of the rotation still required, both in world
  // coordinates so the error lives in the same frame as the Jacobian.
  function twistError(robot, q, target) {
    const e = endPose(robot, q);
    const Rerr = matMul(target.R, matTranspose(e.R));
    const aa = quatToAxisAngle(quatFromMat(Rerr));
    return [
      target.t[0] - e.t[0], target.t[1] - e.t[1], target.t[2] - e.t[2],
      aa.axis[0] * aa.angle, aa.axis[1] * aa.angle, aa.axis[2] * aa.angle
    ];
  }

  // ============================================================
  // Module 7: geometric Jacobian and derivatives
  // Columns are z_i x (p_e - o_i) and z_i for revolute joints, plus
  // z_i and 0 for prismatic joints.
  // ============================================================
  function jointAxis(robot, q, i) {
    if (!Number.isInteger(i) || i < 0 || i > robot.dof - 1) {
      throw new RangeError('jointAxis expects a joint index in [0, ' + (robot.dof - 1) + '], got ' + i);
    }
    const fk = forwardKinematics(robot, q);
    const f = fk.frames[i];
    const z = matVec(f.R, [0, 0, 1]);
    const o = f.t;
    return { z: z, o: o };
  }

  function jacobian(robot, q) {
    const n = robot.dof;
    const fk = forwardKinematics(robot, q);
    const zCols = [];
    const oCols = [];
    for (let i = 0; i < n; i++) {
      zCols.push(matVec(fk.frames[i].R, [0, 0, 1]));
      oCols.push(fk.frames[i].t);
    }
    const pe = fk.end.t;
    const J = matZeros(6, n);
    for (let i = 0; i < n; i++) {
      const link = robot.links[i];
      const z = zCols[i];
      if (link.type === 'prismatic') {
        J[0][i] = z[0];
        J[1][i] = z[1];
        J[2][i] = z[2];
        J[3][i] = 0;
        J[4][i] = 0;
        J[5][i] = 0;
      } else {
        const c = cross(z, vecSub(pe, oCols[i]));
        J[0][i] = c[0];
        J[1][i] = c[1];
        J[2][i] = c[2];
        J[3][i] = z[0];
        J[4][i] = z[1];
        J[5][i] = z[2];
      }
    }
    return J;
  }

  // Analytic translational velocity check helper: J * qdot equals the
  // linear velocity of the tool origin from a finite difference.
  function endpointVelocityNumerical(robot, q, qdot, dt) {
    const h = dt === undefined ? 1e-6 : dt;
    const qa = q.map((v, i) => v + qdot[i] * h);
    const pa = endPose(robot, qa).t;
    const pb = endPose(robot, q).t;
    return vecScale(vecSub(pa, pb), 1 / h);
  }

  // Time derivative of the Jacobian, used for the dynamic term of a
  // resolved-acceleration controller.
  function jacobianDot(robot, q, qdot) {
    const n = robot.dof;
    const h = 1e-6;
    const qa = q.map((v, i) => v + qdot[i] * h);
    const Ja = jacobian(robot, qa);
    const Jb = jacobian(robot, q);
    const Jd = matZeros(6, n);
    for (let i = 0; i < 6; i++) for (let j = 0; j < n; j++) Jd[i][j] = (Ja[i][j] - Jb[i][j]) / h;
    return Jd;
  }

  // ============================================================
  // Module 8: manipulability and singularities
  // w(q) = sqrt(det(J J^T)) is the Yoshikawa kinematic manipulability
  // measure. The ellipsoid axes are the singular values of J.
  // ============================================================
  function manipulability(robot, q) {
    // Product of the singular values: equals sqrt(det(J J^T)) for a square
    // Jacobian and stays meaningful for lower-DOF arms where det(J J^T) is
    // identically zero.
    const sv = singularValues(jacobian(robot, q));
    return sv.reduce((a, v) => a * v, 1);
  }

  function manipulabilityProfile(robot, samples) {
    const n = robot.dof;
    const out = [];
    const count = samples || 48;
    for (let i = 0; i < count; i++) {
      const q = new Array(n);
      for (let j = 0; j < n; j++) {
        const l = robot.links[j];
        q[j] = lerp(l.lo, l.hi, count === 1 ? 0 : i / (count - 1));
      }
      out.push({ q: q, w: manipulability(robot, q) });
    }
    return out;
  }

  function conditionNumber(robot, q) {
    const sv = singularValues(jacobian(robot, q));
    if (!sv.length) return null;
    const smax = sv[sv.length - 1];
    const smin = sv[0];
    if (smin <= 1e-12) return Infinity;
    return smax / smin;
  }

  function ellipsoidAxes(robot, q) {
    const J = jacobian(robot, q);
    const sv = singularValues(J);
    // The translational part of the 6-dof twist space: the three largest
    // singular values and their left singular vectors.
    const JJt = matMul(J, matTranspose(J));
    const e = eigSymmetric(JJt);
    const top = sv.length >= 3 ? 3 : sv.length;
    const axes = [];
    const cols = [];
    const smax = sv[sv.length - 1] || 1;
    for (let k = 0; k < top; k++) {
      // eigSymmetric returns 6 eigenpairs sorted ascending; the left singular
      // direction for the k-th largest sigma is the matching eigenvector.
      const c = e.values.length - 1 - k;
      const sigma = sv[sv.length - 1 - k];
      axes.push(sigma);
      const dir = [];
      for (let r = 0; r < 3; r++) dir.push(e.vectors[r][c]);
      cols.push({ axis: normalize(dir), length: smax > 1e-15 ? sigma / smax : 0, sigma: sigma });
    }
    const smin = sv.length ? sv[0] : 0;
    return { axes: axes, basis: cols, cond: smin > 1e-15 ? smax / smin : Infinity };
  }

  function detectSingularity(robot, q, options) {
    const o = options || {};
    // Tolerance is a numerical rank threshold, not a workspace parameter:
    // values above 1 (or nonfinite/nonpositive) are degenerate and fall back
    // to the default so they cannot zero the rank.
    const tol = Number.isFinite(o.tolerance) && o.tolerance > 0 && o.tolerance <= 1 ? o.tolerance : 1e-3;
    const w = manipulability(robot, q);
    const sv = singularValues(jacobian(robot, q));
    const smin = sv.length ? sv[0] : 0;
    const rank = matrixRank(jacobian(robot, q), tol);
    const full = Math.min(6, robot.dof);
    const singular = smin < tol || rank < full;
    let kind = 'none';
    if (singular) {
      // A wrist singularity collapses two of the last three joint axes
      // (classic q5 = 0 alignment). Otherwise the arm is stretched to its
      // workspace boundary. Only meaningful for 6-axis wrists.
      const n = robot.dof;
      let wrist = false;
      if (n >= 6) {
        const fk = forwardKinematics(robot, q);
        for (let i = n - 3; i < n - 1 && !wrist; i++) {
          const za = matVec(fk.frames[i].R, [0, 0, 1]);
          const zb = matVec(fk.frames[i + 1].R, [0, 0, 1]);
          if (Math.abs(dot(za, zb)) > 0.999) wrist = true;
        }
        if (!wrist) {
          const z4 = matVec(fk.frames[n - 3].R, [0, 0, 1]);
          const z6 = matVec(fk.frames[n - 1].R, [0, 0, 1]);
          if (Math.abs(dot(z4, z6)) > 0.999) wrist = true;
        }
      }
      kind = wrist ? 'wrist' : 'boundary';
    }
    return {
      singular: singular,
      type: kind,
      w: w,
      sigmaMin: smin,
      sigmaMax: sv.length ? sv[sv.length - 1] : 0,
      rank: rank,
      fullRank: full,
      conditionNumber: smin > 1e-15 ? sv[sv.length - 1] / smin : Infinity,
      message: singular
        ? (kind === 'wrist' ? 'Wrist singularity: the last three joint axes align, so roll and pitch collapse.' : 'Boundary singularity: the arm is at the edge of its workspace.')
        : 'Full rank, ' + w.toFixed(4) + ' manipulability'
    };
  }

  // ============================================================
  // Module 9: damped least squares inverse kinematics
  // Solves (J^T J + lambda^2 I) dq = J^T dx with a Levenberg-Marquardt
  // style damping ramp and optional joint-limit and obstacle potentials.
  // ============================================================
  function zeros(n) { const a = new Array(n); for (let i = 0; i < n; i++) a[i] = 0; return a; }

  function jointWithinLimits(robot, q, tol) {
    const t = tol === undefined ? 1e-9 : tol;
    for (let i = 0; i < robot.dof; i++) {
      const l = robot.links[i];
      if (!Number.isFinite(q[i])) return false;
      if (q[i] < l.lo - t || q[i] > l.hi + t) return false;
    }
    return true;
  }

  function clampToLimits(robot, q) {
    const out = q.slice();
    for (let i = 0; i < robot.dof; i++) {
      const l = robot.links[i];
      out[i] = clamp(out[i], l.lo, l.hi);
    }
    return out;
  }

  // Repulsive distance potential, nulling inside the influence radius.
  function obstaclePotential(robot, q, obstacles) {
    if (!obstacles || !obstacles.length) return 0;
    const frames = forwardKinematics(robot, q).frames;
    const n = robot.dof;
    let v = 0;
    for (const ob of obstacles) {
      const p = [ob.x || 0, ob.y || 0, ob.z || 0];
      const r = ob.radius === undefined ? 0.1 : ob.radius;
      const margin = ob.margin === undefined ? 0.05 : ob.margin;
      const rawFrom = Number(ob.from === undefined ? 0 : ob.from);
      const rawTo = Number(ob.to === undefined ? n : ob.to);
      const sampleFrom = clamp(Math.round(Number.isFinite(rawFrom) ? rawFrom : 0), 0, n);
      const sampleTo = clamp(Math.round(Number.isFinite(rawTo) ? rawTo : n), 0, n);
      for (let i = Math.min(sampleFrom, sampleTo); i <= Math.max(sampleFrom, sampleTo); i++) {
        const o = frames[i].t;
        const d = norm(vecSub(o, p));
        if (d < r + margin) {
          const pen = (r + margin - d);
          v += pen * pen;
        }
      }
    }
    return v;
  }

  function ikStepDamping(iter, maxIter) {
    const frac = maxIter > 1 ? iter / (maxIter - 1) : 1;
    return 0.15 + 7.5 * frac * frac;
  }

  // One damped update plus projection onto the joint limits, used as the
  // building block for the hybrid search below. Candidates are ranked by a
  // score that can include the obstacle potential, while err stays the pure
  // twist error used for the convergence test.
  function ikCandidate(robot, q, target, opts, extra, scoreFn) {
    const J = jacobian(robot, q);
    const e = opts.error ? opts.error() : twistError(robot, q, target);
    const wRot = opts.weightRot === undefined ? 1 : opts.weightRot;
    if (wRot !== 1) for (let k = 3; k < 6; k++) e[k] *= wRot;
    let best = null;
    const lam = opts.damping;
    const scales = [lam, lam * 0.25, lam * 4, lam * 0.05, lam * 10];
    for (let s = 0; s < scales.length; s++) {
      const lamS = scales[s];
      const JW = J;
      const A = matAdd(matMul(matTranspose(JW), JW), matScale(matIdentity(robot.dof), lamS * lamS));
      const b = matVec(matTranspose(JW), e);
      const dq = solveLinear(A, b);
      if (!dq) continue;
      // Cap the per-iteration joint excursion so a far target does not
      // create a single huge leap that leaves the optimum behind.
      const cap = opts.maxStep;
      let mag = 0;
      for (let i = 0; i < robot.dof; i++) mag = Math.max(mag, Math.abs(dq[i]));
      if (mag > cap) for (let i = 0; i < robot.dof; i++) dq[i] *= cap / mag;
      const cand = extra ? extra(q, dq) : q.map((v, i) => v + dq[i]);
      if (!cand) continue;
      const err = norm(twistError(robot, cand, target));
      const score = scoreFn ? scoreFn(cand) : err;
      if (!best || score < best.score) best = { q: cand, err: err, score: score, lambda: lamS };
    }
    return best;
  }

  // Reject any candidate that would enter a link-styled joint-limit band.
  function applyLimitGate(robot, q, dq, useLimits, limitMargin) {
    const cand = q.slice();
    for (let i = 0; i < robot.dof; i++) cand[i] = q[i] + dq[i];
    if (!useLimits) return cand;
    for (let i = 0; i < robot.dof; i++) {
      const l = robot.links[i];
      const span = l.hi - l.lo;
      const lo = l.lo + span * limitMargin;
      const hi = l.hi - span * limitMargin;
      cand[i] = clamp(cand[i], lo, hi);
    }
    return cand;
  }

  function solveIK(robot, q0, target, options) {
    const o = options || {};
    const r = getRobot(robot);
    const n = r.dof;
    const maxIter = o.maxIter === undefined ? 300 : o.maxIter;
    const posTol = o.posTol === undefined ? 1e-6 : o.posTol;
    const rotTol = o.rotTol === undefined ? 1e-5 : o.rotTol;
    const damping = o.damping === undefined ? 0.05 : o.damping;
    const obstacles = o.obstacles || null;
    const obsWeight = o.obstacleWeight === undefined ? 1 : o.obstacleWeight;
    const useLimits = o.useLimits !== false;
    const weightRot = o.weightRotation === undefined ? 1 : o.weightRotation;
    if (!Number.isFinite(weightRot) || weightRot < 0) throw new RangeError('weightRotation must be nonnegative');
    const stepScale = o.stepScale === undefined ? 1 : o.stepScale;
    const rawMargin = o.limitMargin === undefined ? 0.01 : o.limitMargin;
    const limitMargin = Number.isFinite(rawMargin) ? clamp(rawMargin, 0, 0.45) : 0.01;
    const maxStep = o.maxStep === undefined ? 0.5 : o.maxStep;
    if (target && target.t && target.R) {
      for (const v of target.t) if (!Number.isFinite(v)) throw new RangeError('IK target translation is nonfinite');
      for (const row of target.R) for (const v of row) if (!Number.isFinite(v)) throw new RangeError('IK target rotation is nonfinite');
    }
    // Convergence is tested on position (metres) and rotation (radians)
    // separately, so relaxing one side, for example a position only solve
    // with rotTol very large, cannot mask a real residual on the other.
    function splitErr(e) {
      return { pos: Math.hypot(e[0], e[1], e[2]), rot: Math.hypot(e[3], e[4], e[5]) };
    }
    function errMet(e) {
      const s = splitErr(e);
      return s.pos < posTol && s.rot < rotTol;
    }
    // Combined descent objective: the pose residual plus a repulsive penalty
    // for configurations that push a link inside an obstacle's margin.
    const scoreOf = obstacles
      ? qq => norm(twistError(r, qq, target)) + obsWeight * obstaclePotential(r, qq, obstacles)
      : null;

    let q = new Array(n);
    for (let i = 0; i < n; i++) {
      const v = q0 && Number.isFinite(q0[i]) ? q0[i] : 0;
      q[i] = useLimits ? clamp(v, r.links[i].lo, r.links[i].hi) : v;
    }
    const gate = (qq, dq) => applyLimitGate(r, qq, dq, useLimits, limitMargin);
    let eVec = twistError(r, q, target);
    let error = norm(eVec);
    let score = scoreOf ? scoreOf(q) : error;
    let converged = errMet(eVec);
    let iter = 0;
    const history = [error];
    const log = [];

    while (iter < maxIter && !converged) {
      // LM-style ramp: light damping early for fast approach, heavy damping
      // late so the tail of the descent cannot overshoot the optimum.
      const opts = { damping: damping * ikStepDamping(iter, maxIter), maxStep: maxStep, weightRot: weightRot };
      const c = ikCandidate(r, q, target, opts, gate, scoreOf);
      iter++;
      if (!c) {
        log.push({ iter: iter, note: 'damped system was singular' });
        break;
      }
      const scaled = c.q.map((v, i) => q[i] + (v - q[i]) * stepScale);
      const eAfter = twistError(r, scaled, target);
      const errAfter = norm(eAfter);
      const scoreAfter = scoreOf ? scoreOf(scaled) : errAfter;
      log.push({ iter: iter, error: errAfter, lambda: c.lambda });
      // Accept only if the combined objective improves, else shrink the step.
      // A nonfinite score counts as "worse" so NaN candidates can never be
      // accepted by accident.
      if (!(Number.isFinite(scoreAfter) && scoreAfter <= score)) {
        let accepted = false;
        for (const shrink of [0.5, 0.25, 0.1]) {
          const trial = c.q.map((v, i) => q[i] + (v - q[i]) * shrink);
          const tev = twistError(r, trial, target);
          const te = norm(tev);
          const ts = scoreOf ? scoreOf(trial) : te;
          if (Number.isFinite(ts) && ts <= score) { q = trial; error = te; eVec = tev; score = ts; accepted = true; break; }
        }
        if (!accepted) {
          log.push({ iter: iter, note: 'no improving step, stopping' });
          break;
        }
      } else {
        q = scaled;
        error = errAfter;
        eVec = eAfter;
        score = scoreAfter;
      }
      history.push(error);
      if (errMet(eVec)) { converged = true; }
    }

    // Final projection onto the hard limits when they are enforced.
    const clamped = useLimits ? clampToLimits(r, q) : q;
    if (norm(vecSub(clamped, q)) > 1e-12) {
      q = clamped;
      eVec = twistError(r, q, target);
      error = norm(eVec);
      converged = errMet(eVec);
    }
    const parts = splitErr(eVec);

    return {
      q: q,
      converged: converged,
      iterations: iter,
      error: error,
      positionError: parts.pos,
      rotationError: parts.rot,
      withinLimits: jointWithinLimits(r, q),
      obstaclePenalty: obstacles ? obstaclePotential(r, q, obstacles) : 0,
      damping: damping,
      history: history,
      trace: log
    };
  }

  // Multi-start IK: tries several seeds and keeps the best solution.
  // options.seeds may be an array of joint vectors, or a count that fills
  // the limits with deterministic low-discrepancy seeds.
  function ikSeeds(robot, count) {
    const r = robot;
    const out = [
      zeros(r.dof),
      r.links.map(l => (l.lo + l.hi) / 2),
      r.links.map(l => l.hi * 0.5),
      r.links.map(l => l.lo * 0.5)
    ];
    // Golden-ratio lattice inside the limits: deterministic, spread out,
    // and stays valid for any joint range.
    for (let k = 0; k < count; k++) {
      const s = [];
      for (let i = 0; i < r.dof; i++) {
        const u = ((k + 1) * 0.6180339887498949 + i * 0.3819660112501051) % 1;
        s.push(r.links[i].lo + u * (r.links[i].hi - r.links[i].lo));
      }
      out.push(s);
    }
    return out;
  }

  function solveIKMultiStart(robot, target, options) {
    const o = options || {};
    const r0 = getRobot(robot);
    let seedList;
    if (typeof o.seeds === 'number' && Number.isFinite(o.seeds)) {
      seedList = ikSeeds(r0, Math.max(0, Math.floor(o.seeds)));
    } else if (Array.isArray(o.seeds)) {
      seedList = o.seeds.filter(s => Array.isArray(s));
    }
    if (!seedList || !seedList.length) seedList = ikSeeds(r0, 16);
    let best = null;
    for (const s of seedList) {
      const r = solveIK(robot, s, target, o);
      if (r.converged) return r;
      if (!best || r.error < best.error) best = r;
    }
    return best;
  }

  function reachable(robot, target) {
    let max = 0;
    for (const l of robot.links) max += Math.abs(l.a) + Math.abs(l.d);
    const base = robot.base.t;
    const d = norm(vecSub(target.t, base));
    // Inner dead-zone estimate: never reject interior targets. Signed link
    // offsets (UR5e-style negative a) used to inflate this bound and produce
    // false negatives, so only magnitudes participate.
    let min = 0;
    for (let i = robot.dof - 1; i >= 0; i--) min = Math.max(min - Math.abs(robot.links[i].a), 0);
    const tool = norm(robot.tool.t);
    return d >= Math.max(0, min - tool - 0.05) && d <= max + tool + 0.05;
  }

  // ============================================================
  // Module 10: robot library
  // Standard DH rows: a, alpha, d, theta with the theta offset folded in
  // as the resting angle of each revolute joint.
  // ============================================================
  const PUMA_LIMITS = [
    [-2.7925, 2.7925],
    [0, 3.1416],
    [-3.7525, 0.68],
    [-3.3161, 3.3161],
    [-1.5708, 1.5708],
    [-3.3161, 3.3161]
  ];

  // PUMA 560. Elbow offset folded into joint 3, forearm on joint 4.
  const PUMA_560 = createRobot({
    name: 'PUMA-560',
    description: 'Six axis anthropomorphic arm, standard Denavit-Hartenberg table with the classic elbow offset on joint 3.',
    links: [
      { name: 'shoulder', a: 0, alpha: -Math.PI / 2, d: 0, theta: 0, lo: PUMA_LIMITS[0][0], hi: PUMA_LIMITS[0][1], velocity: 2.2, radius: 0.075, length: 0.1, color: '#3987e5' },
      { name: 'upper-arm', a: 0.4318, alpha: 0, d: 0, theta: 0, lo: PUMA_LIMITS[1][0], hi: PUMA_LIMITS[1][1], velocity: 2.0, radius: 0.06, length: 0.4318, color: '#d95926' },
      { name: 'elbow', a: 0.0203, alpha: -Math.PI / 2, d: 0.1491, theta: -Math.PI / 2, lo: PUMA_LIMITS[2][0], hi: PUMA_LIMITS[2][1], velocity: 2.6, radius: 0.055, length: 0.15, color: '#199e70' },
      { name: 'forearm', a: 0, alpha: Math.PI / 2, d: 0.4318, theta: 0, lo: PUMA_LIMITS[3][0], hi: PUMA_LIMITS[3][1], velocity: 3.0, radius: 0.05, length: 0.4318, color: '#eda100' },
      { name: 'wrist-bend', a: 0, alpha: -Math.PI / 2, d: 0, theta: 0, lo: PUMA_LIMITS[4][0], hi: PUMA_LIMITS[4][1], velocity: 3.4, radius: 0.042, length: 0.09, color: '#8b5cf6' },
      { name: 'wrist-roll', a: 0, alpha: 0, d: 0, theta: 0, lo: PUMA_LIMITS[5][0], hi: PUMA_LIMITS[5][1], velocity: 3.8, radius: 0.036, length: 0.08, color: '#ef476f' }
    ]
  });

  // UR5e class arm. Widely used standard DH table, shoulder to flange.
  const UR5E = createRobot({
    name: 'UR5e',
    description: 'Six axis collaborative arm, UR5e geometry with the standard published Denavit-Hartenberg parameters.',
    links: [
      { name: 'base', a: 0, alpha: Math.PI / 2, d: 0.089159, theta: 0, lo: -2 * Math.PI, hi: 2 * Math.PI, velocity: 3.14, radius: 0.06, length: 0.09, color: '#3987e5' },
      { name: 'shoulder', a: -0.425, alpha: 0, d: 0, theta: 0, lo: -2 * Math.PI, hi: 2 * Math.PI, velocity: 2.41, radius: 0.06, length: 0.425, color: '#d95926' },
      { name: 'upper-arm', a: -0.39225, alpha: 0, d: 0, theta: 0, lo: -2 * Math.PI, hi: 2 * Math.PI, velocity: 3.14, radius: 0.055, length: 0.39225, color: '#199e70' },
      { name: 'forearm', a: 0, alpha: Math.PI / 2, d: 0.10915, theta: 0, lo: -2 * Math.PI, hi: 2 * Math.PI, velocity: 3.14, radius: 0.05, length: 0.10915, color: '#eda100' },
      { name: 'wrist-1', a: 0, alpha: -Math.PI / 2, d: 0.09465, theta: 0, lo: -2 * Math.PI, hi: 2 * Math.PI, velocity: 3.93, radius: 0.042, length: 0.09465, color: '#8b5cf6' },
      { name: 'wrist-2', a: 0, alpha: 0, d: 0.0823, theta: 0, lo: -2 * Math.PI, hi: 2 * Math.PI, velocity: 3.93, radius: 0.036, length: 0.0823, color: '#ef476f' }
    ]
  });

  // SCARA, four joints: two parallel revolute axes, a vertical prismatic
  // stroke and a tool roll about the same axis as the carriage swing.
  const SCARA = createRobot({
    name: 'SCARA-4',
    description: 'Selective compliance arm, four joints: shoulder swing, elbow swing, vertical stroke and tool roll.',
    links: [
      { name: 'column', a: 0.2, alpha: 0, d: 0.25, theta: 0, lo: -2.3562, hi: 2.3562, velocity: 4.0, radius: 0.05, length: 0.25, color: '#3987e5' },
      { name: 'upper-arm', a: 0.2, alpha: 0, d: 0, theta: 0, lo: -2.6180, hi: 2.6180, velocity: 4.5, radius: 0.045, length: 0.2, color: '#d95926' },
      { type: 'prismatic', name: 'stroke', a: 0, alpha: 0, d: 0, theta: 0, lo: -0.15, hi: 0.15, velocity: 1.2, radius: 0.04, length: 0.2, color: '#199e70' },
      { name: 'tool-roll', a: 0, alpha: 0, d: 0, theta: 0, lo: -3.1416, hi: 3.1416, velocity: 5.0, radius: 0.035, length: 0.06, color: '#eda100' }
    ]
  });

  const ROBOTS = { 'PUMA-560': PUMA_560, 'UR5e': UR5E, 'SCARA-4': SCARA };

  function getRobot(nameOrRobot) {
    if (typeof nameOrRobot === 'string') {
      const r = ROBOTS[nameOrRobot];
      if (!r) throw new RangeError('unknown robot "' + nameOrRobot + '", try one of ' + Object.keys(ROBOTS).join(', '));
      return r;
    }
    return nameOrRobot;
  }

  // ============================================================
  // Module 11: joint limit diagnostics
  // ============================================================
  function jointViolations(robot, q) {
    const out = [];
    for (let i = 0; i < robot.dof; i++) {
      const l = robot.links[i];
      if (q[i] < l.lo) out.push({ index: i, name: l.name, value: q[i], limit: l.lo, side: 'below' });
      else if (q[i] > l.hi) out.push({ index: i, name: l.name, value: q[i], limit: l.hi, side: 'above' });
    }
    return out;
  }

  function limitUtilization(robot, q) {
    const out = [];
    for (let i = 0; i < robot.dof; i++) {
      const l = robot.links[i];
      const span = l.hi - l.lo || 1;
      out.push(clamp((q[i] - l.lo) / span, 0, 1));
    }
    return out;
  }

  // ============================================================
  // Module 12: quintic minimum jerk trajectory planner
  // Position and velocity are pinned at both ends by the boundary
  // conditions. With r = (t - t0)/T the blend is
  //   s(r) = 6r^5 - 15r^4 + 10r^3
  // which has zero first and second derivative at both ends.
  // ============================================================
  function quinticBlend(r) { return 6 * r * r * r * r * r - 15 * r * r * r * r + 10 * r * r * r; }

  function quinticVelocityBlend(r) { return 30 * r * r * r * r - 60 * r * r * r + 30 * r * r; }

  function quinticAccelBlend(r) { return 120 * r * r * r - 180 * r * r + 60 * r; }

  // One joint between two waypoints.
  function quinticSegment(t, T, q0, q1, v0, v1, a0, a1) {
    const dv0 = v0 * T, dv1 = v1 * T;
    const da0 = a0 * T * T, da1 = a1 * T * T;
    const c0 = q0;
    const c1 = dv0;
    const c2 = da0 / 2;
    const c3 = (20 * (q1 - q0) - (12 * dv0 + 8 * dv1) - (3 * da0 - da1)) / 2;
    const c4 = (30 * (q0 - q1) + (16 * dv0 + 14 * dv1) + (3 * da0 - 2 * da1)) / 2;
    const c5 = (12 * (q1 - q0) - (6 * dv0 + 6 * dv1) - (da0 - da1)) / 2;
    if (T <= 1e-12) {
      return { position: q1, velocity: v1, accel: a1 };
    }
    const r = clamp(t / T, 0, 1);
    return {
      position: c0 + c1 * r + c2 * r * r + c3 * r * r * r + c4 * r * r * r * r + c5 * r * r * r * r * r,
      velocity: (c1 + 2 * c2 * r + 3 * c3 * r * r + 4 * c4 * r * r * r + 5 * c5 * r * r * r * r) / T,
      accel: (2 * c2 + 6 * c3 * r + 12 * c4 * r * r + 20 * c5 * r * r * r) / (T * T)
    };
  }

  // Blend factor simplified form, used as a cross check and for tests.
  function quinticSimple(t, T) {
    const r = clamp(t / T, 0, 1);
    return quinticBlend(r);
  }

  // Cost of a segment, the integral of squared jerk over the duration.
  function segmentJerkCost(T, q0, q1, v0, v1, a0, a1) {
    if (!Number.isFinite(T) || T <= 0) throw new RangeError('segmentJerkCost needs a positive duration');
    const dv0 = v0 * T, dv1 = v1 * T;
    const da0 = a0 * T * T, da1 = a1 * T * T;
    const c3 = (20 * (q1 - q0) - (12 * dv0 + 8 * dv1) - (3 * da0 - da1)) / 2;
    const c4 = (30 * (q0 - q1) + (16 * dv0 + 14 * dv1) + (3 * da0 - 2 * da1)) / 2;
    const c5 = (12 * (q1 - q0) - (6 * dv0 + 6 * dv1) - (da0 - da1)) / 2;
    // jerk(r) = c3' + c4'' + c5''' scaled by 1/T^3 each derivative step
    function jerk(r) {
      return (6 * c3 + 24 * c4 * r + 60 * c5 * r * r) / (T * T * T);
    }
    const n = 64;
    let s = 0;
    for (let i = 0; i < n; i++) {
      const r = (i + 0.5) / n;
      const j = jerk(r);
      s += j * j;
    }
    return s / n * T;
  }

  // Independent duration per segment from velocity and acceleration caps.
  function segmentDuration(q0, q1, vLimit, aLimit) {
    const dq = Math.abs(q1 - q0);
    if (dq < 1e-12) return 0;
    const v = vLimit && vLimit > 1e-9 ? vLimit : 10;
    if (!aLimit || aLimit <= 1e-9) return dq / v;
    const tv = dq / v;
    const ta = Math.sqrt(dq / aLimit);
    return Math.max(tv, ta);
  }

  function planTrajectory(robot, waypoints, options) {
    const o = options || {};
    const n = robot.dof;
    // Waypoints may be raw joint arrays or { q, v, a } objects. Velocity and
    // acceleration at a waypoint default to a full stop.
    const pts = (waypoints || []).filter(Boolean).map((w, i) => {
      const p = Array.isArray(w) ? { q: w } : w;
      const q = p.q;
      if (!q || q.length !== n) throw new RangeError('waypoint ' + i + ' needs ' + n + ' joint values');
      for (let k = 0; k < n; k++) if (!Number.isFinite(q[k])) throw new RangeError('waypoint ' + i + ' has a nonfinite joint value');
      const v = p.v || zeros(n);
      const a = p.a || zeros(n);
      if (v.length !== n || a.length !== n) throw new RangeError('waypoint ' + i + ' v/a need ' + n + ' values');
      for (let k = 0; k < n; k++) {
        if (!Number.isFinite(v[k])) throw new RangeError('waypoint ' + i + ' has a nonfinite velocity');
        if (!Number.isFinite(a[k])) throw new RangeError('waypoint ' + i + ' has a nonfinite acceleration');
      }
      return { q: q.slice(), v: v.slice(), a: a.slice() };
    });
    if (pts.length < 2) throw new RangeError('a trajectory needs at least two waypoints');
    const segDur = o.durations || null;
    const vLimits = robot.links.map(l => (o.velocityScale === undefined ? 1 : o.velocityScale) * l.velocity);
    const aLimits = robot.links.map(l => (o.accelScale === undefined ? 1 : o.accelScale) * l.velocity);
    const jLim = robot.links.map(l => l.velocity * 4);

    const segments = [];
    let time = 0;
    for (let s = 0; s < pts.length - 1; s++) {
      const from = pts[s].q;
      const to = pts[s + 1].q;
      const T = segDur && segDur[s] !== undefined && segDur[s] > 0
        ? segDur[s]
        : segmentDurationFor(robot, from, to, vLimits, aLimits, s === 0);
      // Boundary values come straight from the shared waypoint, so position,
      // velocity and acceleration are continuous at every seam by
      // construction.
      segments.push({
        index: s,
        t0: time,
        t1: time + T,
        duration: T,
        from: from.slice(),
        to: to.slice(),
        v0: pts[s].v.slice(),
        v1: pts[s + 1].v.slice(),
        a0: pts[s].a.slice(),
        a1: pts[s + 1].a.slice()
      });
      time += T;
    }

    function sampleAt(t) {
      const tt = clamp(t, 0, time);
      const q = zeros(n), v = zeros(n), a = zeros(n);
      let seg = segments[0];
      for (const s of segments) { if (tt >= s.t0 && tt <= s.t1) { seg = s; break; } seg = s; }
      const local = clamp(tt - seg.t0, 0, seg.duration);
      for (let i = 0; i < n; i++) {
        const r = quinticSegment(local, seg.duration, seg.from[i], seg.to[i], seg.v0[i], seg.v1[i], seg.a0[i], seg.a1[i]);
        q[i] = r.position;
        v[i] = r.velocity;
        a[i] = r.accel;
      }
      return { t: tt, q: q, dq: v, ddq: a, segment: seg.index };
    }

    return {
      robot: robot,
      segments: segments,
      totalTime: time,
      sample: sampleAt,
      sampleRange: function (count) {
        if (count !== undefined && (!Number.isFinite(count) || count < 2 || count > 100000)) {
          throw new RangeError('sampleRange count must be an integer in [2, 100000]');
        }
        const out = [];
        const steps = Math.max(2, count || 64);
        for (let i = 0; i <= steps; i++) out.push(sampleAt(time * i / steps));
        return out;
      },
      velocityLimits: vLimits,
      accelLimits: aLimits,
      jerkLimits: jLim,
      validity: o.validate
        ? validateTrajectory(robot, sampleAt, time, Object.assign({ velocityLimits: vLimits, accelLimits: aLimits, jerkLimits: jLim }, o))
        : null
    };
  }

  function segmentDurationFor(robot, from, to, vLimits, aLimits, first) {
    void first;
    let T = 0;
    for (let i = 0; i < robot.dof; i++) {
      const dt = segmentDuration(from[i], to[i], vLimits[i] * 0.6, aLimits[i] * 1.5);
      T = Math.max(T, dt);
    }
    return Math.max(T, 0.02);
  }

  // Sample the planned path and measure what it actually does: joint-limit
  // compliance, peak jerk from a finite difference of acceleration, and
  // sphere collisions along the way when obstacles are supplied.
  function validateTrajectory(robot, sampleAt, totalTime, options) {
    const o = options || {};
    const n = robot.dof;
    const count = Math.max(8, Math.min(400, Math.round(totalTime / 0.01) || 40));
    const dt = count > 0 ? totalTime / count : 0;
    const violations = [];
    const collisions = [];
    const vLim = o.velocityLimits || null;
    const aLim = o.accelLimits || null;
    const jLim = o.jerkLimits || null;
    let inLimits = true;
    let inVelocityLimits = true;
    let inAccelLimits = true;
    let inJerkLimits = true;
    let collisionFree = true;
    let maxJerk = 0;
    let maxVelocity = 0;
    let maxAccel = 0;
    let allFinite = true;
    let prev = null;
    for (let i = 0; i <= count; i++) {
      const t = Math.min(totalTime, dt * i);
      const s = sampleAt(t);
      for (let j = 0; j < n; j++) {
        const l = robot.links[j];
        if (!Number.isFinite(s.q[j]) || !Number.isFinite(s.dq[j]) || !Number.isFinite(s.ddq[j])) {
          allFinite = false;
          inLimits = false;
          if (violations.length < 32) violations.push({ t: t, joint: j, kind: 'nonfinite', value: s.q[j], lo: l.lo, hi: l.hi });
          continue;
        }
        if (s.q[j] < l.lo - 1e-9 || s.q[j] > l.hi + 1e-9) {
          inLimits = false;
          if (violations.length < 32) violations.push({ t: t, joint: j, kind: 'position', value: s.q[j], lo: l.lo, hi: l.hi });
        }
        if (Math.abs(s.dq[j]) > maxVelocity) maxVelocity = Math.abs(s.dq[j]);
        if (Math.abs(s.ddq[j]) > maxAccel) maxAccel = Math.abs(s.ddq[j]);
        if (vLim && s.dq[j] > vLim[j] * 1.05 + 1e-9) {
          inVelocityLimits = false;
          if (violations.length < 32) violations.push({ t: t, joint: j, kind: 'velocity', value: s.dq[j], limit: vLim[j] });
        }
        if (aLim && s.ddq[j] > aLim[j] * 1.05 + 1e-9) {
          inAccelLimits = false;
          if (violations.length < 32) violations.push({ t: t, joint: j, kind: 'acceleration', value: s.ddq[j], limit: aLim[j] });
        }
      }
      if (prev && dt > 1e-12) {
        for (let j = 0; j < n; j++) {
          const jerk = Math.abs((s.ddq[j] - prev.ddq[j]) / (t - prev.t));
          if (jerk > maxJerk) maxJerk = jerk;
          if (jLim && jerk > jLim[j] * 1.05 + 1e-9) inJerkLimits = false;
        }
      }
      if (o.obstacles && o.obstacles.length) {
        const hit = checkCollisions(robot, s.q, o.obstacles, o);
        if (hit.collided) {
          collisionFree = false;
          for (const col of hit.collisions) if (collisions.length < 32) collisions.push({ t: t, obstacle: col.obstacle, link: col.link, distance: col.distance });
        }
      }
      prev = s;
    }
    return {
      inLimits: inLimits,
      inVelocityLimits: inVelocityLimits,
      inAccelLimits: inAccelLimits,
      inJerkLimits: inJerkLimits,
      collisionFree: collisionFree,
      allFinite: allFinite,
      maxJerk: maxJerk,
      maxVelocity: maxVelocity,
      maxAccel: maxAccel,
      violations: violations,
      collisions: collisions,
      sampleCount: count + 1,
      note: 'sampled quintic trajectory validation'
    };
  }

  // ============================================================
  // Module 13: sphere collision checking
  // Straight-line capsule samples between consecutive link origins,
  // tested against a list of spheres in world coordinates.
  // ============================================================
  function sphereDistance(p, c) {
    const pc = Array.isArray(p) ? p : [p.x || 0, p.y || 0, p.z || 0];
    return norm(vecSub(pc, [c.x || 0, c.y || 0, c.z || 0]));
  }

  function spheresCollide(a, b) {
    const d = sphereDistance(a, b);
    const ra = a.radius === undefined ? 0.05 : a.radius;
    const rb = b.radius === undefined ? 0.05 : b.radius;
    return d < ra + rb;
  }

  // Closest distance from a point to a segment, used to test link tubes.
  function segmentDistance(p, a, b) {
    const ab = vecSub(b, a);
    const len2 = dot(ab, ab);
    if (len2 < 1e-18) return norm(vecSub(p, a));
    const t = clamp(dot(vecSub(p, a), ab) / len2, 0, 1);
    const proj = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    return norm(vecSub(p, proj));
  }

  function linkSamplePoints(robot, q, samplesPerLink) {
    const fk = forwardKinematics(robot, q).frames;
    const pts = [];
    const n = samplesPerLink === undefined ? 8 : Math.max(1, Math.round(samplesPerLink));
    for (let i = 0; i < fk.length - 1; i++) {
      const a = fk[i].t, b = fk[i + 1].t;
      const start = i === 0 ? 0 : 1;
      for (let s = start; s <= n; s++) {
        const r = s / n;
        pts.push({
          link: i,
          point: [lerp(a[0], b[0], r), lerp(a[1], b[1], r), lerp(a[2], b[2], r)]
        });
      }
    }
    return pts;
  }

  function checkCollisions(robot, q, obstacles, options) {
    const o = options || {};
    const clearance = o.clearance === undefined ? 0.04 : o.clearance;
    const links = o.links || null;
    const list = obstacles || [];
    const frames = forwardKinematics(robot, q).frames;
    const collisions = [];
    let minDist = Infinity;
    for (let i = 0; i < list.length; i++) {
      const ob = list[i];
      if (ob.enabled === false) continue;
      const c = [ob.x || 0, ob.y || 0, ob.z || 0];
      const r = (ob.radius === undefined ? 0.1 : ob.radius) + clearance;
      for (let j = 0; j < frames.length - 1; j++) {
        // The optional links filter names LINK segment indices, so an
        // obstacle report can be restricted to a subset of the arm.
        if (links && !links.includes(j)) continue;
        const d = segmentDistance(c, frames[j].t, frames[j + 1].t);
        if (d < minDist) minDist = d;
        if (d <= r) {
          collisions.push({
            obstacle: i,
            link: j,
            distance: d,
            allowed: r,
            center: c
          });
          break;
        }
      }
    }
    return {
      collided: collisions.length > 0,
      collisions: collisions,
      minClearance: minDist === Infinity ? null : minDist,
      checkedObstacles: list.filter(x => x.enabled !== false).length
    };
  }

  function clearanceProfile(robot, q, obstacles) {
    const res = checkCollisions(robot, q, obstacles);
    return res.minClearance;
  }

  // ============================================================
  // Module 14: URDF exporter
  // Emits a ROS compatible robot description with revolute and
  // continuous joints, origin xyz rpy pins, axis lines and a minimal
  // visual plus collision box per link.
  // ============================================================
  function xmlEscape(s) {
    // XML 1.0 forbids most control characters even in escaped form, so they
    // are stripped before the entity passes.
    return String(s)
      .replace(/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]/g, '')
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&apos;');
  }

  function xmlAttribute(k, v) { return ' ' + k + '="' + xmlEscape(v) + '"'; }

  function num(v, p) {
    if (!Number.isFinite(v)) throw new RangeError('URDF numeric field is not finite');
    const f = Math.pow(10, p === undefined ? 6 : p);
    const n = Math.round(v * f) / f;
    return Object.is(n, -0) ? '0' : String(n);
  }

  // URDF joint origins that reproduce the standard DH chain exactly.
  // A_i(q) = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i) with the variable folded
  // into theta (revolute) or d (prismatic). Regrouping
  //   A_i = [Tx(a_{i-1}) Rx(alpha_{i-1})] [Rz(theta_i) Tz(d_i)] Rot_z(q_i)
  // because Rz(q_i) commutes with Tz(d_i). The joint origin in the parent
  // link frame is therefore TxRx_{i-1} * Rz(theta_i) * Tz(d_i), the axis is
  // always the local z, and the trailing TxRx_n lands on the tool joint.
  function dhJointOrigin(prevTxRx, link) {
    const rotZ = pose([
      [Math.cos(link.theta), -Math.sin(link.theta), 0],
      [Math.sin(link.theta), Math.cos(link.theta), 0],
      [0, 0, 1]
    ], [0, 0, 0]);
    const transZ = pose(matIdentity(3), [0, 0, link.d]);
    return poseMul(prevTxRx, poseMul(rotZ, transZ));
  }

  function txRxPose(link) {
    const ca = Math.cos(link.alpha), sa = Math.sin(link.alpha);
    return pose([
      [1, 0, 0],
      [0, ca, -sa],
      [0, sa, ca]
    ], [link.a, 0, 0]);
  }

  function exportURDF(robot, options) {
    const o = options || {};
    const r = robot;
    const pad = o.indent === undefined ? 2 : o.indent;
    const two = ' '.repeat(pad);
    const four = ' '.repeat(pad * 2);
    const six = ' '.repeat(pad * 3);
    const L = [];
    const name = o.name || r.name;
    const basePose = r.base || poseIdentity();
    const baseRpy = matToEuler(basePose.R);
    L.push('<?xml version="1.0" encoding="UTF-8"?>');
    L.push('<robot name="' + xmlEscape(name) + '">');
    L.push(two + '<!-- Generated by KineChain 6DOF, zero dependency Denavit-Hartenberg and URDF tooling. -->');
    L.push(two + '<link name="world"/>');
    L.push('');
    const baseT = basePose.t;
    L.push(two + '<link name="base_link">');
    L.push(four + '<visual>');
    L.push(six + '<geometry><cylinder radius="' + num(0.06) + '" length="' + num(0.09) + '"/></geometry>');
    L.push(six + '<origin' + xmlAttribute('xyz', '0 0 0') + xmlAttribute('rpy', '0 0 0') + '/>');
    L.push(four + '</visual>');
    L.push(four + '<collision>');
    L.push(six + '<geometry><cylinder radius="' + num(0.06) + '" length="' + num(0.09) + '"/></geometry>');
    L.push(four + '</collision>');
    L.push(two + '</link>');
    L.push('');
    L.push(two + '<joint' + xmlAttribute('name', 'world_to_base') + xmlAttribute('type', 'fixed') + '>');
    L.push(four + '<origin' + xmlAttribute('xyz', num(baseT[0]) + ' ' + num(baseT[1]) + ' ' + num(baseT[2])) + xmlAttribute('rpy', num(baseRpy.roll) + ' ' + num(baseRpy.pitch) + ' ' + num(baseRpy.yaw)) + '/>');
    L.push(four + '<parent link="world"/>');
    L.push(four + '<child link="base_link"/>');
    L.push(two + '</joint>');
    L.push('');

    let prevTxRx = poseIdentity();
    for (let i = 0; i < r.dof; i++) {
      const l = r.links[i];
      const linkName = o.linkNames ? (o.linkNames[i] || l.name) : l.name;
      const parent = i === 0 ? 'base_link' : (o.linkNames ? (o.linkNames[i - 1] || r.links[i - 1].name) : r.links[i - 1].name);
      const jointName = o.jointNames ? (o.jointNames[i] || 'joint_' + (i + 1)) : ('joint_' + (i + 1));
      const isPrismatic = l.type === 'prismatic';
      const isContinuous = !isPrismatic && (Math.abs(l.lo + 2 * Math.PI) < 1e-6 || Math.abs(l.hi - 2 * Math.PI) < 1e-6 || (l.lo <= -2 * Math.PI && l.hi >= 2 * Math.PI));
      const jointPose = dhJointOrigin(prevTxRx, l);
      const originRpy = matToEuler(jointPose.R);
      const origin = { xyz: jointPose.t, rpy: [originRpy.roll, originRpy.pitch, originRpy.yaw] };
      const axis = [0, 0, 1];
      const len = l.length === undefined ? 0.1 : l.length;
      const rad = l.radius === undefined ? 0.04 : l.radius;

      L.push(two + '<!-- ' + xmlEscape(l.name) + ': a=' + num(l.a) + ' alpha=' + num(l.alpha) + ' d=' + num(l.d) + ' theta=' + num(l.theta) + ' -->');
      L.push(two + '<link name="' + xmlEscape(linkName) + '">');
      L.push(four + '<visual>');
      L.push(six + '<origin xyz="0 0 ' + num(len / 2) + '" rpy="0 0 0"/>');
      L.push(six + '<geometry><cylinder radius="' + num(rad) + '" length="' + num(len) + '"/></geometry>');
      L.push(four + '</visual>');
      L.push(four + '<collision>');
      L.push(six + '<origin xyz="0 0 ' + num(len / 2) + '" rpy="0 0 0"/>');
      L.push(six + '<geometry><cylinder radius="' + num(rad) + '" length="' + num(len) + '"/></geometry>');
      L.push(four + '</collision>');
      L.push(four + '<inertial>');
      L.push(six + '<mass value="' + num(l.mass === undefined ? 2 : l.mass) + '"/>');
      L.push(six + '<inertia ixx="' + num(0.001) + '" ixy="0" ixz="0" iyy="' + num(0.001) + '" iyz="0" zzz="' + num(0.001) + '"/>');
      L.push(four + '</inertial>');
      L.push(two + '</link>');
      L.push('');
      L.push(two + '<joint' + xmlAttribute('name', jointName) + xmlAttribute('type', isPrismatic ? 'prismatic' : (isContinuous ? 'continuous' : 'revolute')) + '>');
      L.push(four + '<origin' + xmlAttribute('xyz', num(origin.xyz[0]) + ' ' + num(origin.xyz[1]) + ' ' + num(origin.xyz[2])) + xmlAttribute('rpy', num(origin.rpy[0]) + ' ' + num(origin.rpy[1]) + ' ' + num(origin.rpy[2])) + '/>');
      L.push(four + '<parent link="' + xmlEscape(parent) + '"/>');
      L.push(four + '<child link="' + xmlEscape(linkName) + '"/>');
      L.push(four + '<axis xyz="' + num(axis[0]) + ' ' + num(axis[1]) + ' ' + num(axis[2]) + '"/>');
      if (!isContinuous) {
        L.push(four + '<limit effort="' + num(l.effort === undefined ? 100 : l.effort) + '" lower="' + num(l.lo) + '" upper="' + num(l.hi) + '" velocity="' + num(l.velocity === undefined ? 3 : l.velocity) + '"/>');
      } else {
        L.push(four + '<limit effort="' + num(l.effort === undefined ? 100 : l.effort) + '" velocity="' + num(l.velocity === undefined ? 3 : l.velocity) + '"/>');
      }
      L.push(two + '</joint>');
      L.push('');
      prevTxRx = txRxPose(l);
    }

    // The last link's TxRx lands on the fixed tool joint, so the exported
    // chain reproduces forwardKinematics exactly.
    const toolPose = poseMul(prevTxRx, r.tool || poseIdentity());
    const toolT = toolPose.t;
    const toolRpy = matToEuler(toolPose.R);
    const eeName = o.endEffectorName || 'tool0';
    L.push(two + '<link name="' + xmlEscape(eeName) + '"/>');
    L.push(two + '<joint' + xmlAttribute('name', 'tool_joint') + xmlAttribute('type', 'fixed') + '>');
    L.push(four + '<origin' + xmlAttribute('xyz', num(toolT[0]) + ' ' + num(toolT[1]) + ' ' + num(toolT[2])) + xmlAttribute('rpy', num(toolRpy.roll) + ' ' + num(toolRpy.pitch) + ' ' + num(toolRpy.yaw)) + '/>');
    L.push(four + '<parent link="' + xmlEscape(o.linkNames ? (o.linkNames[r.dof - 1] || r.links[r.dof - 1].name) : r.links[r.dof - 1].name) + '"/>');
    L.push(four + '<child link="' + xmlEscape(eeName) + '"/>');
    L.push(two + '</joint>');
    L.push('');
    L.push('</robot>');
    return L.join('\n') + '\n';
  }

  // Lightweight sanity pass over an exported document: it must be a
  // single root element with matched link and joint tags. Self-closing
  // elements such as <link name="world"/> count as their own close.
  function validateURDF(xml) {
    // Strip comments so injected markup inside <!-- --> cannot inflate the
    // counts, then require at least one real link besides the robot shell.
    const raw = String(xml).replace(/<!--[\s\S]*?-->/g, '');
    const links = (raw.match(/<link\b/g) || []).length;
    const linkClose = (raw.match(/<\/link>/g) || []).length;
    const linkSelf = (raw.match(/<link\b[^>]*\/>/g) || []).length;
    const joints = (raw.match(/<joint\b/g) || []).length;
    const jointClose = (raw.match(/<\/joint>/g) || []).length;
    const jointSelf = (raw.match(/<joint\b[^>]*\/>/g) || []).length;
    const revolute = (raw.match(/type\s*=\s*["']revolute["']/g) || []).length;
    const continuous = (raw.match(/type\s*=\s*["']continuous["']/g) || []).length;
    const prismatic = (raw.match(/type\s*=\s*["']prismatic["']/g) || []).length;
    const fixed = (raw.match(/type\s*=\s*["']fixed["']/g) || []).length;
    const robotOpen = (raw.match(/<robot\b/g) || []).length;
    const robotClose = (raw.match(/<\/robot>/g) || []).length;
    // Balance check over real tags: every element must self-close or find a
    // matching close, which catches nested/mismatched markup the counters miss.
    const tags = raw.match(/<\/?[A-Za-z][^>]*>/g) || [];
    const stack = [];
    let balanced = true;
    for (const tag of tags) {
      if (/^<\?/.test(tag) || /\/>$/.test(tag)) continue;
      if (/^<\//.test(tag)) {
        const name = tag.slice(2, -1).trim();
        if (stack.pop() !== name) { balanced = false; break; }
      } else {
        stack.push(tag.slice(1).split(/[\s>]/)[0]);
      }
    }
    if (stack.length) balanced = false;
    const malformed = /<(?![A-Za-z?/!])/.test(raw);
    return {
      valid: balanced && !malformed && links >= 1 && links === linkClose + linkSelf &&
        joints === jointClose + jointSelf &&
        robotOpen === 1 && robotClose === 1 && /^<\?xml/.test(raw) && /<\/robot>\s*$/.test(raw.trim()),
      links: links,
      linkClose: linkClose,
      selfClosedLinks: linkSelf,
      joints: joints,
      jointClose: jointClose,
      revolute: revolute,
      continuous: continuous,
      prismatic: prismatic,
      fixed: fixed,
      bytes: raw.length
    };
  }

  function extractDHRows(xml) {
    const rows = [];
    const numPat = '[-+]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][-+]?\\d+)?';
    const re = new RegExp('<!--\\s*([^<:]+?):\\s*a=(' + numPat + ')\\s+alpha=(' + numPat + ')\\s+d=(' + numPat + ')\\s+theta=(' + numPat + ')\\s*-->', 'g');
    // A row only counts when its name is a real link name in the same
    // document, so attacker-appended comments cannot inject phantom rows.
    const linkNames = new Set();
    const linkRe = /<link\s+name="([^"]+)"/g;
    let lm;
    while ((lm = linkRe.exec(xml)) !== null) linkNames.add(lm[1]);
    let m;
    while ((m = re.exec(xml)) !== null) {
      if (linkNames.size && !linkNames.has(m[1].trim())) continue;
      rows.push({
        name: m[1].trim(),
        a: parseFloat(m[2]),
        alpha: parseFloat(m[3]),
        d: parseFloat(m[4]),
        theta: parseFloat(m[5])
      });
    }
    return rows;
  }

  // ============================================================
  // Module 15: pick and place sequence builder
  // ============================================================
  function buildPickPlace(robot, segments, options) {
    const o = options || {};
    const norm = (s, i, label) => {
      if (!s || !Array.isArray(s.q)) throw new RangeError(label + ' ' + i + ' needs a joint vector q');
      return {
        name: s.name || (label + '-' + (i + 1)),
        q: s.q.slice(),
        duration: Number.isFinite(s.duration) && s.duration > 0 ? s.duration : 1.0,
        mode: s.mode || 'space'
      };
    };
    const segs = (segments || []).map((s, i) => norm(s, i, 'phase'));
    const via = (o.via || []).map((s, i) => norm(s, i, 'via'));
    const home = o.home || zeros(robot.dof);
    // Leg durations: home -> via chain -> segs -> home. The approach leg uses
    // homeDuration (or the first via/phase duration), the return leg uses
    // returnDuration. Nonpositive values defer to the planner's own timing.
    const firstDur = via.length ? via[0].duration : (segs.length ? segs[0].duration : 1.0);
    const durations = [o.homeDuration === undefined ? firstDur : o.homeDuration]
      .concat(via.slice(1).map(s => s.duration))
      .concat(segs.map(s => s.duration))
      .concat([o.returnDuration === undefined ? 2.0 : o.returnDuration]);
    const wps = [home].concat(via.map(s => s.q), segs.map(s => s.q), [home]);
    const plan = planTrajectory(robot, wps, { durations: durations });
    return {
      robot: robot.name,
      moves: via.concat(segs),
      waypoints: wps,
      totalTime: plan.totalTime,
      plan: plan
    };
  }

  // ============================================================
  // Module 16: combined state report
  // ============================================================
  function robotReport(robot, q, options) {
    const o = options || {};
    const r = getRobot(robot);
    const fk = forwardKinematics(r, q);
    const J = jacobian(r, q);
    const w = manipulability(r, q);
    const sing = detectSingularity(r, q, { tolerance: o.tolerance });
    const links = linkFrames(r, q);
    const coll = o.obstacles ? checkCollisions(r, q, o.obstacles, o) : null;
    const tool = fk.end;
    return {
      robot: r.name,
      dof: r.dof,
      q: q.slice(),
      qDeg: q.map(toDeg),
      frames: fk.frames.map(f => ({ t: f.t.slice(), quaternion: poseToQuat(f) })),
      tool: {
        position: tool.t.slice(),
        quaternion: poseToQuat(tool),
        euler: quatToEuler(poseToQuat(tool))
      },
      jacobian: J,
      jacobianDeterminant: matDet(matMul(J, matTranspose(J))),
      manipulability: w,
      conditionNumber: sing.conditionNumber,
      singular: sing.singular,
      singularityType: sing.type,
      sigmaMin: sing.sigmaMin,
      sigmaMax: sing.sigmaMax,
      rank: sing.rank,
      links: links,
      limits: {
        within: jointWithinLimits(r, q),
        violations: jointViolations(r, q),
        utilization: limitUtilization(r, q)
      },
      collisions: coll,
      workspaceDistance: norm(vecSub(tool.t, r.base.t))
    };
  }

  // ============================================================
  // Module 17: presets used by the interactive viewport
  // ============================================================
  const PRESETS = [
    {
      id: 'ready',
      label: 'Ready carry',
      robot: 'PUMA-560',
      q: [0, 1.2, -0.6, 0, -0.4, 0]
    },
    {
      id: 'reach',
      label: 'Long reach',
      robot: 'PUMA-560',
      q: [0.6, 0.5, -0.2, 0, -1.0, 0.4]
    },
    {
      id: 'folded',
      label: 'Folded stow',
      robot: 'PUMA-560',
      q: [0, 2.4, -1.2, 0, 0.8, 0]
    },
    {
      id: 'ur5-home',
      label: 'UR5e home',
      robot: 'UR5e',
      q: [0, -1.7453, 0, 0, 1.5708, 0]
    },
    {
      id: 'ur5-reach',
      label: 'UR5e reach',
      robot: 'UR5e',
      q: [1.1, -1.2, 1.0, -1.2, 1.5708, 0.6]
    },
    {
      id: 'ur5-wrist',
      label: 'UR5e wrist bent',
      robot: 'UR5e',
      q: [0.3, -1.9, 1.9, -1.5, -1.5708, 0.9]
    },
    {
      id: 'scara-park',
      label: 'SCARA park',
      robot: 'SCARA-4',
      q: [0.9, 1.5, 0.05, 0]
    },
    {
      id: 'scara-pick',
      label: 'SCARA pick',
      robot: 'SCARA-4',
      q: [-0.8, 1.9, -0.04, 0.7]
    }
  ];

  function getPreset(id) {
    for (const p of PRESETS) if (p.id === id) return p;
    return null;
  }

  // ============================================================
  // Public API
  // ============================================================
  const api = {
    // constants
    RAD: RAD, DEG: DEG,
    // math helpers
    clamp, lerp, round, toRad, toDeg, approx, sum, mean, dot, cross,
    vecAdd, vecSub, vecScale, norm, normalize, angleBetween,
    // matrices
    matZeros, matIdentity, matAdd, matSub, matScale, matTranspose, matMul,
    matVec, matTrace, matEquals, matClone, matDet, solveLinear, matInverse,
    eigSymmetric, singularValues, matrixRank,
    // quaternions and angles
    quatIdentity, quatNormalize, quatMul, quatConj, quatDot,
    quatFromAxisAngle, quatToAxisAngle, quatRotate, quatFromMat, quatToMat,
    quatSlerp,
    eulerToMat, eulerToQuat, matToEuler, quatToEuler, eulerFromDeg,
    // poses
    pose, poseIdentity, poseMul, poseInv, poseApply, poseFromQuat, poseToQuat,
    poseToHomogeneous, poseFromHomogeneous, poseInterp, poseDistance,
    // Denavit-Hartenberg
    dhMatrix, modifiedDHMatrix, createRobot, forwardKinematics,
    linkFrames, endPose, toolPoint, poseError, twistError, dhParams,
    // Jacobian and manipulability
    jacobian, jacobianDot, jointAxis, endpointVelocityNumerical,
    manipulability, manipulabilityProfile, conditionNumber, ellipsoidAxes,
    detectSingularity,
    // inverse kinematics
    solveIK, solveIKMultiStart, jointWithinLimits, clampToLimits,
    jointViolations, limitUtilization, reachable, obstaclePotential,
    zeros,
    // trajectory
    quinticBlend, quinticVelocityBlend, quinticAccelBlend, quinticSegment,
    quinticSimple, segmentJerkCost, segmentDuration, planTrajectory,
    buildPickPlace,
    // collision
    sphereDistance, spheresCollide, segmentDistance, linkSamplePoints,
    checkCollisions, clearanceProfile,
    // URDF
    exportURDF, validateURDF, extractDHRows, xmlEscape, num,
    // robots
    PUMA_560, UR5E, SCARA, ROBOTS, getRobot, PRESETS, getPreset,
    // reporting
    robotReport
  };
  return api;
}));
