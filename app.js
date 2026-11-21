/* KineChain 6DOF workbench controller.
 * Wires the local KineChain engine (src/robotics.js) to the page. All
 * geometry is computed locally; nothing leaves the browser.
 */
(function () {
  'use strict';
  const K = window.KineChain;
  const $ = id => document.getElementById(id);
  const els = {
    engineBadge: $('engineBadge'), themeToggle: $('themeToggle'), helpToggle: $('helpToggle'),
    helpPanel: $('helpPanel'), robotSelect: $('robotSelect'), presetSelect: $('presetSelect'),
    presetNote: $('presetNote'), jointSliders: $('jointSliders'), homeBtn: $('homeBtn'), randBtn: $('randBtn'),
    tgtX: $('tgtX'), tgtY: $('tgtY'), tgtZ: $('tgtZ'), tgtRoll: $('tgtRoll'), tgtPitch: $('tgtPitch'), tgtYaw: $('tgtYaw'),
    solveIkBtn: $('solveIkBtn'), targetFromPoseBtn: $('targetFromPoseBtn'),
    obsX: $('obsX'), obsY: $('obsY'), obsZ: $('obsZ'), obsR: $('obsR'), obsAddBtn: $('obsAddBtn'),
    obsClearBtn: $('obsClearBtn'), obsList: $('obsList'), wpSelect: $('wpSelect'), planBtn: $('planBtn'),
    playBtn: $('playBtn'), scrubSlider: $('scrubSlider'), scrubOut: $('scrubOut'),
    statPos: $('statPos'), statRpy: $('statRpy'), statManip: $('statManip'), statSigma: $('statSigma'),
    statSing: $('statSing'), statColl: $('statColl'), statusLine: $('statusLine'),
    sideCanvas: $('sideCanvas'), topCanvas: $('topCanvas'), sideReadout: $('sideReadout'), topReadout: $('topReadout'),
    jacHead: $('jacHead'), jacBody: $('jacBody'), urdfOut: $('urdfOut'), urdfStat: $('urdfStat'),
    exportUrdfBtn: $('exportUrdfBtn'), dlUrdfBtn: $('dlUrdfBtn'), dlJsonBtn: $('dlJsonBtn'), dlCsvBtn: $('dlCsvBtn'),
    errorStatus: $('errorStatus'), footNote: $('footNote')
  };

  const state = {
    robotName: 'PUMA-560',
    robot: K.PUMA_560,
    q: K.zeros(6),
    obstacles: [],
    plan: null,
    playing: false,
    playT: 0,
    lastIk: null,
    urdf: ''
  };

  function fail(msg) {
    els.errorStatus.textContent = String(msg);
    els.errorStatus.hidden = false;
  }
  function clearFail() { els.errorStatus.hidden = true; els.errorStatus.textContent = ''; }
  function fmt(v, d) { return Number.isFinite(v) ? v.toFixed(d === undefined ? 3 : d) : 'n/a'; }
  function fmt3(v, d) { return v.map(x => fmt(x, d === undefined ? 3 : d)).join(', '); }
  function degOfLink(link) { return link.type === 'prismatic'; }
  // Rough outer-reach estimate for view scaling: base offset plus every
  // link's a and d magnitudes plus the tool offset.
  function reachOf(robot) {
    let r = Math.hypot(robot.base.t[0], robot.base.t[1], robot.base.t[2]);
    for (const l of robot.links) r += Math.abs(l.a) + Math.abs(l.d);
    return r + Math.hypot(robot.tool.t[0], robot.tool.t[1], robot.tool.t[2]);
  }

  function themeVars() {
    const s = getComputedStyle(document.documentElement);
    return {
      grid: s.getPropertyValue('--grid'), text: s.getPropertyValue('--muted'),
      link: s.getPropertyValue('--link-a'), linkB: s.getPropertyValue('--link-b'),
      joint: s.getPropertyValue('--joint'), obstacle: s.getPropertyValue('--obstacle'),
      target: s.getPropertyValue('--target'), accent: s.getPropertyValue('--accent')
    };
  }

  // ---------- joint sliders ----------
  function buildSliders() {
    const host = els.jointSliders;
    host.replaceChildren();
    const n = state.robot.dof;
    for (let i = 0; i < n; i++) {
      const link = state.robot.links[i];
      const row = document.createElement('div');
      row.className = 'control-row';
      const label = document.createElement('label');
      label.setAttribute('for', 'j' + i);
      const isPrism = degOfLink(link);
      label.textContent = 'j' + (i + 1) + ' ' + link.name + (isPrism ? ' (m)' : ' (rad)');
      const out = document.createElement('output');
      out.id = 'j' + i + 'Out';
      out.textContent = fmt(state.q[i]);
      const input = document.createElement('input');
      input.type = 'range';
      input.id = 'j' + i;
      input.min = String(link.lo);
      input.max = String(link.hi);
      input.step = isPrism ? '0.005' : '0.01';
      input.value = String(state.q[i]);
      input.addEventListener('input', () => {
        state.q[i] = Number(input.value);
        out.textContent = fmt(state.q[i]);
        state.plan = null;
        render();
      });
      row.append(label, out, input);
      host.appendChild(row);
    }
  }

  function syncSliders() {
    for (let i = 0; i < state.robot.dof; i++) {
      const input = $('j' + i);
      const out = $('j' + i + 'Out');
      if (input && out) {
        input.value = String(state.q[i]);
        out.textContent = fmt(state.q[i]);
      }
    }
  }

  // ---------- presets ----------
  function buildPresets() {
    const sel = els.presetSelect;
    sel.replaceChildren();
    const opt = (v, t) => { const o = document.createElement('option'); o.value = v; o.textContent = t; return o; };
    sel.appendChild(opt('custom', 'Custom (sliders)'));
    for (const p of K.PRESETS) {
      if (p.robot !== state.robotName) continue;
      sel.appendChild(opt(p.id, p.label));
    }
  }

  // ---------- canvas drawing ----------
  function drawView(canvas, axes) {
    // axes: [indexForX, indexForY] into the 3D world point
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const V = themeVars();
    ctx.clearRect(0, 0, W, H);
    const reach = reachOf(state.robot);
    const scale = 0.42 * Math.min(W, H) / Math.max(0.15, reach);
    const cx = W / 2, cy = H * 0.55;
    const P = p => [cx + p[axes[0]] * scale, cy - p[axes[1]] * scale];

    // grid circles at 25%, 50%, 100% reach
    ctx.strokeStyle = V.grid;
    ctx.lineWidth = 1;
    for (const f of [0.25, 0.5, 0.75, 1]) {
      ctx.beginPath();
      ctx.arc(cx, cy, reach * f * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    // ground line on the side view
    if (axes[1] === 2) {
      ctx.strokeStyle = V.text;
      ctx.beginPath();
      const g = P([-reach, 0, 0]);
      const g2 = P([reach, 0, 0]);
      ctx.moveTo(g[0], g[1]); ctx.lineTo(g2[0], g2[1]);
      ctx.stroke();
    }

    const fk = K.forwardKinematics(state.robot, state.q);
    const frames = fk.frames;

    // obstacles as circles
    for (const ob of state.obstacles) {
      const c = P([ob.x, ob.y, ob.z]);
      ctx.beginPath();
      ctx.fillStyle = V.obstacle;
      ctx.globalAlpha = 0.25;
      ctx.arc(c[0], c[1], ob.radius * scale, 0, Math.PI * 2);
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = V.obstacle;
      ctx.beginPath();
      ctx.arc(c[0], c[1], ob.radius * scale, 0, Math.PI * 2);
      ctx.stroke();
    }

    // planned path
    if (state.plan) {
      ctx.strokeStyle = V.target;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      let started = false;
      for (const s of state.plan.sampleRange(64)) {
        const e = K.endPose(state.robot, s.q);
        const p = P(e.t);
        if (!started) { ctx.moveTo(p[0], p[1]); started = true; } else ctx.lineTo(p[0], p[1]);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // links
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';
    for (let i = 0; i < frames.length - 1; i++) {
      const a = P(frames[i].t), b = P(frames[i + 1].t);
      ctx.strokeStyle = i % 2 ? V.linkB : V.link;
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
      ctx.stroke();
    }
    // joints
    for (let i = 0; i < frames.length; i++) {
      const p = P(frames[i].t);
      ctx.beginPath();
      ctx.fillStyle = V.joint;
      ctx.arc(p[0], p[1], i === frames.length - 1 ? 7 : 5, 0, Math.PI * 2);
      ctx.fill();
      if (i < state.robot.dof) {
        ctx.fillStyle = V.text;
        ctx.font = '11px ui-monospace, monospace';
        ctx.fillText('j' + (i + 1), p[0] + 8, p[1] - 6);
      } else {
        ctx.fillStyle = V.text;
        ctx.font = '11px ui-monospace, monospace';
        ctx.fillText('ee', p[0] + 8, p[1] - 6);
      }
    }

    // target marker
    const tgt = readTarget();
    const tp = P(tgt.t);
    ctx.strokeStyle = V.target;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(tp[0] - 8, tp[1]); ctx.lineTo(tp[0] + 8, tp[1]);
    ctx.moveTo(tp[0], tp[1] - 8); ctx.lineTo(tp[0], tp[1] + 8);
    ctx.stroke();
    return fk;
  }

  // ---------- readouts ----------
  function readTarget() {
    const num = el => { const v = Number(el.value); return Number.isFinite(v) ? v : 0; };
    return {
      R: K.eulerToMat(num(els.tgtRoll), num(els.tgtPitch), num(els.tgtYaw)),
      t: [num(els.tgtX), num(els.tgtY), num(els.tgtZ)]
    };
  }

  function obstacleList() {
    return state.obstacles.map(o => ({ x: o.x, y: o.y, z: o.z, radius: o.radius }));
  }

  function render() {
    clearFail();
    const r = state.robot;
    const fkSide = drawView(els.sideCanvas, [0, 2]);
    const fk = fkSide;
    drawView(els.topCanvas, [0, 1]);
    const ee = K.endPose(r, state.q);
    const rpy = K.matToEuler(ee.R);
    els.statPos.textContent = fmt3(ee.t);
    els.statRpy.textContent = fmt3([rpy.roll, rpy.pitch, rpy.yaw]);
    const man = K.manipulability(r, state.q);
    els.statManip.textContent = fmt(man);
    const svs = K.singularValues(K.jacobian(r, state.q).map(row => row.slice()));
    els.statSigma.textContent = fmt(svs.length ? svs[0] : NaN, 4);
    const sing = K.detectSingularity(r, state.q);
    els.statSing.textContent = sing.singular ? sing.type : 'none';
    const col = K.checkCollisions(r, state.q, obstacleList(), {});
    els.statColl.textContent = col.collided ? String(col.collisions.length) + ' hit' : 'clear';
    const util = K.limitUtilization(r, state.q);
    const worst = util.indexOf(Math.max.apply(null, util));
    els.statusLine.className = 'status-line' + (sing.singular || col.collided ? ' warn' : ' ok');
    els.statusLine.textContent =
      (sing.singular ? 'Singular configuration (' + sing.type + '), sigma min ' + fmt(svs[0], 4) + '. ' : 'Pose is regular. ') +
      (col.collided ? 'Obstacle contact on link ' + col.collisions[0].link + '. ' : '') +
      'Highest joint utilisation: j' + (worst + 1) + ' at ' + fmt(util[worst] * 100, 0) + '%.';
    els.sideReadout.textContent = 'frames: ' + fk.frames.map(f => '(' + fmt(f.t[0], 2) + ',' + fmt(f.t[1], 2) + ',' + fmt(f.t[2], 2) + ')').join('  ');
    els.topReadout.textContent = 'reach ' + fmt(reachOf(r), 2) + ' m, dof ' + r.dof;
    renderJacobian();
  }

  function renderJacobian() {
    const J = K.jacobian(state.robot, state.q);
    els.jacHead.replaceChildren();
    const h0 = document.createElement('th');
    h0.textContent = 'twist';
    els.jacHead.appendChild(h0);
    for (let i = 0; i < state.robot.dof; i++) {
      const th = document.createElement('th');
      th.textContent = 'j' + (i + 1);
      els.jacHead.appendChild(th);
    }
    els.jacBody.replaceChildren();
    const rows = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'];
    for (let rr = 0; rr < 6; rr++) {
      const tr = document.createElement('tr');
      const td0 = document.createElement('td');
      td0.textContent = rows[rr];
      tr.appendChild(td0);
      for (let c = 0; c < state.robot.dof; c++) {
        const td = document.createElement('td');
        td.textContent = fmt(J[rr][c], 3);
        tr.appendChild(td);
      }
      els.jacBody.appendChild(tr);
    }
  }

  // ---------- actions ----------
  function setRobot(name) {
    state.robotName = name;
    state.robot = K.getRobot(name);
    state.q = K.zeros(state.robot.dof);
    state.plan = null;
    state.lastIk = null;
    state.playing = false;
    els.playBtn.setAttribute('aria-pressed', 'false');
    els.playBtn.textContent = 'Play';
    buildPresets();
    buildSliders();
    refreshUrdf();
    render();
  }

  function refreshUrdf() {
    try {
      state.urdf = K.exportURDF(state.robot);
      els.urdfOut.value = state.urdf;
      const v = K.validateURDF(state.urdf);
      els.urdfStat.textContent = 'validated: ' + (v.valid ? 'yes' : 'NO') + ', links ' + v.links + ', joints ' + v.joints + ', ' + v.bytes + ' bytes';
    } catch (e) {
      fail('URDF export failed: ' + e.message);
    }
  }

  function download(name, mime, text) {
    const blob = new Blob([text], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = name;
    a.click();
    URL.revokeObjectURL(url);
  }

  function solveIk() {
    try {
      const target = readTarget();
      const result = K.solveIKMultiStart(state.robot, target, {
        seeds: 12,
        maxIter: 400,
        obstacles: obstacleList(),
        obstacleWeight: 2
      });
      state.lastIk = { target: target, result: result };
      if (result.converged) state.q = result.q.slice();
      render();
      if (result.converged) {
        syncSliders();
        els.statusLine.className = 'status-line ok';
        els.statusLine.textContent = 'IK converged in ' + result.iterations + ' iterations, residual ' + fmt(result.error, 6) + '.';
      } else {
        els.statusLine.className = 'status-line warn';
        els.statusLine.textContent = 'IK stalled: residual ' + fmt(result.error, 4) + ' (pos ' + fmt(result.positionError, 4) + ' m, rot ' + fmt(result.rotationError === undefined ? NaN : result.rotationError, 4) + ' rad). Try a nearer target or another seed.';
      }
    } catch (e) {
      fail('IK error: ' + e.message);
    }
  }

  function planPickPlace() {
    try {
      const mode = els.wpSelect.value;
      const home = K.zeros(state.robot.dof);
      const pickPreset = K.getPreset(state.robotName === 'SCARA-4' ? 'scara-pick' : (state.robotName === 'UR5e' ? 'ur5-reach' : 'reach'));
      const placePreset = K.getPreset(state.robotName === 'SCARA-4' ? 'scara-park' : (state.robotName === 'UR5e' ? 'ur5-wrist' : 'folded'));
      const segs = [];
      if (pickPreset) segs.push({ name: 'pick', q: pickPreset.q, duration: 2.2 });
      if (mode === 'home-pick-place' && placePreset) segs.push({ name: 'place', q: placePreset.q, duration: 2.2 });
      const built = K.buildPickPlace(state.robot, segs, { home: home, returnDuration: 2.0 });
      state.plan = built.plan;
      els.scrubSlider.max = String(Math.max(0.01, state.plan.totalTime));
      els.scrubSlider.value = '0';
      els.scrubOut.textContent = '0.00 s';
      state.playT = 0;
      render();
      els.statusLine.className = 'status-line ok';
      els.statusLine.textContent = 'Planned ' + state.plan.segments.length + ' segments, ' + fmt(state.plan.totalTime, 2) + ' s total.';
    } catch (e) {
      fail('Planning error: ' + e.message);
    }
  }

  function applyPlanTime(t) {
    if (!state.plan) return;
    const s = state.plan.sample(Math.min(Math.max(0, t), state.plan.totalTime));
    state.q = s.q.slice();
    syncSliders();
    els.scrubOut.textContent = fmt(t, 2) + ' s';
    render();
  }

  // ---------- wiring ----------
  els.robotSelect.addEventListener('change', () => setRobot(els.robotSelect.value));
  els.presetSelect.addEventListener('change', () => {
    const p = K.getPreset(els.presetSelect.value);
    if (p && p.robot === state.robotName) {
      state.q = p.q.slice(0, state.robot.dof);
      while (state.q.length < state.robot.dof) state.q.push(0);
      syncSliders();
      state.plan = null;
      render();
    }
    els.presetNote.textContent = p && p.id !== 'custom' ? 'Preset ' + p.label + ' applied to ' + state.robotName + '.' : 'Sliders drive the joint vector directly.';
  });
  els.homeBtn.addEventListener('click', () => { state.q = K.zeros(state.robot.dof); state.plan = null; syncSliders(); render(); });
  els.randBtn.addEventListener('click', () => {
    state.q = state.robot.links.map(l => l.lo + Math.random() * (l.hi - l.lo));
    state.plan = null;
    syncSliders();
    render();
  });
  els.solveIkBtn.addEventListener('click', solveIk);
  els.targetFromPoseBtn.addEventListener('click', () => {
    const ee = K.endPose(state.robot, state.q);
    const rpy = K.matToEuler(ee.R);
    els.tgtX.value = fmt(ee.t[0], 3); els.tgtY.value = fmt(ee.t[1], 3); els.tgtZ.value = fmt(ee.t[2], 3);
    els.tgtRoll.value = fmt(rpy.roll, 3); els.tgtPitch.value = fmt(rpy.pitch, 3); els.tgtYaw.value = fmt(rpy.yaw, 3);
    render();
  });
  els.obsAddBtn.addEventListener('click', () => {
    const num = el => { const v = Number(el.value); return Number.isFinite(v) ? v : 0; };
    const radius = Math.max(0.01, num(els.obsR));
    if (state.obstacles.length >= 6) { fail('At most 6 obstacles.'); return; }
    state.obstacles.push({ x: num(els.obsX), y: num(els.obsY), z: num(els.obsZ), radius: radius });
    els.obsList.textContent = state.obstacles.map((o, i) => '#' + (i + 1) + ' (' + fmt(o.x, 2) + ',' + fmt(o.y, 2) + ',' + fmt(o.z, 2) + ') r=' + fmt(o.radius, 2)).join('  ');
    render();
  });
  els.obsClearBtn.addEventListener('click', () => { state.obstacles = []; els.obsList.textContent = ''; render(); });
  els.planBtn.addEventListener('click', planPickPlace);
  els.playBtn.addEventListener('click', () => {
    if (!state.plan) { fail('Plan a trajectory first.'); return; }
    state.playing = !state.playing;
    els.playBtn.setAttribute('aria-pressed', String(state.playing));
    els.playBtn.textContent = state.playing ? 'Pause' : 'Play';
    if (state.playing) tick();
  });
  function tick() {
    if (!state.playing || !state.plan) return;
    applyPlanTime(state.playT);
    state.playT += 1 / 30;
    if (state.playT > state.plan.totalTime) {
      state.playing = false;
      els.playBtn.setAttribute('aria-pressed', 'false');
      els.playBtn.textContent = 'Play';
      return;
    }
    requestAnimationFrame(tick);
  }
  els.scrubSlider.addEventListener('input', () => {
    state.playT = Number(els.scrubSlider.value);
    applyPlanTime(state.playT);
  });
  els.exportUrdfBtn.addEventListener('click', refreshUrdf);
  els.dlUrdfBtn.addEventListener('click', () => download('kinechain-' + state.robotName.toLowerCase() + '.urdf', 'application/xml', state.urdf));
  els.dlJsonBtn.addEventListener('click', () => {
    const payload = {
      robot: state.robotName, q: state.q, endPose: K.endPose(state.robot, state.q),
      obstacles: state.obstacles, ik: state.lastIk, planTime: state.plan ? state.plan.totalTime : null
    };
    download('kinechain-run.json', 'application/json', JSON.stringify(payload, null, 2));
  });
  els.dlCsvBtn.addEventListener('click', () => {
    if (!state.plan) { fail('Plan a trajectory first.'); return; }
    const n = state.robot.dof;
    const head = ['t'].concat(Array.from({ length: n }, (_, i) => 'q' + (i + 1)), Array.from({ length: n }, (_, i) => 'dq' + (i + 1)));
    const lines = [head.join(',')];
    for (const s of state.plan.sampleRange(120)) {
      lines.push([fmt(s.t, 4)].concat(s.q.map(v => fmt(v, 5)), s.dq.map(v => fmt(v, 5))).join(','));
    }
    download('kinechain-trajectory.csv', 'text/csv', lines.join('\n') + '\n');
  });
  els.themeToggle.addEventListener('click', () => {
    const dark = document.documentElement.getAttribute('data-theme') !== 'light';
    const next = dark ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', next);
    els.themeToggle.setAttribute('aria-pressed', String(next === 'dark'));
    els.themeToggle.textContent = next === 'dark' ? 'Light theme' : 'Dark theme';
    try { localStorage.setItem('kinechain-theme', next); } catch (e) { /* storage may be blocked */ }
    render();
  });
  els.helpToggle.addEventListener('click', () => {
    const open = els.helpPanel.hidden;
    els.helpPanel.hidden = !open;
    els.helpToggle.setAttribute('aria-expanded', String(open));
  });

  // ---------- boot ----------
  function boot() {
    try {
      let saved = 'dark';
      try { saved = localStorage.getItem('kinechain-theme') || 'dark'; } catch (e) { saved = 'dark'; }
      document.documentElement.setAttribute('data-theme', saved);
      els.themeToggle.setAttribute('aria-pressed', String(saved === 'dark'));
      els.themeToggle.textContent = saved === 'dark' ? 'Light theme' : 'Dark theme';
      setRobot(state.robotName);
      els.engineBadge.textContent = 'engine ready, ' + Object.keys(K.ROBOTS).length + ' robots';
      els.footNote.textContent = 'KineChain engine ' + (K.VERSION || 'local') + ', ' + K.PRESETS.length + ' presets';
    } catch (e) {
      fail('Boot failed: ' + e.message);
      els.engineBadge.textContent = 'engine error';
    }
  }
  boot();
})();
