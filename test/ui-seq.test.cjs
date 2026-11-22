'use strict';
const { test } = require('node:test');
const assert = require('node:assert/strict');
const { boot } = require('../dom-harness.cjs');

function fresh(hash) {
  return boot(require('node:path').join(__dirname, '..'), {
    beforeBoot: ({ sandbox }) => { sandbox.location = { hash: hash || '' }; }
  });
}
const add3 = h => {
  const $ = id => h.get(id);
  [['0.50', '0.00', '0.30'], ['0.40', '-0.15', '0.30'], ['0.35', '0.15', '0.35']].forEach(v => {
    $('seqX').value = v[0]; $('seqY').value = v[1]; $('seqZ').value = v[2];
    $('seqAddBtn').click();
  });
};

// ---------- boot and structure ----------
test('sequencer panel boots cleanly with all new controls', () => {
  const h = fresh();
  assert.deepEqual(h.errors, []);
  for (const id of ['seqMode', 'seqX', 'seqY', 'seqZ', 'seqAddBtn', 'seqAddCurBtn', 'seqClearBtn', 'seqPlanBtn', 'seqList', 'seqStatus', 'riskStrip', 'riskStripNote', 'statRisk', 'riskBar', 'riskPill', 'dlWpJsonBtn', 'dlGcodeBtn', 'selfTestBtn', 'selfTestOut', 'checksumOut']) {
    assert.ok(h.document.getElementById(id), 'missing ' + id);
  }
  h.dispose();
});

test('risk strip canvas has an accessible name', () => {
  const h = fresh();
  const c = h.get('riskStrip');
  assert.equal(c.getAttribute('role'), 'img');
  assert.ok(c.getAttribute('aria-label').length > 10);
  h.dispose();
});

test('new buttons all have visible text names', () => {
  const h = fresh();
  for (const id of ['seqAddBtn', 'seqAddCurBtn', 'seqClearBtn', 'seqPlanBtn', 'dlWpJsonBtn', 'dlGcodeBtn', 'selfTestBtn']) {
    assert.ok(h.get(id).textContent.trim().length > 3, id);
  }
  h.dispose();
});

test('new numeric inputs have labels', () => {
  const h = fresh();
  for (const id of ['seqX', 'seqY', 'seqZ', 'seqMode']) {
    const label = h.document.querySelector('label[for="' + id + '"]');
    assert.ok(label, 'no label for ' + id);
  }
  h.dispose();
});

// ---------- waypoint list ----------
test('add waypoint rows with labels and coordinates', () => {
  const h = fresh();
  const $ = id => h.get(id);
  $('seqX').value = '0.50'; $('seqY').value = '0.00'; $('seqZ').value = '0.30';
  $('seqAddBtn').click();
  const items = $('seqList').children;
  assert.equal(items.length, 1);
  assert.match(items[0].textContent, /W1 \(0\.50, 0\.00, 0\.30\)/);
  h.dispose();
});

test('add current pose seeds a waypoint from FK', () => {
  const h = fresh();
  h.get('seqAddCurBtn').click();
  const items = h.get('seqList').children;
  assert.equal(items.length, 1);
  assert.match(items[0].textContent, /W1 \(/);
  h.dispose();
});

test('delete removes a waypoint', () => {
  const h = fresh();
  add3(h);
  const del = h.get('seqList').children[1].querySelectorAll('button')[3];
  del.click();
  assert.equal(h.get('seqList').children.length, 2);
  assert.match(h.get('seqList').textContent, /W3/);
  h.dispose();
});

test('move buttons reorder the list', () => {
  const h = fresh();
  add3(h);
  const down = h.get('seqList').children[0].querySelectorAll('button')[1];
  down.click();
  const first = h.get('seqList').children[0];
  assert.match(first.textContent, /W2/);
  h.dispose();
});

test('first move-up and last move-down are disabled', () => {
  const h = fresh();
  add3(h);
  const items = h.get('seqList').children;
  assert.ok(items[0].querySelectorAll('button')[0].disabled);
  assert.ok(items[2].querySelectorAll('button')[1].disabled);
  h.dispose();
});

test('edit loads coordinates into the inputs and removes the row', () => {
  const h = fresh();
  add3(h);
  const edit = h.get('seqList').children[1].querySelectorAll('button')[2];
  edit.click();
  assert.equal(h.get('seqList').children.length, 2);
  assert.equal(Number(h.get('seqX').value), 0.40);
  assert.equal(Number(h.get('seqY').value), -0.15);
  h.dispose();
});

test('clear empties the list and status', () => {
  const h = fresh();
  add3(h);
  h.get('seqClearBtn').click();
  assert.equal(h.get('seqList').children.length, 0);
  assert.equal(h.get('seqStatus').textContent, '');
  h.dispose();
});

test('the waypoint cap is enforced', () => {
  const h = fresh();
  for (let i = 0; i < 13; i++) h.get('seqAddCurBtn').click();
  assert.equal(h.get('seqList').children.length, 12);
  assert.match(h.get('errorStatus').textContent, /12/);
  h.dispose();
});

// ---------- planning ----------
test('planning with fewer than two waypoints shows an error', () => {
  const h = fresh();
  h.get('seqPlanBtn').click();
  assert.match(h.get('errorStatus').textContent, /two waypoints/);
  h.dispose();
});

test('cubic plan reports segments, convergence and floor state', () => {
  const h = fresh();
  add3(h);
  h.get('seqPlanBtn').click();
  const status = h.get('seqStatus').textContent;
  assert.match(status, /segments over .*s via cubic/);
  assert.match(status, /All 3 waypoint solves converged|best-effort IK/);
  assert.match(status, /Floor|manipulability/);
  assert.deepEqual(h.errors, []);
  h.dispose();
});

test('plan updates the scrub range and risk strip', () => {
  const h = fresh();
  add3(h);
  h.get('seqPlanBtn').click();
  assert.ok(Number(h.get('scrubSlider').max) > 0.5);
  const ctx = h.get('riskStrip').getContext('2d');
  assert.ok(ctx.calls.filter(c => c.name === 'fillRect').length >= 60);
  h.dispose();
});

test('scrubbing after a plan moves the robot', () => {
  const h = fresh();
  add3(h);
  h.get('seqPlanBtn').click();
  h.get('scrubSlider').value = '0.4';
  h.event(h.get('scrubSlider'), 'input');
  assert.match(h.get('scrubOut').textContent, /0\.40 s/);
  assert.match(h.get('sideReadout').textContent, /frames:/);
  h.dispose();
});

test('trapezoid and quintic modes plan without errors', () => {
  for (const mode of ['trapezoid', 'quintic']) {
    const h = fresh();
    add3(h);
    h.get('seqMode').value = mode;
    h.get('seqPlanBtn').click();
    assert.match(h.get('seqStatus').textContent, new RegExp('via ' + mode));
    h.dispose();
  }
});

test('planning an unreachable waypoint warns honestly', () => {
  const h = fresh();
  const $ = id => h.get(id);
  [['0.50', '0.00', '0.30'], ['5.00', '5.00', '5.00']].forEach(v => {
    $('seqX').value = v[0]; $('seqY').value = v[1]; $('seqZ').value = v[2];
    $('seqAddBtn').click();
  });
  $('seqPlanBtn').click();
  assert.match(h.get('seqStatus').textContent, /best-effort IK/);
  h.dispose();
});

// ---------- risk monitor ----------
test('risk pill and bar update with the pose', () => {
  const h = fresh();
  // PUMA zero pose is singular: critical.
  assert.equal(h.get('riskPill').textContent, 'critical');
  assert.match(h.get('riskBar').style.width, /100/);
  h.get('presetSelect').value = 'folded';
  h.event(h.get('presetSelect'), 'change');
  assert.notEqual(h.get('riskPill').textContent, 'critical');
  assert.match(h.get('statRisk').textContent, /safe|watch|warning|critical/);
  h.dispose();
});

test('statRisk shows level and score', () => {
  const h = fresh();
  assert.match(h.get('statRisk').textContent, /(safe|watch|warning|critical) [01]\.\d+/);
  h.dispose();
});

// ---------- exports ----------
test('waypoint bundle downloads JSON and prints checksums', () => {
  const h = fresh();
  add3(h);
  h.get('seqPlanBtn').click();
  h.get('dlWpJsonBtn').click();
  const dl = h.document.downloads;
  assert.equal(dl[dl.length - 1].filename, 'kinechain-waypoints.json');
  assert.match(h.get('checksumOut').textContent, /sha-256 payload [0-9a-f]{32}/);
  h.dispose();
});

test('g-code downloads a .gcode file with a checksum', () => {
  const h = fresh();
  add3(h);
  h.get('seqPlanBtn').click();
  h.get('dlGcodeBtn').click();
  const dl = h.document.downloads;
  assert.equal(dl[dl.length - 1].filename, 'kinechain-toolpath.gcode');
  assert.match(h.get('checksumOut').textContent, /sha-256 gcode [0-9a-f]{32}/);
  h.dispose();
});

test('exports before planning fail politely', () => {
  const h = fresh();
  h.get('dlWpJsonBtn').click();
  assert.match(h.get('errorStatus').textContent, /Plan a trajectory/);
  h.dispose();
});

// ---------- self tests ----------
test('self test button runs the battery and reports 10/10', () => {
  const h = fresh();
  h.get('selfTestBtn').click();
  const out = h.get('selfTestOut');
  assert.equal(out.hidden, false);
  assert.match(out.textContent, /10\/10 passed/);
  assert.match(out.textContent, /PASS.*Jacobian matches central finite difference/);
  h.dispose();
});

// ---------- URL hash ----------
test('hash updates as waypoints are added', () => {
  const h = fresh();
  const $ = id => h.get(id);
  $('seqX').value = '0.50'; $('seqY').value = '0.10'; $('seqZ').value = '0.30';
  $('seqAddBtn').click();
  assert.match(h.sandbox.location.hash, /w=0\.5,0\.1,0\.3/);
  h.dispose();
});

test('hash restore repopulates robot, pose, waypoints and mode', () => {
  const h = fresh('#r=UR5e&q=0.1,-0.2,0.3,0.4,-0.5,0.6&w=0.3,0.1,0.4;0.2,-0.1,0.3&m=trapezoid');
  assert.equal(h.get('robotSelect').value, 'UR5e');
  assert.equal(h.get('seqList').children.length, 2);
  assert.equal(h.get('seqMode').value, 'trapezoid');
  assert.deepEqual(h.errors, []);
  h.dispose();
});

test('a malformed hash does not break boot', () => {
  const h = fresh('#r=Nope-9&q=a,b&w=1,2&junk=1');
  assert.deepEqual(h.errors, []);
  assert.equal(h.get('seqList').children.length, 0);
  h.dispose();
});

// ---------- accessibility of generated rows ----------
test('waypoint row buttons carry accessible labels', () => {
  const h = fresh();
  h.get('seqAddCurBtn').click();
  const buttons = h.get('seqList').children[0].querySelectorAll('button');
  assert.equal(buttons.length, 4);
  buttons.forEach(b => assert.ok(b.getAttribute('aria-label').length > 3));
  h.dispose();
});

test('status and seq list are live regions', () => {
  const h = fresh();
  assert.equal(h.get('seqList').getAttribute('aria-live'), 'polite');
  assert.ok(h.get('statusLine').getAttribute('role') === 'status');
  h.dispose();
});
