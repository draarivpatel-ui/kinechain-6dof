'use strict';
const { test } = require('node:test');
const assert = require('node:assert/strict');
const { boot } = require('./dom-harness.cjs');

function fresh() { return boot(__dirname); }

// ---------- boot and structure ----------
test('boot: no console errors, engine ready', () => {
  const h = fresh();
  assert.deepEqual(h.errors, []);
  assert.match(h.get('engineBadge').textContent, /engine ready/);
  h.dispose();
});

test('page has a skip link into the workbench', () => {
  const h = fresh();
  const skip = h.document.querySelector('.skip-link');
  assert.ok(skip);
  assert.equal(skip.getAttribute('href'), '#workbench');
  h.dispose();
});

test('CSP meta forbids network connections', () => {
  const h = fresh();
  const csp = h.document.querySelector('meta[http-equiv="Content-Security-Policy"]');
  assert.ok(csp, 'missing CSP meta');
  const content = csp.getAttribute('content');
  assert.match(content, /connect-src 'none'/);
  assert.match(content, /script-src 'self'/);
  assert.match(content, /object-src 'none'/);
  h.dispose();
});

test('no remote resources referenced anywhere', () => {
  const h = fresh();
  assert.ok(!/https?:\/\//.test(h.html), 'index.html references a remote URL');
  assert.ok(!/import\(|fetch\(|XMLHttpRequest/.test(h.html), 'network API used in markup');
  h.dispose();
});

test('robot select lists the three built-ins', () => {
  const h = fresh();
  const opts = h.get('robotSelect').querySelectorAll('option').map(o => o.value);
  assert.deepEqual(opts, ['PUMA-560', 'UR5e', 'SCARA-4']);
  h.dispose();
});

test('preset select is filtered to the active robot', () => {
  const h = fresh();
  const values = h.get('presetSelect').querySelectorAll('option').map(o => o.value);
  assert.ok(values.includes('custom'));
  assert.ok(values.includes('ready'));
  assert.ok(!values.includes('ur5-reach'));
  h.dispose();
});

test('six joint sliders render for PUMA', () => {
  const h = fresh();
  assert.equal(h.get('jointSliders').querySelectorAll('input[type="range"]').length, 6);
  h.dispose();
});

test('SCARA shows four sliders and rebuilds on switch', () => {
  const h = fresh();
  h.get('robotSelect').value = 'SCARA-4';
  h.get('robotSelect').dispatchEvent({ type: 'change', bubbles: true });
  assert.equal(h.get('jointSliders').querySelectorAll('input[type="range"]').length, 4);
  const labels = h.get('jointSliders').querySelectorAll('label').map(l => l.textContent);
  assert.ok(labels.some(t => /\(m\)/.test(t)), 'prismatic joint should be labelled in metres');
  h.dispose();
});

// ---------- interaction ----------
test('joint slider input updates the end position readout', () => {
  const h = fresh();
  const before = h.get('statPos').textContent;
  const slider = h.get('j0');
  slider.value = '1.2';
  slider.dispatchEvent({ type: 'input', bubbles: true });
  assert.notEqual(h.get('statPos').textContent, before);
  assert.match(h.get('j0Out').textContent, /1\.200/);
  h.dispose();
});

test('home button returns to the zero pose', () => {
  const h = fresh();
  h.get('randBtn').click();
  h.get('homeBtn').click();
  assert.match(h.get('statPos').textContent, /0\.864, 0\.149, 0\.020/);
  h.dispose();
});

test('random pose stays inside joint limits', () => {
  const h = fresh();
  h.get('randBtn').click();
  const vals = h.get('jointSliders').querySelectorAll('input[type="range"]').map(s => Number(s.value));
  const robot = h.sandbox.KineChain.PUMA_560;
  vals.forEach((v, i) => {
    assert.ok(v >= robot.links[i].lo && v <= robot.links[i].hi, 'joint ' + i + ' out of limits');
  });
  h.dispose();
});

test('preset applies a named pose', () => {
  const h = fresh();
  h.get('presetSelect').value = 'reach';
  h.get('presetSelect').dispatchEvent({ type: 'change', bubbles: true });
  assert.match(h.get('presetNote').textContent, /reach/i);
  h.dispose();
});

test('singularity stat shows wrist at the zero pose', () => {
  const h = fresh();
  assert.equal(h.get('statSing').textContent, 'wrist');
  h.dispose();
});

test('target=current fills inputs from FK', () => {
  const h = fresh();
  h.get('targetFromPoseBtn').click();
  assert.equal(Number(h.get('tgtX').value).toFixed(3), '0.864');
  h.dispose();
});

test('solve IK on current pose converges', () => {
  const h = fresh();
  // The zero pose is wrist-singular, so drive to a regular preset first.
  h.get('presetSelect').value = 'reach';
  h.get('presetSelect').dispatchEvent({ type: 'change', bubbles: true });
  h.get('targetFromPoseBtn').click();
  h.get('solveIkBtn').click();
  assert.match(h.get('statusLine').textContent, /converged/);
  h.dispose();
});

test('solve IK on a far target reports a stall without throwing', () => {
  const h = fresh();
  h.get('tgtX').value = '5';
  h.get('tgtY').value = '5';
  h.get('tgtZ').value = '5';
  h.get('solveIkBtn').click();
  assert.match(h.get('statusLine').textContent, /stalled/);
  assert.deepEqual(h.errors, []);
  h.dispose();
});

test('nonfinite IK inputs are neutralised to zero not NaN', () => {
  const h = fresh();
  h.get('tgtX').value = 'not-a-number';
  h.get('tgtY').value = '0.1';
  h.get('tgtZ').value = '0.4';
  h.get('solveIkBtn').click();
  assert.deepEqual(h.errors, []);
  assert.ok(!/NaN/.test(h.get('statusLine').textContent));
  h.dispose();
});

test('obstacles list updates and feeds collision stat', () => {
  const h = fresh();
  h.get('obsAddBtn').click();
  h.get('obsAddBtn').click();
  assert.match(h.get('obsList').textContent, /#1/);
  assert.match(h.get('obsList').textContent, /#2/);
  h.get('obsClearBtn').click();
  assert.equal(h.get('obsList').textContent, '');
  h.dispose();
});

test('obstacle radius is clamped to a positive minimum', () => {
  const h = fresh();
  h.get('obsR').value = '-5';
  h.get('obsAddBtn').click();
  assert.match(h.get('obsList').textContent, /r=0\.01\b/);
  h.dispose();
});

test('plan quintic produces a playable trajectory', () => {
  const h = fresh();
  h.get('planBtn').click();
  assert.match(h.get('statusLine').textContent, /Planned \d+ segments/);
  h.dispose();
});

test('scrub slider moves the arm along the plan', () => {
  const h = fresh();
  h.get('planBtn').click();
  const before = h.get('statPos').textContent;
  const scrub = h.get('scrubSlider');
  scrub.value = '2.0';
  scrub.dispatchEvent({ type: 'input', bubbles: true });
  assert.notEqual(h.get('statPos').textContent, before);
  assert.match(h.get('scrubOut').textContent, /2\.00 s/);
  h.dispose();
});

test('play advances frames and pause stops them', () => {
  const h = fresh();
  h.get('planBtn').click();
  h.get('playBtn').click();
  assert.equal(h.get('playBtn').textContent, 'Pause');
  const before = h.get('scrubOut').textContent;
  h.frames(8);
  assert.notEqual(h.get('scrubOut').textContent, before);
  h.get('playBtn').click();
  assert.equal(h.get('playBtn').textContent, 'Play');
  h.dispose();
});

test('play without a plan reports an error not a crash', () => {
  const h = fresh();
  h.get('playBtn').click();
  assert.match(h.get('errorStatus').textContent, /Plan a trajectory/);
  h.dispose();
});

test('Jacobian table renders 6 rows plus dof columns', () => {
  const h = fresh();
  assert.equal(h.get('jacBody').children.length, 6);
  assert.equal(h.get('jacHead').children.length, 7);
  h.dispose();
});

test('Jacobian table shrinks to dof for SCARA', () => {
  const h = fresh();
  h.get('robotSelect').value = 'SCARA-4';
  h.get('robotSelect').dispatchEvent({ type: 'change', bubbles: true });
  assert.equal(h.get('jacHead').children.length, 5);
  h.dispose();
});

// ---------- exports ----------
test('URDF textarea contains a validated document', () => {
  const h = fresh();
  assert.match(h.get('urdfOut').value, /<robot name="PUMA-560">/);
  assert.match(h.get('urdfStat').textContent, /validated: yes/);
  h.dispose();
});

test('URDF refresh after robot switch changes the robot name', () => {
  const h = fresh();
  h.get('robotSelect').value = 'UR5e';
  h.get('robotSelect').dispatchEvent({ type: 'change', bubbles: true });
  assert.match(h.get('urdfOut').value, /<robot name="UR5e">/);
  h.dispose();
});

test('URDF download produces a .urdf blob link', () => {
  const h = fresh();
  h.get('dlUrdfBtn').click();
  assert.equal(h.document.downloads.length, 1);
  assert.match(h.document.downloads[0].filename, /\.urdf$/);
  h.dispose();
});

test('run JSON download contains serialisable state', () => {
  const h = fresh();
  h.get('dlJsonBtn').click();
  assert.match(h.document.downloads[0].filename, /\.json$/);
  h.dispose();
});

test('CSV download needs a plan first', () => {
  const h = fresh();
  h.get('dlCsvBtn').click();
  assert.match(h.get('errorStatus').textContent, /Plan a trajectory/);
  h.get('planBtn').click();
  h.get('dlCsvBtn').click();
  assert.match(h.document.downloads.at(-1).filename, /\.csv$/);
  h.dispose();
});

// ---------- theme and guide ----------
test('theme toggle flips data-theme and stores it', () => {
  const h = fresh();
  h.get('themeToggle').click();
  assert.equal(h.document.documentElement.getAttribute('data-theme'), 'light');
  assert.equal(h.get('themeToggle').textContent, 'Dark theme');
  h.get('themeToggle').click();
  assert.equal(h.document.documentElement.getAttribute('data-theme'), 'dark');
  h.dispose();
});

test('theme survives a storage exception', () => {
  const h = boot(__dirname, { storageBlocked: true });
  assert.deepEqual(h.errors, []);
  h.get('themeToggle').click();
  assert.equal(h.document.documentElement.getAttribute('data-theme'), 'light');
  h.dispose();
});

test('guide toggle flips hidden and aria-expanded', () => {
  const h = fresh();
  assert.equal(h.get('helpPanel').hidden, true);
  h.get('helpToggle').click();
  assert.equal(h.get('helpPanel').hidden, false);
  assert.equal(h.get('helpToggle').getAttribute('aria-expanded'), 'true');
  h.dispose();
});

// ---------- accessibility ----------
test('focus rings exist for every interactive class', () => {
  const h = fresh();
  for (const sel of ['.btn:focus-visible', 'input:focus-visible', 'select:focus-visible', 'canvas:focus-visible', 'textarea:focus-visible']) {
    assert.ok(h.css.includes(sel), 'missing ' + sel);
  }
  h.dispose();
});

test('canvases carry accessible labels and are focusable', () => {
  const h = fresh();
  for (const id of ['sideCanvas', 'topCanvas']) {
    const c = h.get(id);
    assert.equal(c.getAttribute('tabindex'), '0');
    assert.ok(c.getAttribute('aria-label').length > 10);
  }
  h.dispose();
});

test('status region uses role=status and errors use role=alert', () => {
  const h = fresh();
  assert.equal(h.get('statusLine').getAttribute('role'), 'status');
  assert.equal(h.get('errorStatus').getAttribute('role'), 'alert');
  h.dispose();
});

test('no smooth-scroll CSS and no single-letter key shortcuts', () => {
  const h = fresh();
  assert.ok(!/scroll-behavior\s*:\s*smooth/.test(h.css));
  assert.ok(!/scroll-behavior\s*:\s*smooth/.test(h.html));
  h.dispose();
});

test('sticky controls panel is height capped and scrollable on desktop only', () => {
  const h = fresh();
  assert.match(h.css, /\.controls-panel\s*\{[^}]*max-height:\s*calc\(100vh - 2rem\)[^}]*overflow-y:\s*auto/);
  assert.match(h.css, /max-width:\s*980px\)\s*\{\s*\.controls-panel\s*\{[^}]*max-height:\s*none[^}]*overflow-y:\s*visible/,
    'mobile panel must stay uncapped with no inner scroll');
  h.dispose();
});

test('all visible controls have accessible names', () => {
  const h = fresh();
  const unlabelled = h.document.querySelectorAll('button, input, select, textarea')
    .filter(el => {
      // A button's accessible name is its text; other controls need a label or aria-label.
      const named = el.localName === 'button' ? el.textContent.trim().length > 0 : false;
      const labelled = el.getAttribute('aria-label') || (el.id && h.document.querySelector('label[for="' + el.id + '"]'));
      return !named && !labelled && !el.hidden;
    });
  assert.deepEqual(unlabelled.map(el => el.id || el.localName), []);
  h.dispose();
});

test('no em dash or en dash anywhere in shipped text', () => {
  const h = fresh();
  for (const src of [h.html, h.css]) {
    assert.ok(!src.includes('\u2014') && !src.includes('\u2013'), 'dash character found');
  }
  h.dispose();
});
