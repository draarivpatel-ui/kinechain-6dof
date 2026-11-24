'use strict';
const { test } = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const { boot } = require('../dom-harness.cjs');
const K = require('../src/robotics.js');

function fresh(hash) {
  return boot(path.join(__dirname, '..'), {
    beforeBoot: ({ sandbox }) => { sandbox.location = { hash: hash || '' }; }
  });
}

function hexLuminance(hex) {
  const parts = hex.replace('#', '').match(/../g).map(x => parseInt(x, 16) / 255)
    .map(v => (v <= 0.03928 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4)));
  return 0.2126 * parts[0] + 0.7152 * parts[1] + 0.0722 * parts[2];
}
function contrast(a, b) {
  const x = hexLuminance(a), y = hexLuminance(b);
  return (Math.max(x, y) + 0.05) / (Math.min(x, y) + 0.05);
}
function cssVar(css, name) {
  const m = new RegExp(':root\\s*\\{[^}]*' + name.replace(/-/g, '\\-') + '\\s*:\\s*([^;]+);').exec(css);
  return m && m[1].trim();
}

test('skip link target is keyboard focusable', () => {
  const h = fresh();
  const main = h.document.getElementById('workbench');
  assert.equal(main.getAttribute('tabindex'), '-1');
  h.dispose();
});

test('analysis workspace keeps heading order without a level skip', () => {
  const h = fresh();
  const stage = h.document.querySelector('section.stage');
  assert.ok(stage.querySelector('h2'), 'stage needs an h2 before its h3 panels');
  h.dispose();
});

test('risk meter exposes meter semantics and a live value', () => {
  const h = fresh();
  const meter = h.document.querySelector('.risk-meter');
  assert.equal(meter.getAttribute('role'), 'meter');
  assert.equal(meter.getAttribute('aria-valuemin'), '0');
  assert.equal(meter.getAttribute('aria-valuemax'), '1');
  const now = Number(meter.getAttribute('aria-valuenow'));
  assert.ok(Number.isFinite(now) && now >= 0 && now <= 1);
  h.dispose();
});

test('Jacobian row labels are row headers', () => {
  const h = fresh();
  const firstRow = h.get('jacBody').children[0];
  assert.equal(firstRow.children[0].localName, 'th');
  assert.equal(firstRow.children[0].getAttribute('scope'), 'row');
  h.dispose();
});

test('reduced motion users are not given decorative transitions', () => {
  const h = fresh();
  assert.match(h.css, /@media\s*\(prefers-reduced-motion:\s*reduce\)/);
  assert.match(h.css, /prefers-reduced-motion:\s*reduce\)\s*\{[\s\S]*?transition:\s*none/);
  h.dispose();
});

test('compact waypoint buttons keep an accessible touch target', () => {
  const h = fresh();
  assert.match(h.css, /\.seq-item \.seq-actions \.btn\s*\{[^}]*min-height:\s*(?:2[4-9]|[3-9]\d)px/);
  h.dispose();
});

test('light accent text keeps AA contrast on its soft badge surface', () => {
  const h = fresh();
  const accent = cssVar(h.css, '--accent');
  const soft = cssVar(h.css, '--accent-soft');
  assert.ok(accent && soft, 'theme variables missing');
  assert.ok(contrast(accent, soft) >= 4.5, 'contrast ' + contrast(accent, soft).toFixed(2) + ':1');
  h.dispose();
});

test('sequencer exposes and stores a dwell hold input', () => {
  const h = fresh('');
  const dwell = h.get('seqDwell');
  assert.equal(dwell.type, 'number');
  assert.ok(h.document.querySelector('label[for="seqDwell"]'));
  dwell.value = '1.2';
  h.get('seqAddBtn').click();
  assert.match(h.get('seqList').textContent, /hold 1\.2s/);
  assert.match(h.sandbox.location.hash, /0\.5,0,0\.3,1\.2/);
  h.dispose();
});

test('hash joint values are clamped into robot limits on restore', () => {
  const h = fresh('#q=99,99,99,99,99,99');
  const clamped = K.PUMA_560.links.map(l => l.hi);
  const expected = K.endPose(K.PUMA_560, clamped).t.map(v => v.toFixed(3)).join(', ');
  assert.equal(h.get('statPos').textContent, expected);
  h.dispose();
});

test('hash waypoint dwell values restore into the sequencer list', () => {
  const h = fresh('#w=0.5,0,0.3,1.5;0.4,-0.1,0.3');
  assert.match(h.get('seqList').textContent, /hold 1\.5s/);
  h.dispose();
});
