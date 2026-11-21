'use strict';
/*
 * Deterministic DOM and canvas adapter for local controller regression tests.
 * This is not a layout engine. It executes the page's real local scripts and
 * event handlers inside a VM sandbox and records canvas calls, downloads and
 * console errors so UI behaviour can be asserted without a browser.
 * Shared infrastructure, copied verbatim into each repository.
 */
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function decode(text) {
  return String(text).replace(/&(#x[0-9a-f]+|#\d+|amp|lt|gt|quot|apos|nbsp);/gi, (_, code) => {
    if (code[0] === '#') return String.fromCodePoint(code[1].toLowerCase() === 'x' ? parseInt(code.slice(2), 16) : Number(code.slice(1)));
    return { amp: '&', lt: '<', gt: '>', quot: '"', apos: "'", nbsp: ' ' }[code.toLowerCase()];
  });
}
function selectorParts(selector) { return selector.trim().split(/\s+(?=(?:[^\[\]]|\[[^\]]*\])*$)/); }
function simpleMatch(node, selector) {
  if (!node || node.nodeType !== 1) return false;
  let ok = true;
  selector = selector.replace(/:not\(([^)]+)\)/g, (_, excluded) => { if (simpleMatch(node, excluded)) ok = false; return ''; });
  selector = selector.replace(/\[([^\s=\]]+)(?:\s*=\s*["']?([^"'\]]*)["']?)?\]/g, (_, name, value) => {
    if (!node.hasAttribute(name) || (value !== undefined && node.getAttribute(name) !== value.trim())) ok = false;
    return '';
  });
  const tag = /^[A-Za-z][\w-]*|^\*/.exec(selector);
  if (tag && tag[0] !== '*' && node.localName !== tag[0].toLowerCase()) ok = false;
  for (const match of selector.matchAll(/([.#])([\w-]+)/g)) {
    if (match[1] === '#' ? node.id !== match[2] : !node.classList.contains(match[2])) ok = false;
  }
  return ok;
}
function matches(node, selector) {
  return selector.split(',').some(part => {
    const chain = selectorParts(part);
    if (!simpleMatch(node, chain.pop())) return false;
    let parent = node.parentElement;
    while (chain.length) {
      const item = chain.pop();
      while (parent && !simpleMatch(parent, item)) parent = parent.parentElement;
      if (!parent) return false;
      parent = parent.parentElement;
    }
    return true;
  });
}

class Element {
  constructor(tag, document, type = 1) {
    this.localName = tag.toLowerCase(); this.tagName = tag.toUpperCase(); this.nodeName = this.tagName;
    this.nodeType = type; this.ownerDocument = document; this.childNodes = []; this.parentNode = null;
    this.attributes = new Map(); this._props = {};
    this.style = { setProperty(k, v) { this[k] = String(v); }, getPropertyValue(k) { return this[k] || ''; } };
    this.listeners = new Map(); this._text = ''; this._value = null; this._context = null; this.rect = null;
    this.classList = {
      contains: value => this.className.split(/\s+/).includes(value),
      add: (...values) => { this.className = [...new Set(this.className.split(/\s+/).filter(Boolean).concat(values))].join(' '); },
      remove: (...values) => { this.className = this.className.split(/\s+/).filter(v => !values.includes(v)).join(' '); },
      toggle: (value, force) => { const on = force == null ? !this.classList.contains(value) : !!force; if (on) this.classList.add(value); else this.classList.remove(value); return on; }
    };
    this.dataset = new Proxy({}, {
      get: (_, key) => this.getAttribute('data-' + String(key).replace(/[A-Z]/g, c => '-' + c.toLowerCase())) ?? undefined,
      set: (_, key, value) => { this.setAttribute('data-' + String(key).replace(/[A-Z]/g, c => '-' + c.toLowerCase()), value); return true; }
    });
  }
  get id() { return this.getAttribute('id') || ''; } set id(v) { this.setAttribute('id', v); }
  get className() { return this.getAttribute('class') || ''; } set className(v) { this.setAttribute('class', v); }
  get parentElement() { return this.parentNode && this.parentNode.nodeType === 1 ? this.parentNode : null; }
  get children() { return this.childNodes.filter(c => c.nodeType === 1); }
  get firstChild() { return this.childNodes[0] || null; } get lastChild() { return this.childNodes.at(-1) || null; }
  get textContent() { return this.nodeType === 3 ? this._text : this.childNodes.map(c => c.textContent).join(''); }
  set textContent(v) { if (this.nodeType === 3) this._text = String(v); else { this.replaceChildren(); if (String(v)) this.appendChild(this.ownerDocument.createTextNode(String(v))); } }
  get innerHTML() { return this.textContent; }
  set innerHTML(v) { this.replaceChildren(); parseHTML(String(v), this, this.ownerDocument); }
  get value() {
    if (this._value != null) return this._value;
    if (this.localName === 'select') { const options = this.querySelectorAll('option'); const selected = options.find(o => o.hasAttribute('selected')) || options[0]; return selected ? selected.value : ''; }
    if (this.localName === 'textarea' || this.localName === 'option') return this.getAttribute('value') ?? this.textContent;
    return this.getAttribute('value') || '';
  }
  set value(v) {
    if (this.type === 'range' || this.type === 'number') {
      const num = Number(v);
      if (Number.isFinite(num)) {
        const lo = Number(this.min || -Infinity), hi = Number(this.max || Infinity);
        this._value = String(Math.min(hi, Math.max(lo, num)));
        return;
      }
    }
    this._value = String(v);
  }
  get type() { return this.getAttribute('type') || ''; } set type(v) { this.setAttribute('type', v); }
  get min() { return this.getAttribute('min') || ''; } set min(v) { this.setAttribute('min', v); }
  get max() { return this.getAttribute('max') || ''; } set max(v) { this.setAttribute('max', v); }
  get step() { return this.getAttribute('step') || ''; } set step(v) { this.setAttribute('step', v); }
  get hidden() { return this.hasAttribute('hidden'); } set hidden(v) { if (v) this.setAttribute('hidden', ''); else this.removeAttribute('hidden'); }
  get disabled() { return this.hasAttribute('disabled'); } set disabled(v) { if (v) this.setAttribute('disabled', ''); else this.removeAttribute('disabled'); }
  get checked() { return this._checked != null ? this._checked : this.hasAttribute('checked'); }
  set checked(v) { this._checked = !!v; if (v) this.setAttribute('checked', ''); else this.removeAttribute('checked'); }
  get width() { return Number(this.getAttribute('width') || 300); } set width(v) { this.setAttribute('width', v); }
  get height() { return Number(this.getAttribute('height') || 150); } set height(v) { this.setAttribute('height', v); }
  get offsetWidth() { return this.rect?.width || 190; } get offsetHeight() { return this.rect?.height || 130; }
  get isConnected() { let node = this; while (node.parentNode) node = node.parentNode; return node === this.ownerDocument; }
  setAttribute(k, v) { this.attributes.set(String(k), String(v)); }
  getAttribute(k) { return this.attributes.has(k) ? this.attributes.get(k) : null; }
  hasAttribute(k) { return this.attributes.has(k); } removeAttribute(k) { this.attributes.delete(k); }
  appendChild(child) { if (child.parentNode) child.remove(); child.parentNode = this; this.childNodes.push(child); return child; }
  append(...children) { children.forEach(c => this.appendChild(typeof c === 'string' ? this.ownerDocument.createTextNode(c) : c)); }
  removeChild(child) { const index = this.childNodes.indexOf(child); if (index < 0) throw new Error('Child not found'); this.childNodes.splice(index, 1); child.parentNode = null; return child; }
  replaceChildren(...children) { this.childNodes.forEach(c => { c.parentNode = null; }); this.childNodes = []; this.append(...children); }
  remove() { if (this.parentNode) this.parentNode.removeChild(this); }
  contains(node) { return node === this || this.childNodes.some(c => c.contains(node)); }
  querySelectorAll(selector) { const found = []; const walk = node => { for (const child of node.childNodes) { if (matches(child, selector)) found.push(child); walk(child); } }; walk(this); return found; }
  querySelector(selector) { return this.querySelectorAll(selector)[0] || null; }
  closest(selector) { for (let node = this; node; node = node.parentElement) if (matches(node, selector)) return node; }
  addEventListener(type, callback, options) { if (!this.listeners.has(type)) this.listeners.set(type, []); this.listeners.get(type).push({ callback, once: !!options?.once }); }
  removeEventListener(type, callback) { this.listeners.set(type, (this.listeners.get(type) || []).filter(item => item.callback !== callback)); }
  dispatchEvent(event) {
    if (!event.target) event.target = this;
    event.preventDefault ||= function () { this.defaultPrevented = true; };
    event.stopPropagation ||= function () { this.stopped = true; };
    for (let node = this; node; node = event.bubbles === false || event.stopped ? null : node.parentNode) {
      event.currentTarget = node;
      for (const item of [...(node.listeners.get(event.type) || [])]) { item.callback.call(node, event); if (item.once) node.removeEventListener(event.type, item.callback); }
    }
    return !event.defaultPrevented;
  }
  click() {
    if (this.disabled) return;
    if (this.type === 'checkbox') {
      // HTML 'checked' initializes the property; a click toggles it.
      if (this._value == null && this.hasAttribute('checked')) this._value = 'on';
      this.checked = !this.checked;
    }
    this.dispatchEvent({ type: 'click', bubbles: true, button: 0 });
    if (this.type === 'checkbox') this.dispatchEvent({ type: 'change', bubbles: true });
    if (this.localName === 'a' && this.download) this.ownerDocument.downloads.push({ filename: this.download, href: this.href });
  }
  focus() { if (!this.disabled) { this.ownerDocument.activeElement = this; this.dispatchEvent({ type: 'focusin', bubbles: true }); } }
  blur() { this.dispatchEvent({ type: 'focusout', bubbles: true }); }
  select() { this.ownerDocument.selection = this.value; }
  scrollIntoView() {} setPointerCapture() {} releasePointerCapture() {}
  getBoundingClientRect() {
    if (this.rect) return { left: 0, top: 0, ...this.rect, right: this.rect.width, bottom: this.rect.height };
    const box = (this.getAttribute('viewBox') || '').split(/\s+/).map(Number);
    const width = box[2] || (this.localName === 'canvas' ? 880 : 540);
    const height = box[3] || (this.localName === 'canvas' ? width * 9 / 16 : 260);
    return { left: 0, top: 0, width, height, right: width, bottom: height };
  }
  getContext(kind) {
    if (this.localName !== 'canvas' || kind !== '2d') return null;
    if (this._context) return this._context;
    const calls = [];
    const gradient = { addColorStop() {} };
    const special = {
      calls, canvas: this,
      createImageData: (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
      getImageData: (x, y, w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
      measureText: text => ({ width: String(text).length * 7, actualBoundingBoxAscent: 8, actualBoundingBoxDescent: 2 }),
      createLinearGradient: () => gradient, createRadialGradient: () => gradient,
      createConicGradient: () => gradient, createPattern: () => ({}),
      isPointInPath: () => false
    };
    const state = {};
    const context = new Proxy(special, {
      get(target, name) {
        if (name in target) return target[name];
        if (name in state) return state[name];
        return (...args) => {
          if (args.some(arg => typeof arg === 'number' && !Number.isFinite(arg))) throw new Error('Nonfinite canvas coordinate in ' + String(name));
          calls.push({ name: String(name), args });
          if (calls.length > 6000) calls.splice(0, 2000);
          return undefined;
        };
      },
      set(target, name, value) { state[name] = value; return true; }
    });
    this._context = context; return context;
  }
}

class Document extends Element {
  constructor() { super('document', null, 9); this.ownerDocument = this; this.readyState = 'complete'; this.hidden = false; this.activeElement = null; this.downloads = []; }
  createElement(tag) { return new Element(tag, this); }
  createElementNS(namespace, tag) { const node = new Element(tag, this); node.namespaceURI = namespace; return node; }
  createTextNode(text) { const node = new Element('#text', this, 3); node._text = String(text); return node; }
  getElementById(id) { return this.querySelector('#' + id); }
  execCommand() { return true; }
  get documentElement() { return this.querySelector('html'); }
  get body() { return this.querySelector('body'); }
}

function parseHTML(html, host, document) {
  const voidTags = new Set(['area', 'base', 'br', 'col', 'embed', 'hr', 'img', 'input', 'link', 'meta', 'param', 'source', 'track', 'wbr']);
  const stack = [host];
  const token = /<!--[\s\S]*?-->|<![^>]*>|<\/(\w[\w:-]*)\s*>|<(\w[\w:-]*)([^>]*)>|([^<]+)/g;
  let match;
  while ((match = token.exec(html))) {
    if (match[1]) { const tag = match[1].toLowerCase(); let index = stack.length - 1; while (index > 0 && stack[index].localName !== tag) index--; if (index > 0) stack.length = index; }
    else if (match[2]) {
      const tag = match[2].toLowerCase(); const node = document.createElement(tag);
      for (const attr of match[3].matchAll(/([^\s=/>]+)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s>]+)))?/g)) {
        const value = attr[2] ?? attr[3] ?? attr[4];
        // Boolean attributes such as checked, disabled and selected have no
        // value in HTML; setAttribute stores an empty string for them.
        node.setAttribute(attr[1], value === undefined ? '' : decode(value));
      }
      stack.at(-1).appendChild(node);
      if (tag === 'script' || tag === 'style') { const end = html.indexOf('</' + tag, token.lastIndex); if (end >= 0) { node.textContent = html.slice(token.lastIndex, end); token.lastIndex = html.indexOf('>', end) + 1; } }
      else if (!voidTags.has(tag) && !match[3].endsWith('/')) stack.push(node);
    } else if (match[4]) stack.at(-1).appendChild(document.createTextNode(decode(match[4])));
  }
}

function boot(directory, options = {}) {
  const document = new Document();
  const html = fs.readFileSync(path.join(directory, 'index.html'), 'utf8');
  parseHTML(html, document, document);
  const ids = document.querySelectorAll('[id]').map(node => node.id);
  if (new Set(ids).size !== ids.length) throw new Error('Duplicate HTML IDs');
  const css = fs.readFileSync(path.join(directory, 'style.css'), 'utf8');
  const cssValues = text => Object.fromEntries([...text.matchAll(/(--[\w-]+)\s*:\s*([^;\n}]+)/g)].map(m => [m[1], m[2].trim()]));
  const light = cssValues((/:root\s*\{([^}]*)\}/.exec(css) || [])[1] || '');
  const dark = cssValues((/:root\[data-theme="dark"\]\s*\{([^}]*)\}/.exec(css) || [])[1] || '');
  const animation = new Map(), timers = new Map(), intervals = new Map(), blobs = new Map(), storage = new Map();
  const errors = [], copied = []; let next = 1;
  const sandbox = {
    document, navigator: { clipboard: { writeText: text => { copied.push(text); return Promise.resolve(); } }, userAgent: 'node-dom-harness' },
    devicePixelRatio: options.dpr || 1, innerWidth: options.width || 1440, innerHeight: options.height || 900,
    console: { log() {}, info() {}, debug() {}, warn: (...args) => errors.push(args.join(' ')), error: (...args) => errors.push(args.join(' ')) },
    getComputedStyle: () => ({ getPropertyValue: key => (document.documentElement.getAttribute('data-theme') === 'dark' ? dark[key] || light[key] : light[key]) || '' }),
    localStorage: { getItem: key => { if (options.storageBlocked) throw new Error('Storage unavailable'); return storage.get(key) || null; }, setItem: (key, value) => { if (options.storageBlocked) throw new Error('Storage unavailable'); storage.set(key, value); }, removeItem: key => storage.delete(key) },
    requestAnimationFrame: callback => { const id = next++; animation.set(id, callback); return id; },
    cancelAnimationFrame: id => animation.delete(id),
    setTimeout: (callback, ms) => { const id = next++; timers.set(id, callback); return id; }, clearTimeout: id => timers.delete(id),
    setInterval: (callback, ms) => { const id = next++; intervals.set(id, callback); return id; }, clearInterval: id => intervals.delete(id),
    URL: { createObjectURL: blob => { const name = 'blob:local-' + next++; blobs.set(name, blob); return name; }, revokeObjectURL: name => blobs.delete(name) },
    Blob, print() {}, matchMedia: () => ({ matches: false, addEventListener() {}, removeEventListener() {} }),
    fetch: () => { throw new Error('Network access attempted'); }, performance: { now: () => 0 },
    alert: msg => errors.push('alert: ' + msg), confirm: () => true
  };
  const windowEvents = new Element('window', document);
  sandbox.addEventListener = windowEvents.addEventListener.bind(windowEvents);
  sandbox.removeEventListener = windowEvents.removeEventListener.bind(windowEvents);
  sandbox.dispatchEvent = windowEvents.dispatchEvent.bind(windowEvents);
  sandbox.window = sandbox; sandbox.globalThis = sandbox; sandbox.self = sandbox;
  const context = vm.createContext(sandbox);
  if (options.beforeBoot) options.beforeBoot({ document, sandbox });
  for (const script of document.querySelectorAll('script')) {
    const src = script.getAttribute('src');
    if (src && (!/^[\w./-]+\.(js|cjs)$/.test(src) || src.includes('..'))) throw new Error('Nonlocal script source: ' + src);
    const code = src ? fs.readFileSync(path.join(directory, src), 'utf8') : script.textContent;
    const compiled = new vm.Script(code, { filename: src || 'index-inline.js' });
    compiled.runInContext(context, { timeout: 20000 });
  }
  if (document.readyState === 'loading') { document.readyState = 'complete'; document.dispatchEvent({ type: 'DOMContentLoaded', bubbles: false }); }
  return {
    document, sandbox, errors, animation, timers, intervals, blobs, copied, html, css,
    get: id => { const node = document.getElementById(id); if (!node) throw new Error('Missing DOM ID: ' + id); return node; },
    event: (node, type, extra = {}) => node.dispatchEvent({ type, bubbles: true, ...extra }),
    key: (node, key, extra = {}) => { const event = { type: 'keydown', key, bubbles: true, ...extra }; node.dispatchEvent(event); if (!event.defaultPrevented && (key === 'Enter' || key === ' ') && node.localName === 'button') node.click(); return event; },
    windowEvent: (type, extra = {}) => windowEvents.dispatchEvent({ type, bubbles: false, ...extra }),
    resize: () => windowEvents.dispatchEvent({ type: 'resize', bubbles: false }),
    frames: count => { for (let frame = 0; frame < count; frame++) { const pending = [...animation.values()]; animation.clear(); pending.forEach(callback => callback(frame * 16)); } },
    flushTimers: () => { const pending = [...timers.values()]; timers.clear(); pending.forEach(callback => callback()); },
    dispose: () => { animation.clear(); timers.clear(); intervals.clear(); blobs.clear(); }
  };
}
module.exports = { boot, Element, Document, parseHTML };
