'use strict';
// Dashcam Connect web client: read-only, same pages and freshness rules as the
// Android app. CSP-compatible: no inline code, DOM built with textContent only.
(() => {
  const DELAYS = [1, 2, 4, 8, 15];
  const LIVENESS_MS = 15000, FETCH_MS = 5000;   // parity with the Android read/connect timeouts
  const GIB = 1073741824;
  const TOKEN = /^[A-Za-z0-9_-]{32,128}$/;
  const PAGES = { live: 'Live', vehicle: 'Vehicle', system: 'System', connection: 'Connection' };
  const WAKE_HINT = 'Monitoring pauses while this page is in the background.';
  const $ = id => document.getElementById(id);

  let identity = null, unpaired = false;      // identity = {id, name, capabilities}
  let ws = null, snap = null, lastMs = 0, openedMs = 0, lastSession = '', lastSeq = -1;
  let status = 'Starting', attempt = 0, retryTimer = 0, epoch = 0;
  let wakeWanted = false, wakeLock = null, wakeBusy = false;
  let message = null, wakeNote = null;
  const binds = [];

  // ---- helpers -------------------------------------------------------------
  function h(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    return e;
  }
  const isObj = x => x !== null && typeof x === 'object' && !Array.isArray(x);
  const num = x => (typeof x === 'number' && Number.isFinite(x) ? x : null);
  const per = (x, by) => (num(x) === null ? null : x / by);
  function fmt(x, decimals, unit) {
    const n = num(x);
    return n === null ? '—' : n.toFixed(decimals) + (unit ? ' ' + unit : '');
  }
  function setText(el, text) { if (el.textContent !== text) el.textContent = text; }
  function bind(el, fn) { binds.push(v => setText(el, fn(v))); return el; }
  function safeDecode(text) { try { return decodeURIComponent(text); } catch (_) { return ''; } }
  // A hung TCP/TLS handshake must end in the normal error path, not stall start().
  const deadline = () => (typeof AbortSignal !== 'undefined' && AbortSignal.timeout ? AbortSignal.timeout(FETCH_MS) : undefined);

  function card(title, ...kids) {
    const c = h('div', 'card');
    c.append(h('h2', null, title), ...kids);
    return c;
  }
  function metric(label, fn) {
    const row = h('div', 'metric');
    row.append(h('span', 'label', label), bind(h('span', 'value', '—'), fn));
    return row;
  }
  const hint = text => h('p', 'hint', text);
  const big = fn => bind(h('p', 'big', '—'), fn);
  function duo(a, b) { const d = h('div', 'duo'); d.append(a, b); return d; }
  function button(text, cls, onClick) {
    const b = h('button', cls, text);
    b.type = 'button';
    b.addEventListener('click', onClick);
    return b;
  }

  // ---- view model ----------------------------------------------------------
  function view() {
    const now = performance.now();
    const fresh = ws !== null && ws.readyState === WebSocket.OPEN && lastMs > 0 && now - lastMs <= 2000;
    const s = fresh ? snap : null;
    const runtime = s && s.runtime_fresh === true && isObj(s.runtime) ? s.runtime : null;
    const vehicle = runtime && runtime.bridge_fresh === true && isObj(runtime.vehicle) ? runtime.vehicle : null;
    const system = s && isObj(s.system) ? s.system : null;
    return { now, fresh, s, runtime, vehicle, system };
  }
  function stateText(v) {
    if (v.fresh) return v.runtime ? 'Live' : 'Jetson online · runtime unavailable';
    if (ws && lastMs > 0) return 'Data stale · waiting for Jetson';
    return status;
  }
  function detailText(v) {
    if (v.fresh) return v.runtime ? 'Updated ' + (Math.max(0, v.now - lastMs) / 1000).toFixed(1) + ' s ago'
      : 'System health available; recorder data has expired';
    return unpaired ? 'Open Connection to pair this browser' : '';
  }
  function render() { const v = view(); for (const b of binds) b(v); }
  function setStatus(text) { status = text; render(); }

  // ---- cards (labels, units and decimals match the Android app) -----------
  function speedCard() {
    const row = h('div', 'speed');
    row.append(bind(h('span', 'num', '—'), v => fmt(v.vehicle?.speed_kmh, 1, '')), h('span', 'unit', 'km/h'));
    const source = h('p', 'hint source');
    binds.push(v => {
      const raw = v.vehicle?.speed_source;
      const src = typeof raw === 'string' && raw ? raw.slice(0, 32) : 'Unavailable';
      const sim = v.vehicle !== null && src === 'SIMULATED';
      setText(source, !v.vehicle ? (v.runtime ? 'Waiting for a fresh vehicle source' : 'Waiting for recorder telemetry')
        : sim ? 'Source: SIMULATED · not real vehicle data' : 'Source: ' + src);
      source.classList.toggle('sim', sim);
    });
    return card('Vehicle speed', row, source, metric('Acceleration', v => fmt(v.vehicle?.acceleration_ms2, 2, 'm/s²')));
  }
  const recordingCard = () => card('Recording',
    big(v => (!v.runtime ? 'Unavailable' : v.runtime.recording === true ? 'Recording' : 'Waiting')),
    metric('Actual frame rate', v => fmt(v.runtime?.fps, 1, 'fps')),
    metric('Frames in session', v => fmt(v.runtime?.frames, 0, '')),
    metric('Bridge sample age', v => fmt(v.runtime?.bridge_age_ms, 0, 'ms')));
  const positionCard = () => card('GNSS position',
    metric('Latitude', v => fmt(v.vehicle?.latitude, 6, '°')),
    metric('Longitude', v => fmt(v.vehicle?.longitude, 6, '°')),
    metric('Heading', v => fmt(v.vehicle?.heading_deg, 1, '°')),
    metric('Altitude', v => fmt(v.vehicle?.altitude_m, 1, 'm')),
    metric('Satellites', v => fmt(v.vehicle?.satellites, 0, '')));
  const healthCard = () => card('Jetson health',
    metric('CPU utilization', v => fmt(v.system?.cpu_percent, 1, '%')),
    metric('GPU utilization', v => fmt(v.system?.gpu_percent, 1, '%')),
    metric('Uptime', v => fmt(per(v.system?.uptime_s, 3600), 1, 'h')),
    hint('Host measurements · unsupported counters display as —'));
  const adasCard = () => card('ADAS', h('p', 'big', 'Not available'),
    hint('Lane detection and driver monitoring are unavailable on this Jetson.'));
  const powertrainCard = () => card('Powertrain',
    metric('Engine speed', v => fmt(v.vehicle?.rpm, 0, 'rpm')),
    metric('Coolant', v => fmt(v.vehicle?.coolant_c, 1, '°C')),
    metric('Fuel level', v => fmt(v.vehicle?.fuel_percent, 1, '%')),
    metric('Throttle', v => fmt(v.vehicle?.throttle_percent, 1, '%')),
    metric('Engine load', v => fmt(v.vehicle?.engine_load_percent, 1, '%')));
  const linkCard = () => card('Telemetry link',
    big(v => (!v.runtime ? 'Runtime unavailable' : v.runtime.bridge_fresh === true ? 'Fresh' : 'No fresh bridge data')),
    metric('Bridge sample age', v => fmt(v.runtime?.bridge_age_ms, 0, 'ms')),
    hint('Missing or stale sensor readings display as —.'));
  const memoryCard = () => card('Memory & storage',
    metric('RAM used', v => fmt(per(v.system?.ram_used_bytes, GIB), 2, 'GiB')),
    metric('RAM total', v => fmt(per(v.system?.ram_total_bytes, GIB), 2, 'GiB')),
    metric('Swap used', v => fmt(per(v.system?.swap_used_bytes, GIB), 2, 'GiB')),
    metric('Swap total', v => fmt(per(v.system?.swap_total_bytes, GIB), 2, 'GiB')),
    metric('Footage disk free', v => fmt(per(v.system?.storage_free_bytes, GIB), 1, 'GiB')),
    metric('Footage disk total', v => fmt(per(v.system?.storage_total_bytes, GIB), 1, 'GiB')));
  function temperaturesCard() {
    const list = h('div', 'temps'), empty = hint('Temperature sensors unavailable');
    let signature = null, cells = [];
    binds.push(v => {
      const t = isObj(v.system?.temperatures_c) ? v.system.temperatures_c : null;
      const names = t ? Object.keys(t).sort().slice(0, 64) : [];
      const next = JSON.stringify(names);
      if (next !== signature) {      // rebuild rows only when the sensor set changes
        signature = next;
        const rows = names.map(name => {
          const row = h('div', 'metric'), value = h('span', 'value', '—');
          row.append(h('span', 'label', name.slice(0, 64)), value);
          return row;
        });
        cells = rows.map(row => row.lastChild);
        list.replaceChildren(...rows);
        empty.hidden = names.length > 0;
      }
      names.forEach((name, i) => setText(cells[i], fmt(t[name], 1, '°C')));
    });
    return card('Temperatures', empty, list);
  }

  // ---- connection page -----------------------------------------------------
  function connectionPage() {
    const steps = h('ol', 'steps');
    const step = (...parts) => {
      const li = h('li');
      parts.forEach((p, i) => li.append(i % 2 ? h('code', null, p) : document.createTextNode(p)));
      steps.append(li);
    };
    step('Install the Dashcam Connect CA on this device: on the Jetson run ', 'service.py ca --out dashcam-ca.crt',
      ' and check that its fingerprint matches.');
    step('On the Jetson run ', 'service.py pair --host <Jetson IP>', '.');
    step('Open the printed Browser link on this device, or paste it below.');
    const notPaired = card('This browser is not paired', hint('Browser access needs a session from the Jetson.'), steps);
    binds.push(() => { notPaired.hidden = !unpaired; });

    const device = card('This Jetson',
      metric('Device name', () => identity ? identity.name : '—'),
      metric('Device ID', () => identity ? identity.id.slice(0, 8) : '—'),
      metric('Connection', v => stateText(v)),
      metric('Last message', v => (ws && lastMs > 0 ? (Math.max(0, v.now - lastMs) / 1000).toFixed(1) + ' s ago' : '—')),
      metric('Invalid packets', v => fmt(v.s?.invalid_packets, 0, '')),
      metric('Session', v => (typeof v.s?.session === 'string' ? v.s.session.slice(0, 16) : '—')));

    const input = h('input', 'field');
    Object.assign(input, { id: 'pair-link', type: 'password', autocomplete: 'off', spellcheck: false, maxLength: 4096 });
    input.setAttribute('autocapitalize', 'off');
    const label = h('label', 'field-label', 'Pairing link (browser or Android link)');
    label.htmlFor = 'pair-link';
    const submit = () => { const token = extractToken(input.value); input.value = ''; pair(token); };
    input.addEventListener('keydown', e => { if (e.key === 'Enter') submit(); });
    message = h('p', 'message');
    message.setAttribute('aria-live', 'polite');
    const actions = h('div', 'actions');
    actions.append(button('Pair', 'btn', submit));
    const pairCard = card('Pair', hint('Paste the private link printed by the pair command. ' +
      'It contains the device credential: do not share it.'), label, input, actions, message);

    const signOutActions = h('div', 'actions');
    signOutActions.append(button('Sign out', 'btn secondary', signOut));
    const signOutCard = card('Sign out', hint('Removes this browser’s session. The Android app and other browsers stay paired.'),
      signOutActions);
    binds.push(() => { signOutCard.hidden = !identity; });

    const box = h('input');
    Object.assign(box, { type: 'checkbox', id: 'wake' });
    box.setAttribute('role', 'switch');
    const toggle = h('label', 'switch');
    toggle.htmlFor = 'wake';
    toggle.append(h('span', null, 'Keep screen on'), box);
    wakeNote = hint(WAKE_HINT);
    if (!('wakeLock' in navigator)) { box.disabled = true; wakeNote.textContent = 'Keeping the screen on is not supported by this browser.'; }
    box.addEventListener('change', () => { wakeWanted = box.checked; syncWake(); });
    return [notPaired, device, pairCard, signOutCard, card('Display', toggle, wakeNote)];
  }
  function say(text, error) {
    message.textContent = text;
    message.classList.toggle('error', !!error);
  }
  function extractToken(text) {
    text = text.trim();
    const at = text.indexOf('#pair=');
    if (/^https:\/\//i.test(text) && at >= 0) return safeDecode(text.slice(at + 6));
    if (/^dashcam:\/\/pair\?/i.test(text)) {
      return new URLSearchParams(text.slice(text.indexOf('?') + 1).split('#')[0]).get('token') || '';
    }
    return '';
  }

  // ---- pairing and session -------------------------------------------------
  async function pair(token) {
    if (!TOKEN.test(token)) { say('That is not a Dashcam Connect pairing link.', true); return start(); }
    say('Pairing…', false);
    let r = null;
    try {
      r = await fetch('/v1/session', { method: 'POST', headers: { Authorization: 'Bearer ' + token },
        credentials: 'same-origin', cache: 'no-store', signal: deadline() });
    } catch (_) {
      say('Could not reach the Jetson. Check that this device trusts the Dashcam Connect CA.', true);
    }
    if (r) {
      if (r.ok) say('This browser is paired.', false);
      else if (r.status === 401) say('Pairing rejected: the link does not match this Jetson’s identity.', true);
      else if (r.status === 403) say('Pairing refused: open this page at the address given to pair --host.', true);
      else say('Pairing failed (HTTP ' + r.status + ').', true);
    }
    return start();
  }
  async function signOut() {
    stop('Signing out');
    let ok = false;
    try {
      const r = await fetch('/v1/session', { method: 'DELETE', credentials: 'same-origin', cache: 'no-store',
        signal: deadline() });
      ok = r.ok;
    } catch (_) { /* reported below */ }
    if (!ok) { say('Could not reach the Jetson to sign out. Try again.', true); return start(); }
    identity = null; unpaired = true;
    say('Signed out. Open a new browser link to pair again.', false);
    setStatus('Not paired');
    showPage('connection');
  }
  async function checkIdentity() {
    let r;
    try {
      r = await fetch('/v1/identity', { credentials: 'same-origin', cache: 'no-store', signal: deadline() });
    } catch (_) { return 'error'; }
    if (r.status === 401 || r.status === 403) { identity = null; unpaired = true; return 'unpaired'; }
    if (!r.ok) return 'error';
    let j;
    try { j = await r.json(); } catch (_) { return 'error'; }
    if (!isObj(j) || typeof j.id !== 'string' || !/^[A-Fa-f0-9-]{36}$/.test(j.id)) return 'error';
    identity = { id: j.id, name: typeof j.name === 'string' && j.name.trim() ? j.name.slice(0, 80) : 'Jetson dashcam',
      capabilities: isObj(j.capabilities) ? j.capabilities : {} };
    unpaired = false;
    return 'ok';
  }

  // ---- stream --------------------------------------------------------------
  async function start() {
    const e = ++epoch;
    clearTimeout(retryTimer); retryTimer = 0;
    if (document.hidden) return setStatus('Disconnected');
    if (!ws) setStatus(identity ? 'Connecting' : 'Checking pairing');
    const result = await checkIdentity();
    if (e !== epoch) return;
    if (result === 'ok') openSocket();
    else if (result === 'unpaired') { closeSocket(); setStatus('Not paired'); showPage('connection'); }
    else { closeSocket(); retryLater(e, 'Jetson unreachable · retrying'); }
  }
  function retryLater(e, text) {
    setStatus(text);
    const delay = DELAYS[Math.min(attempt, DELAYS.length - 1)];
    attempt += 1;
    clearTimeout(retryTimer);
    retryTimer = setTimeout(() => { retryTimer = 0; if (e === epoch) start(); }, delay * 1000);
  }
  function stop(text) {
    epoch += 1;
    clearTimeout(retryTimer); retryTimer = 0;
    closeSocket();
    setStatus(text);
  }
  function closeSocket() {
    const sock = ws;
    ws = null; snap = null; lastMs = 0;
    if (sock) sock.close(1000);
  }
  function parseSnapshot(text) {
    if (typeof text !== 'string' || text.length > 65536 || !identity) return null;
    let j;
    try { j = JSON.parse(text); } catch (_) { return null; }
    if (!isObj(j) || j.schema !== 1 || j.type !== 'snapshot' || j.device_id !== identity.id) return null;
    if (!Number.isSafeInteger(j.seq) || j.seq < 0 || typeof j.session !== 'string' || !j.session.trim()) return null;
    return j;
  }
  function openSocket() {
    if (ws) return;
    lastSession = ''; lastSeq = -1;
    let sock;
    try {
      sock = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/v1/stream');
    } catch (_) { return retryLater(epoch, 'Connection lost · retrying'); }
    ws = sock; openedMs = performance.now();
    setStatus('Connecting');
    sock.addEventListener('message', ev => {
      if (ws !== sock) return;
      const data = parseSnapshot(ev.data);
      if (!data) { closeSocket(); return retryLater(epoch, 'Unsupported or invalid monitoring data · retrying'); }
      if (data.session === lastSession && data.seq <= lastSeq) return;
      lastSession = data.session; lastSeq = data.seq;
      snap = data; lastMs = performance.now(); attempt = 0;
      render();
    });
    sock.addEventListener('close', () => {
      if (ws !== sock) return;
      ws = null; snap = null; lastMs = 0;
      retryLater(epoch, 'Connection lost · retrying');   // start() re-checks /v1/identity first
    });
  }

  // Browsers never ping, so a dead peer (Jetson power cycle, silent Wi-Fi drop) leaves the
  // socket OPEN; a silent socket is closed and retried like a lost connection.
  function tick() {
    if (ws && performance.now() - (lastMs || openedMs) > LIVENESS_MS) {
      closeSocket();
      retryLater(epoch, 'Connection lost · retrying');
    }
    render();
  }

  // ---- wake lock -----------------------------------------------------------
  async function syncWake() {
    const want = () => wakeWanted && !document.hidden;
    if (!want()) {
      if (wakeLock) { const l = wakeLock; wakeLock = null; l.release().catch(() => {}); }
      return;
    }
    if (wakeLock || wakeBusy || !('wakeLock' in navigator)) return;
    wakeBusy = true;
    try {
      const l = await navigator.wakeLock.request('screen');
      l.addEventListener('release', () => { if (wakeLock === l) wakeLock = null; });
      if (want()) wakeLock = l; else l.release().catch(() => {});
      wakeNote.textContent = WAKE_HINT;
    } catch (_) {
      wakeNote.textContent = 'The browser refused to keep the screen on (for example in battery saver).';
    } finally { wakeBusy = false; }
  }

  // ---- navigation and startup ----------------------------------------------
  function showPage(name) {
    for (const tab of document.querySelectorAll('.tab')) {
      if (tab.dataset.page === name) tab.setAttribute('aria-current', 'page');
      else tab.removeAttribute('aria-current');
    }
    for (const page of Object.keys(PAGES)) $('page-' + page).hidden = page !== name;
    $('title').textContent = PAGES[name];
  }
  function pairFromHash() {
    if (!location.hash.startsWith('#pair=')) return false;
    const token = safeDecode(location.hash.slice(6));
    history.replaceState(null, '', location.pathname + location.search);   // strip the token at once
    pair(token);
    return true;
  }
  function init() {
    $('page-live').append(speedCard(), duo(recordingCard(), adasCard()), duo(positionCard(), healthCard()));
    $('page-vehicle').append(speedCard(), duo(powertrainCard(), positionCard()), linkCard());
    $('page-system').append(duo(healthCard(), memoryCard()), temperaturesCard(), adasCard());
    $('page-connection').append(...connectionPage());
    const banner = $('banner'), statusLine = $('status'), detail = $('detail'), subtitle = $('subtitle');
    binds.push(v => {
      setText(statusLine, stateText(v));
      setText(detail, detailText(v));
      banner.classList.toggle('ok', v.fresh);
      banner.classList.toggle('bad', !v.fresh && unpaired);
      setText(subtitle, identity ? identity.name : 'Your Jetson, at a glance');
    });
    for (const tab of document.querySelectorAll('.tab')) {
      tab.addEventListener('click', () => { showPage(tab.dataset.page); window.scrollTo(0, 0); });
    }
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) stop('Disconnected');
      else { attempt = 0; start(); }
      syncWake();
    });
    window.addEventListener('hashchange', pairFromHash);
    setInterval(tick, 250);
    if (!pairFromHash()) start();
  }
  init();
})();
