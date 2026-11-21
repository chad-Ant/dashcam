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
  // Each connection attempt (start(), a stop, a failed video upgrade) verifies the identity
  // afresh: video opens only once /v1/identity has answered for the current attempt, and an
  // answer from an older attempt is dropped before it touches identity.
  let verified = false, idGen = 0, idCheck = null;
  let ws = null, snap = null, lastMs = 0, openedMs = 0, lastSession = '', lastSeq = -1;
  let status = 'Starting', attempt = 0, retryTimer = 0, epoch = 0;
  let wakeWanted = false, wakeLock = null, wakeBusy = false;
  let message = null, wakeNote = null, page = 'live', signingOut = false;
  const binds = [];
  // Camera video (/v1/video): open only while live view is on, paired and verified, on Live,
  // the page visible and the camera box on screen.
  const VIDEO_STALE_MS = 2000, VIDEO_MAGIC = [0x44, 0x43, 0x56, 0x31];   // "DCV1"
  let vws = null, vAttempt = 0, vTimer = 0, vOpenedMs = 0, vMsgMs = 0, vLastSeq = 0;
  let vImg = null, vUrl = null, vLoadingUrl = null, vLoading = null, vPending = null;
  let vMeta = null, vShownMs = 0, vTimes = [];
  // Video also pauses while the camera box is scrolled fully out of view (after a short grace).
  const OFFSCREEN_GRACE_MS = 2000;
  let vOnScreen = true, vOffTimer = 0, updateCamera = () => {};
  // Live view starts off on every load and is never stored, so video only streams on demand
  // (a cloud relay bills by the byte); it stops by itself LIVE_VIEW_MAX_MS after the tap.
  const LIVE_VIEW_MAX_MS = 10 * 60 * 1000;
  let liveView = false, liveStartMs = 0, liveStartWall = 0, liveExpired = false, liveDevice = null;

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
  function cameraCard() {
    const box = h('div', 'video-box'), note = h('p', 'video-note', 'No video');
    vImg = h('img');
    vImg.alt = 'Dashcam camera';
    vImg.hidden = true;
    vImg.draggable = false;   // with pointer-events: none, the context menu and drags target the box, not the picture
    vImg.addEventListener('load', () => frameDone(true));
    vImg.addEventListener('error', () => frameDone(false));
    box.append(vImg, note);
    const caption = h('p', 'hint video-caption', '—');
    const toggle = button('Start live view', 'btn', toggleLiveView), actions = h('div', 'actions');
    actions.append(toggle);
    // A decoded frame refreshes only this card; everything else waits for render().
    updateCamera = (now = performance.now()) => {
      const open = vws !== null && vws.readyState === WebSocket.OPEN;
      const showing = open && vUrl !== null && vMeta !== null && now - vShownMs <= VIDEO_STALE_MS;
      if (vImg.hidden === showing) vImg.hidden = !showing;   // never keep showing a stale picture
      note.hidden = showing;
      setText(note, canVideo() && !liveView ? 'Live view off' : open && vShownMs > 0 ? 'Video stale' : 'No video');
      setText(caption, videoCaption(now, open, showing));
      setText(toggle, liveView ? 'Stop live view' : 'Start live view');
      toggle.setAttribute('aria-pressed', String(liveView));
      toggle.classList.toggle('secondary', liveView);
      toggle.disabled = !canVideo();
    };
    binds.push(v => updateCamera(v.now));
    // Any visible pixel counts as on screen; without IntersectionObserver the camera is always on screen.
    if (typeof IntersectionObserver === 'function') {
      new IntersectionObserver(entries => setOnScreen(entries[entries.length - 1].isIntersecting),
        { threshold: 0 }).observe(box);
    }
    return card('Camera', box, caption, actions);
  }
  function videoCaption(now, open, showing) {
    if (showing) {
      const span = vTimes.length > 1 ? vTimes[vTimes.length - 1] - vTimes[0] : 0;
      const fps = span > 0 ? fmt((vTimes.length - 1) * 1000 / span, 1, 'fps') : '— fps';
      return vMeta.width + '×' + vMeta.height + ' · ' + fps + ' · ' + vMeta.ageMs + ' ms old' + stopsIn(now);
    }
    if (!identity) return 'Pair this browser to see the recording camera';
    if (identity.capabilities.video !== true) return 'This Jetson does not offer camera video';
    if (!liveView) {
      return liveExpired ? 'Live view stopped after ' + LIVE_VIEW_MAX_MS / 60000 + ' min · tap Start live view to resume'
        : 'Live view off · tap Start live view to watch';
    }
    if (!open) return (vAttempt > 0 ? 'Video connection lost · retrying' : 'Connecting to the camera') + stopsIn(now);
    if (vShownMs > 0) return 'Last frame ' + (Math.max(0, now - vShownMs) / 1000).toFixed(1) + ' s ago' + stopsIn(now);
    return 'Waiting for frames · the Jetson must have MonitoringVideoEnabled set' + stopsIn(now);
  }
  function stopsIn(now) {
    const left = LIVE_VIEW_MAX_MS - liveElapsed(now);
    return ' · stops in ' + (left < 60000 ? '< 1 min' : Math.ceil(left / 60000) + ' min');
  }
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

  // ---- vehicle map ---------------------------------------------------------
  // A small slippy map: OpenStreetMap tiles as <img> (pixels are never read, so no CORS),
  // fetched straight from OSM only while Vehicle is visible and a fix has been received:
  // the viewport plus half a tile, at most four at a time, a failed tile again after 30 s.
  const TILE = 256, MIN_ZOOM = 3, MAX_ZOOM = 19, DEFAULT_ZOOM = 16, MAX_LAT = 85.0511;
  const TILE_URL = 'https://tile.openstreetmap.org/', ATTRIBUTION_URL = 'https://www.openstreetmap.org/copyright';
  const TILE_INFLIGHT = 4, TILE_RETRY_MS = 30000, TILE_TIMEOUT_MS = 15000, TILE_SETTLE_MS = 200, TILE_KEEP = 128;
  const MAP_MARGIN = TILE / 2, DRAG_SLOP = 6, WHEEL_STEP = 100, WHEEL_NOTCH_PX = 40, WHEEL_MIN_GAP_MS = 150;
  const clampZoom = z => Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, z));
  const clamp01 = x => Math.min(1, Math.max(0, x));
  const wrap01 = x => x - Math.floor(x);
  // Web Mercator, normalised to 0..1 (x east, y south); null for unusable coordinates.
  function mapFix(vehicle) {
    const lat = num(vehicle?.latitude), lon = num(vehicle?.longitude);
    if (lat === null || lon === null || Math.abs(lat) > MAX_LAT || Math.abs(lon) > 180 || (lat === 0 && lon === 0)) {
      return null;
    }
    const r = lat * Math.PI / 180, heading = num(vehicle.heading_deg);
    return { x: (lon + 180) / 360, y: (1 - Math.log(Math.tan(Math.PI / 4 + r / 2)) / Math.PI) / 2,
      heading: heading === null ? null : ((heading % 360) + 360) % 360 };
  }
  function mapCard() {
    const box = h('div', 'map-box empty'), layer = h('div', 'map-layer');
    box.setAttribute('role', 'group');
    box.setAttribute('aria-label', 'Map of the car position');
    const marker = h('div', 'map-marker'), arrow = h('div', 'map-arrow');
    const label = h('span', 'map-label', 'Last known position');
    marker.append(arrow, h('div', 'map-dot'), label);
    marker.hidden = true;
    const note = h('p', 'map-note', 'No GNSS fix yet'), offline = h('p', 'map-offline', 'Map tiles need internet');
    offline.hidden = true;
    const zoomIn = button('+', 'map-btn', () => zoomStep(1)), zoomOut = button('−', 'map-btn', () => zoomStep(-1));
    zoomIn.setAttribute('aria-label', 'Zoom in');
    zoomOut.setAttribute('aria-label', 'Zoom out');
    const recentre = button('Recentre', 'map-btn map-recentre', () => { follow = true; requestDraw(); render(); });
    recentre.setAttribute('aria-label', 'Recentre on the car');
    const controls = h('div', 'map-controls');
    controls.append(zoomIn, zoomOut);
    const link = h('a', null, '© OpenStreetMap contributors');
    Object.assign(link, { href: ATTRIBUTION_URL, target: '_blank', rel: 'noopener noreferrer' });
    const attribution = h('p', 'map-attrib');
    attribution.append(link);
    box.append(layer, marker, note, offline, controls, recentre, attribution);
    const caption = h('p', 'hint');

    // fix: last usable position, client memory only (dropped when unpaired or the Jetson changes).
    let fix = null, fixDevice = null, live = false, follow = true, wanted = false;
    let cx = 0.5, cy = 0.5, zoom = DEFAULT_ZOOM, tileZoom = -1, tileZoomMs = 0, rafId = 0;
    let wheelAcc = 0, wheelMs = 0, wheelStepMs = 0;
    const tiles = new Map();      // "z/x/y" -> {z, x, y, el, img, state: idle|loading|ok|failed, ...}
    const pointers = new Map();   // pointerId -> {x, y, sx, sy}

    binds.push(v => {
      if (!identity || (fixDevice !== null && fixDevice !== identity.id)) {
        fix = null; fixDevice = null; follow = true;
      }
      const f = identity ? mapFix(v.vehicle) : null;   // v.vehicle is null unless fresh and bridge_fresh
      live = f !== null;
      if (f) { fix = f; fixDevice = identity.id; }
      wanted = fix !== null && page === 'vehicle' && !document.hidden;
      setText(caption, fix === null ? 'The map appears with the first GNSS fix'
        : !live ? 'No fresh position · showing the last known position'
          : follow ? 'Following the car' : 'Free view · Recentre to follow the car');
      draw(v.now);
    });

    function requestDraw() {
      if (!rafId) rafId = requestAnimationFrame(() => { rafId = 0; draw(performance.now()); });
    }
    function draw(now) {
      const W = box.clientWidth, H = box.clientHeight;
      box.classList.toggle('empty', fix === null);
      note.hidden = fix !== null;
      layer.hidden = fix === null;
      zoomIn.disabled = fix === null || zoom >= MAX_ZOOM;
      zoomOut.disabled = fix === null || zoom <= MIN_ZOOM;
      recentre.hidden = fix === null || follow;
      if (!wanted || W === 0 || H === 0) {
        for (const t of tiles.values()) if (t.state === 'loading') cancel(t);
        marker.hidden = true; offline.hidden = true;
        return;
      }
      if (follow) { cx = fix.x; cy = fix.y; }
      const scale = TILE * 2 ** zoom, z = Math.round(zoom), n = 2 ** z;
      if (z !== tileZoom) { tileZoom = z; tileZoomMs = now; }
      const hw = (W / 2 + MAP_MARGIN) / scale, hh = (H / 2 + MAP_MARGIN) / scale;
      const x0 = Math.floor((cx - hw) * n), x1 = Math.floor((cx + hw) * n);
      const y0 = Math.max(0, Math.floor((cy - hh) * n)), y1 = Math.min(n - 1, Math.floor((cy + hh) * n));
      const need = [];
      for (let y = y0; y <= y1; y += 1) {
        for (let x = x0; x <= x1; x += 1) {
          const key = z + '/' + x + '/' + y;
          let t = tiles.get(key);
          if (!t) { t = newTile(z, x, y); tiles.set(key, t); }
          t.usedMs = now;
          need.push(t);
        }
      }
      const cxt = cx * n, cyt = cy * n;   // nearest to the centre loads first
      const away = t => (t.x + .5 - cxt) ** 2 + (t.y + .5 - cyt) ** 2;
      need.sort((a, b) => away(a) - away(b));
      const needed = new Set(need), complete = need.every(t => t.state === 'ok');
      for (const t of tiles.values()) {
        if (!needed.has(t) && t.state === 'loading') cancel(t);
        if (t.state === 'loading' && now - t.startedMs > TILE_TIMEOUT_MS) fail(t, now);
        const s = scale / 2 ** t.z;
        const l = Math.round(W / 2 + t.x * s - cx * scale), r = Math.round(W / 2 + (t.x + 1) * s - cx * scale);
        const tp = Math.round(H / 2 + t.y * s - cy * scale), b = Math.round(H / 2 + (t.y + 1) * s - cy * scale);
        // Until this zoom is fully loaded, loaded tiles of nearby zooms fill the gaps (never fetched for it).
        const show = needed.has(t) || (!complete && t.state === 'ok' && Math.abs(t.z - z) <= 2 &&
          r > 0 && l < W && b > 0 && tp < H);
        if (!show) { if (t.el.parentNode) t.el.remove(); continue; }
        place(t, l, tp, r - l, b - tp, t.z === z ? 10 : 5 - Math.abs(t.z - z));
        if (t.el.parentNode !== layer) layer.append(t.el);
      }
      if (now - tileZoomMs >= TILE_SETTLE_MS) {   // no requests while a zoom is still changing
        let inflight = 0;
        for (const t of tiles.values()) if (t.state === 'loading') inflight += 1;
        for (const t of need) {
          if (inflight >= TILE_INFLIGHT) break;
          if (t.state !== 'idle' && !(t.state === 'failed' && now - t.failedMs >= TILE_RETRY_MS)) continue;
          load(t, now, n);
          inflight += 1;
        }
      }
      offline.hidden = !need.some(t => t.state === 'failed');
      evict(needed);
      let ddx = fix.x - cx;
      ddx -= Math.round(ddx);   // shortest way round: the copy of the car nearest the view (±180°)
      const mx = Math.round(W / 2 + ddx * scale), my = Math.round(H / 2 + (fix.y - cy) * scale);
      marker.hidden = mx < -48 || mx > W + 48 || my < -48 || my > H + 48;
      marker.style.transform = 'translate(' + mx + 'px, ' + my + 'px)';
      marker.classList.toggle('stale', !live);
      label.hidden = live;
      arrow.hidden = !live || fix.heading === null;
      if (!arrow.hidden) arrow.style.transform = 'rotate(' + fix.heading.toFixed(1) + 'deg)';
    }
    function newTile(z, x, y) {
      return { z, x, y, el: h('div', 'map-tile'), img: null, state: 'idle', startedMs: 0, failedMs: 0, usedMs: 0,
        geometry: '' };
    }
    function place(t, left, top, width, height, z) {
      const geometry = left + ',' + top + ',' + width + ',' + height + ',' + z;
      if (geometry === t.geometry) return;
      t.geometry = geometry;
      Object.assign(t.el.style, { left: left + 'px', top: top + 'px', width: width + 'px', height: height + 'px',
        zIndex: String(z) });
    }
    function load(t, now, n) {
      cancel(t);
      const img = h('img');   // a fresh element per attempt: events of an aborted one are ignored
      img.alt = '';
      img.draggable = false;
      img.decoding = 'async';
      img.hidden = true;
      // The document policy is no-referrer; OSM's tile policy wants a Referer (only the page origin is sent).
      img.referrerPolicy = 'strict-origin-when-cross-origin';
      img.addEventListener('load', () => {
        if (t.img === img && t.state === 'loading') { t.state = 'ok'; img.hidden = false; requestDraw(); }
      });
      img.addEventListener('error', () => {
        if (t.img === img && t.state === 'loading') { fail(t, performance.now()); requestDraw(); }
      });
      t.img = img; t.state = 'loading'; t.startedMs = now;
      t.el.classList.remove('failed');
      t.el.replaceChildren(img);
      img.src = TILE_URL + t.z + '/' + (((t.x % n) + n) % n) + '/' + t.y + '.png';
    }
    function cancel(t) {             // drops the picture; removing src aborts a pending request
      const img = t.img;
      t.img = null; t.state = 'idle';
      if (img) { img.removeAttribute('src'); img.remove(); }
    }
    function fail(t, now) {
      cancel(t);
      t.state = 'failed'; t.failedMs = now;
      t.el.classList.add('failed');
    }
    function evict(needed) {
      if (tiles.size <= TILE_KEEP) return;
      const old = [...tiles.entries()].filter(([, t]) => !needed.has(t)).sort((a, b) => a[1].usedMs - b[1].usedMs);
      for (const [key, t] of old.slice(0, tiles.size - TILE_KEEP)) {
        cancel(t);
        t.el.remove();
        tiles.delete(key);
      }
    }

    // ---- gestures: any pan or zoom gesture stops following; +/- keep following the car
    const at = e => { const r = box.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top }; };
    function panBy(dx, dy) {
      const s = TILE * 2 ** zoom;
      cx = wrap01(cx - dx / s); cy = clamp01(cy - dy / s);   // x wraps like the tiles do
    }
    function zoomAround(next, px, py) {           // keeps the map point under (px, py) in place
      next = clampZoom(next);
      if (next === zoom) return false;
      const dx = px - box.clientWidth / 2, dy = py - box.clientHeight / 2;
      const s0 = TILE * 2 ** zoom, s1 = TILE * 2 ** next;
      cx = wrap01(cx + dx / s0 - dx / s1); cy = clamp01(cy + dy / s0 - dy / s1);
      zoom = next;
      return true;
    }
    function zoomStep(step) {
      if (fix !== null && zoomAround(Math.round(zoom) + step, box.clientWidth / 2, box.clientHeight / 2)) requestDraw();
    }
    function goFree() {
      if (!follow) return;
      follow = false;
      render();
    }
    box.addEventListener('pointerdown', e => {
      if (fix === null || pointers.size >= 2 || e.target.closest('button, a')) return;
      if (e.pointerType === 'mouse' && e.button !== 0) return;
      try { box.setPointerCapture(e.pointerId); } catch (_) { /* pointer already gone */ }
      const p = at(e);
      pointers.set(e.pointerId, { x: p.x, y: p.y, sx: p.x, sy: p.y });
      box.classList.add('dragging');
      e.preventDefault();
    });
    box.addEventListener('pointermove', e => {
      const prev = pointers.get(e.pointerId);
      if (!prev || fix === null) return;
      const p = at(e);
      if (pointers.size === 1) {
        if (follow && Math.hypot(p.x - prev.sx, p.y - prev.sy) < DRAG_SLOP) return;   // a tap is not a pan
        goFree();
        panBy(p.x - prev.x, p.y - prev.y);
        prev.x = p.x; prev.y = p.y;
      } else {
        const [a0, b0] = [...pointers.values()].map(q => ({ x: q.x, y: q.y }));
        prev.x = p.x; prev.y = p.y;
        const [a1, b1] = [...pointers.values()];
        const d0 = Math.hypot(a0.x - b0.x, a0.y - b0.y), d1 = Math.hypot(a1.x - b1.x, a1.y - b1.y);
        const mx = (a1.x + b1.x) / 2, my = (a1.y + b1.y) / 2;
        goFree();
        panBy(mx - (a0.x + b0.x) / 2, my - (a0.y + b0.y) / 2);
        if (d0 > 0 && d1 > 0) zoomAround(zoom + Math.log2(d1 / d0), mx, my);
      }
      requestDraw();
    });
    const lift = e => {
      if (!pointers.delete(e.pointerId)) return;
      if (pointers.size === 0) box.classList.remove('dragging');
    };
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) box.addEventListener(type, lift);
    // One zoom level per wheel notch (a line/page event or one large pixel event) or per 100 px of
    // trackpad scroll, at most one level every 150 ms so trackpad momentum cannot zoom several at once.
    box.addEventListener('wheel', e => {
      if (fix === null) return;
      e.preventDefault();
      const now = performance.now();
      if (now - wheelMs > 300) wheelAcc = 0;
      wheelMs = now;
      const mode = e.deltaMode;                     // read deltaMode first so Firefox keeps line mode
      const px = e.deltaY * (mode === 1 ? 40 : mode === 2 ? 800 : 1);
      wheelAcc += px;
      const notch = mode !== 0 || Math.abs(px) >= WHEEL_NOTCH_PX;
      if (!notch && Math.abs(wheelAcc) < WHEEL_STEP) return;
      if (now - wheelStepMs < WHEEL_MIN_GAP_MS) return;
      const step = wheelAcc < 0 ? 1 : -1, p = at(e);
      wheelAcc = 0;
      wheelStepMs = now;
      if (zoomAround(Math.round(zoom) + step, p.x, p.y)) { goFree(); requestDraw(); }
    }, { passive: false });

    return card('Map', box, caption);
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
    const submit = () => {
      const link = extractToken(input.value);
      input.value = '';
      // A link for another Jetson must never send its credential here: this page's
      // own agent would receive another device's token.
      if (link.target && link.target !== location.host.toLowerCase()) {
        say('This link is for ' + link.target + ', but this page is ' + location.host +
            '. Nothing was sent: open the link on that address instead.', true);
        return;
      }
      pair(link.token);
    };
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
  // The token and the host[:port] the link was made for ('' when unknown or not a link).
  function extractToken(text) {
    text = text.replace(/\s+/g, '');   // terminals wrap long links; whitespace is never part of one
    const at = text.indexOf('#pair=');
    if (/^https:\/\//i.test(text) && at >= 0) {
      let target = '';
      try { target = new URL(text).host.toLowerCase(); } catch (_) { return { token: '', target: '' }; }
      return { token: safeDecode(text.slice(at + 6)), target };
    }
    if (/^dashcam:\/\/pair\?/i.test(text)) {
      const q = new URLSearchParams(text.slice(text.indexOf('?') + 1).split('#')[0]);
      const host = (q.get('host') || '').toLowerCase(), port = q.get('port') || '443';
      const shown = host.includes(':') ? '[' + host + ']' : host;   // as location.host writes IPv6
      return { token: q.get('token') || '', target: host ? (port === '443' ? shown : shown + ':' + port) : '' };
    }
    return { token: '', target: '' };
  }

  // ---- pairing and session -------------------------------------------------
  async function pair(token) {
    if (!TOKEN.test(token)) { say('That is not a Dashcam Connect pairing link.', true); return start(); }
    resetLiveView(); syncVideo();
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
    resetLiveView();
    signingOut = true; syncVideo();
    let ok = false;
    try {
      const r = await fetch('/v1/session', { method: 'DELETE', credentials: 'same-origin', cache: 'no-store',
        signal: deadline() });
      ok = r.ok;
    } catch (_) { /* reported below */ }
    signingOut = false;
    if (!ok) { say('Could not reach the Jetson to sign out. Try again.', true); return start(); }
    identity = null; unpaired = true;
    syncVideo();
    say('Signed out. Open a new browser link to pair again.', false);
    setStatus('Not paired');
    showPage('connection');
  }
  // Reads /v1/identity without touching shared state: {result: ok|unpaired|error, identity}.
  async function fetchIdentity() {
    let r;
    try {
      r = await fetch('/v1/identity', { credentials: 'same-origin', cache: 'no-store', signal: deadline() });
    } catch (_) { return { result: 'error' }; }
    if (r.status === 401 || r.status === 403) return { result: 'unpaired' };
    if (!r.ok) return { result: 'error' };
    let j;
    try { j = await r.json(); } catch (_) { return { result: 'error' }; }
    if (!isObj(j) || typeof j.id !== 'string' || !/^[A-Fa-f0-9-]{36}$/.test(j.id)) return { result: 'error' };
    return { result: 'ok', identity: { id: j.id,
      name: typeof j.name === 'string' && j.name.trim() ? j.name.slice(0, 80) : 'Jetson dashcam',
      capabilities: isObj(j.capabilities) ? j.capabilities : {} } };
  }
  function newAttempt() { idGen += 1; idCheck = null; verified = false; }
  // One in-flight check per attempt, shared by telemetry and video; it resolves to 'stale' (and
  // changes nothing) once a newer attempt has started.
  function checkIdentity() {
    if (idCheck) return idCheck;
    const gen = idGen;
    verified = false;
    idCheck = fetchIdentity().then(({ result, identity: confirmed }) => {
      if (gen !== idGen) return 'stale';
      idCheck = null;
      if (result === 'unpaired') { identity = null; unpaired = true; }
      else if (result === 'ok') { identity = confirmed; unpaired = false; verified = true; }
      return result;
    });
    return idCheck;
  }

  // ---- stream --------------------------------------------------------------
  // A telemetry retry joins a check still in flight: it began after the last start()/stop(), so
  // it belongs to this attempt, and two failing sockets then share one /v1/identity request.
  async function start(fromRetry) {
    const e = ++epoch;
    clearTimeout(retryTimer); retryTimer = 0;
    if (fromRetry && idCheck) verified = false;
    else newAttempt();
    if (document.hidden) return setStatus('Disconnected');
    if (!ws) setStatus(identity ? 'Connecting' : 'Checking pairing');
    const result = await checkIdentity();
    if (e !== epoch) return;
    if (result === 'ok') openSocket();
    else if (result === 'unpaired') { closeSocket(); setStatus('Not paired'); showPage('connection'); }
    else { closeSocket(); retryLater(e, 'Jetson unreachable · retrying'); }
    syncVideo();   // after retryLater, so video waits for telemetry's next check instead of starting one
  }
  function retryLater(e, text) {
    setStatus(text);
    const delay = DELAYS[Math.min(attempt, DELAYS.length - 1)];
    attempt += 1;
    clearTimeout(retryTimer);
    retryTimer = setTimeout(() => { retryTimer = 0; if (e === epoch) start(true); }, delay * 1000);
  }
  function stop(text) {
    epoch += 1;
    clearTimeout(retryTimer); retryTimer = 0;
    newAttempt();
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
    const now = performance.now();
    if (ws && now - (lastMs || openedMs) > LIVENESS_MS) {
      closeSocket();
      retryLater(epoch, 'Connection lost · retrying');
    }
    // A silent video socket is normal while video is disabled on the Jetson; it only counts as
    // a dead peer when the telemetry stream is not fresh either.
    const telemetryFresh = ws !== null && lastMs > 0 && now - lastMs <= 2000;
    if (vws && !telemetryFresh && now - (vMsgMs || vOpenedMs) > LIVENESS_MS) {
      closeVideo();
      retryVideo();
    }
    syncVideo();
    render();
  }

  // ---- camera video --------------------------------------------------------
  // Binary message: "DCV1", u32 header length (>= 24), u64 sequence, u32 age_ms,
  // u16 width, u16 height (all little-endian), then the JPEG.
  const canVideo = () => identity !== null && identity.capabilities.video === true;
  function videoWanted() {
    return liveView && canVideo() && page === 'live' && !document.hidden && !signingOut && vOnScreen;
  }
  // Leaving the viewport pauses video only after OFFSCREEN_GRACE_MS; coming back reopens at
  // once. Live view, its button and its 10-minute deadline are untouched either way.
  function setOnScreen(visible) {
    if (visible) {
      clearTimeout(vOffTimer); vOffTimer = 0;
      if (!vOnScreen) { vOnScreen = true; syncVideo(); updateCamera(); }
    } else if (vOnScreen && !vOffTimer) {
      vOffTimer = setTimeout(() => { vOffTimer = 0; vOnScreen = false; syncVideo(); updateCamera(); },
        OFFSCREEN_GRACE_MS);
    }
  }
  function toggleLiveView() {
    if (liveView) liveView = false;
    else if (canVideo()) {
      liveView = true; liveExpired = false; liveDevice = identity.id;
      liveStartMs = performance.now(); liveStartWall = Date.now();
    }
    syncVideo();
    render();
  }
  function resetLiveView() { liveView = false; liveExpired = false; liveDevice = null; }
  // performance.now() pauses while some devices sleep; the wall clock also counts that time,
  // and a wall-clock jump can only end the window early, which is the cheap direction.
  function liveElapsed(now = performance.now()) {
    return Math.max(now - liveStartMs, Date.now() - liveStartWall);
  }
  // Runs on every tick against the monotonic clock instead of one 10-minute timer, which a
  // background tab may throttle; reopening after a hidden spell checks it first.
  function syncLiveView() {
    if (liveDevice !== null && (!canVideo() || identity.id !== liveDevice)) resetLiveView();
    else if (liveView && liveElapsed() >= LIVE_VIEW_MAX_MS) { liveView = false; liveExpired = true; }
  }
  function syncVideo() {
    syncLiveView();
    if (!videoWanted()) {
      clearTimeout(vTimer); vTimer = 0; vAttempt = 0;
      if (vws) closeVideo();
      return;
    }
    if (vws || vTimer) return;
    if (verified) openVideo();      // only for the current attempt's confirmed identity
    else revalidate();
  }
  // A failed video upgrade carries no readable HTTP status, so /v1/identity decides: a rejected
  // session ends live view, a device without video clears it (syncLiveView), and a confirmed
  // device keeps the bounded backoff. A pending or scheduled telemetry check is shared instead.
  function revalidate() {
    verified = false;
    if (!videoWanted() || idCheck || retryTimer) return;
    checkIdentity().then(result => {
      if (result === 'stale') return;   // a newer attempt decides
      if (result === 'unpaired') { resetLiveView(); stop('Not paired'); showPage('connection'); return; }
      if (result === 'error' && !vTimer) retryVideo();
      syncVideo();
      render();
    });
  }
  function retryVideo() {
    clearTimeout(vTimer); vTimer = 0;
    if (!videoWanted()) return;
    const delay = DELAYS[Math.min(vAttempt, DELAYS.length - 1)];
    vAttempt += 1;
    vTimer = setTimeout(() => { vTimer = 0; syncVideo(); }, delay * 1000);
  }
  function closeVideo() {
    const sock = vws;
    vws = null;
    resetVideoFrames();
    if (sock) sock.close(1000);
  }
  function resetVideoFrames() {
    vPending = null; vLoading = null; vMeta = null; vShownMs = 0; vTimes = []; vLastSeq = 0;
    if (vImg) { vImg.hidden = true; vImg.removeAttribute('src'); }   // no load/error event follows
    if (vLoadingUrl) { URL.revokeObjectURL(vLoadingUrl); vLoadingUrl = null; }
    if (vUrl) { URL.revokeObjectURL(vUrl); vUrl = null; }
  }
  function parseFrame(data) {
    if (!(data instanceof ArrayBuffer) || data.byteLength < 24) return null;
    const d = new DataView(data);
    for (let i = 0; i < 4; i += 1) if (d.getUint8(i) !== VIDEO_MAGIC[i]) return null;
    const headerLength = d.getUint32(4, true);
    if (headerLength < 24 || headerLength >= data.byteLength) return null;
    const seq = d.getUint32(12, true) * 4294967296 + d.getUint32(8, true);
    const width = d.getUint16(20, true), height = d.getUint16(22, true);
    if (!Number.isSafeInteger(seq) || width < 1 || width > 4096 || height < 1 || height > 4096) return null;
    const blob = new Blob([new Uint8Array(data, headerLength)], { type: 'image/jpeg' });
    return { seq, width, height, ageMs: d.getUint32(16, true), blob, recvMs: performance.now() };
  }
  function openVideo() {
    let sock;
    try {
      sock = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/v1/video');
    } catch (_) { return retryVideo(); }
    sock.binaryType = 'arraybuffer';
    vws = sock; vOpenedMs = performance.now(); vMsgMs = 0;
    resetVideoFrames();
    sock.addEventListener('message', ev => {
      if (vws !== sock) return;
      const frame = parseFrame(ev.data);
      if (!frame) { closeVideo(); return retryVideo(); }
      if (frame.seq <= vLastSeq) return;            // replayed or out of order
      vLastSeq = frame.seq; vMsgMs = frame.recvMs; vAttempt = 0;
      if (vLoadingUrl) vPending = frame;            // still decoding: keep only the newest
      else loadFrame(frame);
    });
    sock.addEventListener('close', () => {
      if (vws !== sock) return;
      const delivered = vMsgMs > 0;
      vws = null;
      resetVideoFrames();
      retryVideo();
      if (!delivered) revalidate();                 // failed upgrade or closed before any frame
      render();
    });
  }
  function loadFrame(frame) {
    vLoading = frame;
    vLoadingUrl = URL.createObjectURL(frame.blob);
    vImg.src = vLoadingUrl;
  }
  function frameDone(ok) {
    if (!vLoadingUrl) return;
    const url = vLoadingUrl, frame = vLoading;
    vLoadingUrl = null; vLoading = null;
    if (vUrl) URL.revokeObjectURL(vUrl);           // the previous picture is no longer displayed
    if (ok) {
      vUrl = url; vMeta = frame; vShownMs = frame.recvMs;
      vTimes.push(frame.recvMs);
      while (vTimes.length > 32 || (vTimes.length > 1 && frame.recvMs - vTimes[0] > 3000)) vTimes.shift();
    } else {
      URL.revokeObjectURL(url);                    // undecodable frame: nothing to show until the next
      vUrl = null;
    }
    const next = vPending;
    vPending = null;
    if (next) loadFrame(next);
    updateCamera();
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
    for (const p of Object.keys(PAGES)) $('page-' + p).hidden = p !== name;
    $('title').textContent = PAGES[name];
    page = name;
    syncVideo();
    render();
  }
  function pairFromHash() {
    if (!location.hash.startsWith('#pair=')) return false;
    const token = safeDecode(location.hash.slice(6));
    history.replaceState(null, '', location.pathname + location.search);   // strip the token at once
    pair(token);
    return true;
  }
  function init() {
    $('page-live').append(cameraCard(), speedCard(), duo(recordingCard(), adasCard()), duo(positionCard(), healthCard()));
    $('page-vehicle').append(speedCard(), mapCard(), duo(powertrainCard(), positionCard()), linkCard());
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
      syncVideo();
      syncWake();
    });
    window.addEventListener('hashchange', pairFromHash);
    setInterval(tick, 250);
    if (!pairFromHash()) start();
  }
  init();
})();
