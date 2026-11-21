import {initializeApp} from 'https://www.gstatic.com/firebasejs/11.10.0/firebase-app.js';
import {getAuth, GoogleAuthProvider, signInWithPopup, signOut, inMemoryPersistence, setPersistence}
  from 'https://www.gstatic.com/firebasejs/11.10.0/firebase-auth.js';

const el = id => document.getElementById(id);
const auth = getAuth(initializeApp(await (await fetch('/v1/config')).json()));
await setPersistence(auth, inMemoryPersistence);
let vehicles = [], version = 0, socket, renewal, retry, watching = false;
let cameraStart, frameUrl, frameTime = 0, sampleTime = 0, start = 0, latency;
const status = message => { el('status').textContent = message; };
const path = () => '/v1/vehicles/' + encodeURIComponent(el('vehicle').value);
async function api(url, options) {
  const r = await fetch(url, options);
  if (!r.ok) throw Error('Request failed (' + r.status + ')');
  return r.status === 204 ? null : r.json();
}
function action(id, fn) {
  el(id).onclick = () => Promise.resolve().then(fn).catch(e => status(e.message));
}
function clearFrame() {
  el('frame').hidden = true;
  el('frame').removeAttribute('src');
  if (frameUrl) URL.revokeObjectURL(frameUrl);
  frameUrl = null;
}
function disconnect() {
  watching = false;
  clearTimeout(retry);
  clearInterval(renewal);
  const old = socket;
  socket = null;
  old?.close();
  clearFrame();
  el('summary').textContent = el('sample').textContent = '';
  sampleTime = 0;
  el('leave').disabled = el('camera').disabled = true;
  el('camera').checked = false;
  el('vehicle').disabled = false;
  el('connect').disabled = !vehicles.length;
  el('request').textContent = el('timing').textContent = '';
  status('Disconnected');
}
async function policy() {
  const key = el('vehicle').value;
  const owner = vehicles.some(v => v.id === key && v.role === 'owner');
  el('owner').hidden = !owner;
  el('code').textContent = '';
  if (!key) return;
  const p = await api(path() + '/policy');
  if (el('vehicle').value === key) {
    version = p.version;
    el('mode').value = p.mode;
  }
}
async function list() {
  vehicles = await api('/v1/vehicles');
  el('vehicle').replaceChildren(...vehicles.map(v => new Option(v.label, v.id)));
  el('fleet').hidden = el('logout').hidden = false;
  el('login').hidden = true;
  el('connect').disabled = !vehicles.length;
  status(vehicles.length ? 'Select a vehicle' : 'No vehicles assigned');
  await policy();
}
action('login', async () => {
  const {user} = await signInWithPopup(auth, new GoogleAuthProvider());
  await api('/v1/session', {method:'POST', headers:{Authorization:'Bearer ' + await user.getIdToken()}});
  await list();
});
action('logout', async () => {
  disconnect();
  await api('/v1/session', {method:'DELETE'});
  await signOut(auth);
  location.reload();
});
function demand() {
  if (socket?.readyState === WebSocket.OPEN)
    socket.send(JSON.stringify({channels: el('camera').checked ? ['telemetry','video'] : ['telemetry']}));
}
function open() {
  if (!watching) return;
  status('Connecting');
  const ws = new WebSocket('wss://' + location.host + '/v1/view/' + encodeURIComponent(el('vehicle').value));
  socket = ws;
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => {
    if (socket !== ws) return;
    demand();
    renewal = setInterval(demand, 10000);
  };
  ws.onmessage = ({data}) => {
    if (socket !== ws) return;
    if (typeof data === 'string') {
      const s = JSON.parse(data);
      if (s.type === 'connecting') {
        el('request').textContent = 'Request ' + s.requestId;
        return;
      }
      if (s.type !== 'snapshot') return;
      sampleTime = performance.now();
      if (s.runtime_fresh && latency === undefined) latency = Math.round(sampleTime - start);
      status(s.runtime_fresh ? 'Live · first fresh sample ' + latency + ' ms' : 'Online · recorder data unavailable');
      const speed = s.runtime?.bridge_fresh && s.runtime?.vehicle?.speed_kmh;
      const metrics = ['Sample ' + s.seq];
      if (Number.isFinite(speed)) metrics.push(Math.round(speed) + ' km/h');
      if (s.runtime_fresh) metrics.push(s.runtime.recording ? 'Recording' : 'Recorder idle');
      el('summary').textContent = metrics.join(' · ');
      el('sample').textContent = JSON.stringify({seq:s.seq, runtime:s.runtime, system:s.system}, null, 2);
    } else if (el('camera').checked && data.byteLength >= 24) {
      const header = new DataView(data);
      const offset = header.getUint32(4,true);
      if (header.getUint32(0) !== 0x44435631 || offset < 24 || offset >= data.byteLength || header.getUint32(16,true)>2000) return;
      clearFrame();
      frameTime = performance.now();
      if (cameraStart !== undefined) {
        el('timing').textContent = 'First camera frame ' + Math.round(frameTime - cameraStart) + ' ms';
        cameraStart = undefined;
      }
      frameUrl = URL.createObjectURL(new Blob([data.slice(offset)],{type:'image/jpeg'}));
      el('frame').src = frameUrl;
      el('frame').hidden = false;
    }
  };
  ws.onclose = async e => {
    if (socket !== ws) return;
    clearInterval(renewal);
    clearFrame();
    sampleTime = 0;
    el('summary').textContent = el('sample').textContent = '';
    try {
      if (e.code === 1008 || !(await api('/v1/vehicles')).some(v => v.id === el('vehicle').value))
        throw Error('Session ended. Check vehicle access and sign in again.');
      if (socket === ws && watching) {
        status('Reconnecting');
        retry = setTimeout(open, 2000);
      }
    } catch (err) { if (socket === ws) { disconnect(); status(err.message); } }
  };
}
action('connect', () => {
  watching = true;
  el('connect').disabled = el('vehicle').disabled = true;
  el('leave').disabled = el('camera').disabled = false;
  start = performance.now();
  latency = undefined;
  open();
});
action('leave', disconnect);
el('camera').onchange = () => {
  cameraStart = el('camera').checked ? performance.now() : undefined;
  el('timing').textContent = '';
  if (!el('camera').checked) clearFrame();
  demand();
};
el('vehicle').onchange = () => policy().catch(e => status(e.message));
action('save', async () => {
  try { await api(path()+'/policy',{method:'PUT',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:el('mode').value,version})}); }
  finally { await policy(); }
});
action('issue', async () => { el('code').textContent = (await api(path()+'/enrollment',{method:'POST'})).code; });
document.addEventListener('visibilitychange', () => { if (document.hidden && watching) disconnect(); });
addEventListener('pagehide', disconnect);
setInterval(() => {
  if (sampleTime && performance.now() - sampleTime > 2000) {
    sampleTime = 0; el('summary').textContent = el('sample').textContent = ''; status('Waiting for fresh telemetry');
  }
  if (frameUrl && performance.now() - frameTime > 2000) clearFrame();
},250);
try { await list(); } catch { status('Sign in to view your vehicles'); }
