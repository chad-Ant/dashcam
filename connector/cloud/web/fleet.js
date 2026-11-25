import {initializeApp} from 'https://www.gstatic.com/firebasejs/11.10.0/firebase-app.js';
import {getAuth, GoogleAuthProvider, signInWithPopup, signOut, inMemoryPersistence, setPersistence}
  from 'https://www.gstatic.com/firebasejs/11.10.0/firebase-auth.js';

const el = id => document.getElementById(id);
const auth = getAuth(initializeApp(await (await fetch('/v1/config')).json()));
await setPersistence(auth, inMemoryPersistence);
// Owner actions are bound to the vehicle that was selected when they started. `selection`
// changes with every vehicle choice, so a late response for an earlier vehicle is dropped;
// `loaded` is the policy read for the current selection, and Apply stays disabled without it.
let vehicles = [], selection = 0, reads = 0, issues = 0, loaded = null;
const status = message => { el('status').textContent = message; };
const path = key => '/v1/vehicles/' + encodeURIComponent(key);
async function api(url, options) {
  const r = await fetch(url, options);
  if (!r.ok) throw Error('Request failed (' + r.status + ')');
  return r.status === 204 ? null : r.json();
}
function action(id, fn) {
  el(id).onclick = () => Promise.resolve().then(fn).catch(e => status(e.message));
}
function selected() {
  selection += 1;
  loaded = null;
  el('code').textContent = '';
  return policy();
}
async function policy() {
  const key = el('vehicle').value, mine = ++reads;
  const owner = vehicles.some(v => v.id === key && v.role === 'owner');
  el('owner').hidden = !owner;
  loaded = null;
  el('save').disabled = el('mode').disabled = true;
  if (!key) return;
  const p = await api(path(key) + '/policy');
  if (mine !== reads || el('vehicle').value !== key) return;
  loaded = {key, version: p.version};
  el('mode').value = p.mode;
  el('save').disabled = el('mode').disabled = false;
}
async function list() {
  vehicles = await api('/v1/vehicles');
  el('vehicle').replaceChildren(...vehicles.map(v => new Option(v.label, v.id)));
  el('fleet').hidden = el('logout').hidden = false;
  el('login').hidden = true;
  el('connect').disabled = !vehicles.length;
  status(vehicles.length ? 'Select a vehicle' : 'No vehicles assigned');
  await selected();
}
action('login', async () => {
  const {user} = await signInWithPopup(auth, new GoogleAuthProvider());
  await api('/v1/session', {method:'POST', headers:{Authorization:'Bearer ' + await user.getIdToken()}});
  await list();
});
action('logout', async () => {
  await api('/v1/session', {method:'DELETE'});
  await signOut(auth);
  location.reload();
});
action('connect', () => {
  location.assign('/dashboard?vehicle=' + encodeURIComponent(el('vehicle').value));
});
el('vehicle').onchange = () => selected().catch(e => status(e.message));
action('save', async () => {
  const target = loaded, mine = selection;
  if (!target || target.key !== el('vehicle').value) return;
  loaded = null;
  el('save').disabled = el('mode').disabled = true;
  try {
    await api(path(target.key) + '/policy', {method:'PUT', headers:{'Content-Type':'application/json'},
      body:JSON.stringify({mode:el('mode').value, version:target.version})});
  } finally { if (mine === selection) await policy(); }
});
// Only the newest code for the selected vehicle is shown: issuing replaces the vehicle's
// pending code, and a code shown under another vehicle would enroll a Jetson as the wrong one.
action('issue', async () => {
  const key = el('vehicle').value, mine = ++issues, chosen = selection;
  el('code').textContent = '';
  const {code} = await api(path(key) + '/enrollment', {method:'POST'});
  if (mine === issues && chosen === selection && el('vehicle').value === key) el('code').textContent = code;
});
try { await list(); } catch { status('Sign in to view your vehicles'); }
