import {initializeApp} from 'https://www.gstatic.com/firebasejs/11.10.0/firebase-app.js';
import {getAuth, GoogleAuthProvider, signInWithPopup, signOut, inMemoryPersistence, setPersistence}
  from 'https://www.gstatic.com/firebasejs/11.10.0/firebase-auth.js';

const el = id => document.getElementById(id);
const auth = getAuth(initializeApp(await (await fetch('/v1/config')).json()));
await setPersistence(auth, inMemoryPersistence);
let vehicles = [], version = 0;
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
  await api('/v1/session', {method:'DELETE'});
  await signOut(auth);
  location.reload();
});
action('connect', () => {
  location.assign('/dashboard?vehicle=' + encodeURIComponent(el('vehicle').value));
});
el('vehicle').onchange = () => policy().catch(e => status(e.message));
action('save', async () => {
  try { await api(path()+'/policy',{method:'PUT',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:el('mode').value,version})}); }
  finally { await policy(); }
});
action('issue', async () => { el('code').textContent = (await api(path()+'/enrollment',{method:'POST'})).code; });
try { await list(); } catch { status('Sign in to view your vehicles'); }
