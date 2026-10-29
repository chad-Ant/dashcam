# Dashcam Connect — Android, browser + Jetson monitoring

Read-only vehicle telemetry, recorder state and host health for `dashcam_v0_4`.
Phone portrait and tablet landscape use the same adaptive Android app; any other
device (desktop, laptop, iPad, a phone without the app) can use the built-in web
client in a browser, see [Browser access](#browser-access). No internet, map,
commands or background monitoring is required. An optional low-rate preview of
the recording camera is off by default, see [Camera video](#camera-video).
v0.4 has no integrated ADAS: lane/driver capabilities explicitly remain false.

## Architecture and protocol

`v0.4 → nonblocking Unix datagrams → host Python agent → TLS WebSocket → Android app / browser`

The camera preview takes the same path on a second socket and a second
WebSocket route (`/v1/video`), see [Camera video](#camera-video).

The recorder emits one JSON snapshot per supervisor tick (~5 Hz), including
vehicle data even without a recording camera. It uses the existing bridge
overlay source selection. Unavailable/nonfinite readings become JSON `null`.
Its datagram send never waits for a reader. No network/disk work runs inside
camera or bridge callbacks. The publisher is disabled by default.

The agent runs **on the host**, samples health at 1 Hz and sends snapshots at
5 Hz. It accepts up to four `/v1/stream` viewers in total (Android and browser
clients share the limit; `/v1/video` has its own separate limit of four), gives
slow sends two seconds, and retains only the
latest local runtime snapshot. The agent uses the sender's
monotonic stamp, not its own receipt time, to discard queued/expired data.
v0.4 runtime data expires after two seconds; host health remains independently
available. Android and the web client expire the entire display after two
seconds without a message, using Android's monotonic clock or the browser's
`performance.now()`. Clock corrections cannot rejuvenate readings.

Authenticated routes. Android sends `Authorization: Bearer <device credential>`
(no Origin/Host requirement); a browser sends its session cookie instead, see
[Browser security](#browser-security):

| Route | Response |
|---|---|
| `GET /v1/identity` | Schema, device ID/name and capabilities; no secrets |
| `GET /v1/snapshot` | Current snapshot |
| `GET /v1/stream` | WebSocket, same snapshot envelope at 5 Hz; read-only (any client message closes it with 1008) |
| `GET /v1/video` | WebSocket, binary camera frames, up to 10 fps per viewer; same auth as `/v1/stream`; read-only (1008); 503 with four video viewers, see [Camera video](#camera-video) |
| `POST /v1/session` | Bearer token + same-origin request → 204 and the browser session cookie |
| `DELETE /v1/session` | Same-origin request → 204 and the cookie cleared (sign out) |

The `GET` routes return 401 without valid credentials, 403 when a cookie
request fails the Origin/Host check. Any
other path that is not a static file also needs credentials (401, or 404 when
authorised). Unauthenticated static files, which contain no data or secrets:
`GET /` (`/index.html`), `/app.js`, `/app.css`, `/icon.svg` from `connector/web/`
(exact whitelist, no listing; `serve --web-dir` overrides the directory).

`capabilities` in `/v1/identity`: `vehicle`, `system` and `video` are `true`;
`lane`, `driver` and `controls` are `false`. `video=true` means the agent offers
`/v1/video`; frames only arrive when v0.4 has the preview enabled. With an older
agent (no `video` capability, `/v1/video` answers 404) both clients show that
this Jetson offers no video instead of a picture.

Envelope fields: `schema=1`, `type=snapshot`, `device_id`, `session` (agent-run
UUID), `seq`, `runtime_fresh`, `runtime_age_ms`, `runtime`, `system` and
`invalid_packets`. `runtime=null` when expired or absent. Its `vehicle` values
are usable only when `bridge_fresh=true`; each individual invalid value is null.
Numeric keys include units: `speed_kmh`, `acceleration_ms2`, `latitude`,
`longitude`, `altitude_m`, `heading_deg`, `rpm`, `coolant_c`, `fuel_percent`,
`throttle_percent`, `engine_load_percent`, `satellites`. Runtime also carries
`recording`, session `frames`, measured `fps`, `bridge_age_ms`, `source_mono_s`.
Vehicle selection is CAN speed → GNSS → OBD; the existing source validity rules
remain authoritative. Detailed source-specific ages beyond the bridge's age
are not available from the present upstream protocol.

System fields include CPU/GPU percentage, byte counts for RAM/swap/storage,
uptime and a thermal-zone-name → Celsius map. Missing sysfs/proc counters are
null. Storage measures the filesystem containing the configured footage path;
an inaccessible path is unavailable. GPU load reads Tegra sysfs in 0.1% units.
Memory reports unified host RAM, not separate GPU VRAM. No power mode or fan
settings are changed. Unknown future JSON fields may be ignored; unknown schema
versions are rejected. Phone monitoring never implies an active ADAS detector.

## Jetson setup

Build v0.4 in the existing project container with normal `make dashcam_v0_4`.
Install this agent separately on the host (Python 3.10+):

```bash
cd ~/dashcam
python3 -m venv ~/.local/share/dashcam-monitor/venv
~/.local/share/dashcam-monitor/venv/bin/pip install -r connector/requirements.txt
mkdir -p ~/.config/dashcam-monitor
chmod 700 ~/.config/dashcam-monitor
~/.local/share/dashcam-monitor/venv/bin/python connector/service.py init \
  --state-dir ~/.config/dashcam-monitor --name 'Car Jetson' --san 192.168.0.15
```

`init` refuses an existing `identity.json`. It creates the identity (ID, name,
reusable token), a local CA (`ca.pem`, `ca-key.pem`) and a TLS leaf certificate
(`key.pem`, `cert.pem` = leaf followed by the CA) for the `--san` addresses.
`--san` is only needed for browser access; the Android app pins the leaf and
does not check names. For an existing state directory see
[Browser setup on the Jetson](#browser-setup-on-the-jetson).

Configure the paths for your rig. `LAN_ADDRESS` must be the Jetson's current
host LAN IPv4; reserve its address on the router for predictable reconnection.
The user's current address is `192.168.0.15`; it is not embedded in the APK.

```bash
printf 'LAN_ADDRESS=192.168.0.15\nFOOTAGE_PATH=/user/output/footage\n' \
  > ~/.config/dashcam-monitor/service.env
chmod 600 ~/.config/dashcam-monitor/service.env
# Edit FOOTAGE_PATH above to the actual HOST path backing the footage mount.
sudo cp connector/dashcam-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dashcam-monitor
```

The example service assumes user/group `jetson` and checkout `~/dashcam`.
Adjust these if your installation differs. Allow inbound TCP 8443 and mDNS
UDP 5353 on the LAN if a host firewall is enabled. No router port forwarding.

Add to the recording container's launch options:

```text
--volume /run/dashcam-monitor:/run/dashcam-monitor
```

Mount the **directory**, not the individual socket. A bind mount keeps
pointing at the directory that existed when the recorder started, so the
example unit sets `RuntimeDirectoryPreserve=yes`: without it systemd removes
and recreates `/run/dashcam-monitor` whenever the agent stops (`systemctl
restart`, `Restart=on-failure`), and telemetry and video stay silent until the
recorder is restarted too. The agent recreates its sockets inside the kept
directory. Start the recorder after the agent's first start, so the directory
exists when it is mounted (it lasts until reboot; `/run` is a tmpfs).

The recorder must run as the same UID as the agent, or as container root.
In the active dashcam XML file, under `<Network>`, add:

```xml
<MonitoringEnabled>true</MonitoringEnabled>
<MonitoringSocket>/run/dashcam-monitor/telemetry.sock</MonitoringSocket>
```

For the optional camera preview add `<MonitoringVideoEnabled>true</MonitoringVideoEnabled>`
as well, see [Camera video](#camera-video).

Restart the recorder during a suitable recording interruption. Its existing
network-control channel does not need to be enabled. Wi-Fi without internet is
supported; both devices must be reachable on the same router (no client isolation).

Display the pairing links on the Jetson:

```bash
~/.local/share/dashcam-monitor/venv/bin/python connector/service.py pair \
  --state-dir ~/.config/dashcam-monitor --host 192.168.0.15
```

It prints the Android link (`dashcam://pair?...`), then the browser link
(`https://<host>:8443/#pair=<token>`), the CA fingerprint and a warning if the
host is not a SAN of the leaf certificate. The warning's `cert` command lists the
current SANs plus the host, because `cert` replaces the SAN list.
Paste the Android link into Android's **Connection → Pair a Jetson** field.
Transfer it through a channel you trust: it contains a reusable credential and
the SHA-256 fingerprint of the exact TLS leaf certificate. Android checks the
pin and certificate validity before sending credentials, then checks device ID
on every snapshot. It never falls back to unauthenticated HTTP or accepts a new
certificate automatically. Profiles are encrypted with an Android Keystore key;
app backup is disabled. Forgetting a device removes its local pairing. To revoke
all clients (Android pairings and browser sessions), stop the agent and provision
a new identity in a **new** state directory, update the service path, then pair
each client again. A new state directory also has a new CA: install it on the
browser devices again.

Optional mDNS advertises `_dashcam._tcp` with device ID and schema only. The app
can update a paired device's address from discovery; it keeps its original pin
and credential. Discovery does not grant trust. Manual pairing works if multicast
is blocked. Browsers do not use this discovery. Android 17 requests
local-network permission; denial has a recovery button. No GPS, microphone or
camera permission is needed.

## Camera video

The **Live** page of the Android app and of the web client starts with a
**Camera** card: a live preview of the **recording USB dashcam camera only**
(the pinned UGREEN camera v0.4 records by passthrough; no CSI or cabin camera —
if v0.4 ever falls back to recording the cabin camera, the preview pauses and
logs `monitor preview: paused, recording the cabin camera`),
scaled to **640×360** (16:9, letterboxed if the source is not), **up to 10 fps**,
with a caption such as `640×360 · 9.8 fps · 120 ms old`. Measured on the bench
(2026-10-29, real UGREEN at 1080p30): 9.8 fps, about 11 KB per frame (≈ 0.9 Mbit/s
per viewer; detailed road scenes are larger, capped at 64 KB, so budget up to
~5 Mbit/s), 110–160 ms old when sent. Converting costs v0.4 about **0.3 of a CPU
core** while someone watches (27 ms p95 per frame: a 1080p source is decoded at
half scale for 640×360) and nothing otherwise; recording is passthrough and is
not affected. It is a monitoring aid, not footage: nothing is stored by the agent
or the clients.

**Off by default.** It needs the existing `MonitoringEnabled` plus, under
`<Network>` in the active dashcam XML file:

```xml
<MonitoringEnabled>true</MonitoringEnabled>
<MonitoringVideoEnabled>true</MonitoringVideoEnabled>
<!-- Optional, these are the defaults: -->
<MonitoringVideoSocket>/run/dashcam-monitor/video.sock</MonitoringVideoSocket>
<MonitoringVideoDemand>/run/dashcam-monitor/video-demand</MonitoringVideoDemand>
<MonitoringVideoFps>10</MonitoringVideoFps>
```

`MonitoringVideoFps` is the conversion ceiling (1–15; out of range logs a
warning and uses 10); the agent still sends each viewer at most 10 fps.
`MonitoringVideoEnabled` without `MonitoringEnabled` only logs a warning. The
video socket and the demand file live in the `/run/dashcam-monitor` directory,
so the `--volume /run/dashcam-monitor:/run/dashcam-monitor` option from Jetson
setup above is required (and enough: no extra container option); the
agent binds the video socket like the telemetry socket (a stale socket is
replaced, mode 0600). The agent's `serve --video-socket` / `--video-demand` default to `video.sock` and
`video-demand` next to `--socket`. Restart the recorder (as above) to apply.

**Converts only while someone watches.** While at least one `/v1/video` viewer
is connected, the agent touches the demand file (created mode 0600 if missing,
mtime updated about twice a second). Once per supervisor tick v0.4 `stat()`s
it: the preview converter runs only while it is a regular file whose mtime is
within 3 s of the wall clock (a missing file, a directory, or an mtime more than
3 s old or in the future means nobody watches). When the last viewer leaves, the
agent stops touching it and the conversion stops by itself within about 3 s; a
crashed agent therefore cannot leave it running. Demand-file write errors are
logged once by the agent and are never fatal.

**Never affects recording.** v0.4 hands every recorded camera JPEG to its own
preview worker (a second `VideoPreview`, like the ES3C28P display preview)
without waiting; the worker decodes, scales and re-encodes it (quality 65,
down to 45, at most 64 KB) and sends each frame as one nonblocking Unix
datagram from a preallocated buffer. A missing, full or stalled agent only
drops preview frames (counted, with a stats line at shutdown); a converter
start failure logs a warning and recording continues. The agent keeps only the
latest frame and never queues frames for a viewer: a slow viewer skips frames,
and a send that takes over 2 s closes that viewer.

**Clients stream only while Live is visible.** The web client opens
`/v1/video` only while it is paired, the Live page is shown and the tab is
visible; the Android app only while a device is selected, the Live page is
selected and the app is in the foreground. Leaving Live, hiding the tab or
backgrounding the app, signing out or losing the pairing closes it; reconnects
use the 1, 2, 4, 8, 15 s backoff. Both clients decode off the main path and
keep only the newest pending frame while one is still decoding.

**Privacy.** The page and the app show the **road camera live** to anyone with
the credential: the same Android pairing or browser session that already shows
the live GPS position. There is no separate video credential; sign out on
borrowed devices and revoke as described above.

**Limits.** At most **four video viewers** (`503` for a fifth), separate from
the four `/v1/stream` viewers. The agent never sends a frame older than **1 s**
(capture to send) and drops received frames older than 2 s. Clients show
**No video** before the first frame (or while not connected) and **Video
stale**, with the old picture hidden, when the last frame arrived more than
**2 s** ago (by `performance.now()` / `SystemClock.elapsedRealtime`).

### Video protocol

All integers are **little-endian**. Both headers are 24 bytes, start with the
ASCII magic `DCV1` and carry the header length, so a future version can grow
the header; the JPEG starts at that offset.

v0.4 → agent, one Unix datagram per frame (`SOCK_DGRAM`, unbound nonblocking
sender, like the telemetry publisher), at most 24 + 65536 bytes:

| Bytes | Type | Field |
|---|---|---|
| 0–3 | ASCII | `DCV1` |
| 4–7 | uint32 | header length = 24 |
| 8–15 | uint64 | capture time, `CLOCK_MONOTONIC` microseconds (camera frame) |
| 16–19 | uint32 | source sequence (recorder's frame counter, truncated) |
| 20–21 | uint16 | width (640) |
| 22–23 | uint16 | height (360) |
| 24– | bytes | JPEG (`FF D8` … `FF D9`) |

The agent rejects (counting `invalid_video`) a datagram shorter than 24 bytes
or longer than 24 + 262144, a wrong magic or header length, a JPEG without the
`FF D8` start and `FF D9` end markers, a width or height outside 1–4096, and a
capture time that is not 0–2 s in the past of its own monotonic clock (host and
container share `CLOCK_MONOTONIC`). A sequence that is not newer than the
current frame's is ignored, unless the capture time went back by more than 5 s
(a restarted v0.4 is a new producer) or the current frame has expired.

Agent → client, `GET /v1/video` (WebSocket, `wss://<host>:8443/v1/video`), one
**binary** message per frame:

| Bytes | Type | Field |
|---|---|---|
| 0–3 | ASCII | `DCV1` |
| 4–7 | uint32 | header length = 24 |
| 8–15 | uint64 | frame sequence of this agent run (starts at 1, increases) |
| 16–19 | uint32 | frame age at send time, milliseconds (≤ 1000) |
| 20–21 | uint16 | width |
| 22–23 | uint16 | height |
| 24– | bytes | JPEG |

Authorization is exactly that of `/v1/stream`: `Authorization: Bearer`
(Android), or the browser session cookie with an allowed `Host` **and** a
mandatory same-origin `Origin`. The channel is read-only (any client message
closes it with 1008). Clients accept a message only if it is binary, at least
24 bytes, has the magic, a header length from 24 to the message length, width
and height 1–4096 and a sequence greater than the last one shown. The web
client displays it through a `blob:` object URL (the Content-Security-Policy
allows `img-src 'self' data: blob:`) and revokes the previous URL once the new
image has loaded.

## Browser access

The agent also serves a small web client (`connector/web/`: plain HTML/JS/CSS,
no build step, no frameworks, no external resources, no analytics) with the same
pages as the Android app: **Live** (camera preview, see
[Camera video](#camera-video), speed and source, recording, fps, frames,
bridge age, connection state), **Vehicle**, **System** and **Connection**
(device, session, pairing, sign out, optional "Keep screen on" where the browser
supports the Wake Lock API). It adapts from phone portrait (bottom navigation)
to tablet landscape/desktop (side navigation from ~840 px) and follows the
system light/dark setting. Like the app it is read-only, blanks every value two
seconds after the last message, uses vehicle values only while
`bridge_fresh=true`, shows `SIMULATED` speed with a warning style and reconnects
with 1, 2, 4, 8, 15 s backoff. Like the app's 15 s read timeout, a stream that
stays silent for 15 s (for example after the Jetson was power-cycled) is closed
and reconnected, and HTTP requests time out after 5 s.

### Certificate model

A browser cannot pin a certificate, so it must trust the agent through a CA.
The agent creates its own **local CA** per state directory (EC P-256, 10 years,
`ca-key.pem` mode 0600, never leaves the Jetson) and you install only its public
certificate on each browser device. The CA carries critical **name
constraints**: it can only vouch for private and link-local addresses
(`10/8`, `172.16/12`, `192.168/16`, `127/8`, `169.254/16`, `::1`, `fd00::/8`,
`fe80::/10`) and names under `local`, `lan`, `home.arpa` and `localhost`. The
list is fixed: `--san` never widens it, so use the LAN IP or a name such as
`jetson.lan` or `dashcam.home.arpa`. A stolen `ca-key.pem` therefore cannot
impersonate public sites to devices that trust it. Major verifiers enforce
these constraints; treat them as defence in depth and protect the key anyway.
`--san` values outside the constraints are refused, and an existing CA is never
silently replaced.

The CA signs the server's **leaf certificate** (new key each time, valid for
825 days from one day before issue, Apple's maximum for any TLS certificate).
Its SANs are the addresses or names you open in the browser; the subject CN is
the first DNS SAN of at most 64 characters, else `Dashcam Monitor`. Renew it with `cert` before it expires, or
when the Jetson's LAN address changes; browser devices keep trusting the same
CA. Every `cert` run changes the leaf pin, so **each Android device must be
paired again** (forget the device, paste the new Android link). Certificate
dates use the Jetson's clock: run `init`/`cert` only after its time is set.

### Browser setup on the Jetson

Reserve the Jetson's LAN address on the router: the browser must open exactly a
SAN of the leaf.

```bash
PY=~/.local/share/dashcam-monitor/venv/bin/python
STATE=~/.config/dashcam-monitor
# New install: init creates the CA and a leaf for the address
$PY connector/service.py init --state-dir $STATE --name 'Car Jetson' --san 192.168.0.15
# Existing state dir (no CA yet, or a renewal / new address):
$PY connector/service.py cert --state-dir $STATE --san 192.168.0.15
sudo systemctl restart dashcam-monitor    # the agent loads the new certificate
# Export the public CA certificate for the browser devices
$PY connector/service.py ca --state-dir $STATE --out dashcam-ca.crt
# Both links: Android and browser
$PY connector/service.py pair --state-dir $STATE --host 192.168.0.15
```

`cert` needs at least one `--san` (IPv4/IPv6 address or a DNS name such as
`jetson.lan`; repeat for several). It keeps the identity ID and token, creates
the CA if it is missing (replacing an old self-signed `cert.pem`/`key.pem`),
writes every new file and fsyncs it before renaming any of them (then fsyncs the
directory), prints the new pin and the re-pairing warning. The SAN list is
replaced, not extended: give every address you still use. A `ca-key.pem` left
without `ca.pem` by an interrupted upgrade (it never signed the served leaf) is
replaced on the next `cert`. If a SAN is outside an existing CA's constraints it fails
without changing anything. `ca` refuses to overwrite `--out`, never writes key
material, and prints the CA's SHA-256 fingerprint (`AB:CD:...`). Copy
`dashcam-ca.crt` to each device through a channel you trust (USB, your own
cloud drive, AirDrop); it is public, but a substituted file would make the
device trust someone else's CA. No port or firewall change is needed (still TCP
8443). `serve --browser-session-days N` (1–365, default 30) sets the browser
session lifetime.

### Installing the CA on a device

Install `dashcam-ca.crt` as a **CA / root certificate for websites**, not as a
VPN, Wi-Fi or app certificate. During or after the install, compare the SHA-256
fingerprint the device shows with the `ca` output (any computer:
`openssl x509 -in dashcam-ca.crt -noout -fingerprint -sha256`). The Android app
itself does not need the CA.

- **Android 14–16** (Chrome): Settings > Security and privacy > More security
  settings > Encryption and credentials > Install a certificate > CA
  certificate, accept the warning, pick the file. **Samsung One UI**: Settings >
  Security and privacy > Other security settings > Install from device storage >
  CA certificate. A screen lock is required; Android then shows a "network may be
  monitored" notice. Remove it later under Encryption and credentials > Trusted
  credentials > User. Firefox for Android ignores user CAs unless its
  third-party CA option is enabled; use Chrome.
- **Windows** (Edge, Chrome): double-click the `.crt` > Install Certificate >
  Current User > Place all certificates in the following store > Browse >
  Trusted Root Certification Authorities > Finish, confirm the security
  warning. Or `certmgr.msc` > Trusted Root Certification Authorities >
  Certificates > All Tasks > Import. Windows' dialog shows a SHA-1 thumbprint;
  check the SHA-256 with `certutil -dump dashcam-ca.crt` (`Cert Hash(sha256)`,
  same hex without colons).
- **macOS** (Safari, Chrome): double-click the `.crt` and add it to the login
  (or System) keychain in Keychain Access, open the certificate, expand Trust,
  set "When using this certificate" to **Always Trust**, close and authenticate.
- **iOS/iPadOS** (all browsers): open the file (AirDrop, Files or Safari) and
  allow the profile download, install it under Settings > Profile Downloaded
  (or Settings > General > VPN & Device Management), then enable full trust in
  Settings > General > About > Certificate Trust Settings.
- **Linux, Chrome/Chromium**: `chrome://certificate-manager` > Local
  certificates > Installed by you > Trusted certificates > Import; or with
  NSS tools: `certutil -d sql:$HOME/.pki/nssdb -A -t "C,," -n "Dashcam Connect"
  -i dashcam-ca.crt`. Restart the browser.
- **Firefox** (desktop): Settings > Privacy & Security > Certificates > View
  Certificates > Authorities > Import, tick "Trust this CA to identify
  websites", then restart Firefox. Firefox keeps its own store: a CA installed
  into the Windows Current User store (the certificate wizard's default) is
  **not** seen by Firefox, which keeps warning. `security.enterprise_roots.enabled
  = true` in `about:config` makes it read the OS roots, but on Windows only the
  machine-wide (Local Machine, administrator) Trusted Root store.

### Browser pairing

Open the **browser link** printed by `pair` on the device (paste it into the
address bar; it is as sensitive as the Android link). The token is in the URL
fragment (`#pair=...`), which browsers never send to the server in the request
line. The page removes it from the address bar immediately and exchanges it
once (`POST /v1/session` with the Bearer header) for a session cookie
`__Host-dcsession` (`Secure; HttpOnly; SameSite=Strict`, 30 days by default).
The script never stores the token. Alternatively open `https://<host>:8443/`
and paste either link into **Connection → Pair**. The pasted link may remain in
the clipboard or the address-bar history of that device: clear them on shared
devices. Bookmark `https://<host>:8443/` afterwards.

**Sign out** deletes the cookie on that browser. Sessions are stateless (an
HMAC of the expiry with a key derived from the identity token), so agent
restarts keep them and a copied cookie stays valid until it expires; only a new
identity revokes all browser sessions (and all Android pairings). Expiry is
checked against the Jetson's wall clock: if it is set wrong at sign-in, or later
jumps, sessions can expire early or last longer than 30 days. An expired or
revoked session shows the "not paired" state with instructions.

### Browser security

- **Host check**: cookie requests must address the agent as one of the leaf's
  SANs plus the serving port (`192.168.0.15:8443`). With TLS name verification
  this also blocks DNS rebinding: a rebinding name never matches the
  certificate or the Host whitelist.
- **Same-origin check**: `POST`/`DELETE /v1/session` and every cookie request
  that carries `Origin` need `Origin: https://<that host>`; the WebSocket
  always needs it. This prevents cross-site WebSocket hijacking and, together
  with `SameSite=Strict`, cross-site use of the cookie. No CORS headers are ever
  sent.
- **Headers on every response** (also errors): a strict Content-Security-Policy
  (`default-src 'none'`, scripts/styles/connections only from the agent itself,
  images only from the agent, `data:` or `blob:` (the camera frames), no
  framing, no forms), `nosniff`, `no-referrer`, `X-Frame-Options: DENY`,
  same-origin COOP/CORP, a Permissions-Policy denying geolocation, camera and
  microphone, and `Cache-Control: no-store`. The page builds all DOM with
  `textContent`; there is no inline script, `eval` or HTML injection of data.
- **Read-only**: the browser can only read identity, snapshots and camera
  frames; both WebSockets accept no input and close on any client message.
- The page shows the car's **live GPS position**, speed and, with
  `MonitoringVideoEnabled`, the **live road camera**: treat a browser session
  like an Android pairing and sign out on borrowed devices.

Limits: browsers have no mDNS discovery here; open the IP (or a router DNS name
added as a SAN) and bookmark it. The four-viewer limits (stream and video,
counted separately) are shared with the Android clients, and a hidden browser
tab closes its stream and video until it is shown again (video also closes when
leaving the Live page).

## Android build

Open `android/` in Android Studio, or use JDK 17+ (tested locally with JDK 25),
Android SDK 37 and the checked-in Gradle wrapper:

```powershell
cd android
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest :app:lintDebug
```

APK: `android/app/build/outputs/apk/debug/app-debug.apk`. Install it on the phone
and tablet, allow local-network access when prompted, and pair the same Jetson
on both. This is a debug-signed personal-use build; a distribution release needs
your own signing key. No signing secret is included in this repository.

## Simulator and tests

```bash
python -m unittest discover -s connector -p test_service.py -v
python connector/service.py init --name 'Development simulator'
python connector/service.py serve --simulate --bind 127.0.0.1 --storage .
python connector/service.py pair --host 127.0.0.1
```

To try the web client locally, add `--san 127.0.0.1` to `init`, trust the CA
from `service.py ca --out dashcam-ca.crt` in the desktop browser and open the
browser link printed by `pair --host 127.0.0.1`.
For an Android emulator, use host `10.0.2.2` in the simulator pairing command.
For a physical phone, bind the simulator to a LAN interface and pair that host's
LAN IP. Simulator readings are explicitly labelled `SIMULATED` in speed source;
the running service name also carries `[SIMULATOR]`. Do not confuse them with
vehicle evidence. For the Camera card the simulator reads
`connector/testcard.jpg` (a synthetic test pattern, no camera image) once at
start and re-sends it at 10 fps with an increasing sequence and the current
capture time; without that file it produces no video. It opens no video socket
and writes no demand file.

`src/tests/test_monitoring.cpp` covers serialization, nonfinite/stale readings,
socket delivery and bounded behavior with a missing/stalled agent, plus the
video datagram header byte by byte, the demand-file rule (missing, fresh, 4 s
old, future, directory), non-blocking drops without a receiver, byte-exact
delivery and 1000 offers against a stalled receiver. Plain `make` builds it as
`monitoring_test`.
Python tests cover auth denial, certificate pinning, two viewers, read-only
commands, client limits, runtime expiry, bad packets and host metric deltas,
and for video: each `accept_video` rule, latest-frame-only and the
sequence/new-producer rule, `/v1/video` auth like `/v1/stream`, the binary
message byte by byte, the 10 fps cap, no stale (> 1 s) frames, the separate
viewer limit, 1008 on client messages, the demand file while a viewer is
connected and after it leaves, `blob:` in the CSP and `video` in identity.
Android unit tests cover pairing validation, schema/device identity and freshness.
The opt-in Android instrumentation test accepts a base64-encoded **simulator**
pairing link using `-e pairingBase64`, verifies the real TLS stream and wrong-pin
rejection, and saves `files/dashboard.png` in the test app's private files.

`PYTHONPATH=<dependency-directory> python3 connector/smoke_jetson.py` is an
opt-in test for an **isolated validation checkout** on the Jetson host after
building v0.4 there. It launches an unprivileged container without host camera
or serial devices, checks real v0.4 IPC through pinned TLS, stops the test
runtime, and checks expiry while host health continues. It uses temporary
identities/paths and does not change deployed services. Normal venv users can
omit `PYTHONPATH`.

Hardware acceptance: compare displayed speed with the bridge, unplug the C3,
stop/restart v0.4, restart the agent, interrupt Wi-Fi, rotate both device types,
deny/regrant LAN permission, and verify recording throughput with both clients
connected, including four camera viewers and a stalled one (no recorder frame
drops; the camera preview stops about 3 s after the last viewer leaves). These checks must be distinguished from simulated/software results.

See [VALIDATION.md](VALIDATION.md) for the checks performed for this implementation.
