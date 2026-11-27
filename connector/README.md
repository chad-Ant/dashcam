# Dashcam Connect — Android, browser + Jetson monitoring

Read-only vehicle telemetry, recorder state and host health for `dashcam_v0_4`.
Phone portrait and tablet landscape use the same adaptive Android app; any other
device (desktop, laptop, iPad, a phone without the app) can use the built-in web
client in a browser, see [Browser access](#browser-access). No internet, commands
or background monitoring is required; only the Vehicle map's tiles need internet
on the phone or browser device, see [Vehicle map](#vehicle-map). An optional
low-rate preview of the recording camera is off by default, see
[Camera video](#camera-video).
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
| `GET /v1/recordings` | Finished recorded segments, newest first (see [Recordings](#recordings-direct-lan)); 404 when the agent offers none |
| `GET /v1/recordings/<segment>.mkv` / `.ass` | One segment's video or captions, byte ranges supported; `?download=1` adds `Content-Disposition: attachment`; 409 while that segment is still recording |
| `GET /v1/video?quality=hd` | WebSocket, the camera's original frames at its own rate; 503 with two HD viewers; 400 for another `quality`; closes with 4001 when no HD frame arrives within 5 s, see [HD](#hd-direct-lan-only) |
| `POST /v1/session` | Bearer token + same-origin request → 204 and the browser session cookie |
| `DELETE /v1/session` | Same-origin request → 204 and the cookie cleared (sign out) |

The `GET` routes return 401 without valid credentials, 403 when a cookie
request fails the Origin/Host check. Any
other path that is not a static file also needs credentials (401, or 404 when
authorised). Unauthenticated static files, which contain no data or secrets:
`GET /` (`/index.html`), `/app.js`, `/app.css`, `/icon.svg` from `connector/web/`
(exact whitelist, no listing; `serve --web-dir` overrides the directory).

`capabilities` in `/v1/identity`: `vehicle`, `system`, `video` and `video_hd` are `true`;
`lane`, `driver` and `controls` are `false`. `video_hd=true` means the agent offers
`/v1/video?quality=hd`; without it the clients offer SD only. `recordings=true` means `/v1/recordings` lists the
footage folder (`serve --storage`; off with `--no-recordings`). `video=true` means the agent offers
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

Configure the paths for your rig. `LAN_ADDRESS` is the address mDNS advertises (**Find Jetsons**):
- **`auto` (recommended):** the IPv4 address of the interface holding the default route, found by a route
  lookup that sends no packet. Only private LAN addresses are advertised. It is re-checked every 30 s, the
  agent re-advertises when it changes (bench and car networks, a new DHCP lease), and it withdraws the
  advertisement while there is no LAN.
- **A fixed address:** pins it. On another network, mDNS then stays off (`mDNS advertisement off … is <address>
  assigned to this host?`).
- **Empty:** passes `--advertise-address ""`, which also leaves mDNS off.

Manual pairing works either way, and no address is embedded in the APK. Reserving the Jetson's address on the
router keeps paired devices' saved address valid. The service's `IPAddressAllow` must also include each LAN
it uses (the rig has a drop-in for 192.168.1.0/24).

```bash
printf 'LAN_ADDRESS=auto\nFOOTAGE_PATH=/mnt/sdcard/footage\n' \
  > ~/.config/dashcam-monitor/service.env
chmod 600 ~/.config/dashcam-monitor/service.env
# FOOTAGE_PATH is the HOST directory the recorder writes to (the SD card's
# footage folder), not the container path.  Segments recorded to
# <repo>/footage_fallback while the card was missing are not listed.
sudo cp connector/dashcam-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dashcam-monitor
```

The example service assumes user/group `jetson` and checkout `~/dashcam`.
It also limits peers to localhost, `192.168.0.0/24` and mDNS (`IPAddressAllow=`):
edit that line for another LAN. It does not rely on ufw, which is not enabled on
the rig, and hides the Docker/jtop sockets and the CA key from the agent.
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
is blocked. The advertisement is best-effort: if the advertised address is not
assigned to the Jetson (another network, DHCP not done yet), the agent logs
`mDNS advertisement off (...)` and keeps serving; restart it once the address is
back to advertise again. Browsers do not use this discovery. Android 17 requests
local-network permission; denial has a recovery button. No GPS, microphone or
camera permission is needed.

## Camera video

The **Live** page of the Android app and of the web client starts with a
**Camera** card: a live preview of the **recording USB dashcam camera only**
(the pinned UGREEN camera v0.4 records by passthrough; no CSI or cabin camera —
if v0.4 ever falls back to recording the cabin camera, the preview pauses and
logs `monitor preview: paused, recording the cabin camera`),
in **SD, 720×404** (16:9; a source of another shape is letterboxed into a 720×406
canvas), **up to 10 fps**, with a caption such as `720×404 · 9.8 fps · 120 ms old`.
This is the **SD** choice in both clients (`quality=low` on the wire; until
2026-11-26 it was 640×360). A 1080p frame is decoded straight at 3/8 scale
(libjpeg-turbo DCT scaling: 720×405, one row cropped), so nothing is resampled.
Measured on the Jetson with the converter on real UGREEN drive frames (2026-11-26):
**day** 43 KB per frame, **3.6 Mbit/s** at 10 fps, 10.9 ms per frame; **night**
23 KB, **1.9 Mbit/s**, 10.4 ms; SSIM 0.948 against a lanczos downscale. The old
640×360 path took 14.7 ms for 2.6 Mbit/s (day), so SD costs about a quarter less
CPU while watching. Frames are capped at 64 KB (re-encoded at quality 45 if
larger; no drive frame needed it). Converting runs only while someone watches;
recording is passthrough and is not affected. It is a monitoring aid, not footage: nothing is stored by the agent
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

**Clients stream only on demand, while Live is visible.** Video starts only
when the user taps **Start live view** on the Camera card. It is off on every
page load and app start (never remembered) and stops by itself **10 minutes
after the tap** ("Live view stopped after 10 min · tap Start live view to
resume"), so a phone left on the dashboard cannot stream for hours; a future
cloud relay bills for that traffic. Telemetry is not affected. While live
view is on, the web client opens `/v1/video` only while it is paired, the
Live page is shown and the tab is visible; the Android app only while a
device is selected, the Live page is selected and the app is in the
foreground. Live view turns off when the device changes, on sign-out or
unpairing, and when the Jetson rejects the credential or has no video. Leaving Live, hiding the tab or
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

### HD (direct LAN only)

Both clients show an **SD / HD** choice under the camera and a **Fullscreen**
button (enabled while live view is on). SD is the 720×404 preview above. **HD**
is the recording camera's **original JPEG frames, byte for byte, at the camera's
rate** (1920×1080 at 30 fps for the UGREEN): no decoding or re-encoding on the
Jetson. The choice is never stored: every app start and page load begins at SD.
Changing it reconnects the video socket; live view and its 10-minute window carry
on.

- **Direct LAN only.** A cloud session always shows SD, with HD disabled: the
  uplink requests `/v1/video` without `quality`, so original frames never leave
  the LAN. At 16–75 Mbit/s (bench 66 KB frames; night drive 2026-10-03 ≈ 5.2 MB/s;
  day drive 2026-09-26 ≈ 9.1 MB/s, 297 KB per frame ≈ 71 Mbit/s) they exceed a
  car's mobile uplink and would cost about 7–32 GB of relay egress per viewer-hour.
- **Network.** One HD viewer needs 16–75 Mbit/s between the Jetson and the client
  (daylight driving is the top of that range). At most two HD viewers (503 for a
  third), separate from the four SD viewers.
  On a slow link the newest-frame-only rule lowers the frame rate instead of
  building up delay.
- **Cost on the Jetson.** v0.4 copies each frame into a latest-frame slot on the
  camera's tap thread and a worker sends it; nothing is converted, so HD costs
  v0.4 less than SD. With HD enabled the recorder's preview tap runs at the
  camera rate in every session (the converters still limit themselves). Measured
  on the Jetson over loopback (2026-11-26, the C++ sender against this agent, 12 s
  runs at 30 fps): 66 KB frames, one viewer 29.3 fps, 4% of a core in the agent,
  5 ms median age; 578 KB frames, two viewers 28.0 fps each, 20% of a core; every
  frame byte-exact, none dropped. Not yet measured with the real camera or Wi-Fi.
- **Recording is never affected.** As for SD: nonblocking sends, a worker that
  waits at most 100 ms per frame for the agent to drain its queue, and counted
  drops (`monitor HD: … frames taken, … sent` at shutdown). While it waits it
  retries at most every 2 ms: `POLLOUT` on the unconnected socket does not see a
  full agent-side queue, so a poll-only loop would spin. Measured with the queue
  held full: the frame is given up after 101 ms using 2.4 ms of CPU.
- **Fallback.** If no fresh original frame has been sent for 5 s, the agent closes the HD socket with **4001**.
  That covers the start (HD off in v0.4, or a recorder without HD: reason `HD not offered`) and any later
  stop (reason `HD stopped`). Both clients then switch to SD and say which it was. A shorter gap keeps HD.
- **Capability.** Clients offer HD only when the Jetson's `/v1/identity` has `video_hd: true`; an older agent
  omits it while answering `quality=hd` with SD frames. Android resets the choice to SD when another Jetson
  is selected.
- **Fullscreen.** Android: its own window, landscape, system bars hidden, the
  SD/HD choice and **Exit fullscreen** on top (a tap hides or shows them; Back
  exits). Web: the browser's Fullscreen API with a landscape lock where allowed,
  or a fixed full-window view where the browser lacks or ignores it (iPhone
  Safari, embedded browsers). Leaving live view leaves fullscreen.
- **Phones.** Android decodes HD frames at full size (frames over 1920 px are
  subsampled). Expect more battery use and heat than SD; older phones may show
  fewer than 30 fps.

v0.4 settings, under `<Network>` (needs `MonitoringVideoEnabled`):

```xml
<MonitoringVideoHdEnabled>true</MonitoringVideoHdEnabled>   <!-- default -->
<MonitoringVideoHdDemand>/run/dashcam-monitor/video-demand-hd</MonitoringVideoHdDemand>   <!-- default -->
```

The agent touches the HD demand file (`serve --video-hd-demand`, default
`video-demand-hd` next to `--socket`) while an HD viewer is connected, by the
same 3-second rule as the preview's. An HD viewer does not touch the preview's
demand file, so the SD conversion stays idle.

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
| 20–21 | uint16 | width (720) |
| 22–23 | uint16 | height (404) |
| 24– | bytes | JPEG (`FF D8` … `FF D9`) |

**HD chunks**, v0.4 → agent on the same socket. A 1080p frame (60–600 KB) does not
fit one Unix datagram (`net.core.wmem_max` is 212992), so v0.4 cuts each frame
into chunks of up to 61,440 JPEG bytes, sent in order, one datagram each:

| Bytes | Type | Field |
|---|---|---|
| 0–3 | ASCII | `DCH1` |
| 4–7 | uint32 | header length = 32 |
| 8–15 | uint64 | capture time, `CLOCK_MONOTONIC` microseconds |
| 16–19 | uint32 | source sequence |
| 20–21 | uint16 | width |
| 22–23 | uint16 | height |
| 24–27 | uint32 | total JPEG bytes (4 – 1,048,576) |
| 28–31 | uint32 | offset of this chunk in the JPEG |
| 32– | bytes | the chunk |

The agent reassembles a frame only from consecutive chunks of the same frame
(capture time, sequence, size and total), starting at offset 0. A gap drops that
frame (`hd_incomplete`). A complete frame needs the JPEG start and end markers,
and replaces the current HD frame under the same sequence and new-producer rules
as the preview. A chunk with a bad header counts as `invalid_video`. HD viewers
receive the same `DCV1` client message as the preview.

A `DCV1` preview datagram is rejected (counting `invalid_video`) when it is shorter than 24 bytes
or longer than 24 + 262144, or has a wrong magic or header length, a JPEG without the
`FF D8` start and `FF D9` end markers, a width or height outside 1–4096, or a
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

## Recordings (direct LAN)

Both clients have a **Recordings** page that lists v0.4's finished segments by day, newest first.
- **Each segment** is a 3-minute `dashcam_<seq>_<YYYYmmdd>_<HHMMSS>.mkv` (1080p MJPEG, 0.4–1.6 GB) with its
  `.ass` telemetry captions (0.3–0.8 MB). The time in the name is the Jetson's local start time; a segment made
  before the clock was set shows under 1970-01-01.
- **Play** opens a full-window player, with fullscreen, pause and seek. Captions are drawn where the recorder's
  styles put them (corners, top centre, left middle), and can be turned off.
- **Download** saves the original files:
  - Android: both files in *Download/Dashcam*. On Android 9, the app's own Download folder.
  - Web: two links.

  VLC loads the captions automatically when they sit next to the video with the same name.

**The client does the playback; the Jetson only moves bytes.** The agent lists the folder and serves the two
files by byte range. It never decodes, converts or deletes anything. The client reads the Matroska itself:
1. the header (first 64 KiB): codec `V_MS/VFW/FOURCC` MJPG, size, duration;
2. the Cues at the end of the file (about 170 KB): where each 0.5 s cluster starts;
3. whole clusters by range, one ahead, decoding each frame's JPEG and showing it at its timestamp.

When decoding falls behind, frames are skipped rather than delayed, and a seek jumps straight to the cluster
that holds the target time. The Android reader (`MkvReader.kt`, `AssCaptions.kt`) is unit-tested against a
segment written by the recorder's own muxer (`src/test/resources/recorder_fixture.mkv`). The web client has
the same reader in `app.js`.

**What it costs.**
- **Wi-Fi:** playback needs the original bitrate, about 16 Mbit/s for a bench scene, 41 at night, 73 for
  detailed daylight. On a slower link the player buffers.
- **Jetson CPU:** serving costs the agent about 16 ms of CPU per MB over TLS (measured 2026-11-26 on the
  Jetson, real footage). That is about 0.1 of a core at playback rates; a full-speed download uses more.
- **Speed:** listing 790 segments takes about 70 ms.

**What is not served.** Only names matching the recorder's pattern, as regular files in the folder, never
symlinks. Requests are by name, never by path. The segment still being written (modified in the last 10 s,
with no Cues yet) is neither listed nor served (409). A segment that retention deletes while it plays or
downloads ends with "Not found".

**Cut-short segments.** A segment the recorder never finished (power lost mid-segment) has no Cues and a zero
Duration, so the players cannot seek it. The agent reads each segment's header and its Cues position as the
players do, caches the result per file, and lists such a segment with `playable: false`. Both clients then
disable Play but keep Download, for recovery. On the bench Jetson, 5 of 831 segments were cut short
(2026-11-27). The first listing reads every header (1.25 s for 831); later ones use the cache (70 ms).

**Privacy.** Recordings are a location and time history. On the LAN they need the same pairing credential or
browser session as live view. Run the agent with `--no-recordings` to withhold them. That also withholds them from cloud sessions.

## Cloud recordings

In a cloud session the Recordings page lists the same segments, but it does not stream the original files.
The Jetson's uplink is often on mobile data, and the originals run at 16–73 Mbit/s. A viewer instead exports a
**clip**: part of one segment, made into an H.264 MP4 on the Jetson and uploaded to Cloud Storage. Owners and
viewers granted the vehicle may both export clips.

1. The viewer asks for the list. The relay forwards `{type: "list_recordings"}` to the uplink. The uplink reads
   the agent's `/v1/recordings`, keeping at most 1,500 segments.
2. The viewer chooses the start (seconds into the segment), a length (10 s, 30 s, 1 min or 3 min), a quality and
   whether to burn in the captions. The relay validates the request. It makes a random object name under
   `clips/<vehicle>/` and signs a PUT URL for it (15 minutes). It answers `clip_status: exporting` and sends the
   uplink `export_clip` with that URL.
3. The uplink accepts only an `https://storage.googleapis.com/` upload URL. It POSTs the request to the agent's
   `POST /v1/recordings/<segment>.mkv/clip` and PUTs the returned MP4 to the signed URL.
4. The relay signs a play link and a download link, each valid for an hour. The download link carries the clip's
   file name. The page shows a player and a **Download MP4** button. On Android the player is fullscreen and the
   download goes through the system download manager to *Download/Dashcam*.

**The export on the Jetson** runs `ffmpeg` (Ubuntu's 4.4 package) at `nice 15` with two x264 threads, one export
at a time (a second one answers 503 "busy"). The JPEG decoder runs at half size for **Small**.

| Quality | Output | 30 s clip | 3 min clip | Size per 30 s |
|---|---|---|---|---|
| Small | 960×540, 10 fps | 9.7 s | 82 s | 3.8 MB |
| 720p | 1280×720, 30 fps | 21.4 s | 127 s | 5.5 MB |

These were measured 2026-11-26 on the Jetson from a night-drive segment with captions burned in. Each export uses
about 2.2–2.4 cores while it runs. The recorder keeps priority, but the CPU is shared, so avoid long 720p exports
while the Jetson is under load. Burned-in captions keep the recorder's styles and clock. Their timing follows the
clip's place in the segment: at 5 s into a clip that starts 60 s into a segment from 22:35:50, the caption reads
22:36:55.

**Limits.** A clip is at most 180 s and comes from one finished segment. The segment still being written
answers 409. A start at or past the segment's end answers 416, and ffmpeg runs with `-abort_on empty_output`,
so an empty MP4 is never uploaded. Export time is bounded at 60 s + 3 × the clip length.

**When things go wrong.** Each pending request belongs to the vehicle socket it was sent on. If that socket
closes or is replaced, the viewer gets an error at once. A silent vehicle gets one at a deadline: 60 s for a
list, 10 min for a clip. A signing failure ends only that request; the vehicle and viewer sockets keep serving.
In the browser, export progress and links are kept per segment, so a list refresh keeps them. A request whose
socket closes (for example a hidden tab) fails at once. An interrupted list reloads on the next connection.
An interrupted export offers **Export again** and is never resent automatically, since resending could make
the clip twice. Clip objects are deleted by the bucket's
lifecycle rule a day after upload. Links expire after an hour; export again for new ones. Setup:
cloud/README.md "Cloud recordings bucket".

**Privacy.** A clip is a location and time record that leaves the vehicle. Only a signed-in user with a grant for
the vehicle can ask for one. The relay re-checks the sign-in and the grant on every request. Anyone holding a
link can fetch the clip until the link expires, so share links with care.

## Vehicle map

The **Vehicle** page of the Android app and of the web client shows a **Map**
card above the GNSS position: the car as a blue dot with a heading arrow while
the vehicle data is fresh, a grey dot labelled "Last known position" once it goes
stale (kept in client memory only, never stored), and "No GNSS fix yet" over a
plain background until the first fix. It follows the car by default; dragging,
pinching or (web) mouse-wheel zooming switches to a free view, **Recentre**
follows the car again at the current zoom, and **+/−** zoom (levels 3–19,
default 16). The Live page is unchanged.

- **Tiles come straight from OpenStreetMap** (`https://tile.openstreetmap.org`,
  256 px Web Mercator), not through the Jetson: the phone or browser device needs
  its own internet connection. The bench router's DNS blocks openstreetmap.org,
  so there the map shows its plain background (plus any tiles already loaded at
  nearby zooms, which a failed tile never covers) with "Map tiles need internet";
  everything else on the page is unaffected, and a failed tile is retried no
  sooner than 30 s later.
- **Tile usage policy**: requests start only after a fix and only while the
  Vehicle page is shown and the app/page is in the foreground; only the tiles
  covering the view plus a small margin, at most four at a time, no prefetching
  of other zooms or areas, no bulk download. Android identifies itself as
  `DashcamConnect/<version> (Android; com.dashcam.connector)` through a separate
  HTTP client (platform trust, no Jetson pin or credentials) with a ~50 MB
  HTTP cache; a browser sends its own User-Agent and, for tile images only, the
  page origin as Referer (`referrerpolicy="strict-origin-when-cross-origin"`;
  the page itself stays `no-referrer`).
- **Attribution**: "© OpenStreetMap contributors" is always visible on the map
  and opens <https://www.openstreetmap.org/copyright>.
- **Privacy**: tile requests reveal the car's approximate position (the tiles
  around it) and the device's IP address to the OSM tile servers. Nothing is
  requested before the first fix or while the Vehicle page is not shown; stay off
  the Vehicle page if that is not acceptable.
- **CSP**: for the web client the agent's Content-Security-Policy adds exactly
  `https://tile.openstreetmap.org` to `img-src`; `connect-src` stays `'self'`.
  Tiles are plain `<img>` elements (no pixel reads), so no CORS is involved.

## Browser access

The agent also serves a small web client (`connector/web/`: plain HTML/JS/CSS,
no build step, no frameworks, no analytics, no external resources except the
OpenStreetMap tiles of the [Vehicle map](#vehicle-map)) with the same
pages as the Android app: **Live** (camera preview, see
[Camera video](#camera-video), speed and source, recording, fps, frames,
bridge age, connection state), **Vehicle** (speed, map, powertrain, GNSS
position, telemetry link), **System** and **Connection**
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
and paste either link into **Connection → Pair**. A pasted link made for another
address (its host:port differs from the page's) is refused before anything is
sent, so this agent never receives another Jetson's token. The pasted link may remain in
the clipboard or the address-bar history of that device: clear them on shared
devices. Bookmark `https://<host>:8443/` afterwards.

**Sign out** deletes the cookie on that browser. Sessions are stateless (an
HMAC of the expiry with a key derived from the identity token), so agent
restarts keep them and a copied cookie stays valid until it expires; only a new
identity revokes all browser sessions (and all Android pairings). Expiry is
checked against the Jetson's wall clock: if it is set wrong at sign-in, or later
jumps, sessions can expire early or last longer than 30 days. An expired or
revoked session shows the "not paired" state with instructions. A page left open
does not outlive its session: the agent closes its telemetry and video sockets
(WebSocket close 1008, "Session expired") when the cookie's expiry passes.

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
  images only from the agent, `data:` or `blob:` (the camera frames) and
  `https://tile.openstreetmap.org` (the [Vehicle map](#vehicle-map) tiles), no
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

## Cloud uplink library (Plan A)

For the pilot prerequisites, Firebase setup, Android signing configuration and first mobile-data test, see the [Google Cloud integration guide](GOOGLE_CLOUD_SETUP.md).

For native Jetson integration, [libclouduplink](../lib/libclouduplink/README.md)
provides the C++17 static library, event-loop contract and build instructions.

The [on-demand uplink core](cloud_on_demand.py) implements the Jetson-side
control lifecycle from [Plan A](CLOUD_ON_DEMAND_PLAN.md). It uses Python 3.10+
and the standard library. Firestore sign-in/listening, relay transport and local
media subscriptions are adapter responsibilities. The current LAN agent runs
independently.

Create one `OnDemandUplink(device_id, on_change)` per device on its asyncio
loop. Pass `ControlSnapshot.from_document(path, fields)` to `apply_control`
with the listener's trusted `server_confirmed` flag. Use the actual document
path and snapshot metadata from the authenticated listener. For SDK callbacks
on another thread, dispatch onto the owning loop. Control fields are
`desiredMode`, `configVersion`, `demandGeneration`, `channels` (a list
containing `telemetry` and/or `video`), and `expiresAt` (timezone-aware
timestamp or Unix seconds). Empty demand may omit expiry.

The callback receives the complete desired `RelayTarget`. Schedule I/O from
the callback and serialize reconciliation against the latest target:

1. A new attempt requests one authenticated relay socket. Register the listed
   generation and channels. Open local readers only after `relay_confirmed`
   grants them for that attempt and generation.
2. Follow `active_channels` for the existing local `/v1/stream` and
   `/v1/video` readers. An idle target releases all readers and the relay.
   Check the current target before publishing an asynchronous result.
3. Report `relay_lost(attempt)` on disconnect. After backoff, reconcile a
   fresh authoritative snapshot. A rejected generation stays idle until demand
   advances. Lease renewal with the same generation and a later expiry keeps
   a healthy grant alive; silent expiry releases it automatically.
4. Call `close()` on shutdown, deletion of the enrolled device's control
   document, or credential revocation. A future mode supervisor owns the
   handoff to Ready while powered.

Run the offline lifecycle tests from the repository root:

```sh
python3 -m unittest discover -s connector -p test_cloud_on_demand.py -v
```
