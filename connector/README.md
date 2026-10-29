# Dashcam Connect — Android + Jetson monitoring

Read-only vehicle telemetry, recorder state and host health for `dashcam_v0_4`.
Phone portrait and tablet landscape use the same adaptive Android app. No
internet, camera preview, map, commands or background monitoring is required.
v0.4 has no integrated ADAS: lane/driver capabilities explicitly remain false.

## Architecture and protocol

`v0.4 → nonblocking Unix datagrams → host Python agent → TLS WebSocket → Android`

The recorder emits one JSON snapshot per supervisor tick (~5 Hz), including
vehicle data even without a recording camera. It uses the existing bridge
overlay source selection. Unavailable/nonfinite readings become JSON `null`.
Its datagram send never waits for a reader. No network/disk work runs inside
camera or bridge callbacks. The publisher is disabled by default.

The agent runs **on the host**, samples health at 1 Hz and sends snapshots at
5 Hz. It accepts up to four WebSocket viewers, gives slow sends two seconds,
and retains only the latest local runtime snapshot. The agent uses the sender's
monotonic stamp, not its own receipt time, to discard queued/expired data.
v0.4 runtime data expires after two seconds; host health remains independently
available. Android expires the entire display after two seconds without a message,
using Android's monotonic clock. Clock corrections cannot rejuvenate readings.

Authenticated routes (all require `Authorization: Bearer <device credential>`):

| Route | Response |
|---|---|
| `GET /v1/identity` | Schema, device ID/name and capabilities; no secrets |
| `GET /v1/snapshot` | Current snapshot |
| `GET /v1/stream` | WebSocket, same snapshot envelope at 5 Hz |

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
  --state-dir ~/.config/dashcam-monitor --name 'Car Jetson'
```

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

Mount the **directory**, not the individual socket, so agent restarts work.
The recorder must run as the same UID as the agent, or as container root.
In the active dashcam XML file, under `<Network>`, add:

```xml
<MonitoringEnabled>true</MonitoringEnabled>
<MonitoringSocket>/run/dashcam-monitor/telemetry.sock</MonitoringSocket>
```

Restart the recorder during a suitable recording interruption. Its existing
network-control channel does not need to be enabled. Wi-Fi without internet is
supported; both devices must be reachable on the same router (no client isolation).

Display a pairing link on the Jetson:

```bash
~/.local/share/dashcam-monitor/venv/bin/python connector/service.py pair \
  --state-dir ~/.config/dashcam-monitor --host 192.168.0.15
```

Paste that private link into Android's **Connection → Pair a Jetson** field.
Transfer it through a channel you trust: it contains a reusable credential and
the SHA-256 fingerprint of the exact TLS leaf certificate. Android checks the
pin and certificate validity before sending credentials, then checks device ID
on every snapshot. It never falls back to unauthenticated HTTP or accepts a new
certificate automatically. Profiles are encrypted with an Android Keystore key;
app backup is disabled. Forgetting a device removes its local pairing. To revoke
all clients, stop the agent and provision a new identity in a **new** state
directory, update the service path, then pair each client again.

Optional mDNS advertises `_dashcam._tcp` with device ID and schema only. The app
can update a paired device's address from discovery; it keeps its original pin
and credential. Discovery does not grant trust. Manual pairing works if multicast
is blocked. Android 17 requests local-network permission; denial has a recovery
button. No GPS, microphone or camera permission is needed.

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

For an Android emulator, use host `10.0.2.2` in the simulator pairing command.
For a physical phone, bind the simulator to a LAN interface and pair that host's
LAN IP. Simulator readings are explicitly labelled `SIMULATED` in speed source;
the running service name also carries `[SIMULATOR]`. Do not confuse them with
vehicle evidence.

`src/tests/test_monitoring.cpp` covers serialization, nonfinite/stale readings,
socket delivery and bounded behavior with a missing/stalled agent. Build with the
project's include flags and GStreamer headers inside the Jetson toolchain.
Python tests cover auth denial, certificate pinning, two viewers, read-only
commands, client limits, runtime expiry, bad packets and host metric deltas.
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
connected. These checks must be distinguished from simulated/software results.

See [VALIDATION.md](VALIDATION.md) for the checks performed for this implementation.
