# ES3C28P position map — implementation plan

Date: 2026-10-14. Status: Jetson side implemented 2026-10-14 (see "Implementation
record" near the end); firmware side not implemented. Extension v2, board-side
panning (smooth movement), was added the same day at the user's request: see
"Extension v2: board-side panning" near the end (Jetson side implemented,
firmware side not). Covers the Jetson application and ES3C28P firmware. Existing
VIDEO_STREAMING_PLAN.md and VOICE_COMMAND_PLAN.md remain unchanged. Reviewed 2026-10-14 against the
code: the user's UI/network answers, the float-precision result, the build
dependencies, the five-page touch points and the STATUS polling cost are folded
in below; everything else was checked and stands.

## 1. Scope and decisions

Show the vehicle's current GPS position and direction of travel on an
OpenStreetMap-based map on the ES3C28P, over its existing USB connection.
Version 1 has no destination search, route calculation, turn instructions,
road snapping, compass fusion, or dead reckoning through GPS outages.

Confirmed by the user:

- Online maps with a local tile cache on the Jetson.
- GPS direction of travel is sufficient; actual vehicle facing direction
  while stopped or reversing is not required.
- Show a position dot whenever heading is unreliable or speed is below the
  heading threshold; show an arrow only with a reliable moving course.
- Stop the camera preview stream while the map display is in use. Camera
  recording continues; camera preview resumes only when returning to LIVE.

Confirmed by the user on 2026-10-14 (review pass):

- **A new MAP tab**, appended as page 5 (tab label `MAP`); existing pages keep
  their IDs.
- **North-up** map, rotating direction arrow, automatic centering on position.
  Heading-up is out of scope for v1.
- **Internet in the car comes from a phone hotspot or car WiFi** (the Jetson's
  existing NetworkManager path, `<Network><WifiConnectEnabled>`). The online
  tiles + cache design stands; see section 4 for data use and offline gaps.

Append page 5 without renumbering existing pages. Replace hard-coded five-page limits and 64-pixel tab hit testing with
shared tab boundaries: `edge(i) = floor(i * 320 / pageCount)`. Update host,
firmware, Python validation, capability reporting and tests together. Future
voice implementation must add MAP aliases and capability-gate page 5; record
that dependency here rather than rewriting the existing voice plan.

Places that hard-code five pages today (checked 2026-10-14) and must change
together, gated on the map capability so old firmware keeps working:
`lib/libes3display` `ControlRequest::page()` rejects index 5 and
`es3display_test` [18] asserts that; the firmware's `updatePageText()` and tab
drawing/hit testing in `es3Graphics`; the Python client's page validation in
`host/es3_display.py`; and the voice plan's page aliases. With six tabs each is
53-54 px wide (edges 0, 53, 106, 160, 213, 266, 320); the four-letter labels
(`LIVE`, `TRIP`, `CAM`, `SYS`, `SET`, `MAP`) still fit at the current font.

## 2. Existing code and integration constraints

| Area | Current behavior and consequence |
| --- | --- |
| `lib/libcommlink/HostProtocol.h` | Telemetry carries WGS-84 float latitude/longitude, `gpsSpeedKmh`, `heading`, fix validity/type and GPS presence/fix flags. No new GPS device reader is needed. |
| `peripherals/mkr_zero/lib/CommunicationFunctions.cpp` | Publishes `derived.headingDeg`: filtered course, not raw compass orientation. |
| `peripherals/mkr_zero/mkr_zero.ino` | Heading filter uses fresh PVT samples only, resets below the configured minimum speed, and can publish NaN while warming up or inconsistent. |
| `peripherals/mkr_zero/lib/DataDictionary.h` | Currently 4 Hz GNSS, 1,000 ms fix expiry, 5 km/h minimum heading speed. Reuse these semantics instead of introducing a competing heading filter. |
| `src/dashcam_v0_4.cpp` | Owns the C3 bridge, time/CSV callback, display link, preview and recorder. Display startup currently depends on DisplayPreview being enabled. |
| `lib/libes3display/` | Single tty owner, control scheduling, one pending JPEG, credit-based E3 29 transport. Extend this owner; never open a second display connection. |
| `lib/libvideopreview/` | Move-only `EncodedFrameLease` and bounded `EncodedFramePool` can carry a composed map JPEG without going through camera conversion. |
| `peripherals/es3c28p/lib/es3Graphics/` | `updateVideo()` activates images only on page 0; the current viewport placement/status messages are video-specific. A map cannot just be offered to this implementation unchanged. |
| `peripherals/es3c28p/lib/es3Video/` | Baseline JPEG decode, RGB565 image buffers, token/credit lifecycle and partial strip drawing are reusable. |
| `lib/libdeadreckoning/` | Local odometry only; it supplies no absolute geographic position for this feature. |

Reading telemetry must preserve time synchronization, CSV logging and existing
overlay consumers. Add navigation to the existing callback fan-out or add an
atomic snapshot-with-receive-time API. Do not call the callback setter a second
time and silently replace existing consumers. The callback only copies a
small snapshot into a latest-value mailbox; it does no HTTP, rendering or
filesystem work. Register it whenever navigation needs it, including when
GPS clock fallback and CSV logging are disabled.

## 3. Architecture

```text
MKR GNSS -> C3 bridge -> CommLink -> latest navigation snapshot
                                               |
OSM raster tile endpoint -> HTTP/cache worker -> compositor + JPEG worker
                                               |
camera preview producer ----------------> display source coordinator
                                               |
                                      Es3DisplayLink (one tty)
                                               | USB
                               bounded JPEG receiver + JPEGDEC
                                               |
                                  graphics loop -> map page
```

The Jetson owns projection, tile fetching/caching, composition, marker and
direction drawing, map labels and JPEG encoding. The ESP32 owns USB framing,
decoding, display, touch, local stale indication and page transitions. It needs
no internet connection, API key, tile database or geographic projection code.

Only one image source owns the receiver at a time: camera preview on LIVE,
map on its configured page. Entering the map stops camera preview production
and USB transmission, including preview conversion work; it does not merely
hide arriving camera frames. Gate the recorder's optional preview tap before
copying frames, and discard queued preview work. The recorder's capture and
recording pipeline continues. Returning to LIVE restarts preview demand and
streams a fresh camera frame; other pages keep preview stopped. The planned
voice interaction temporarily suspends both image sources through the same
coordinator. Telemetry collection continues in every display mode.

## 4. Map service and cache

Use a configurable HTTPS XYZ raster tile endpoint returning 256 x 256 PNG.
The prototype can use `https://tile.openstreetmap.org/{z}/{x}/{y}.png` for
visible-map requests under its policy. No OSM editing API, Overpass or
Nominatim requests are needed for this feature. The service is best effort;
keep provider URL, attribution and policy settings configurable. [1][2]

OSM public tiles require application identification, visible attribution and
HTTP-aware caching; bulk/offline downloads are prohibited. Request only the
visible footprint, stop fetching when the map is hidden, honor validators and
expiry, and never add region-prefetch or offline-download features to this
backend. A future offline mode needs a provider that permits it. [2]

Implementation choices:

- C++17 library, libcurl for HTTP, libpng for tiles, libjpeg-compatible encoder
  already used by preview, SQLite for cache metadata/blobs. The `l4t-ml-gpio`
  image already has the development packages (checked 2026-10-14): libcurl
  7.81.0, libpng 1.6.37, sqlite3 3.37.2. `curl/curl.h` lives under
  `/usr/include/aarch64-linux-gnu`, so take flags from `pkg-config libcurl`.
- Application User-Agent, for example `DashcamES3Map/1.0`, with a real contact
  URL if supplied; no invented contact or generic library User-Agent.
- Cache key includes provider/style revision plus z/x/y. Store successful PNG,
  ETag, Last-Modified, freshness metadata and access time transactionally.
- Initial persistent cache budget: 256 MiB on Jetson storage, configurable.
  Evict least-recently-used eligible entries, excluding the visible working set.
  Honor cache directives; if retention requirements and space conflict, stop
  admitting new tiles rather than repeatedly downloading evicted fresh tiles.
- Revalidate expired tiles conditionally; never send blanket no-cache requests.
  Separate the persistent HTTP cache from the decoded-tile RAM cache.
- At most two active fetches and eight queued distinct tile keys. Deduplicate
  requests; discard obsolete queued work after a view change. An obsolete
  successful response may populate the cache but cannot replace a newer view.
- Connect timeout 2 s, total request timeout 5 s. Backoff with jitter, bounded
  at 60 s; honor longer Retry-After. No retry storm on 403/429/5xx. A 403 disables
  the provider for the session with a visible error; retry only after correction
  or explicit restart. Missing tiles get a short negative-cache entry.
- Verify TLS certificates; cap body at 512 KiB and require valid 256 x 256 PNG
  before allocation/decode. Reject HTML errors, corrupt data and oversized images.
- Online state comes from the existing network layer (hotspot up or down);
  while offline, issue no requests, serve the cache, and resume fetching on
  reconnect without a backlog burst (only the then-visible footprint).
- Hotspot data use, as an engineering estimate to verify: at zoom 16 a tile is
  about 600 m across (at lat 10.8), so at 60 km/h the view crosses a tile edge
  roughly every 36 s and needs about 3 new tiles; with 10-30 KiB per tile that
  is about 5-15 MiB per hour of new road, near zero on cached routes. Log the
  bytes fetched per drive.
- Fresh cached views still work through transient network loss. For expired
  entries, serve only where response directives permit stale reuse; otherwise
  show missing-map placeholders. Do not promise offline coverage.

Show `© OpenStreetMap contributors` legibly in a reserved strip, plus any
provider credit. A tap on the strip opens local map-information text with
`openstreetmap.org/copyright` and ODbL information; no browser is required.
Keep attribution visible whenever map imagery is visible, including retained
images and error states. Account for a copyright glyph in the chosen font. [3][4]

## 5. Position, course and freshness

Define a host-only `NavSnapshot` with latitude/longitude in double (promoted
from existing telemetry), course degrees, speed, validity flags, bridge receive
monotonic timestamp and connection generation. Promotion does not recover the
precision lost in float telemetry, but that loss is below the map's resolution
(computed 2026-10-14 from float32 spacing): at zoom 16 the coordinate step is at
most 0.71 px anywhere on Earth (0.36 px east-west at lat 10.8, lon 106.7); at
zoom 18 at most 2.8 px, near lon ±180. Both are smaller than GNSS error (a few
metres). **No telemetry change is needed.**

Accept coordinates only if GPS_FIX, fixValid and an accepted GNSS fix type
agree; require finite latitude in [-90, 90] and longitude in [-180, 180].
Do not reject (0, 0) merely because it is zero. Use GPS_PRESENT to distinguish
missing receiver from acquiring a fix. Accept heading only when position is
usable, course is finite and within its defined range, and GPS speed meets
the upstream 5 km/h gate. Normalize 360 degrees to zero after validation.

Use a fresh course as supplied. Do not apply another long smoothing window
on the Jetson: the upstream eight-sample filter already has latency. Handle
359-to-1 degree transitions circularly if visual interpolation is later added.
The arrow indicates motion, including reverse motion; it never claims to show
the vehicle's bonnet direction. With valid position but unreliable course or
speed below 5 km/h, draw a round position dot and `Direction --`. This includes
filter warm-up, inconsistent course samples, missing speed and non-finite
heading. Do not retain a previous arrow while these conditions apply. Restore
the arrow only when the upstream course is reliable and speed meets the gate.

Initial freshness policy:

- No telemetry or no valid fix: `Waiting for GPS` / `GPS unavailable`.
- Bridge sample older than 1,500 ms, disconnect, or explicit invalid fix:
  retain the last location as a hollow/gray marker labeled `Last known`.
- No dead reckoning, extrapolation or automatic road snapping. Never substitute
  zero coordinates on errors. Clear validity immediately on a new invalid fix.
  (Extension v2 moves the marker smoothly by interpolating between two received
  fixes, shown one fix interval late: still no extrapolation.)
- Existing MKR code invalidates a silent GNSS fix after 1,000 ms even while
  bridge telemetry continues. Test that behavior end to end. Transport receive
  time is not GNSS measurement time, and repeated bridge packets are not fixes.
- Current telemetry has no explicit GNSS sample age/sequence. Therefore display
  qualitative freshness rather than an invented precise fix-age value. If the
  current bridge can replay old payloads without marking them invalid, fix that
  provenance gap before claiming live position; negotiate additional sample
  age/sequence separately if required. Do not change the frozen display framing.

Position and map availability are independent. A fresh position can be shown
on a correctly projected blank grid when tiles are missing, with `Map unavailable`.
An old cached tile is not evidence that GPS is old, and a fresh tile is not
evidence that the vehicle position is current.

## 6. Viewport and rendering

Proposed map frame: 320 x 224, anchored at (0, 0). Reserve y=0..15 for compact
status, y=16..207 for map imagery, y=208..223 for attribution. The existing
local tab strip occupies y=224..239 and is always accessible. Compose all map
pixels on the Jetson; firmware may overwrite only the reserved status region
with local connection/stale messages. The marker remains inside the map area.

Start at zoom 16, configurable within 12..18 and provider-supported levels.
Automatic centering and fixed zoom are sufficient for v1; touch pan, pinch,
autozoom and zoom controls are deferred. Draw a north indicator, a high-contrast
direction triangle or position dot, and a small scale bar. Test the actual
256-pixel tile style for label readability on the 2.8-inch display.

Use double-precision Web Mercator world pixels: world width = 256 * 2^z;
x = (lon + 180) / 360 * width;
y = (1 - asinh(tan(latRadians)) / pi) / 2 * width. Clamp projection latitude
to approximately +/-85.05112878 degrees, wrap x modulo world width, clamp y,
and use shortest wrapped x differences near the antimeridian. Never alter the
stored geographic fix to implement projection clamping. Derive tile indexes
and source crop offsets from world pixels; test fractional edges. [5]

North-up needs at most six 256 x 256 tiles for a 320 x 192 view. Use a bounded
12-tile decoded RGB LRU (2,359,296 bytes). Compose into one of two RGB buffers
(430,080 bytes total for 320 x 224), then encode a baseline JPEG directly into
one of two 65,536-byte leased output slots. Allocate buffers at startup.

Target 4 fps maximum, driven by the current GNSS rate; coalesce updates to the
newest snapshot. Refresh status/health at least once per second even when
stationary. Network fetches happen only on cache misses/revalidation, not once
per rendered frame. Tile arrival can trigger a redraw within the same 4 fps cap.

**Warm map while hidden (added 2026-10-14).** The navigation service is part of
the long-running app and never stops when the MAP tab is hidden: it keeps
following the position, keeps the decoded working set of tiles around the
current view in the RAM LRU, and re-renders a fresh composed frame about once
per second (`WarmFps`, default 1), from the disk cache only. While hidden it
sends **no** network requests (OSMF policy: fetch only for the visible map);
tiles missing from the cache are fetched when the map becomes visible. Hidden
frames are not transmitted. When the MAP tab is selected, the service renders
from the warm working set at once (a RAM-only composition and encode, about
5-10 ms) and offers it as soon as START_MAP returns, so the first map frame
does not wait on disk reads, PNG decodes or the network. Switching costs the
two control round trips, the USB transfer and the board's decode and draw;
the 750 ms first-map target becomes a ceiling rather than the expected time.

Start with JPEG quality 80 and 4:4:4 sampling for map text. Measure JPEGDEC
compatibility/readability and encode time; allow bounded quality retries at
70 then 60 if the 65,536-byte cap is exceeded. Never silently truncate a JPEG.
If all attempts fail, retain the previous frame with an error indicator.
Map bandwidth ceiling from frame size and rate is about 256 KiB/s plus framing;
actual throughput and SPI latency must be measured, not assumed from USB speed.

## 7. Display protocol and source coordination

Keep existing E3 28 framing, E3 29 record bytes, CRCs, reason codes, golden
vectors and START_VIDEO semantics intact. Map frames reuse START/CHUNK/END,
RESULT/PRESENTED/CREDIT and JPEG limits. A source switch creates a new image
session token; camera and map never share an active token or credit pool.

Add a separately negotiated control extension in a new shared
`Es3MapProtocol.h`, mirrored byte-for-byte on host and firmware. The names
below are proposed; allocate unused command IDs only after checking voice
extension reservations. Unknown commands must retain their existing response.

| Proposed command | Contract |
| --- | --- |
| MAP_CAPABILITIES | Extension version, map page ID, supported viewport/format, map feature flags; bounded response <=128 bytes. Old firmware answers unsupported and remains usable for existing features. |
| START_MAP | Validated fixed-size settings using existing image limits plus target page and host view generation. Returns a fresh token and initial grant only after the graphics loop accepts the binding. Does not select a tab. |
| STOP_MAP | Stops only the specified map token; cancels buffered map work and unbinds the surface after active leases are safely released. Must not terminate a newer session. |

START_MAP binds token, map content kind, page ID, viewport (0,0,320,224) and
generation atomically. It uses the same receiver implementation as video, with
mode-specific configuration. Reject start while another image source owns it;
the coordinator must stop the old source first. Reject page/viewport/version
errors before changing the existing session. A timed-out start has unknown
outcome: reset/reconcile the session rather than replaying a mutation blindly.

Use existing STATUS polling to observe the active page; do not overload
unchanged reply bytes. STATUS is answered by the firmware's Arduino loop, and
the board's USB task reads nothing until that answer is ready (host reply
timeout 2 s, `loopReplyTimeoutMs`), so polling pauses image intake briefly
every time. Poll at up to 4 Hz only while no bound surface is visible (the
host must notice LIVE or MAP being selected); while a surface is visible, its
VISIBILITY record reports leaving the page, so poll at 1 Hz at most. Firmware
immediately hides a surface on a local page change and emits existing VISIBILITY
for the currently bound surface. Host polling selects which producer to start;
firmware never waits for that poll to prevent the old image drawing on a new page.

Source switch sequence:

1. Close old producer demand and bump the host generation; reject late offers.
   On entry to the map, gate camera preview input and conversion immediately;
   any already-running conversion may finish but its result must be discarded.
2. Cancel unsent frames; finish a partially written record before any next
   control record. Stop the old token using its source-specific stop command.
3. Firmware invalidates queued work while retaining actively leased decode/draw
   memory until release. The graphics loop clears the outgoing surface.
4. Start and bind the new surface, observe its token/grant, then offer only the
   latest matching-generation frame. Visibility depends on page, calibration
   and voice ownership, never simply on `_activePage == 0`.
5. Recheck the latest selected page after every completion. Rapid tab changes,
   voice cancellation or reconnect may obsolete the transition in progress.

Map stream defaults: assembly deadline 500 ms, image stale after 2,000 ms,
unavailable placeholder after 5,000 ms. These are separate from GPS freshness
and camera's existing shorter thresholds. Continued heartbeat/control traffic
must not keep a stopped map producer looking current. A local stale overlay
clearly qualifies the retained marker; no need for an extra marker wire format.

The existing host centers camera frames automatically: add explicit placement
for map (0,0), rather than letting a 224-high frame center at y=8 and overwrite
tabs. Preserve camera letterboxing and original status strings on LIVE. Maps
must work with preview disabled or no camera connected. With old firmware or
no PSRAM, show a useful map-unavailable state and keep ordinary controls usable.

Future tap-to-speak pauses the active image source via this coordinator. On
completion, use the final selected page to resume map or camera; do not resume
an obsolete pre-voice page. Map-information overlays and calibration own the
graphics loop until dismissed and also suppress image demand.

## 8. Libraries, ownership and configuration

Proposed host library `lib/libnavigation/`:

- `NavigationService`: lifecycle, latest snapshot, render pacing and statistics.
- `NavState`: pure coordinate/course validation and freshness transitions.
- `Mercator`: pure projection, tile selection and scale calculations.
- `TileProvider` / `TileCache`: bounded asynchronous HTTP and persistent cache.
- `MapCompositor`: crops tiles, draws overlays, emits a bounded JPEG lease.
- `DisplaySourceCoordinator`: camera/map/voice ownership and generation gates;
  place in a shared display component if that avoids navigation owning voice.

Proposed firmware `lib/es3Map/`: map mode/binding, extension parsing, pure
surface-state tests and local map statuses. Reuse `es3Video` buffers and decode
worker; do not allocate a second full image pipeline. Extend `es3Graphics`
surface routing and `es3UsbLink` dispatch. Core 0 remains USB/decode; the Arduino
loop remains sole TFT/touch/I2C owner. Do not perform filesystem operations or
network work from the USB task or decode callback.

Host render and HTTP/cache workers operate independently with bounded queues.
Hold mutexes only to swap small state or buffer ownership. Coalesce view/position
updates, cap all response buffers, check library return values, and keep callback
lifetimes explicit. Shutdown order: disable producers/callback delivery, stop
navigation and preview workers, release coordinator work, then stop display;
join the bridge before destroying any state its callback can reference.

Add `<Navigation>` configuration with enabled (default false), provider URL,
User-Agent, attribution, cache path/budget, zoom and max FPS. Runtime destination
page comes from negotiated capabilities. Keep provider credentials out of logs,
firmware and committed config. Store cache on a persistent container volume.
Navigation failure disables only navigation, without preventing application
startup or recording. Keep the display USB path explicit and distinct from C3.

Extract common display-link configuration/startup from the preview-only branch.
Start one link when any configured display consumer needs it. Existing
DisplayPreview configuration remains backward compatible; preview failure must
not stop a link still required by map or voice. No graphical desktop, display
server, CUDA or hardware video encoder is required for the map compositor.

## 9. Work packages and acceptance

1. **Lock UI and extension contract.** Tab and orientation are decided (MAP tab,
   north-up); specify
   command byte layouts, bounds, capability failure and token transitions;
   add golden vectors for the extension while preserving all existing vectors.
2. **Pure navigation state and renderer.** Synthetic fixes and checked-in tiny
   synthetic tiles; projection, marker, attribution and baseline JPEG output.
   Produce sample 320 x 224 artifacts for visual inspection.
3. **Tile provider/cache.** Fake HTTP server tests for miss/hit/304/expiry,
   invalid PNG, timeouts, 403/429/5xx, disk full, cancellation and restart.
   Real endpoint smoke test requests only the displayed view.
4. **Firmware surface support.** Add capability/start/stop map and mode-aware
   graphics routing, with simulated image streams before wiring real GPS.
5. **Host coordinator/application.** Share the existing tty, add independent
   navigation startup, connect telemetry and preserve preview/recording behavior.
6. **Hardware validation.** USB/GPS/network disconnects, tab changes, memory,
   display readability and concurrent recording. Update an implementation record
   with measured results, remaining limits and exact deployed build versions.

Tests and measurable targets:

- Projection landmarks, tile boundaries, fractional crops, dateline wrap,
  polar clamp and 0/90/180/270-degree marker directions. No flipped axes.
- No-fix, NaN/out-of-range coordinates, legitimate zero coordinates, NaN course,
  stop/start, reverse travel and 359-to-1 transitions. No arrow from invalid data.
  With a valid position, verify dot display below 5 km/h or with unreliable
  heading, including a finite but unusable course; restore the arrow only when
  both speed and course are usable.
- Freeze GNSS while bridge continues; disconnect C3; adjust system wall clock;
  ensure monotonic freshness and last-known states still work. Include source
  invalidation time in reported GPS-loss latency, not just host timers.
- Extension C++/Python vectors, fragmented/CRC-bad input, timeout recovery,
  unsupported firmware, credit exhaustion, invalid token and reconnect tests.
- Rapid map/LIVE/TRIP/SET switches, page change during partial draw/decode,
  token replacement and planned voice mode transitions: no cross-page image,
  credit leak, stale late frame or unexpected calibration.
- While the map is active, verify zero new camera preview input copies,
  conversions or camera USB frames after the transition has drained. Recording
  continues; returning to LIVE displays fresh footage, not a queued old frame.
- Map enabled with preview disabled, no camera, no SD, missing display, missing
  GPS or failed provider. Recording remains operational in every case.
- Aim for <=500 ms p95 from receiving usable bridge telemetry to presenting a
  cached-map update at 4 fps; report upstream heading-filter delay separately.
  Aim for <=750 ms cached first map after tab selection and <=150 ms local
  touch feedback. These are targets requiring hardware measurement.
- Measure CPU, RSS, cache size, frame bytes, render/encode/decode/blit p95,
  USB throughput and recording drops with and without maps. Initial navigation
  budget: <32 MiB incremental host RSS, no unbounded queues, no duplicate ESP32
  image pool. Reduce refresh rate before affecting capture/recording.
- Run >=100 page transitions and >=30 minutes with recording active and forced
  network/GPS outages. No memory growth, missing attribution or live-looking
  position after its source becomes invalid. Retest normal camera streaming.

No board flashing, package installation, API registration or service deployment
is part of this planning task.

## Implementation record: Jetson side (2026-10-14)

Implemented and tested without the board; nothing is flashed. The firmware
side of the map is **not** implemented yet (see "Firmware work" below).

**Map extension v1, frozen** (`Es3MapProtocol.h`, byte-identical in
`lib/es3UsbLink/` and `lib/libes3display/`; vectors `tests/map_golden_vectors.txt`,
C++ `tests/map_protocol_test.cpp` and Python `test_usb_client.py`):

| Cmd | Request | Reply |
| --- | --- | --- |
| 0A MAP_CAPABILITIES | empty | 16 B: version u8, page_count u8, map_page u8, viewport x/y/w/h u16, max_jpeg_bytes u32, flags u8 (bit0 map tab) |
| 0B START_MAP | 27 B: version u8, page u8, viewport x/y/w/h u16 (must equal the advertised one), max_jpeg_bytes u32, chunk_bytes u16, target_fps u8, assembly/stale/placeholder ms u16, generation u32 | 8 B: session_token u32, initial_grant u32 (as START_VIDEO) |
| 0C STOP_MAP | session_token u32 | empty; a stale token is a no-op |

Map frames are ordinary E3 29 records under the map token. One receiver: a
START_MAP while a video session exists (or START_VIDEO while a map session
exists) is refused with NotReady. The voice plan's future commands must take
ids above 0E (0D/0E are extension v2).

**What the Jetson does:**
- `lib/libes3display`: asks MAP_CAPABILITIES after CAPABILITIES (`DisplayLinkConfig::map`);
  BadCommand/NotReady = no map, everything else unchanged. With a map board it
  polls STATUS for the active page (250 ms while nothing is shown, 1 s while a
  surface is shown, and at once when the shown surface turns hidden), and binds
  the receiver to the camera on LIVE and to the map on page 5: STOP_x then
  START_y. Other tabs keep the current session, paused by VISIBILITY. The camera
  preview's demand is inactive whenever the map owns the receiver (no converter
  work, the recorder tap's frames refused before copying). `mapSink()` is the
  map's IPreviewSink; PAGE index 5 is accepted only when the board has 6 pages.
- `lib/libnavigation`: NavState (freshness: Live / Last known after GpsStaleMs or
  an invalid fix / Acquiring / No receiver / No data; arrow only with a usable
  course at >= 5 km/h), Web Mercator, tile PNG validation, HTTP caching rules,
  SQLite cache (LRU budget sparing the view), compositor (status strip, north-up
  tiles, marker, north arrow, scale bar, attribution strip; 5x7 font from
  Adafruit GFX glcdfont, BSD), 4:4:4 baseline JPEG with retries at 70/60, and
  NavigationService: 4 fps shown, **warm** 1 fps hidden (compose only, disk
  only, no requests), 2 tile workers, newest-view queue of 8, 403 = provider off
  for the run, 429/5xx/offline = jittered backoff up to 60 s (Retry-After
  honoured), stale-if-error/stale-while-revalidate honoured, no request while
  the map is hidden.
- `<Navigation>` config (off by default); `dashcam_v0_4` starts one link when
  the preview or the map is enabled, fans the bridge telemetry out to the map
  (no second callback), marks the position "last known" when the bridge drops,
  and stops navigation -> preview -> link at shutdown. `launchcode_v0_4.sh`
  mounts `$MEDIA_DIR/navcache` for the tile cache.

**Deviations from the plan (deliberate):**
- HTTP timeouts: connect 4 s, abort when a transfer stalls (< 1 KiB/s for 6 s),
  cap 20 s, instead of connect 2 s / total 5 s. Measured on this bench's link a
  34 KiB tile needed more than 5 s while still arriving; a short total timeout
  would abort and re-download it (wasted hotspot data, unfriendly to the server).
- Eviction: plain LRU beyond the budget, sparing the visible tiles; it does not
  stop admitting new tiles when fresh ones would be evicted.
- Attribution and small labels use the 5x7 font at 1x (legible at arm's length;
  check on the panel), status at 2x.

**Measured (synthetic tiles; `navigation_test`, `es3display_test` [20]):**
compose 0.6-1.0 ms, encode 1.4-3.9 ms (320x224 q80 4:4:4); warm map on screen
4 ms after the tab is shown (cold: 53 ms incl. disk + PNG); tap on MAP -> first
map frame at the board 40 ms on a pty (STATUS poll + STOP_VIDEO + START_MAP).
Real OSM tiles (2026-10-14, `navigation_test --live 48.8584 2.2945 16`, Paris,
zoom 16): the view needed 4 tiles, 128 KiB (about 32 KiB each, ~0.45 s each over
this link); the composed frame is 33.8 KB at q80 4:4:4 (half the 64 KiB cap),
encode 5.9-6.2 ms, compose 0.6 ms. Dense city areas may be larger: the q70/q60
retries cover the cap. DNS note: this bench's router (192.168.0.1) answers
NXDOMAIN for every openstreetmap.org name while 8.8.8.8 / 1.1.1.1 resolve them
(the test used `docker run --dns 1.1.1.1`); a phone hotspot uses its own DNS, but
check `getent hosts tile.openstreetmap.org` on the car's network before relying
on the map.

**Firmware work (not done; for the firmware author):**
1. Answer 0A with page_count 6, map_page 5, viewport 0,0,320,224, flags 1; add
   the MAP tab (six tabs, edges 0/53/106/160/213/266/320) and accept PAGE index 5.
2. 0B/0C on the USB task like START/STOP_VIDEO: validate with
   `es3map::decodeStartMapRequest`, bind the receiver to the map surface with
   `es3map::sessionLimits()` as the VideoState config, NotReady while a video
   session exists (and the reverse for START_VIDEO), STOP_MAP no-op on a stale token.
3. VISIBILITY of the bound surface: active only on its tab (map on page 5),
   hidden elsewhere, calibrating during calibration; re-sent after START_MAP.
4. Draw map images at 0,0 (320x224, above the tabs); never draw over the bottom
   attribution strip (y 208-223). Local stale / "Map unavailable" notices go in
   the top strip only, with the START_MAP stale/placeholder times (2 s / 5 s).
5. Report the active page in STATUS byte 4 (already done).

## Extension v2: board-side panning (2026-10-14)

Asked by the user: the map should move like a video, not in 4 Hz steps. Chosen
(the user's "plan B"): the board scrolls the map itself. Whole frames at 10-15 fps
were the alternative; they would cost about 340-510 KB/s of USB (real map frames
are ~34 KB) and probably exceed what the board can decode and draw.

**How it works.** The Jetson sends a background larger than the map area
(512x384 by default, map imagery only) once every several seconds, the two fixed
strips ("chrome": status on top, attribution + scale at the bottom, 320x32) when
they change, and a 32-byte pose 20 times a second. The board crops the
background around the pose and draws the marker itself. At 60 km/h in Paris (real
OSM tiles, zoom 16) that was one background every ~12 s, 51-60 KB each, so about
5 KB/s of images plus 0.6 KB/s of poses, instead of hundreds of KB/s.

**Wire contract** (`Es3MapProtocol.h` v2 section + one record type in
`Es3VideoProtocol.h`; vectors `tests/pan_golden_vectors.txt`, C++
`tests/pan_protocol_test.cpp`; v1 bytes and vectors unchanged):

| Item | Bytes | Meaning |
| --- | --- | --- |
| 0D PAN_CAPABILITIES | request empty; reply 9 B | pan_version u8 (1), max_background_w u16, max_background_h u16, chrome_top_h u8, chrome_bottom_h u8, max_pose_hz u8, flags u8 (none). Firmware without v2: BadCommand, and the Jetson uses START_MAP + whole frames as before |
| 0E START_PAN | request 31 B; reply as START_MAP | the 27 START_MAP bytes, then background_w u16, background_h u16 (map area <= background <= the advertised maximum). Opens a map session: STOP_MAP ends it, START_VIDEO refused while it exists |
| background image | ordinary E3 29 frame | exactly background_w x background_h; START `source_timestamp_us` = `packOrigin(x, y)`, the world pixel of its top-left corner (u32 each) |
| chrome image | ordinary E3 29 frame | exactly viewport_w x (top_h + bottom_h); first top_h rows -> top of the viewport, the rest -> its bottom |
| E3 29 type 10 MAP_POSE | 16 B payload, frame_id 0 | seq u32 (1, 2, 3...; keep only increasing), x_q4 u32, y_q4 u32 (world pixels x 16: map-area centre and marker), course_cdeg u16 (0-35999, clockwise from north, for the arrow), marker u8 (0 none, 1 dot, 2 arrow, 3 last known), flags u8. Uncredited, never answered |

`es3map::computePanView()` is the shared integer arithmetic for the crop and the
marker position (no floating point on the board); `es3map::checkPanStart()`
classifies a START in a panning session.

**Firmware work (v2, for the firmware author; not done):**
1. Answer 0D with 1, the background maximum the PSRAM budget allows (the Jetson
   asks for 512x384; two RGB565 copies are 786 KB), 16, 16, 30, 0.
2. 0E like 0B (`es3map::decodeStartPanRequest`, `panSession()` for the geometry,
   `sessionLimits(request.map)` for the VideoState limits), same token, credit,
   NotReady and replacement rules.
3. In a panning session check STARTs with `checkPanStart()` instead of
   `checkStartForSession()`. A newer image may supersede only a not yet presented
   image of the same kind. Backgrounds need a decode target of background size
   (RGB565 in PSRAM), not a strip blit; chrome decodes like a small frame.
4. MAP_POSE: accept for the current panning token with seq > last seq and
   `decodeMapPose()` == Ok; otherwise ignore it. Keep the newest pose only.
5. Drawing (application loop, as today): top strip = chrome rows 0-15 at y 0-15,
   bottom strip = chrome rows 16-31 at y 208-223, map area y 16-207 = the
   background crop from `computePanView(origin, bgW, bgH, 320, 192, pose)`, then
   the marker at markerX/16, 16 + markerY/16 when markerVisible. Redraw the map
   area when the crop or the marker changes (up to the panel's rate; the Jetson
   sends at most max_pose_hz). Marker shapes as the Jetson drew them (blue
   20,100,255; white halo 255,255,255; grey 120,120,120): dot = white disc r 8.5
   + blue disc r 6; last known = white disc r 8.5 + grey ring r 7, 2.5 px wide;
   arrow = white triangle (0,14) (-10,-10) (10,-10) + blue triangle (0,11)
   (-7.5,-8) (7.5,-8) in units where y points up, rotated clockwise by the course.
6. Poses, like complete images, restart the stale/placeholder timers (images can
   be seconds apart while poses keep coming). Until a background and a chrome
   have arrived, show the local waiting notice. The board may drop both when the
   map is hidden: the Jetson resends them whenever VISIBILITY turns active.
7. Test with JPEGDEC that 4:2:0 baselines decode (backgrounds are 4:2:0 to fit
   the 64 KB slot; chrome is 4:4:4).

**Jetson side (implemented 2026-10-14, uncommitted):**
- `lib/libes3display`: asks 0D after 0A when `DisplayLinkConfig::mapPan`;
  START_PAN instead of START_MAP when the board has v2; `panSink()`
  (`navigation::IMapPanSink`): one pending background, one chrome (sent first),
  one pose (newest wins, at most max_pose_hz, sent between any two records,
  also between the chunks of a background). A START_PAN refused other than
  NotReady falls back to whole frames until reconnect. The whole-frame map sink
  stays inactive during a panning session.
- `lib/libnavigation`: `PoseInterpolator` (keeps only new fixes from the 10 Hz
  telemetry, smooths their 100 ms-quantised arrival times, shows the position one
  fix interval + 50 ms late, linearly between the two real fixes, holds past the
  newest; restarts after a 2 s gap or a 2000 px jump), the background planner
  (new background when the map area, now or 1 s ahead, would come within 8 px of
  its edge; centred 2 s ahead along the velocity, lead capped at 60% of the
  margin), `composePanBackground()` / `composePanChrome()` (scale bar and north
  mark move into the bottom strip; LOADING in the status strip while tiles are
  missing), and the service's panning loop (20 Hz poses, chrome every 0.5 s and
  at once on a marker/GPS-state change, sent only when it changed; background 4:2:0
  q75 with three retries 10 lower). The tiles of the next background are requested
  about 3 s before it is due, so it is complete when sent.
- `<Navigation>`: `Panning` (on), `PoseHz` 20, `PanBackgroundWidth/Height`
  512x384, `PanJpegQuality` 75. v0.4 passes `display.panSink()`.

**Deliberate choices / deviations:**
- Interpolation, not extrapolation: about 0.35 s of lag, but the marker is never
  somewhere the car was not. CAN speed / wheel-count odometry (`lib/libdeadreckoning`)
  could later cut the lag; it needs the calibration and telemetry plumbing first.
- Tile use: a background covers 512x384 instead of 320x192 (up to 9 tiles instead
  of 4-6), and the next background's tiles are fetched ~3 s early. Still only around
  the current position (no route prefetch, nothing while hidden); in the Paris
  demo 25 tile downloads for 1.9 km.
- Dense city backgrounds come close to the 64 KB slot at q75 4:2:0 (largest
  59.9 KB); the quality retries cover denser areas.

**Measured / tested:** `pan_protocol_test` (20 vectors, C++11/17);
`es3display_test` [21] (50 checks vs a panning fake board: 30 poses/s cap, seq
order, poses interleaved with a 55 KB background over a 120 KB/s reader, nothing
while hidden, fallback when START_PAN is refused, mapPan off) and [22]
(NavigationService -> link -> fake board over the pty); `navigation_test` [8]
(interpolation steadiness 1.86-2.17 px per 50 ms against 2.0 true, never off the
path; planner; images) and [9] (20.0 poses/s, east at 35.0 px/s against 35.4 true,
marker centred in the background offered before every pose). `navigation_test
--pan-drive OUTDIR` renders a simulated board at 30 fps over the real OSM drive
(output kept outside the repo).

## 10. Sources checked for this plan

1. [OSM API purpose](https://wiki.openstreetmap.org/wiki/API): editing/raw data
   API, not the display-tile service.
2. [OSMF raster tile policy](https://operations.osmfoundation.org/policies/tiles/):
   endpoint, identification, caching, availability and offline restrictions.
3. [OpenStreetMap copyright](https://www.openstreetmap.org/copyright): data credit
   and licence information.
4. [OSMF attribution guidance](https://osmfoundation.org/wiki/Licence/Attribution_Guidelines):
   readable attribution and access to licence details.
5. [OSM tile coordinates](https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames):
   XYZ tile addressing and Web Mercator projection.

External service policy is current to this research pass and should be checked
again when implementing or changing the provider. Numerical performance and
resource limits above are proposed engineering budgets, not measured results.
