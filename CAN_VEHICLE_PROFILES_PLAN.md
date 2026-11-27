# Driver-selectable CAN vehicle profiles

Implementation plan for dashcam v0.4 · 27 November 2026

## Release scope

The driver chooses a vehicle in **ES3 Settings**. The Jetson remembers the choice, loads that vehicle’s DBC and supplies decoded values to the live recording overlay. A headless operator can make the same choice. Offline replay uses the same C++17 decoder and can reproduce the profile in effect when each frame was captured.

The first two targets are the **2019 Honda Brio DD1 CVT** already measured by this project and the **2022 Hyundai Tucson diesel sold in Vietnam**. There is currently no DBC or capture from the target Tucson. Deliver the Brio and native acquisition independently of that evidence. List the Tucson as selectable for live use once its capture path and advertised signals have been verified.

The owner chose native C++ for **acquisition and raw logging as well as decoding**. Keep the MKR in its existing listen-only raw capture mode. The ESP32-C3 continues supplying GNSS, IMU and bridge telemetry. The ES3 needs a firmware change for the picker. The proposed process split, XML package format and activation rules below are engineering recommendations.

## Architecture and ownership

```text
MKR native USB ──► C++ can_capture (sole tty owner)
                    ├── bounded archive queue ──► raw log, stats, sync, console
                    ├── immediate frame decode ─► timestamped vehicle snapshot
                    └── profile authority ──────► selection events + profile assets
                                                    │
                         local Unix socket          │
                    can_capture ◄────────────────► dashcam_v0_4 ◄──► ES3 Settings
                          ▲                           ▲
                    headless C++ CLI             C3 bridge telemetry

raw log + recorded profile assets/events ──► C++ replay ──► decoded CSV
```

Run acquisition as one headless process so restarting the camera app, losing the display, or stopping video recording does not interrupt a drive capture. Keep DBC parsing, storage writes and USB reads off the camera and display threads. Use the same decoding library in the service and replay executable. The existing Python logger and legacy C++ decoder serve as reference implementations during migration.

| Change | Existing seam | Proposed work |
|---|---|---|
| Native acquisition and logging | [Firmware line contract](../peripherals/mkr_zero/lib/CANRawStream.h), [Python logger](../peripherals/mkr_zero/tools/canstream/mkr_stream_log.py), [logger tests](../peripherals/mkr_zero/tools/canstream/tests) | New C++ service and standalone capture entry point. Match the existing drive tools’ files, timestamps and loss accounting. |
| Profile packages and decoding | [Brio map](../peripherals/mkr_zero/config/canmap.brio.txt), [legacy C++ decoder](../peripherals/mkr_zero/tools/canstream/can_decode.cpp), [CAN frame type](../lib/libcan/libcan.h) | Add `lib/libcandb/`, `config/vehicles/<profile-id>/` and a small C++ CLI. Reuse `dashcam::can::CanFrame` with capture time and stream identity. |
| Recorder and settings | [v0.4 supervisor](../src/dashcam_v0_4.cpp), [overlay mapping](../src/bridge_overlay.h), [telemetry log](../src/telemetry_log.h), [config](../lib/libconfig/libconfig.h) | Subscribe to snapshots through a nonblocking local client. Merge by field and freshness. Add config for catalog, socket, writable state and initial profile. |
| In-car UI | [Jetson display owner](../lib/libes3display/libes3display.h), [ES3 Settings](../peripherals/es3c28p/lib/es3Graphics/es3GraphicsSettings.cpp), [control framing](../peripherals/es3c28p/lib/es3UsbLink/Es3Protocol.h) | Extend the existing display link with a capability-negotiated catalog, selection request and result. |
| Build and deployment | [Makefile](../Makefile), [capture launcher example](../tools/lane_eval/lane_run.sh), [canstream usage](../peripherals/mkr_zero/tools/canstream/README.md) | Add scoped build targets and one service owner. Update launcher/supervisor integration and the separately deployed drive-session launcher at cutover. |

## 1. Establish the vehicle and decoder contracts

**Tucson feasibility comes first.** Hyundai’s Vietnam specification identifies the diesel variant. Upstream opendbc puts its 2022 fourth-generation Tucson entry in a CAN FD platform, and separately lists a 2019 diesel in a different platform. Neither entry establishes the traffic available at this particular Vietnam vehicle’s connector. The MKR’s MCP2515 path and USB format currently support Classical CAN at the configured 500 kbit/s. [Hyundai Vietnam specification](https://hyundai-api.thanhcong.vn/storage/uploads/product/myTIhZsMPNIoJpb2JvkJIltqppccNjTJ5oKzDCzE.pdf) · [opendbc Hyundai platform definitions](https://github.com/commaai/opendbc/blob/master/opendbc/car/hyundai/values.py) · [Firmware bitrate](../peripherals/mkr_zero/lib/DataDictionary.h) · [Microchip MCP2515](https://www.microchip.com/en-us/product/MCP2515)

Record the Tucson trim, connector or approved tap, accessible bus, frame format and bitrate(s). Obtain a passive trace with an interface suited to the documented bus. Compare stationary brake, selector and indicator observations and an independently referenced speed trace with candidate definitions. Record which signals were visible and which could be independently checked. If the exposed bus is compatible Classical CAN, proceed on the MKR. If required traffic is CAN FD, plan a separate FD-capable acquisition input using the existing Jetson SocketCAN wrapper and suitable hardware. If the connector exposes insufficient broadcast data, offer only verified fields or keep the profile experimental. None of these outcomes blocks Brio delivery.

**Package one DBC and one XML manifest per supported variant.** Use pugixml, already in the build. Give each package an immutable ID and revision, driver-facing label, market/model years/powertrain, capture requirements, DBC source and license, and bindings for the fields it provides. Keep bit layout, signedness, byte order, scale and offset in the DBC. Put signal aliases, canonical units, gear enum mapping, invalid values, freshness windows, indicator behavior and optional vehicle-specific calibration in the manifest. For redistribution, record the upstream revision and use a self-contained generated DBC if its source uses generator imports.

For example, the proposed Brio package can bind the canonical `speed_kmh` field to `XMISSION_SPEED.VehicleSpeed` in a locally authored DBC whose names we control. Encode the measured Brio layout there. Verify that the Brio pedal’s raw counts convert to the percentage displayed today, EPS assist stays labeled as relative effort, gear codes are mapped explicitly, and missing hazard or steering-angle signals stay unavailable. Retain the documented indicator hold for parity. Expose wheel-derived yaw only with the appropriate calibration and label it as derived. [Measured Brio definitions](../peripherals/mkr_zero/config/canmap.brio.txt) · [Existing derived-signal logic](../peripherals/mkr_zero/lib/CANSniffFunctions.cpp)

Use **dbcppp** behind a small C++ adapter. Pin a tested revision, its build dependencies and notices after an ARM64 build spike. At load time, resolve each binding to a message and signal, validate types/units and build an immutable lookup keyed by bus, standard/extended flag and ID. Validate message length before decoding. Support signed values, both byte orders, factor/offset, enums and tested multiplexor branches. Reject bindings that require unsupported features with a profile-specific diagnostic. Decode only mapped signals, skip remote frames, and publish unavailable for unknown enum codes or stale values. [dbcppp API and supported constructs](https://github.com/xR3b0rn/dbcppp)

**First executable milestone:** `candb list`, `candb validate PROFILE` and `candb replay --profile PROFILE INPUT`. Compare the normalized Brio outputs with the legacy decoder at identical sample times. Use a small synthetic second profile with overlapping IDs and different encoding until a Tucson capture is available. A synthetic fixture tests the software and gives no vehicle-compatibility claim.

## 2. Port the raw logger and prove parity

1. Give the native reader exclusive ownership of the MKR tty. Match by board identity or configured device path, preserve DTR/open/close behavior, and handle unplug/replug, partial reads, interleaved console lines and a stale first USB packet. Parse the firmware’s `F` and `FS` records with explicit standard/extended ID, RTR, DLC and bounds checks.
2. Keep the MKR drain timestamp, host monotonic arrival time, capture instance and timeline ID. Port timer-wrap, board-reset, drift, delivery-backlog and wall-clock-step handling. The raw archive keeps the existing delayed timestamp correction. Live decoding uses a causal estimate of frame age, invalidates values if the clock is uncertain after reconnect, and discards stale bursts for display even when they remain valuable for replay.
3. Fan out before the disk hold. The USB reader must continue during a stalled writer or disconnected subscriber. Bound memory and count whole-record drops on each branch. Preserve the existing candump representation and the schemas of `can_raw.log`, `can_stats.csv`, `can_sync.csv` and `mkr_console.txt`, including per-file drops and hardware overrun counters. Distinguish an overrun event from a known number of lost frames. Preserve raw output even with no profile selected.
4. Run the same pseudo-terminal input against both implementations in separate runs. Compare raw IDs/flags/payload/order, counter deltas, timeline splits, timestamps within the current test tolerances, console content and queue-loss reports. Exercise hung/full disk, host disconnect, reset, timer wrap, late bursts, wall-clock jumps and orderly shutdown. Measure CPU, memory and losses against the Python baseline on the Jetson with cameras recording. [Reference scenarios](../peripherals/mkr_zero/tools/canstream/tests/test_stream_log.py) · [Capture performance harness](../peripherals/mkr_zero/tools/canstream/tests/perf_stream_log.py)

**Cutover criterion:** the native logger passes the current stream contracts and sustained-load comparison. Transfer ownership from the legacy launcher to the native service in one deployment step. Configure the lane and drive launchers to consume the owner’s session files or run the native standalone mode when the service is stopped. On an already-owned tty, report the conflict and leave the current owner running. Rollback stops the native owner before restarting the original logger.

## 3. Implement selection, persistence and replay

- The capture service is the single writer for the selected profile. Initial boot uses a saved valid selection, otherwise the explicitly configured initial profile on first setup. With neither, start in **Raw capture only**. A missing/corrupt saved package enters raw-only mode with its reason visible. Headless selection uses `candb select PROFILE` through the same control path as the ES3. Keep the writable selection outside the installed catalog.
- Expose a versioned local Unix-domain protocol for catalog, status, selection and latest snapshot. Include request ID, service instance, profile revision/generation, validity and monotonic timestamps. Limit message size and queued output. A slow subscriber can skip superseded snapshots. After reconnect it reads current status and a full snapshot. Selection retries with the same request ID are idempotent.
- Validate and prepare a candidate off the capture thread. Serialize concurrent selections and atomically persist the chosen revision (temporary file, flush, rename and directory sync). Commit the change at a recorded frame boundary, clear all old signal and derived state, and acknowledge success after commit. A failed selection reports its reason and keeps the previous valid selection. Define recovery for a crash between durable intent and activation using a fresh stream instance and an explicit initial activation record.
- Assign frames an unambiguous sequence within their capture instance. Store the active profile ID, revision and DBC/manifest content hashes at session start and on every switch. Keep an event index tying each change to the first affected raw record, accounting for archive drops, file rotations and restarts. Archive the exact package bytes once per content hash when permitted, or require an exact externally retained package for replay. Link the video session/segment to the capture instance and time range. A missing event index or missing exact package produces an explicit replay error or raw-only interval.
- Default replay to recorded selection history. For old logs without metadata, require an explicit profile. Provide an override for retrospective decoding and mark the override in its output. Keep the original capture intact.

## 4. Connect the recorder and driver controls

Give each canonical field a value, source, validity and last sample time. The overlay samples a coherent snapshot on its normal update tick. Prefer fresh profile-derived speed, then the existing GNSS and OBD-II fallback policy. Preserve legacy bridge behavior when the feature is disabled. With profile mode enabled, suppress stale or unrelated bridge-sniffed fields after a switch. Keep bridge GNSS, IMU and independently sourced fields available during CAN loss. The detail overlay should show the same speed, gear, RPM and flags as the normalized recording output. Keep the existing raw bridge CSV interpretable and write normalized vehicle data with profile provenance separately. Recording should start and continue with either USB link absent.

Extend the existing ES3 host-controlled request/reply direction. Add a capability probe, bounded catalog pages, a polled pending selection carrying a request token, and a host reply with the applied profile and status. Fit the existing 128-byte control payload and reserve command numbers against the video, map and pan extensions. Update both protocol header copies and golden vectors. Start the Jetson display link for the selector even when preview and navigation are disabled. Older ES3 firmware can keep using its existing tabs while headless selection works.

In Settings, show model/year/powertrain and market where needed, highlight the active variant, and distinguish **receiving signals**, **waiting for expected frames**, **capture offline** and **raw capture only**. Confirm a selection only when the Jetson acknowledges it. After either side reboots, refresh the catalog and active selection from the Jetson. This avoids two SD cards disagreeing on which car is active.

## 5. Sequence, effort and release evidence

| Deliverable | Architectural effort | Evidence to move on |
|---|---|---|
| A. Brio package, C++ decoder and replay CLI | One library, one CLI and a profile package. Local to the Jetson. | Legacy Brio parity, independent bit-level vectors, synthetic second profile and actionable rejection of malformed input. |
| B. Native acquisition and archive | Largest work item. Ports a mature Python logger’s clock and loss behavior, adds process lifecycle and deployment ownership. Can proceed alongside A. | Old/new logger comparison, preserved tool compatibility, bounded disk-stall behavior, measured Jetson load. |
| C. Selection service and recorder | Crosses one local process boundary and the v0.4 overlay/recording boundary. Depends on A and B. | Switch A→B with the same numeric CAN ID carrying a different value. Verify no A values survive, failed switch keeps A, selection survives power cycle, and replay reproduces each interval. |
| D. ES3 picker and headless operation | Touches the host and display firmware on the existing USB link. Depends on C. | Select/reboot/reconnect from both entry points. Check old display firmware, UI-only link mode, raw-only mode and absent MKR. |
| E. Vietnam Tucson package | Data-only when its accessible bus and signal semantics fit the implemented transport and bindings. Transport work is conditional on evidence. | Publish exact variant, connector, frame format, source revision and per-signal validation results. Demonstrate the supported fields on the actual car. |

For release, replay an annotated Brio drive and a verified Tucson capture, compare the live and replayed **decoded values at matching frames**, and exercise a drive with an unsupported selection or silent bus. The live overlay can suppress a delayed sample that offline replay decodes correctly at its historical time. Verify fresh data appears by the next normal overlay tick after publication and use frame-age measurements to detect accumulated buffering. Verify camera recording continues through capture-service restart and profile errors.

In code-volume terms, budget for **several thousand lines across production C++, tests and capture fixtures**, with the broadest review on native logging and the two ends of the display protocol. This is a planning allowance, not a measured diff. Additional cars with an already supported capture path generally add a package and replay fixtures. The Tucson transport decision is the main item that could introduce another hardware-facing module.
