# Original CYD navigation port

Implemented for the classic ESP32-2432S028, with ILI9341 and XPT2046 touch.
The ES3C28P implementation and its plans remain unchanged. This port uses the
existing whole-map fallback, as agreed, rather than retaining a large panning
background. It does not require PSRAM.

## Display and transport

The sixth tab, MAP (page 5), displays a 320 × 224 baseline JPEG; the bottom
16 pixels remain the local touch tabs. The Jetson renders the car position,
direction-of-travel arrow or uncertain-heading dot, status and OpenStreetMap
attribution into that image. The CYD does not calculate GPS heading or fetch tiles.

UART0 runs at 115200 baud through the board's CH340 USB bridge. Runtime text
logging is disabled so it cannot corrupt the binary protocol. Do not open a
serial terminal while the Jetson owns this port. Boot ROM output can still occur;
the protocol parser resynchronizes after startup.

The board identifies itself as `CYD`. The Jetson display link now explicitly
accepts that identity as well as `ES3C28P`. The protocol headers are included from
the ES3C28P directory, keeping the same wire format. Capabilities advertise one
credit, 1024-byte chunks, and a 32768-byte maximum JPEG. MAP_CAPABILITIES,
START_MAP, STOP_MAP, heartbeat/status, dashboard PAGE text and notification-audio
commands are supported. Panning and camera-start commands return BadCommand;
the Jetson's existing panning negotiation falls back to whole frames.

A single compressed-image buffer is owned by the application loop. The receiver
checks session tokens, frame/grant ordering, dimensions, offsets, length, CRC,
timeouts and visibility. It returns credit only after rendering finishes or the
frame is rejected. JPEGDEC 1.8.4 writes bounded MCU rows directly to the TFT,
without allocating a full RGB framebuffer. Progressive JPEGs are rejected.
Partial drawing is possible before a decode failure; the controller then clears
the map area and does not report PRESENTED. Decoder writes cannot touch tabs.

Leaving MAP or entering calibration stops demand and releases an unfinished
frame. Missing heartbeats reset the session after five seconds. Stale images get
a top-strip warning while retaining attribution; the configured placeholder
timeout clears the old map. Re-entering MAP waits for a fresh image. Display,
parser and receiver operations run on the application loop, avoiding display
access from another task.

## Jetson configuration

Merge these settings into the existing configuration; replace the device path
with the actual CYD serial device. Keep the existing tile-provider, cache and
attribution settings. No deployed configuration was changed by this port.

```xml
<DisplayPreview>
  <Enabled>false</Enabled>
  <Device>/dev/serial/by-id/REPLACE_WITH_CYD_DEVICE</Device>
  <MaxJpegBytes>32768</MaxJpegBytes>
  <ChunkDataBytes>1024</ChunkDataBytes>
  <MaxFramesInFlight>1</MaxFramesInFlight>
  <AssemblyTimeoutMs>5000</AssemblyTimeoutMs>
  <StaleAfterMs>5000</StaleAfterMs>
  <PlaceholderAfterMs>10000</PlaceholderAfterMs>
  <HeartbeatMs>1000</HeartbeatMs>
</DisplayPreview>
<Navigation>
  <Enabled>true</Enabled>
  <MaxFps>1</MaxFps>
  <WarmFps>0</WarmFps>
  <Panning>false</Panning>
  <StaleAfterMs>5000</StaleAfterMs>
  <PlaceholderAfterMs>10000</PlaceholderAfterMs>
</Navigation>
```

Navigation can use the link with DisplayPreview disabled. Keep camera preview
disabled for this map-only port. Setting Panning false skips negotiation; leaving
it true also works through the tested fallback. The host must include the CYD
identity change in this port.

At 115200 baud, a 32 KiB JPEG takes about 2.85 seconds for payload alone, plus
protocol overhead and decode/display time. MaxFps is a ceiling, not a promised
frame rate. Expect seconds of transport lag for large images; this port does not
provide the ES3C28P's smooth local panning or 10–15 fps video. The longer assembly
and freshness timeouts above accommodate this link. Smaller JPEGs refresh faster.

## Validation

Firmware compiled with Arduino ESP32 core 3.2.0 and JPEGDEC 1.8.4: 475042 bytes
flash; 79744 bytes static RAM (247936 bytes left for stack and dynamic allocations,
not a measurement of runtime free heap). No firmware was flashed.

```bash
bash peripherals/cyd/tests/run_tests.sh
# Requires Pillow, C++ compiler and JPEGDEC 1.8.4 source:
JPEGDEC_SRC=/path/to/JPEGDEC/src bash peripherals/cyd/tests/run_decoder_tests.sh
# In the project's development container:
make es3display_test
# Run the executable in the selected BUILD_DIR, optionally with --cyd-map.
```

Receiver tests pass in C++11/C++17 and under ASan/UBSan, including exact-buffer
bounds, CRC, timeout rollover, credit ownership, hidden/calibrating states,
reconnect and bounded event overflow. Real JPEGDEC tests cover colour/subsampling,
grayscale, corrupt/progressive input and tab-strip bounds. A native test of the
actual controller covers partial serial writes, handshake, fallback, frame
presentation, freshness, hidden demand and heartbeat recovery. Vendor JPEGDEC is
ASan-instrumented; wrappers also use UBSan. The full Jetson display-link suite
passes, including a CYD identity/32 KiB whole-frame fallback regression.

Remaining physical checks: flash deliberately, select MAP, verify real CH340
throughput and reconnect, inspect marker/attribution and stale warnings, test all
six touch tabs and recalibration, and soak while exercising SD/audio/backlight.
Native tests and compilation do not establish runtime heap margin or electrical
and display timing on the board. Windows helper and PlatformIO were not executed;
the verified build path is the Linux Arduino CLI helper with core 3.2.0.
