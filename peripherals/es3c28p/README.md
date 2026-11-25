# ES3C28P dashboard firmware

The Ugreen camera preview design is documented in
[VIDEO_STREAMING_PLAN.md](VIDEO_STREAMING_PLAN.md): 10–15 fps over USB,
letterboxed display, library APIs, buffering, protocol changes, integration
steps and acceptance tests. The board-side receiver, decoder and renderer are
implemented; hardware performance remains unmeasured. The plan is preserved
as the design record. See [VIDEO_IMPLEMENTATION.md](VIDEO_IMPLEMENTATION.md)
for the current firmware behavior and remaining board checks.

The [voice-command plan](VOICE_COMMAND_PLAN.md) covers ES3C28P microphone
capture and offline Jetson recognition for **English, tap-to-speak tab
navigation**, with voice and video preview operating in separate modes. Voice is not implemented.

The [navigation plan](NAVIGATION_PLAN.md) covers an OpenStreetMap-based position
and direction-of-travel display: online tiles cached and rendered on the Jetson,
then sent over USB to the ES3C28P and shown on a new MAP tab (north-up).
The Jetson side, frozen map wire extension (`Es3MapProtocol.h`) and firmware
receiver are implemented. See [NAVIGATION_IMPLEMENTATION.md](NAVIGATION_IMPLEMENTATION.md)
for board behavior, validation and remaining hardware checks. The plan remains
the unchanged design record.
The board also supports panning: a retained background up to 512x384, separate
status/attribution images and small pose updates for local cropping and marker
drawing. Whole-frame mode remains available if panning memory cannot be allocated.

Dashboard firmware for the LCDWiki **ES3C28P N16R8** capacitive-touch board
(ESP32-S3, 16 MB flash, 8 MB OPI PSRAM).
The dashboard has Live, Trip, Camera, System, Settings and Map tabs,
320 x 224 SD splash above the tab strip, touch calibration with SD persistence,
brightness controls, and streamed notification WAV playback. Page values are
placeholders until the Jetson supplies page text over the native USB channel.
The USB client can update pages, request status and play/stop alerts. Automatic
publishing of the dashcam application's vehicle telemetry is not wired in yet.

The [2026-10-16 review record](REVIEW_2026-10-16.md) documents fixes, ownership,
refactoring, validation and remaining hardware checks.

## Layout

```text
es3c28p.ino                 Arduino CLI entry marker
src/main.cpp               brightness configuration, setup/loop
src/DashboardConfig.h      tested default page table and auto-brightness enable
lib/es3Board/              board pins and shared touch/codec I2C bus
lib/es3Graphics/           dashboard, touch, assets and stream presentation sources
lib/es3Storage/            four-bit SDMMC storage and calibration records
lib/es3Audio/              ES8311 codec and modern ESP-IDF I2S DMA output
lib/es3UsbLink/            USB task, multiplexed controls/video and frozen protocol
lib/es3Video/              bounded PSRAM pools, assembly and JPEG decode worker
lib/es3Map/                map/camera surface binding and shared six-tab geometry
bench/                     USB sink and JPEG-from-SD feasibility sketches
host/es3_display.py        Jetson USB client and command-line tool (stdlib only)
host/99-dashcam-display.rules serial-specific persistent-device rule template
assets/                    splash.rgb565, splash.bmp, alert.wav
tools/bmp24_to_rgb565.py   320 x 224 BMP to wire-order RGB565 converter
tests/calibration_test.cpp host checks for invalid calibration records
BuildAndUpload.cmd         Windows Arduino CLI build/upload
build_and_upload.sh        Linux Arduino CLI build/upload
platformio.ini             alternative PlatformIO/pioarduino configuration
```

## Hardware

Pin assignments follow the [manufacturer's board page](https://www.lcdwiki.com/2.8inch_ESP32-S3_Display)
and [schematic](https://www.lcdwiki.com/res/ES3C28P/2.8inch_ESP32-S3_Display_Schematic.pdf).
All pins are centralized in `lib/es3Board/es3Board.h`.

| Function | GPIOs / device |
| --- | --- |
| Display SPI | SCLK 12, MOSI 11, MISO 13, CS 10, DC 46 |
| Display reset / backlight | Reset tied to CPU EN; backlight 45, active high |
| Shared I2C | SDA 16, SCL 15; FT6336G at 0x38, ES8311 at 0x18 |
| Touch reset / IRQ | 18 / 17 |
| SDMMC | CLK 38, CMD 40, D0 39, D1 41, D2 48, D3 47 |
| Audio I2S output | MCLK 4, BCLK 5, WS 7, DOUT 8 |
| Amplifier | GPIO1: low enables, high disables |
| Optional external LDR | GPIO2 on expansion connector |

Display initialization applies the vendor's IPS power, VCOM, gamma and
inversion settings from [ILI9341V_Init.txt](https://www.lcdwiki.com/res/ES3C28P/ILI9341V_Init.txt)
after Adafruit startup. Touch and audio share one initialized `Wire` bus;
all component calls belong to the Arduino loop task. The FT6336G is polled
without relying on its interrupt level, so held touches and releases work
with pulse-mode IRQ configurations.

The S3 has no classic ESP32 DAC. WAV samples are converted from unsigned
8-bit mono into signed 16-bit stereo slots for the ES8311. The codec uses
external MCLK at 256 times the sample rate, with register settings derived
from [Espressif's ES8311 driver](https://github.com/espressif/esp-adf/blob/release/v2.x/components/esp_codec_dev/device/es8311/es8311.c).
The driver uses `driver/i2s_std.h`, bounded buffers and zero-timeout writes.
An empty DMA queue outputs silence; the playback tail drains before the
amplifier is disabled. `stop()` disables the amplifier immediately.

## SD assets and touch

Use a FAT32 microSD card. Copy `assets/splash.rgb565` and `assets/alert.wav`
to its root. The image is 143,360 bytes, big-endian RGB565, 320 x 224 pixels.
The old 24-bit `splash.bmp` is supported when the RGB565 file is absent.
The optional boot tone is PCM WAV, mono, unsigned 8-bit, 8--16 kHz;
`playWav()`, `stop()` and `setVolume(0..100)` provide the notification API.
The supplied tone uses 8 kHz.

SDMMC tries 20 MHz, then 10 MHz, without formatting on failure. Insert the
card before boot; avoid swapping it during playback or image rendering.

Missing SD assets fail without remounting the filesystem under active audio
readers. After replacing a card, restart the board or explicitly close all
files before `Es3Storage::remount()`. WAV metadata scanning is limited to 256
chunks whose headers start within the first 64 KiB; normal PCM alert files
remain supported.

On first boot, tap and release each of the twelve calibration targets, then
use the touch-check screen to verify the mapping and choose SAVE. Although
capacitive coordinates are already pixels, this keeps the calibration wizard
and learns the panel's orientation. The record lives at
`/es3c28p_touch_ips.cal`. Cards with the earlier `/es3c28p_touch.cal` record
need one fresh calibration for the corrected display orientation.
Saving verifies a temporary record before replacing the primary and retains
`.cal.bak` for recovery; boot falls back to that backup if the primary is invalid.
This protects against failed writes/renames, not every SD power-loss failure.
Tap RECALIBRATE on the SET tab (or re-tap the active SET tab) to recalibrate. With no SD card the wizard still
works, but calibration is lost on reboot. Out-of-range, repeated or unordered
calibration knots are rejected before piecewise mapping.

## Brightness

This board has **no onboard LDR**. By default, the Settings BRIGHTNESS
slider sets the manual PWM brightness (8..255), saved in `/settings.xml`.
The lower limit keeps the controls visible.

For automatic brightness, connect a 3.3 V LDR voltage divider to
GPIO2 and enable `kAutoBrightness` in `src/DashboardConfig.h`. Measure the
divider's dark and bright ADC readings, then set `kBrightness.darkRaw`,
`brightRaw`, and
`brightThresholdRaw` accordingly; the supplied values are examples.
The external divider uses 11 dB ADC attenuation. GPIO9 is the battery
monitor, not a light sensor. The BRIGHTNESS slider then sets the bright level;
`L:<raw> B:<duty>` shows the filtered input and smoothly faded PWM.

## Settings tab and /settings.xml

The SET tab holds the user's controls, top to bottom:

- **MASTER VOLUME** slider (0-100 %, 0 = muted) with a **TEST** button. It
  scales every sound linearly, including the boot tone and alerts requested
  over USB (`ALERT` volume x master). TEST plays `/alert.wav` at full alert
  volume, the loudest any notification can be at that setting.
- **BRIGHTNESS** slider: backlight PWM 8-255 (never fully dark). With the
  optional LDR (`kAutoBrightness`) it sets the bright level and the dim level
  never exceeds it; the LDR threshold is then fixed by `kBrightness`.
- **MIC THRESHOLD** slider with a **MIC IN** indicator and a live level bar
  under the slider. The onboard microphone is captured only while the SET tab
  is visible (16 kHz, gain 54 dB, see [MIC_LEVEL_PLAN.md](MIC_LEVEL_PLAN.md));
  the indicator turns green when the level reaches the threshold and back to
  grey after 300 ms at least 5 points below it. It shows "paused" while a
  sound plays (playback and capture take turns) and "no input" otherwise.
- **RECALIBRATE** starts the touch wizard (re-tapping the SET tab still works).

Drag or tap a slider; the value is written when the finger lifts. The title
bar shows DEFAULTS, FROM SD, SAVED or NOT SAVED (no card or write failure).
Values persist on the card as `/settings.xml`, read at boot (before the boot
tone, so it plays at the saved master volume):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<settings version="1">
  <masterVolume>10</masterVolume>
  <brightness>100</brightness>
  <micThreshold>50</micThreshold>
</settings>
```

Every value is a percentage. A missing or invalid element keeps its default
from `kDefaultSettings` (`src/DashboardConfig.h`); a file without the
`<settings>` root is ignored. Saving uses the calibration's scheme: verify
`/settings.xml.tmp`, keep the previous file as `/settings.xml.bak`, promote;
boot falls back to the backup. STATUS does not report these values.

## Display orientation

`kRotateDisplay180` (`src/DashboardConfig.h`) turns the image and touch
input upside down for an inverted mounting. Touch coordinates are mirrored
in pixel space, so an existing calibration remains valid.

## Build and upload

Requires Arduino ESP32 **3.x** (compiled with 3.2.0), Adafruit ILI9341 1.6.4,
Adafruit GFX Library 1.12.6, Adafruit BusIO 1.17.4, and JPEGDEC 1.8.4. Install missing libraries:

```sh
arduino-cli lib install "Adafruit ILI9341@1.6.4" "Adafruit GFX Library@1.12.6" "Adafruit BusIO@1.17.4"
```

The scripts select ESP32S3 Dev Module, 16 MB QIO flash, the 3 MB app / 9 MB
FAT partition scheme, OPI PSRAM, and hardware USB CDC on boot. Both scripts
install the pinned JPEGDEC dependency with `--no-deps`. Set
`ES3_USB_MODE=default` to compile TinyUSB CDC for comparison (PlatformIO:
`es3c28p-tinyusb`); hardware CDC remains the default.

From the repository root on Linux:

```sh
peripherals/es3c28p/build_and_upload.sh                # compile only
peripherals/es3c28p/build_and_upload.sh /dev/ttyACM0   # compile and upload
```

On Windows:

```powershell
peripherals\es3c28p\BuildAndUpload.cmd
peripherals\es3c28p\BuildAndUpload.cmd COM7
```

The repository's Jetson toolchain container already has these dependencies:

```sh
docker run --rm -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
  /user/dashcam/peripherals/es3c28p/build_and_upload.sh
docker run --rm --device=/dev/ttyACM0 -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
  /user/dashcam/peripherals/es3c28p/build_and_upload.sh /dev/ttyACM0
```

Use `arduino-cli board list` to identify the actual port. The board's USB-C
port is native ESP32-S3 USB, normally `/dev/ttyACM*` on Linux. If needed, hold
BOOT, press/release RESET, then release BOOT to enter download mode. The
scripts upload only when an explicit port is supplied. Serial logs use
115200 baud is the nominal CDC setting; startup never waits for a host.
USB is now a binary communication channel. Firmware debug logs go to
`Serial0` on the UART connector at 115200 baud, not to USB.

Alternatively run `pio run -d peripherals/es3c28p` (add `-t upload
--upload-port PORT` to flash). The PlatformIO config pins pioarduino
[54.03.20](https://github.com/pioarduino/platform-espressif32/releases/tag/54.03.20),
which supplies Arduino 3.2.0; it is not the classic Arduino 2.x platform.
The Arduino CLI path is the verified build; PlatformIO and Windows execution
have not been tested here.

## USB communication with the Jetson

Connect a **USB data cable** from a Jetson USB host port to the ES3C28P USB-C
connector. This is the board's native USB Serial/JTAG CDC interface; Linux
uses `cdc_acm` and exposes `/dev/ttyACM*` ([Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/v5.1.3/esp32s3/api-guides/usb-serial-jtag-console.html)).
No external USB/UART adapter or Jetson GPIO/I2C connection is needed. The
board's internal I2C bus remains dedicated to touch and audio-codec control.

Identify the display using `arduino-cli board list` and `/dev/serial/by-id/`.
Pass its exact by-id path to `--device`; do not assume it is ACM0, since the
C3 and MKR can also create ACM ports. The client never probes other devices.
For `/dev/dashcam-display`, replace `DISPLAY_SERIAL_HERE` in the supplied udev
template with the ES3C28P's serial, install it in `/etc/udev/rules.d/`, reload
rules with `sudo udevadm control --reload-rules`, and replug the board. The
template does nothing until customized. Both C3 and S3 share VID/PID
303a:1001, so serial matching is required; the alias then follows the board
to any USB port. The C3's `99-dashcam-bridge.rules` is a template
matched on the C3's own serial in the same way, so the display never gets
`/dev/dashcam-bridge` (udev 249 on JetPack 6 has no `SYMLINK-=`, so the serial
match is the only way to keep the two apart). Pin the C3
link's `CommLinkConfig::device` to its own by-id path or `/dev/dashcam-bridge`
when running both links, to avoid its generic `USB_JTAG` discovery probing
the display. The firmware rejects the C3 protocol independently of naming.

From the repository root, with the display's actual port substituted:

```sh
python3 peripherals/es3c28p/host/es3_display.py --device /dev/ttyACM0 ping
python3 peripherals/es3c28p/host/es3_display.py --device /dev/ttyACM0 status
python3 peripherals/es3c28p/host/es3_display.py --device /dev/ttyACM0 watch
python3 peripherals/es3c28p/host/es3_display.py --device /dev/ttyACM0 page 0 \
  --title 'DASHCAM LIVE' --line 'Record: ON' --line 'Speed: 42 km/h' \
  --line 'GPS: fix' --line 'Event: none'
python3 peripherals/es3c28p/host/es3_display.py --device /dev/ttyACM0 alert --volume 60
python3 peripherals/es3c28p/host/es3_display.py --device /dev/ttyACM0 stop-audio
```

`watch` polls status once per second and reconnects after USB loss. Use a
by-id path or the udev alias so reconnect still works if the tty number
changes. Other commands execute once and are never replayed on timeout.
Page indices are Live=0, Trip=1, Camera=2, System=3, Settings=4, Map=5.
The Python client capability-checks page 5 before sending it. Page commands
copy a title and four printable ASCII lines, each at most 23 characters.
Updating Live replaces its splash with those fields; all tabs retain local
touch behavior. Page content is held in RAM until reboot or replacement.
Alerts use the existing `/alert.wav` on the SD card.

Import `Es3Display` from `host/es3_display.py` in a Jetson publisher and use
`set_page(index, title, four_lines)`, `alert(volume)`, `stop_audio()`,
`status()` and `ping()`. Keep one session open and send a request at least
once every five seconds (one-second heartbeats recommended). The footer on
text pages shows USB online/offline and marks retained data stale when the
host stops communicating. `status()` returns uptime, selected tab, brightness,
SD/calibration state and audio playback state. It can be used to observe tab
and brightness changes made on the touchscreen. Serialize client calls; only
one request may be outstanding. Close the client before flashing the board.

For Docker, pass the device as in the upload example. A single `--device`
mapping may need the container restarted after unplug/replug; the project's
existing privileged `/dev` bind mount supports changing device nodes. The
client can also run directly on the Jetson host with normal serial-port
permissions.

### Wire contract (version 1)

`Es3Protocol.h` is the firmware contract; the Python codec is checked against
it with a shared test vector. This is separate from `HostProtocol.h` on the C3.

```text
E3 28 | version=01 | sequence:u8 | type:u8 | length:u8 | payload | CRC16_LE
```

Payloads are limited to 128 bytes. CRC-16/CCITT-FALSE (initial FFFF, polynomial
1021) covers version through the last payload byte. Replies echo the sequence
and set type bit 7; type FF carries a one-byte error (1 unknown command,
2 bad payload, 3 unavailable, 4 no session). HELLO is required after opening
or session expiry. Partial frames expire after 250 ms; the session expires
after five seconds without a request. A dedicated core-0 task services up to
4096 RX bytes per turn, retains partial TX writes, and yields each turn.
Only the application loop executes graphics/audio commands.

| Command | Type | Request payload | Reply payload |
| --- | --- | --- | --- |
| HELLO | 01 | empty | ASCII `ES3C28P` (7 bytes) |
| PING | 02 | empty | empty |
| STATUS | 03 | empty | uptime u32 LE, then u8 page, brightness, SD, calibrated, playing |
| PAGE | 04 | page u8 + title and 4 lines, each a 24-byte NUL-terminated field | empty |
| ALERT | 05 | volume u8 (0..100) | empty |
| STOP_AUDIO | 06 | empty | empty |
| CAPABILITIES | 07 | empty | 18-byte video limits (or unavailable) |
| START_VIDEO | 08 | 23-byte negotiated settings | session token u32, initial cumulative grant u32 |
| STOP_VIDEO | 09 | session token u32 | empty |
| MAP_CAPABILITIES | 0A | empty | 16-byte frozen map capabilities |
| START_MAP | 0B | 27-byte map settings | session token u32, initial grant u32 |
| STOP_MAP | 0C | session token u32 | empty; wrong source/stale token is a no-op |
| PAN_CAPABILITIES | 0D | empty | 9-byte background/strip/pose limits |
| START_PAN | 0E | 31-byte map settings and background dimensions | session token u32, initial grant u32 |

Video records use the frozen `Es3VideoProtocol.h` framing and golden vectors;
the initial credit window is one. See [VIDEO_IMPLEMENTATION.md](VIDEO_IMPLEMENTATION.md).
Map images use that same receiver, bound to MAP instead of LIVE. The viewport
is exactly (0,0,320,224); tabs stay visible. Tap the attribution strip for local
map/licence information and tap again to close it. Stale map notices occupy
only the top 16 pixels, preserving map attribution. Camera/map ownership,
GPS direction/dot selection and tile rendering are coordinated by the Jetson.
In panning mode, the board draws the selected marker and crops the background
from uncredited MAP_POSE records (E3 29 type 10). It advertises up to 30 poses/s,
with panel redraws coalescing to the newest pose. Accepted poses refresh map
freshness even when image transfers are seconds apart. STOP_MAP ends either
map mode. The screen viewport and existing control/video formats are unchanged.

## Validation

The firmware compiles with `--warnings all` in `l4t-ml-gpio:latest` using
Arduino ESP32 3.2.0. Run the host tests from the repository root:

```sh
peripherals/es3c28p/tests/run_tests.sh
peripherals/es3c28p/tests/run_sanitizer_tests.sh
```

These cover calibration validation, C++/Python framing compatibility, CRC
failures, fragmented frames, timeouts, wrong-device rejection, port reopening
and pseudo-tty disconnects. Physical ES3C28P validation is still required:
check HELLO/status, page updates, USB unplug/replug and the offline indicator;
verify colors/orientation,
all six tabs, map/camera switching, attribution information, stale map notices,
the calibration SAVE/reboot cycle, manual brightness, SD
splash and missing-card fallback, and the complete boot tone without a
repeating tail. With an external LDR, also verify both sides of the threshold
and the gradual fade. Compilation does not establish that these hardware
checks pass.
