# CYD dashboard firmware

For the ESP32-S3 ES3C28P capacitive-touch board, use the separate
[ES3C28P firmware](../es3c28p/README.md).

This is the ESP32-2432S028 dashboard firmware, structured the same way as
`peripherals/esp32-c3`:

```
cyd.ino                     Arduino CLI marker
src/main.cpp                application-specific page data and lifecycle
lib/cydStorage/             SD-card initialisation and persistent file I/O
lib/cydGraphics/            reusable display/touch/tab-dashboard component
lib/cydAudio/               SD-streaming notification-audio component
BuildAndUpload.cmd          Windows compile/upload path
build_and_upload.sh         Linux compile/upload path
platformio.ini              PlatformIO configuration
assets/                     ready-to-copy SD-card files (splash, alert tone)
tools/bmp24_to_rgb565.py    splash converter
cyd_bringup_test/           standalone display + read-only SD check
host/ch341_module.sh        Jetson host: CH340 USB-serial driver (build, check)
```

`CydStorage` owns the SD SPI bus and all card file access. It currently stores
and validates `/cyd_touch.cal`, but is the single place for future SD logs,
settings, assets, and data read/write routines. `CydGraphics` owns the ILI9341
display, XPT2046 touch overlay, and five-tab renderer; it receives `CydStorage`
by reference solely to load/save calibration. The application injects a fixed
`CydDashboardPage` array, so later dashcam data/control code can change page
content without touching display, touch, or storage hardware code.

The current demo has five 64 x 16 px white bottom tabs: Live, Trip, Camera,
System, and Settings. It displays only placeholder data; no I2C link exists
yet.

## Ambient-light backlight control

`CydGraphics` supports a filtered, rate-limited LDR-to-PWM backlight mapping on
the display backlight's GPIO21. On this ESP32-2432S028, the onboard LDR uses
ADC1 **GPIO34**, so the demo enables it before `begin()`:

```cpp
const CydBrightnessConfig brightness = {
    true, 34, 1200, 400, 32, 255, 20, 4, 50
};
gGraphics.setBrightnessConfig(brightness);
```

`darkRaw` and `brightRaw` may be reversed for the opposite divider polarity.
This board reads lower in bright light, so the supplied demo uses 1200 for dark
and 400 for bright. Its high-resistance LDR divider has a much smaller useful
raw range than the ADC's nominal 0--4095 range. The graphics library uses ESP32
0 dB ADC attenuation for the required sensitivity and averages samples. It
selects only the dim (`minDuty`) or bright (`maxDuty`) target at the actual
`brightThresholdRaw` sensor value; the demo's target is `L = 20`.
`transitionStep` controls the gradual PWM fade at each `updateIntervalMs`
interval. GPIO34 is ADC1 and input-only, which suits this sensor.

Every dashboard page shows `L:<raw> B:<duty>` at lower-right. Cover and uncover
the LDR: after crossing the `L = 20` threshold, `B` fades smoothly between 32 and
255 rather than jumping. If `L` is outside the default 400--1200 window, use
the dark and bright displayed values as `darkRaw` and `brightRaw` in
`src/main.cpp`.

The Settings tab contains `-` and `+` buttons for the raw LDR threshold. Each
tap changes it by 10 ADC points; the selected value takes effect immediately. It is currently
an in-memory setting and returns to the configured default after a reboot.

## Notification audio

`CydAudio` streams **RIFF/WAVE PCM, mono, unsigned 8-bit, 8--16 kHz** from the
SD card. The supplied 8 kHz form is the recommended notification format: it
needs no MP3/AAC decoder, uses only a small fixed buffer, and sends samples to
the ESP32's I2S/DMA-backed DAC on GPIO26, which feeds the CYD speaker amplifier.

`assets/alert.wav` is a ready-to-copy 0.35-second test tone. Copy it to the SD
card root as `/alert.wav`; the demo plays it once at boot when present. A future
dashcam event can call `gAudio.playWav("/alert.wav")`; `gAudio.update()` is
already serviced by the main loop. `stop()` ends an alert immediately, and
`setVolume(0..100)` applies software volume scaling.

The samples go through ESP-IDF's DAC continuous-mode driver
(`driver/dac_continuous.h`) on the APLL clock: the default clock cannot run
below 19.6 kHz on the ESP32. Do not go back to the legacy `driver/i2s.h`
built-in-DAC mode. It links ESP-IDF's legacy ADC driver, and with
`analogRead()` (the LDR) also linked, the firmware aborts about 0.16 s after
every boot with `ADC: CONFLICT! driver_ng is not allowed to be used with the
legacy driver`. That is what the board was running until 2026-10-12.

## SD splash image

The Live tab streams `/splash.rgb565` from the SD card and displays it at
exactly 320 x 224 px, leaving the 16 px tab strip visible. This is a headerless
RGB565 image in **big-endian display-wire order**, stored top-to-bottom. It is
143,360 bytes: 33% smaller than the previous 24-bit BMP and requires no
per-pixel colour conversion. The loader sets one display window and streams
the pixels in 320-pixel blocks, then logs its elapsed render time and selected
SD frequency to Serial.

The ready-to-copy artwork is `assets/splash.rgb565`. Copy that file to the root
of the CYD SD card so its card path is `/splash.rgb565`. A legacy
`/splash.bmp` remains supported as a compatibility fallback, but is slower.

To make a new asset from a 320 x 224 uncompressed 24-bit BMP, run:

```powershell
python peripherals\cyd\tools\bmp24_to_rgb565.py input.bmp splash.rgb565
```

The converter uses only Python's standard library and rejects incorrect image
dimensions or BMP variants. `CydStorage` now attempts the CYD SD socket at
20 MHz, falling back automatically to 10 MHz if the card cannot initialise at
the faster speed. A successful mount logs the chosen rate; stable operation
with the target card is still the final verification for 20 MHz.

## Touch calibration

On first boot, or when `/cyd_touch.cal` is absent/malformed, a twelve-point
4 x 3 wizard appears. Tap the center of each crosshair and lift your finger
between taps. It samples four evenly distributed horizontal positions and three
evenly distributed vertical positions, then uses a three-segment horizontal
and two-segment vertical mapping rather than one coarse full-screen fit.

The wizard next shows a **TOUCH CHECK** screen. Tap the displayed brightness
controls and tab targets: a cyan dot should appear under your touch. Choose
**RETRY** to repeat the twelve-point wizard or **SAVE** to persist the new
calibration to `/cyd_touch.cal`. From the dashboard, enter Settings and tap the
active **SET** tab again to launch the wizard later. Older calibration files
intentionally trigger the new wizard once.

The board's display, SD, and touch pins are fixed inside their owning library:

- Display: GPIO 12/13/14/15, DC GPIO2; backlight GPIO21; reset tied to EN
  (GPIO4 is the red LED, not the display reset).
- SD: GPIO 18/19/23/5.
- Touch: GPIO 25/32/33/36/39.

## Build and upload

From Windows, run:

```powershell
peripherals\cyd\BuildAndUpload.cmd
peripherals\cyd\BuildAndUpload.cmd COM7
```

The board selection is **ESP32 Dev Module** (`esp32:esp32:esp32`). This CYD is
a classic ESP32 with a CH340 serial adapter, not the ESP32-C3 XIAO.

### From the Jetson

The `l4t-ml-gpio` image has the toolchain: the esp32 core 3.2.0 (it carries
the Xtensa compiler a classic ESP32 needs) and Adafruit ILI9341 1.6.4, on top
of the GFX and BusIO libraries the MKR layer installs. The Windows build used
core 3.3.11; the app also builds on 3.2.0 without warnings.

```bash
# compile only
docker run --rm -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
  /user/dashcam/peripherals/cyd/build_and_upload.sh
# compile and flash the one CH340 under /dev/serial/by-id
docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
  /user/dashcam/peripherals/cyd/build_and_upload.sh auto
# serial console at 115200 (Ctrl-A Ctrl-X quits)
docker run --rm -it --privileged -v /dev:/dev l4t-ml-gpio:latest picocom -b 115200 /dev/ttyUSB0
```

**The host needs the CH340 driver once.** The CYD's USB port is a CH340
(`1a86:7523`). The Tegra kernel ships no `ch341` module, so the board shows in
`lsusb` but no `/dev/ttyUSB*` appears. Ubuntu's `brltty` also claims that USB
ID (a braille display uses the same chip) and would take the port away.
`host/ch341_module.sh` handles both:

```bash
peripherals/cyd/host/ch341_module.sh check    # read-only: driver, tty, brltty
peripherals/cyd/host/ch341_module.sh build    # builds and verifies ch341.ko, prints the sudo install
```

`build` takes `drivers/usb/serial/ch341.c` from the kernel.org stable tree at
the running kernel's version, refuses a source whose sha256 is not pinned, and
checks vermagic and every imported symbol CRC. For 5.15.199-tegra the source
is byte-identical to NVIDIA's R36.x `kernel-jammy-src` copy. A new kernel needs
a new build. Install it on the host (root; one line at a time, so a sudo
password prompt cannot swallow the next line):

```bash
sudo systemctl mask --now brltty-udev.service      # keep brltty off 1a86:7523
sudo install -D -m 0644 ~/drive_logs/tools/ch341_driver/out-5.15.199-tegra/ch341.ko /lib/modules/5.15.199-tegra/updates/ch341.ko
sudo depmod -a 5.15.199-tegra
sudo modprobe ch341                                # binds a CYD that is already plugged in
peripherals/cyd/host/ch341_module.sh check         # expect a /dev/ttyUSB* and OK
```

Undo, unloading first (once `depmod` has forgotten the module, `modprobe -r`
cannot find it; the unload fails while the port is open):

```bash
sudo modprobe -r ch341
sudo rm /lib/modules/$(uname -r)/updates/ch341.ko && sudo depmod -a $(uname -r)
sudo systemctl unmask brltty-udev.service
```

## Bring-up test

`cyd_bringup_test/` is a small standalone sketch, separate from the app. It
draws colour stripes and reads the SD card's capacity without writing to it:

```bash
docker run --rm -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
  arduino-cli compile --fqbn esp32:esp32:esp32 /user/dashcam/peripherals/cyd/cyd_bringup_test
```

(add `--upload --port /dev/ttyUSB0` and `--privileged -v /dev:/dev` to flash).
It prints one line about 0.2 s after a reset, e.g.
`[cyd] display OK; SD OK (953 MiB)`. Only the SD half is a check; `display OK`
is printed whatever the panel does.
