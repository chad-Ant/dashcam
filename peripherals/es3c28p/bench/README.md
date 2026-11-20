# Board feasibility measurements

These sketches replace the dashboard when uploaded. Reflash the main firmware
when finished. Neither sketch touches the plan or starts a camera capture.
UART0 diagnostics require an external 3.3 V UART adapter; native USB remains
the measured data channel. Do not open the production publisher at the same time.

Compile (append the exact display port to upload):

```sh
peripherals/es3c28p/bench/build.sh usb_sink
ES3_USB_MODE=default peripherals/es3c28p/bench/build.sh usb_sink
peripherals/es3c28p/bench/build.sh jpeg_from_sd
```

For the sink, send 1 KiB chunks for at least 30 seconds:

```sh
python3 peripherals/es3c28p/bench/send_sink.py /dev/dashcam-display --seconds 30
```

Record the UART **received** byte rate for hardware CDC and TinyUSB separately;
the host's queued-byte counter includes kernel buffering. Stay with hardware
CDC if it sustains three times the representative JPEG byte rate. TinyUSB can
change device identity; recheck the serial-specific udev rule. A failed TinyUSB
application may require BOOT while resetting to reflash.

For the SD bench, copy a baseline 320×180 JPEG no larger than 65,536 bytes to
`/preview.jpg`. Run red/green/blue bars, gradients, and representative day/night
frames in both 4:2:0 and 4:2:2. Record decode and eight-row-strip blit times and
check color order, orientation and scan tearing. It uses the production decoder,
panel setup and strip path, but does not service touch/audio during timing.

Then run the production firmware with the Jetson sender at 10 fps and 15 fps.
Log its UART counters and host PRESENTED latency while playing alerts, changing
tabs, calibrating, and unplugging USB. Verify stale at 500 ms and placeholder at
2 s, bounded memory, camera recording health, and reconnect for 30 minutes.
No board performance or hardware correctness is claimed by a successful build.

## Microphone capture (MIC_LEVEL_PLAN.md, P1)

```sh
peripherals/es3c28p/bench/build.sh mic_capture /dev/dashcam-display
```

Captures the ES8311 ADC at 16 kHz (I2S0, DIN GPIO6) and shows, for each I2S
slot (L/R), the RMS and peak level in dBFS, the 3 s RMS maximum, the DC offset
and clipped samples, plus DMA overflows, all on the TFT (UART0 prints the same
line every 200 ms). A tap anywhere steps the gain: PGA 0/12/24/30 dB, then
PGA 30 dB plus ADC scale 6/12/18/24 dB; 300 ms after each change is discarded.
The amplifier stays disabled and the DAC muted. Note for each gain step the
level in a quiet room, for speech at about 30 cm, and for a clap (clipping),
and which slot carries the signal. Reflash the main firmware afterwards.
