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
