# CYD dashboard firmware

This folder starts the dashboard work for the common ESP32-2432S028R **Cheap
Yellow Display** (2.8-inch ILI9341, resistive touch, and microSD).

`cyd_bringup_test/` is deliberately small and read-only. It verifies the LCD
and detects the SD card without creating or changing a file. On a working
standard CYD it displays colour stripes, `SD card: OK`, and roughly 950 MiB for
a nominal 1 GB card.

Build from the repository root:

```powershell
& 'C:\Program Files\Arduino CLI\arduino-cli.exe' compile --fqbn esp32:esp32:esp32 peripherals/cyd/cyd_bringup_test
```

Upload by adding `--upload --port COMx` once the board's serial port is known.
The expected board selection is **ESP32 Dev Module**; this is a classic ESP32
CYD, not the ESP32-C3 XIAO.

The future CYD-to-XIAO C3 I2C bus intentionally is not initialised yet. Before
adding it, decide the electrical wiring, which controller is bus master, the
CYD's I2C address if it is a target, and the message/command format.
