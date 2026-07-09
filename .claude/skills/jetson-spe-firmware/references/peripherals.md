# SPE Peripherals Reference

Per-peripheral facts verified from the SPE Developer Guide (r36.4.3). For each: the enable
flag, the header to read for the real API, instances/pins, config fields, and the success
output to expect. **For verified driver call sequences and struct fields, see
`app-patterns.md`** (extracted from the actual sources) — this file is the hardware/pin/flag
view; that file is the code view.

Contents: [GPIO](#gpio) · [I2C](#i2c) · [SPI](#spi) · [AODMIC](#aodmic-dmic) ·
[Timer](#timer) · [GTE](#gte) · [IVC](#ivc) · [Not supported](#not-supported-on-spe)

Every app is compiled in via a flag in `soc/t23x/target_specific.mk`, and **Orin Nano
additionally requires `ENABLE_SPE_FOR_ORIN_NANO := 1`** on top of the app flag.

---

## GPIO
`app/gpio-app.c` — access/manipulate AON GPIO from the R5.

- **Enable:** `ENABLE_GPIO_APP := 1`
- **Header:** `fsp/source/include/gpio/tegra-gpio.h`
- **Demo behavior:** drives an output pin, which is wired to an input pin configured to
  fire an interrupt; the ISR then clears the output. Expected log:
  - `gpio_app_task - Setting GPIO_APP_OUT to 1 - IRQ should trigger`
  - `can_gpio_irq_handler - gpio irq triggered - setting GPIO_APP_OUT to 0`
- **Wiring:**
  - **AGX Orin (J30):** Pin 16 = PBB1 (output), Pin 32 = PBB0 (input) — jumper them.
  - **Orin Nano (J12):** Pin 5 = PDD1 (output), Pin 3 = PDD2 (input) — jumper them.
- **DT specifics (see device-tree.md):** the input pin's interrupt line must be routed to
  an SPE interrupt in the `gpioint` DT (the demo routes it to INT2), the pins must sit in
  the correct `gpio-input` / `gpio-output-low` lists, and pinmux must set the pins to GPIO
  with input enabled as appropriate.

---

## I2C
`app/i2c-app.c` — access/manipulate AON I2C from the R5.

- **Enable:** `ENABLE_I2C_APP := 1`
- **AON I2C instances:** 2, 8, 10 (availability depends on platform). Demo bus is selected by
  `I2C_AON_BUS_NUM` (bus 8), not a literal; clock is **100 kHz** (`I2C_TEST_BUS_CLKRATE`).
- **Demo sensor:** Bosch **BMI160** 6-axis IMU at I2C address **0x68** (SAO tied to J30
  Pin 34). The app reads the sensor's chip-ID via `i2c_test()` — **note:** that transfer/read
  logic lives in `i2c-app-priv` (not in the provided sources); `i2c_controller_init` is
  verified, the transfer API is in `i2c/i2c.h`. See `app-patterns.md`.
- **Wiring (J30, both platforms use bus 8 pins):** Pin 1 = 3.3 V, Pin 3 = SDA, Pin 5 =
  SCL, Pin 34 = SAO, Pin 39 = GND.
- **Success output:** `I2C test successful`
- **DT specifics:** disable `i2c@c250000` (bus 8) in the **kernel** DT (already disabled on
  Nano; must be disabled on AGX Orin), then open the firewall/SCR for I2C8
  (`CLK_RST_CONTROLLER_AON_SCR_I2C8_0`) so the R5 can reach the registers. Compile DT +
  full reflash.

---

## SPI
`app/spi-app.c` (master) and `app/spi-slv-app.c` (slave) — AON SPI from the R5.

- **Enable:** `ENABLE_SPI_APP := 1` (master) **or** `ENABLE_SPI_SLV_APP := 1` (slave).
  **Master and slave are mutually exclusive — one at a time.**
- **Instance:** **SPI2** is the AON SPI on both AGX Orin and Orin Nano.
- **Frequency:** configured to **12 MHz**.
- **Logic level:** **SPI2 is 1.8 V.** If your master/peripheral is 3.3 V, add a level
  shifter, or you risk damage / no comms.
- **Master demo:** loopback — **short MISO and MOSI**, sends predefined bytes, compares.
  Success: `SPI test successful`.
- **Slave demo:** needs an external SPI master to drive SCLK/CS; bring the master up
  **first** (the slave has no clock of its own). Success: `** SPI Slave test Passed **`
  (master demo prints `SPI test successful`).
- **Pin maps:**
  - **AGX Orin (J3):** E61=CLK, D62=MISO, F60=MOSI, D60=CS0.
  - **Orin Nano (J2):** 126=CLK, 127=MISO, 128=MOSI, 130=CS0.
- **DT specifics:** pinmux the four `spi2_*_pcc*` pins to function `spi2`, remove those AON
  GPIOs from conflicting gpio lists, and open the firewall/SCR for SPI2
  (`CLK_RST_CONTROLLER_AON_SCR_SPI2_0`). Full reflash.

---

## AODMIC (DMIC)
`app/aodmic-app.c` — capture from the Always-On Digital Microphone. **DMIC5** lives in the
AON domain, hence "AODMIC." Also demonstrates **system wake on loud sound**.

- **Enable:** `ENABLE_AODMIC_APP := 1`
- **Header (config + limits):** `fsp/source/include/aodmic/tegra-aodmic.h`
- **Transport:** samples captured via **GPCDMA**. The **driver owns cache coherency** — the
  app passes a plain `gpcdma_buf` and reads via `tegra_aodmic_read`, no app-side cache ops
  (see `app-patterns.md`). Manual invalidate/clean is only your job if you hand-roll GPCDMA.
- **Config struct fields (verified against `aodmic-app.c` — the docs showed only the first
  three):**
  - `sample_rate`: `TEGRA_AODMIC_RATE_8KHZ` / `16KHZ` (default) / `44KHZ` (44.1) / `48KHZ`.
  - `channel_config`: `TEGRA_AODMIC_CHANNEL_STEREO` (default) / `MONO_LEFT` / `MONO_RIGHT`.
  - `num_periods`: 2 … `AODMIC_MAX_NUM_PERIODS`; typical 2 = double-buffered.
  - `period_size` (e.g. 256, **must be a multiple of 16**), `sample_width`
    (`TEGRA_AODMIC_BITS_PER_SAMPLE_16`), `dma_id` (`&gpcdma_id_aon`), `dma_chan_num` (0–7).
  - `gpcdma_buf` / `gpcdma_buf_size` — **the app allocates the driver's DMA buffer itself**;
    size = `period_size * num_periods * 4 * num_channels` (driver uses 4 bytes/sample).
  - Changing `sample_width` ripples into data types, buffer sizes, and the wake threshold.
  - **Precondition:** `gpcdma_init()` must run before `tegra_aodmic_open()`; all AODMIC calls
    return `error_t`. Rates are literal Hz (8000/16000/44100/48000).
- **Clocking gotcha:** fixed **64× oversampling**. Clock = `sample_rate × 64`. The mic must
  support that clock — e.g. 8 kHz → 512 kHz clock; if the mic's floor is 1 MHz, 16 kHz is
  the lowest usable rate. The app prints R5 tick count, per-channel zero-crossing counts,
  and computed volume (mean-square) to verify capture rate.
- **Wake:** if volume exceeds `SPE_CCPLEX_WAKE_THRESHOLD` (defined in `aodmic-app.c`) it
  triggers a system wake. On AGX Orin the relevant wake event is **wake83** — CCPLEX/BPMP
  must enable it (verify bit 83 set in both the wake mask and the Tier2 routing mask in the
  BPMP UART log at suspend).
- **Wiring:** stereo PDM mics on the AODMIC pins.
  - **AGX Orin (J30):** Pin 16 = DMIC5_DAT, Pin 32 = DMIC5_CLK (these are the CAN1 pins
    remuxed to `dmic5`); power/GND e.g. Pin 1 (3.3 V), Pin 6 (GND).
- **DT specifics:** pinmux the CAN1 pins to `dmic5`, remove them from the AON GPIO map,
  and open the firewall/SCR for DMIC5 (`CLK_RST_CONTROLLER_AON_SCR_DMIC5_0`).
- **Suspend caveat:** on system wake, BPMP disables the dmic5 clock, so capture fails after
  the first suspend. Either (quick) write `1` to the dmic5 clock state in
  `/sys/kernel/debug/bpmp/debug/clk/dmic5/state` before the first suspend, or (persistent)
  patch the BPMP DT `dmic5` late-init clock entry (3rd arg = `sample_rate × 64`) and reflash
  the bpmp-fw-dtb partition.

---

## Timer
`app/timer-app.c` — uses the **TKE (Time Keeping Engine)** driver (`tke/tke-tegra.h`,
id `tegra_tke_id_timer2`), timer2 in periodic mode. See `app-patterns.md` for the calls.

- **Enable:** `ENABLE_TIMER_APP := 1`
- **Behavior:** with clock source `TEGRA_TKE_CLK_SRC_USECCNT`, **`TIMER2_PTV` is in
  microseconds** (demo `5000000` = 5 s). The ISR-context callback (uses `printf_isr`) prints
  `Timer2 irq triggered, usec=… osc=… tsc_hi=… tsc_lo=…` and calls `tegra_tke_stop_timer`
  once an internal count exceeds `STOP_TIMER` (10).
- No special wiring or firewall notes — a pure on-chip timer, good as a first "is my
  toolchain/flash path working" smoke test. **Why timer2 and not timer0:** the FreeRTOS tick
  runs on TKE `timer0` (`setup_timer_interrupt`), so timer0 is reserved — the demo uses
  timer2. Enabling this app needs no GPCDMA.

---

## GTE
`app/gte-app.c` — **Generic Time-stamping Engine**: snoops a set of signals and delivers
timestamped events to software via FIFOs.

- **Enable:** `ENABLE_GTE_APP := 1`
- **Header (signal→slice map):** `gte-tegra-hw.h`
- **Instances:** **3 GTE slices** available to AON/SPE, **32 bits each**. To find which bit
  in slice 2 corresponds to a pin, use the hidden **"GTE" column** in the pinmux spreadsheet —
  or the verified signal map in `gte-tegra-hw.h` (see `app-patterns.md`). Occupancy threshold
  applies only in no-DMA mode; GTE can also run DMA-driven.
- **Works with gpio-app:** monitors the interrupt on the `GPIO_APP_IN` line (wire per the
  GPIO app). The GTE ISR fires when the FIFO hits `GTE_FIFO_OCCUPANCY`. Expected log
  includes a line like: `Slice Id: xx, Event Id: xx (Other), Edge = rising, Raw Time
  stamp = xx, Timestamp in nanosec = xx`.
- **DT specifics:** disable the kernel `hardware-timestamp@c1e0000` node (ensure no other
  processor uses GTE), rebuild the DT, and set the GTE firewall/SCR
  (`GTE_GPIO_SCR_TESCR_0`) so the R5 can read/write GTE registers. Full reflash.

---

## IVC
`app/ivc-echo-task.c` — **Inter-VM Communication**: message-passing between on-chip
processors over a shared-memory channel, synchronized by **HSP doorbells**.

- **Demo:** CCPLEX → AON over a data channel created by the AON Echo driver; AON echoes it
  back to CCPLEX.
- **Enable channel (kernel DT):** set the `aon_echo` node `status = "okay"` under `bus@0`,
  recompile DT, reflash. Then build the SPE app and copy `spe.bin`.
- **Test from Linux:**
  - `sudo su -c 'echo tegra > /sys/devices/platform/bus@0/bus@0:aon_echo/data_channel'`
  - `cat /sys/devices/platform/bus@0/bus@0:aon_echo/data_channel` → echoes `tegra`
- **Source of truth for channels:** `rt-aux-cpu-demo-fsp/app/ivc-echo-task.c`,
  `drivers/ivc-channels.c`, channel descriptions in `platform/ivc-channel-ids.c` and
  `soc/t23x/include/ivc-config.h`. Edit channel definitions there to add/enable channels.

---

## Not supported on SPE
The AON domain physically has CAN and UART, **but the SPE SDK does not support CAN or UART**
on AGX Orin / Orin Nano (per the supported-features table). Do not scaffold SPE CAN/UART
drivers — say it isn't supported and redirect (e.g. to the CAN controller on the main SoC /
kernel side) rather than producing firmware that can't work.
