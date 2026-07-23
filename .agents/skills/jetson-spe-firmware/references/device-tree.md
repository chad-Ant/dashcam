# SPE Device-Tree Reference

The device tree is where most "my SPE peripheral doesn't work" bugs live. A peripheral is
usable from the R5 only when **four bootloader-side DT layers agree** *and* the **kernel
releases the module**. This file gives the layers, the exact per-platform file names, the
firewall/SCR pattern, and the kernel-side flow.

Contents: [Platform naming](#platform-naming) · [The four layers](#the-four-bootloader-dt-layers) ·
[Firewall / SCR](#layer-4-firewall--scr-in-depth) · [Kernel release](#kernel-side-release-the-module) ·
[Reflash rule](#reflash-rule) · [Debug map](#dt-debug-map)

> The clean, supported way to change pinmux/GPIO is to edit the **pinmux spreadsheet** for
> your module and regenerate the BCT dtsi files. Hand-editing the files below works for a
> quick bring-up/test; regenerate from the spreadsheet for anything you'll keep.

## Platform naming

Filenames encode module + carrier + board revision. Get these right or you'll edit a file
that isn't in your flash:

| | AGX Orin | Orin Nano |
|---|---|---|
| Module | `p3701` | `p3767` |
| Carrier | `p3737` | `p3768` |
| Board suffix | `-a04` | `-dp-a03` |
| Extra build flag | — | `ENABLE_SPE_FOR_ORIN_NANO := 1` |

All bootloader BCT files live under `${L4T}/bootloader/` (some under
`${L4T}/bootloader/generic/BCT/`). `${L4T}` is your `Linux_for_Tegra` directory.

## The four bootloader DT layers

For a peripheral on a given set of pins, **all four** must be consistent with each other
and with the app's expectations.

### Layer 1 — Pinmux
File: `.../BCT/tegra234-mb1-bct-pinmux-<module><suffix>.dtsi`
(e.g. `tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi`,
`tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi`).

Set the pin's `nvidia,function` to the peripheral (e.g. `spi2`, `dmic5`, `i2c8`) instead of
`rsvd*`/its default, and set `nvidia,pull` / `nvidia,tristate` / `nvidia,enable-input`
appropriately for signal direction. Illustrative shape (confirm exact node name for your
pin):

```
spi2_sck_pcc0 {
    nvidia,pins = "spi2_sck_pcc0";
    nvidia,function = "spi2";        /* was rsvd1 */
    nvidia,pull = <TEGRA_PIN_PULL_NONE>;
    nvidia,tristate = <TEGRA_PIN_DISABLE>;
    nvidia,enable-input = <TEGRA_PIN_DISABLE>;
};
```

Rules of thumb: an **output** pin → tristate DISABLE, enable-input DISABLE; an **input**
pin (or a bidirectional bus line like MISO/SDA) → enable-input ENABLE. To *free* a pin from
a bus (e.g. the GPIO app reclaiming the I2C8 pins on Nano), set its function to `rsvd*`.

### Layer 2 — GPIO map
File: `tegra234-mb1-bct-gpio-<module><suffix>.dtsi`.

Pins are grouped into `gpio-input`, `gpio-output-low`, `gpio-output-high`. Move a pin into
the list matching how the app drives it, and **remove it from any list where it conflicts**
(a common bug is leaving a pin in `gpio-output-*` while also muxing it to a bus). Entries
use `TEGRA234_AON_GPIO(<PORT>, <N>)`, e.g. `TEGRA234_AON_GPIO(BB, 1)`.

### Layer 3 — GPIO interrupt map
File: `.../BCT/tegra234-mb1-bct-gpioint-<module>.dts`
(e.g. `...-p3701-0000.dts`, `...-p3767-0000.dts`).

Routes a pin's interrupt to an interrupt *line*. To make an input pin's IRQ reach the SPE,
set its line to an SPE-routed interrupt (the GPIO demo routes the input pin to **INT2**):

```
port@BB {
    pin-0-int-line = <2>;   /* BB0 → INT2 (was INT0) so SPE sees the IRQ */
    ...
};
```

If interrupts silently never fire in firmware, this layer is the first suspect — verify the
pin, its port node, and the line number against the ID-struct IRQ your app registered with
the VIC.

### Layer 4 — Firewall / SCR
File (AGX Orin): `tegra234-mb2-bct-scr-<module>-override.dts` and/or
`tegra234-firewall-config-base.dtsi`. File (Orin Nano): `tegra234-mb2-bct-scr-p3767-0000.dts`
and/or `tegra234-firewall-config-base.dtsi`. See next section.

## Layer 4 — Firewall / SCR in depth

Each protected module has a Security Configuration Register (SCR) that gates who may touch
its register block. **If the R5 accesses a module whose SCR doesn't permit it, the R5 crashes
or hangs** (this is the FAQ "module fails to run / crashes" cause). You add or edit a `reg@`
entry for the module's AON SCR with the right `value` and `exclusion-info`:

```
reg@2135 {                 /* CLK_RST_CONTROLLER_AON_SCR_SPI2_0  — address is register-specific */
    exclusion-info = <3>;
    value = <0x30001410>;
};
```

Known SCR entries from the guide (address offset `reg@`, register, and the demo `value`):

| Peripheral | Register | `reg@` | Example value | exclusion-info |
|---|---|---|---|---|
| SPI2 | `AON_SCR_SPI2_0` | `2135` | `0x30001410` | 3 |
| I2C8 | `AON_SCR_I2C8_0` | `2130` | `0x30001610` | 3 |
| DMIC5 | `AON_SCR_DMIC5_0` | `2126` | `0x30001400` | 3 |
| GTE | `GTE_GPIO_SCR_TESCR_0` | `1359` | `0x38001232` | 2 (override) / 3 (base) |

The `reg@` offset and `value` are register-specific — treat the table as the demo's known-good
values and confirm against the file you're editing; don't transplant a value onto a different
register. `exclusion-info` differs between the `-override.dts` files and
`tegra234-firewall-config-base.dtsi`, so match the convention of the file you edit.

## Kernel-side: release the module

SPE runs **before** the Linux kernel. If the kernel later probes/re-inits the same
controller, it overwrites SPE's setup — the classic **"works for a few seconds then stops."**
So the module must be **disabled in the kernel device tree**.

Typical flow (peripheral-dependent — see `peripherals.md` for which need it):

1. Run `source_sync.sh` to fetch the kernel DT overlay sources.
2. Set the module node `status = "disabled"` (e.g. `i2c@c250000` for I2C8), or for GTE
   disable `hardware-timestamp@c1e0000`, or for IVC set `aon_echo` to `okay` (IVC is the
   opposite case — you're *enabling* a channel, not releasing a controller). Overlay files
   live under `Linux_for_Tegra/sources/hardware/nvidia/t23x/nv-public/overlay/` with
   platform-specific names (AGX Orin `tegra234-p3737-0000+p3701-0000.dts`, Orin Nano
   `tegra234-p3768-0000+p3767-0000.dts`).
3. Recompile the device tree, copy the resulting `.dtb` to `Linux_for_Tegra/dtb/`.
4. Reflash (DT changes require a real flash — see below).

Confirm the kernel isn't the owner whenever a peripheral works briefly then dies, and
whenever two processors might contend for the same controller.

## Reflash rule

- Changing **app C only** (already-plumbed peripheral) → rebuild, copy `spe.bin` →
  `spe_t234.bin`, and you can flash just the SPE-FW partition (`flash.sh -k A_spe-fw ...`).
- Changing **pinmux / gpio / gpioint / firewall-SCR / kernel DT** → **full reflash of all
  partitions**. These live outside the SPE-FW partition; an `A_spe-fw`-only flash will not
  apply them, and the peripheral will keep failing in a way that looks like a code bug.

See `build-flash.md` for the exact commands.

## DT debug map

| Symptom | Most likely layer |
|---|---|
| Peripheral totally silent, app seems fine | Enable flag not set, or pinmux still on `rsvd*` |
| Interrupt never fires | Layer 3 `gpioint` line not routed to SPE / VIC IRQ mismatch |
| Bus present but garbled / wrong direction | Layer 1 pull/tristate/enable-input, or Layer 2 list conflict |
| R5 crashes/hangs on peripheral access | Layer 4 SCR/firewall wrong, or clock not enabled |
| Works ~seconds then stops | Kernel didn't release the module |
| Edited DT but no change on target | Flashed only `A_spe-fw` instead of full reflash |
