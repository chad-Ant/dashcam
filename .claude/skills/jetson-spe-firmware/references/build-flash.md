# SPE Build & Flash Reference

Everything to compile SPE firmware and get it onto the board. Verified from the
Compiling-and-Flashing guide (r36.4.3).

## Toolchain

NVIDIA does **not** ship the toolchain — download it yourself:
**arm-gnu-toolchain-13.2.rel1**, x86_64 Linux host, target **arm-none-eabi** (from ARM's
developer downloads). SPE is bare-metal-style ARMv7-R, so it's the `arm-none-eabi`
(EABI) cross-compiler, prefix `arm-none-eabi-`.

Also useful: `sudo apt-get install doxygen` if you want the generated API docs (`make docs`),
which is one practical way to browse the real driver/OSA/CPL signatures.

## Environment

Point the build at the source root and the cross-compiler:

```
export SPE_FREERTOS_BSP=<root holding rt-aux-cpu-demo-fsp, fsp, and the freertos source dirs>
export CROSS_COMPILE=<toolchain install>/bin/arm-none-eabi-
cd ${SPE_FREERTOS_BSP}/rt-aux-cpu-demo-fsp
```

`SPE_FREERTOS_BSP` is the directory where `rt-aux-cpu-demo-fsp/`, `fsp/`, and the FreeRTOS
tree all sit side by side. `CROSS_COMPILE` is the **prefix** (trailing dash), not a
directory.

## Build targets (run from `rt-aux-cpu-demo-fsp/`)

| Command | Result |
|---|---|
| `make bin_t23x` | Build SPE firmware for the T23x SoC (Orin family). This is the usual one. |
| `make` / `make all` | Build firmware for all SoCs **and** the Doxygen docs. |
| `make docs` | Doxygen only → `out/docs/index.html`. |

Add `-j<N>` to parallelize — e.g. `make -j16 bin_t23x` is markedly faster.

**Before building, set the app flag(s)** in `soc/t23x/target_specific.mk`:
`ENABLE_<PERIPH>_APP := 1` (and `ENABLE_SPE_FOR_ORIN_NANO := 1` for Orin Nano), then rebuild.
All flags default to **0** (a fresh build has no demo app). Verified dependencies:

- `ENABLE_SPI_APP` / `ENABLE_SPI_SLV_APP` / `ENABLE_AODMIC_APP` each auto-set
  `ENABLE_GPCDMA_FUNC := 1` and add their `ids/aon` + `port/aon` sources.
- `ENABLE_GTE_APP` forces `ENABLE_GPIO_APP := 1`.
- `ENABLE_I2C_APP` also defines `-DI2C_CUSTOM_PORT_INIT`.
- Enabling an app also requires wiring its `<x>_app_init()` into `main_task()` (see
  `app-patterns.md`). **Don't repurpose TKE timer0** — it's the RTOS tick.

Compiler (from `Makefile`): `arm-none-eabi-gcc -std=c99 -pedantic -mcpu=cortex-r5
-mthumb-interwork -mfloat-abi=softfp -mfpu=vfpv3-d16 -Wall -Werror` plus `-Wpointer-arith
-Wfloat-equal -Wshadow -Wbad-function-cast`. Note **softfp + VFPv3-D16**: hardware FP exists
but FreeRTOS saves no FPU context across switches — keep `float` out of shared paths (see
SKILL.md).

## Build artifacts

- Firmware binary: `out/<soc>/spe.bin` (for T234, `out/t234/spe.bin`).
- Docs: `out/docs/index.html`.

## Clean targets

| Command | Cleans |
|---|---|
| `make clean` | Everything (firmware + docs). |
| `make clean_t23x` | T23x firmware objects/artifacts only. |
| `make clean_docs` | Doxygen output only. |

## Flash

1. **Back up** the stock firmware: save `Linux_for_Tegra/bootloader/spe_t234.bin`.
2. **Copy** your build over it: `out/t234/spe.bin` → `Linux_for_Tegra/bootloader/spe_t234.bin`
   (the T234 SoC filename is `spe_t234.bin` on both AGX Orin and Orin Nano).
3. **Flash the SPE-FW partition only** (fast path, valid only when *just* the firmware
   changed):

```
sudo ./flash.sh -k A_spe-fw <your-jetson-board-config> internal
```

## When you need a FULL reflash instead

`A_spe-fw` flashes only the SPE firmware partition. **Any device-tree change** — pinmux,
gpio, gpioint, firewall/SCR, or kernel DT — lives in other partitions and will **not** be
applied by an `A_spe-fw`-only flash. After those, reflash **all** partitions (the normal
full `flash.sh`/`initrd` flash for your board), or the peripheral will keep behaving as if
your code is broken when in fact the plumbing never reached the board.

Rule of thumb:
- Touched only `app/*.c` on an already-plumbed peripheral → `A_spe-fw` is enough.
- Touched any DT / pinmux / firewall / enable-that-needs-new-DT → full reflash.

## Flash-failure debugging

If the stock firmware flashed fine but your modified firmware won't flash: your firmware
most likely **crashed the R5**, the device hung, and the host can no longer communicate —
so the flash can't proceed. It's not the flash command. Revert to the known-good binary to
confirm the board still flashes, then re-review your diff (bad SCR access, unbounded
hardware poll, uninitialized clock, etc. — see the review checklist in SKILL.md).
