---
name: jetson-spe-firmware
description: >-
  Develop, review, debug, and fix firmware for the NVIDIA Jetson Sensor Processing
  Engine (SPE) — the always-on (AON) ARM Cortex-R5 on Jetson AGX Orin and Orin Nano.
  Use whenever the user works with SPE / AON firmware: any mention of "SPE", "AON
  cluster", "Cortex-R5" on Jetson, "rt-aux-cpu-demo-fsp", the FSP, the OSA/CPL layers,
  spe.bin / spe_t234.bin, or the SPE demo apps (GPIO, I2C, SPI, AODMIC, Timer, GTE, IVC). Also trigger for the device-tree plumbing SPE
  peripherals need (pinmux, gpioint, gpio, firewall/SCR override dtsi), for building or
  flashing SPE firmware (arm-none-eabi toolchain, make bin_t23x, flash.sh -k A_spe-fw),
  and whenever an SPE peripheral "works then stops after a few seconds", crashes/hangs
  the R5, or a custom SPE firmware fails to flash. Prefer over generic FreeRTOS or
  Cortex-R5 help when the target is Jetson SPE/AON — its FSP driver model, mandatory
  device-tree changes, firewall/SCR rules, and "kernel must release the module"
  constraint trip up code written as if the R5 owned the pins.
---

# Jetson SPE / AON Cortex-R5 Firmware

Firmware for the **Sensor Processing Engine (SPE)**, the always-on ARM Cortex-R5 inside
NVIDIA Jetson (AGX Orin, Orin Nano). Runs **FreeRTOS V10.4.3** on top of the **FSP
(Firmware Support Package)**. This skill covers three things and nothing else was asked
for: **(1)** firmware C / FreeRTOS app code, **(2)** the device-tree config that every
SPE peripheral depends on, and **(3)** the build & flash workflow.

## The one thing to internalize first

**Working SPE firmware is never just C code.** For any peripheral to function from the
R5, four things must all agree, and getting the C right while any one of them is wrong
is the single most common failure:

1. **The app is enabled** — `ENABLE_<PERIPH>_APP := 1` in `soc/t23x/target_specific.mk`
   (plus `ENABLE_SPE_FOR_ORIN_NANO := 1` for Orin Nano).
2. **Pinmux** routes the physical pins to the peripheral function (not `rsvd*`).
3. **GPIO / interrupt maps** put the pins in the right list and route the IRQ line to an
   SPE interrupt.
4. **Firewall / SCR** grants the R5 access to that peripheral's register block.

**And the Linux kernel must release the module.** SPE runs *before* the kernel; if the
kernel owns/re-inits the same controller, it will yank the config out from under SPE —
this is exactly the "works for a few seconds then dies" bug. The module must be disabled
in the kernel device tree.

So whenever you review or fix "SPE code that doesn't work," you are debugging a **system**,
not a source file. Walk the four layers + the kernel-release requirement every time.
Details and per-platform file names live in `references/device-tree.md`.

## Hardware constraints that shape all code

The SPE R5 (from the TRM / dev guide):

- **ARMv7-R ISA**, Cortex-R5. 32 KB I-cache, 32 KB D-cache (**L1-only**), **256 KB TCM
  SRAM**, **2 VICs × 32 = 64 interrupts**, 200 MHz max core clock. Runtime platform/SoC is
  readable via `chip-id.h` (`tegra_get_platform()` → `0x234` on Orin).
- 64-bit AXI master for DRAM, 32-bit AXI master for MMIO.
- AON peripherals *exist* for timers/WDT/timestamp, I2C/SPI/CAN/UART/GPIO/PWM, DMIC —
  but the SDK **does not support all of them**. On Orin, supported = GPIO, I2C, SPI,
  AODMIC, Timer, GTE, IVC, SC7 AODMIC-wake. **CAN and UART are not supported** by the SPE
  SDK on these platforms — do not scaffold SPE CAN/UART drivers; say so and stop.

Implications you must respect:
- **Memory is tiny (256 KB).** No large static buffers; watch stack sizes and the total.
  **Gotcha:** the OSA task field is named `uxStackDepthBytes` but is passed as FreeRTOS's
  stack depth in **words** (StackType_t = 4 B on the R5) — so a demo's `512` is ~2 KB, not
  512 B (the idle/timer static stacks confirm word semantics). Prefer the smallest scope and
  smallest type that works.
- **Floating point exists, but is unsafe to share.** The build is `-mfpu=vfpv3-d16
  -mfloat-abi=softfp` → the R5 has a **VFPv3-D16 FPU** and `float` compiles to VFP
  instructions (softfp = float args in integer registers). **But `FreeRTOSConfig.h`
  configures no FPU context save**, so VFP registers are *not* preserved across task
  switches: using `float`/`double` in more than one task, or in an ISR, can silently corrupt
  another context's FP state. Keep FP inside a single task, or (better) use integer/
  fixed-point as every demo does. `-Wfloat-equal` is on.
- **Tick = 1 ms; preemptive, no time-slicing.** `configTICK_RATE_HZ = 1000`, so every
  `rtosTaskDelay(n)` is n ms and 1 tick = 1 ms. 32 priorities (0 = idle … 31; the software-
  timer daemon runs at 31). `configUSE_TIME_SLICING = 0` → **equal-priority tasks do not
  round-robin**; a running task holds the CPU until it blocks or yields. The RTOS tick uses
  **TKE timer0 — reserved, don't use it** (the timer demo deliberately uses timer2).
  Stack-overflow checking is on and **halts** the R5; `configASSERT` is defined and also
  halts, so any assertion you add is a hard stop.
- **Caches are on (L1-only).** For a buffer you DMA yourself, use the CPL cache ops
  (`dcache_clean` before HW reads it, `dcache_invalidate` before you read HW-written data)
  and **align to a cache line** — a partial line clobbers neighbors. The FSP drivers
  (e.g. AODMIC) already handle coherency for their own buffers; don't double up.
- **MMIO needs barriers.** Use the CPL accessors `readl(addr)` / `writel(data, addr)`
  (note: data first) with `barrier_*` from `barriers.h` where ordering matters — never a
  hand-cast `volatile` pointer.

## Where things live (source tree mental model)

Two roots under `$SPE_FREERTOS_BSP`: `fsp/` (common, reusable) and `rt-aux-cpu-demo-fsp/`
(demo apps + platform glue), plus the FreeRTOS tree.

- **App code + entry point**: apps in `rt-aux-cpu-demo-fsp/app/<periph>-app.c`; build flags
  in `soc/t23x/target_specific.mk`. `main.c` has two stages: `main()` does bare-metal init
  (`spe_vic_init` → `lic_init` → `init_padctrl` → `tegra_tsc_init` → `debug_early_init` →
  `tegra_hsp_init` → `bpmp_ipc_init` → `spe_late_init`) then starts the scheduler;
  **`main_task()`** does clk/debug/IVC init, then `gpcdma_init` and each enabled app's
  `<x>_app_init()`, then deletes itself. `main_task` is where a new app is wired in (see the
  develop workflow).
- **Platform layer** (from `target_specific.mk`): boot/reloc (`boot.S`, `relocate-dma.S`),
  interrupts via **LIC** (Legacy Interrupt Controller: `spe-lic.c`, `lic-map.c`, `tegra-lic.c`
  — SoC IRQ → LIC → VIC → R5), **HSP** doorbells (`hsp-tegra-*`), **AST** address translation,
  console over **TCU** (Tegra Combined UART, `tcu-ids.c`/`uart-t234.c`), and per-instance ID
  tables under `soc/t234/ids/aon/`.
- **OSA (RTOS abstraction)**: app code includes `<osa/rtos-task.h>`,
  `<osa/rtos-semaphore.h>`, `<osa/rtos-mutex.h>`, `<osa/rtos-event-group.h>` and calls the
  **`rtos*`-prefixed** wrapper (`rtosTaskCreate`, `rtosTaskDelay`, `rtosSemaphoreAcquire`,
  `rtosEventGroupSetBitsFromISR`, …) — **not** raw FreeRTOS (`xTaskCreate`) and **not**
  `osa_*`. The FreeRTOS implementation lives under `fsp/source/include/osa/freertosv10/osa/`.
  Full verified surface (task params, stack-in-bytes, the static-vs-dynamic buffer arg) is in
  `references/app-patterns.md` — consult it before writing or reviewing any task/sync code.
- **CPL (CPU abstraction)** — verified primitives: registers `readl`/`writel(data,addr)`
  (`reg-access.h`), barriers `barrier_memory_order`/`_complete`/`_instruction_synchronization`
  (`barriers.h`), cache `dcache_clean`/`dcache_invalidate` (`cache.h`), VIC `arm_vic_enable`/
  `arm_vic_set_isr_vect_addr` (`arm-vic.h`, ISR type `void(*)(void)`), chip-id
  `tegra_get_platform`/`tegra_platform_is_silicon` (`chip-id.h`). Full list in
  `references/app-patterns.md`.
- **Peripheral drivers**: `fsp/source/include/<periph>/tegra-<periph>.h` (e.g.
  `gpio/tegra-gpio.h`, `aodmic/tegra-aodmic.h`).
- **Per-SoC glue**: port layer `fsp/source/soc/<soc>/port/aon`; **ID structs**
  (base address + IRQ per instance) `fsp/source/soc/<soc>/ids/aon`. **Never hardcode a
  base address or IRQ — take it from the ID struct.**

> Signature discipline: `references/app-patterns.md` is the **fully header-verified** surface
> — every supported peripheral (GPIO, TKE, I2C core + HW, SPI master/slave, AODMIC, GTE, IVC,
> wake), the OSA and CPL layers, the entry point, and the RTOS config. Use it as ground truth;
> prefer it over anything restated elsewhere in the skill. The only things not fully mapped are
> deeper platform internals not needed for app work (AST carveout mechanics, BPMP/HSP protocol,
> the linker script). If the user provides those, fold them in; otherwise name the file and
> confirm rather than inventing.

## Coding standard: NASA/JPL "Power of Ten", enforced

Apply these to all SPE C, and call out violations explicitly in review (cite the rule #).
Adaptations for FreeRTOS/AON are noted.

> The FSP/driver layer is already **MISRA-C with logged deviations** (headers carry
> `START_RFD_BLOCK(MISRA, DEVIATE, …)` and `CT_ASSERT` signature guards); the **demo apps**
> are the looser sample layer. Power-of-Ten overlaps MISRA heavily — enforce it *with* the
> existing MISRA posture (so rule 10 = MISRA-clean + zero warnings), and target your findings
> at the app layer, not the already-annotated drivers.

1. **Simple control flow.** No `goto`, `setjmp`/`longjmp`, or recursion.
2. **Bounded loops.** Every loop has a statically provable upper bound. **Any poll of a
   hardware status bit or DMA/FIFO state MUST have an iteration cap or timeout** — an
   unbounded `while(!ready)` on AON hardware is a hang, and a hung R5 also bricks the next
   flash (see FAQ). Fail out and report, don't spin forever.
3. **No dynamic allocation after init.** No `malloc`/`pvPortMalloc` in steady state. The
   reference apps **do** allocate dynamically — `rtosTaskCreate` (with `.pxTCB = NULL`),
   `malloc`/`calloc`, `rtos*Create(NULL, …)` — but only during init, so they meet this rule's
   *"after init"* letter, not its no-heap spirit. To harden: set `.pxTCB` + `.pcStackBuffer`
   to file-scope static buffers (`StaticTask_t` + a stack array) and pass static buffers to
   the sync-object create calls. **`configSUPPORT_STATIC_ALLOCATION = 1`** (confirmed in
   `FreeRTOSConfig.h`; the idle/timer tasks already use `static rtosTaskBuffer` +
   `static rtosStackType[]` — copy that idiom), so no config change is needed — just supply
   the buffers. Footprint then known at build time, critical with only 256 KB.
4. **Short functions.** ~60 lines / one screen each; one job per function.
5. **Assertion-dense.** ≥2 checks per function; validate parameters and post-conditions.
   Note the base apps use **return-code checks + `error_hook`**, not `configASSERT` — so
   assertions are something you *add* when hardening. Either way, **check every driver return
   code** (see rule 7). Assertions are side-effect-free.
6. **Smallest scope.** Declare data at the tightest scope; `static` file-locals over
   globals; no shared mutable global without a documented concurrency guard.
7. **Check every return value / validate every parameter.** FSP driver calls return
   status — check it. Ignoring an init/transfer error is a defect, not a style nit.
8. **Restrained preprocessor.** Includes and simple constant/config macros only; no
   token-pasting cleverness, recursion, or macro control flow.
9. **Restricted pointers.** ≤1 level of dereference in a statement. Function pointers are
   *only* permitted where the RTOS/driver requires them — task entry points, timer and
   ISR callbacks, VIC handler registration — and those must be `static` and file-scope.
10. **Zero-warning build + MISRA-clean.** The real build is `-std=c99 -pedantic -Wall
    -Werror` plus `-Wpointer-arith -Wfloat-equal -Wshadow -Wbad-function-cast`
    (`-Wno-unknown-pragmas` for the IWYU pragmas). Keep it warning-free and align with the
    FSP's existing MISRA posture; treat every new warning as a bug.

### SPE-specific safety rules (verified against the demo sources)

- **ISR hygiene.** ISRs/callbacks are short and non-blocking, use only `...FromISR` wrapper
  variants (e.g. `rtosEventGroupSetBitsFromISR`) taking a `rtosPortBaseType
  *higher_prio_task_woken`, and log via **`printf_isr()`** (from `<printf-isr.h>`) — *not*
  `printf`. (Correction to the earlier draft: there **is** an ISR-safe printf and the code
  uses it; the rule is "`printf_isr` in ISR, `printf` in task," not "no logging in ISRs.")
- **DMA cache coherency — driver-mediated by default.** The FSP drivers own coherency: the
  AODMIC app allocates a plain `gpcdma_buf`, hands it to the driver, and reads via
  `tegra_aodmic_read` with **no app-side cache ops**. You only owe cache work if you
  **hand-roll DMA** — then `dcache_clean(buf,len)` before HW reads it, `dcache_invalidate
  (buf,len)` before you read HW-written data, and **cache-line-align** the buffer (a partial
  line clobbers neighbors; R5 is L1-only). Note SPI `tx_buf`/`rx_buf` *do* require DMA
  alignment even via the driver. Don't add spurious cache calls around driver-owned buffers.
- **Register access via CPL.** For any *direct* MMIO use `readl(addr)` / `writel(data, addr)`
  (data first) from `reg-access.h`, with `barrier_memory_order()`/`barrier_memory_complete()`
  where ordering matters; never a hand-cast `volatile` pointer. App code rarely needs this.
- **Interrupts are driver-managed at app level.** Register handlers through the peripheral
  driver (e.g. `tegra_gpio_set_irq_handler`), not raw VIC. `arm-vic.h` (2 VICs × 32 = 64
  IRQs; `arm_vic_enable`/`arm_vic_set_isr_vect_addr`, ISR type `void(*)(void)`) is used
  *inside* drivers; if you drop to it, take the IRQ from the ID struct and match the
  `gpioint`/SCR routing.
- **Prefer board/pin macros over literals.** `GPIO_APP_*` come from `gpio-aon.h`,
  `I2C_AON_BUS_NUM` from the app priv/config — use those (ultimately backed by the ID structs
  in `soc/<soc>/ids/aon`) rather than hardcoding pins, buses, or addresses.
- **Reference code is not safety-hardened.** The demos use `goto` cleanup, `sprintf`, dynamic
  allocation, and an intentionally infinite capture loop. Reproduce their **driver call
  sequences** faithfully, but flag those idioms when enforcing Power of Ten — the deltas table
  in `references/app-patterns.md` says exactly what to change.

## Review / fix workflow

When asked to review, debug, or fix SPE code, work top-down and be concrete about which
layer is at fault:

1. **Pin down target + peripheral.** AGX Orin (module `p3701` / carrier `p3737`, board
   `-a04`) vs Orin Nano (module `p3767` / carrier `p3768`, board `-dp-a03`). Which
   peripheral(s) and instance(s)? (I2C AON instances: 2/8/10. SPI: SPI2. AODMIC: DMIC5.)
2. **Firmware layer.** Run the Power of Ten + SPE-specific checklist above: OSA (not raw
   FreeRTOS), return-code checks, static allocation, ISR/`FromISR` correctness, DMA cache
   ops, barriers, bounded polls, ID-struct addresses. Flag each finding with the rule it
   breaks.
3. **Enablement layer.** Is `ENABLE_<PERIPH>_APP := 1` set? For Nano, is
   `ENABLE_SPE_FOR_ORIN_NANO := 1` also set? A perfectly correct app that was never
   enabled produces silence.
4. **Device-tree layer.** Do pinmux function, gpio lists, gpioint routing, and
   firewall/SCR entries all match this peripheral and these pins? Is the **kernel module
   disabled**? Use `references/device-tree.md` for the exact files and patterns per platform.
5. **Symptom → cause (FAQ model).** Map the reported behavior:
   - **Works, then stops after a few seconds** → the kernel re-configured the module SPE
     was using. Disable that module in the kernel DT.
   - **R5 crashes / hangs, module never runs** → almost always **wrong SCR/firewall
     settings** or **clocks not enabled/configured**. Recheck the SCR `reg@` value and the
     clock setup.
   - **Custom firmware fails to flash** (default flashed fine) → your firmware crashed the
     R5, the device hung, and the host can no longer talk to it — the bug is in the code
     you changed, not the flash command. Review the diff.
6. **State the fix per layer.** Give the specific edit(s): the C change, the
   `target_specific.mk` flag, and the exact DT hunks — kept mutually consistent.

## Develop (new app) workflow

To scaffold a new SPE peripheral app:

1. Confirm the peripheral is **SDK-supported** on the target (refuse SPE CAN/UART).
2. Provide an app skeleton following the demo pattern (see `app-patterns.md` for the exact
   idiom): an `init()` that fills an `rtosTaskParameters` struct and calls `rtosTaskCreate`
   (checking `!= rtosPASS`), and a task that configures the peripheral via its FSP driver
   (checking every return with `error_hook`, taking pins/bus from board macros or the ID
   struct). If interrupt-driven, register the handler through the **driver** (e.g.
   `tegra_gpio_set_irq_handler`) and log from it with `printf_isr`. Offer a static-allocation
   variant if Power-of-Ten strictness is wanted.
3. **Wire it into the build and `main_task`** (verified from `main.c` + `target_specific.mk`):
   add `ENABLE_<X>_APP := 1` in `soc/t23x/target_specific.mk`, and in `main_task()` add the
   `#include "app/<x>-app.h"` and the `<x>_app_init()` call, both under
   `#if defined(ENABLE_<X>_APP)`. The flag also pulls in that peripheral's `ids/aon` + `port/
   aon` sources and defines `-DENABLE_<X>_APP`. Known dependencies to respect: **SPI, SPI-slave,
   and AODMIC auto-enable `ENABLE_GPCDMA_FUNC`** (and need `gpcdma_init(&gpcdma_id_aon)`, which
   `main_task` already calls when GPCDMA is on); **GTE forces `ENABLE_GPIO_APP`**; **I2C adds
   `-DI2C_CUSTOM_PORT_INIT`**; Nano needs `ENABLE_SPE_FOR_ORIN_NANO := 1`. Don't touch **TKE
   timer0** (RTOS tick).
4. Enumerate the required device-tree edits for the chosen pins (pinmux + gpio + gpioint
   + firewall/SCR) and the kernel-side disable, referencing `device-tree.md`.
5. State the build + flash steps (`build-flash.md`), and note that pinmux/gpio/firewall
   changes require a **full reflash**, not just `A_spe-fw`.
6. Enforce Power of Ten throughout and mark any inferred driver call as "verify against
   `<header>`."

## References — read the one you need

- **`references/app-patterns.md`** — **verified** API surface and idioms from the actual demo
  sources: the `rtos*` OSA wrapper, error/logging conventions, per-peripheral driver calls +
  struct fields (GPIO, TKE, I2C, SPI master/slave, AODMIC, GTE, IVC), and the Power-of-Ten
  deltas vs the reference style. **Ground truth — read this before writing or reviewing any
  SPE C.**
- **`references/peripherals.md`** — per-peripheral facts: GPIO, I2C, SPI, AODMIC, Timer,
  GTE, IVC. Enable flags, instances, config-struct fields we can verify, header to
  consult, pin maps (Orin + Nano), and expected success output. Read this whenever a
  specific peripheral is in play.
- **`references/device-tree.md`** — the four DT layers in depth, exact per-platform file
  names, the firewall/SCR pattern, the kernel-disable + `source_sync.sh` + reflash flow,
  and worked field-level examples. Read this for any "why won't my peripheral come up"
  question or any DT edit.
- **`references/build-flash.md`** — toolchain, env vars, make targets, build artifacts,
  flash commands, clean targets, and exactly when a full reflash is required. Read this
  for any build or flash step.
