# SPE App Patterns — Verified API Surface

Verified against the actual **demo app sources** *and* the **OSA / CPL / driver headers**
(r36.x; mixed BSD-3-Clause and NVIDIA-proprietary — `chip-id.h`, `i2c-tegra.h` are
proprietary). This is ground truth and **overrides any guess elsewhere in the skill.**
Signatures / struct fields / constant values are stated as interface facts; snippets are
minimal derived illustrations. Where a needed header wasn't provided, that gap is marked.

Contents: [OSA/RTOS wrapper](#osartos-wrapper) · [House idioms & MISRA](#house-idioms--misra) ·
[Entry point & integration](#entry-point--app-integration) · [CPL layer](#cpl-layer-cpu-abstraction) ·
[GPIO](#gpio) · [TKE/Timer](#tke--timer) · [I2C](#i2c) · [SPI master](#spi-master) ·
[SPI slave](#spi-slave) · [AODMIC](#aodmic) · [GTE](#gte) · [IVC](#ivc) ·
[Power-of-Ten deltas](#power-of-ten-deltas)

---

## OSA/RTOS wrapper

App code uses the `rtos*` wrapper (`<osa/rtos-task.h>`, `<osa/rtos-semaphore.h>`,
`<osa/rtos-mutex.h>`, `<osa/rtos-event-group.h>`, `<osa/rtos-queue.h>`) — **not** raw
FreeRTOS, **not** `osa_*`. **Every wrapper returns `rtosError`** (`rtosPASS`/`rtosFAIL`/
`E_RTOS_*`); the demos assign to `int` and compare `!= rtosPASS`.

**Task creation** — struct-based. **Stack-depth units gotcha:** the field is *named*
`uxStackDepthBytes` but is passed straight to FreeRTOS's stack-depth argument, which is in
**words** (`StackType_t` = 4 B on the R5); the idle/timer static stacks in `main.c` confirm
word semantics. So `512` ≈ **2 KB**, not 512 B. The struct also carries optional static
buffers (`rtos-port.h` defines `rtosTaskParameters`):
```c
rtosTaskParameters p = {
    .pvTaskCode        = &my_task,          /* void my_task(void *pv) */
    .pcTaskName        = (char *)"name",
    .uxPriority        = rtosIDLE_PRIORITY, /* == tskIDLE_PRIORITY = 0; range 0..31 */
    .pvParameters      = NULL,
    .uxStackDepthBytes = 512,               /* WORDS despite the name → ~2 KB (aodmic 1024) */
    /* .pxTCB / .pcStackBuffer  → see static-vs-dynamic below */
};
rtosError e = rtosTaskCreate(&p, &handle /* or NULL */);
if (e != rtosPASS) { error_hook("..."); }
```
Other task calls (all `rtosError`): `rtosTaskDelay(ticks)`, **`rtosTaskDelayUntil(&prev,
incr)`** (use this for fixed-rate periodic loops — deterministic, no drift),
`rtosTaskDelete(NULL)`, `rtosTaskPriorityGet/Set`, `rtosTaskSuspend/Resume`,
`rtosTaskGetCurrentTaskHandle()`, `rtosTaskGetTickCount()` (→ `rtosTick`; elapsed ms ≈
`(end-start) * rtosTICK_RATE_MS`). `rtosTaskDelay(0)` yields instead of blocking.

**Timing & scheduling (`FreeRTOSConfig.h`):** `configTICK_RATE_HZ = 1000` → **1 tick = 1 ms**,
so `rtosTaskDelay(n)` = n ms (demo delays: gpio 3000 = 3 s, spi 5000 = 5 s, i2c 10000 = 10 s;
aodmic's `period_ms = 0` → `rtosTaskDelay(0)` = a yield). 32-bit ticks → max delay ~49.7 days.
**Preemptive but `configUSE_TIME_SLICING = 0`**, so equal-priority tasks don't round-robin.
32 priorities (0 = idle, 31 = software-timer daemon). `configASSERT` is defined and **halts**
(disable IRQ + spin); `configCHECK_FOR_STACK_OVERFLOW = 1` with a hook that halts on overflow
(but `uxTaskGetStackHighWaterMark` is off — no runtime headroom query).

**Static vs dynamic — now definitive.** `rtosTaskCreate` branches on `.pxTCB`: **`NULL` ⇒
dynamic `xTaskCreate`; non-NULL ⇒ static `xTaskCreateStatic`** using `.pcStackBuffer` +
`.pxTCB`. The demos leave both unset ⇒ dynamic. To satisfy Power-of-Ten rule 3, declare a
file-scope `StaticTask_t` and stack array and point `.pxTCB`/`.pcStackBuffer` at them. The
wrapper compiles *both* branches, so **static allocation is confirmed enabled in this
build** (both `configSUPPORT_STATIC_ALLOCATION` and `..._DYNAMIC_ALLOCATION` are on). Same
`buffer==NULL ? dynamic : static` pattern applies to every create call below.

**Semaphore** (`rtosError`, handle `rtosSemaphoreHandle`, static size `rtosSemaphoreSize()`
= 4 B): `rtosSemaphoreCreateBinary(buf, &h)`, `rtosSemaphoreCreateCounting(buf, max, init,
&h)`, `rtosSemaphoreAcquire(h, ticks)`, `rtosSemaphoreRelease(h)`,
`rtosSemaphoreAcquireFromISR/ReleaseFromISR(h, &woken)`, `rtosSemaphoreDelete(h)`,
`rtosSemaphoreGetCountDepth(h, &n)`.

**Mutex is RECURSIVE** (`rtosMutexCreate` → `xSemaphoreCreateMutex[Static]`;
`rtosMutexAcquire`→`xSemaphoreTakeRecursive`, `rtosMutexRelease`→`xSemaphoreGiveRecursive`):
`rtosMutexCreate(buf, &h)`, `rtosMutexAcquire(h, blocktime)`, `rtosMutexRelease(h)`.
(Note: `ivc-echo-task.c` creates its lock with `rtosMutexCreate` but then uses the
*semaphore* Acquire/Release on it — legal since both are FreeRTOS semaphore handles, but the
recursive semantics are bypassed.)

**Event group** (`rtosEventGroupCreate(buf, &h)`; ISR variants take `&woken`):
`rtosEventGroupWaitBits(h, bits, clearOnExit, waitForAll, &bitsOut, ticks)`,
`rtosEventGroupSetBits/ClearBits(h, bits)` and `...FromISR(h, bits, &woken)`,
`rtosEventGroupGetBits[FromISR]`, `rtosEventGroupDelete(h)`. **Hard R5 limit: only bits
0–23 are valid** — the top byte is reserved for kernel control. WaitBits/SetBits/ClearBits
return the event-group value, not pass/fail.

**Queue** (unused by demos but available): `rtosQueueCreate(...)`, size helper
`rtosQueueSize(n, elem_size)`.

**OSA software timers vs TKE.** `<osa/rtos-timer.h>` wraps **FreeRTOS software timers**
(`rtosTimerCreate(rtosTimerInitParametersType*, &h)`, `rtosTimerStart/Stop/ChangePeriod/
Delete`, `...FromISR`). These are **distinct** from the **TKE hardware timer** the timer
demo uses — don't conflate them. Software timers run in the timer-service task; TKE fires a
hardware ISR.

## House idioms & MISRA

- **`error_t` + `E_SUCCESS`** (from `error/common-errors.h`) is the driver return convention;
  check every call and `error_hook(const char *)` (`<err-hook.h>`) on failure. The demos do
  **not** use `configASSERT` — assertions are a Power-of-Ten *addition*.
- **Logging by context:** `printf()` in tasks, **`printf_isr()`** (`<printf-isr.h>`) in
  ISRs/callbacks, `dbgprintf()` (`<dbgprintf.h>`) for debug.
- **Helpers:** `BIT(n)` (`<misc/bitops.h>`), `ARRAY_SIZE(a)` (`<misc/macros.h>`).
- **Board/pin macros** centralize hardware: `GPIO_APP_*` (`gpio-aon.h`), `I2C_AON_BUS_NUM`
  (i2c priv/config). Backed ultimately by the ID structs in `soc/<soc>/ids/aon`.
- **The FSP is MISRA-C, not Power-of-Ten.** The driver/CPL headers carry formal deviation
  records — `START_RFD_BLOCK(MISRA, DEVIATE, Rule_x_y, "Approval: Bug ...")` — and each
  header self-guards with `CT_ASSERT(FSP__MODULE__FILE_H, ...)` to reject wrong same-named
  headers. So the *production* layer already targets MISRA (they overlap Power-of-Ten
  heavily); the *demo apps* are the looser sample layer (goto/malloc/sprintf). When you
  enforce Power-of-Ten, align it with the existing MISRA posture rather than fighting it.

## Entry point & app integration

From `main.c` + `target_specific.mk` — the canonical structure and the exact place to add an app.

- **`main()`** (bare-metal, before scheduler): `spe_vic_init()` → `lic_init()` →
  `init_padctrl()` → `tegra_tsc_init()` → `debug_early_init()` →
  `tegra_hsp_init(&tegra_hsp_id_aon)` → `bpmp_ipc_init()` → `spe_late_init()` →
  `rtosTaskInitializeScheduler(NULL)`. Failures `error_hook` + halt.
- **`main_task()`** (first RTOS task): `tegra_clk_init()` → `debug_init(...)` →
  `ivc_init_channels_ccplex()` → `gpcdma_init(&gpcdma_id_aon)` (if GPCDMA on) → each enabled
  app's `<x>_app_init()` → `rtosTaskDelete(NULL)` (self-delete). GPIO also needs
  `tegra_gpio_init(&chips[1], 1)` (AON chip = index 1) before `gpio_app_init()`.
- **To add an app:** put `#include "app/<x>-app.h"` and the `<x>_app_init()` call in
  `main_task`, both guarded by `#if defined(ENABLE_<X>_APP)`; add `ENABLE_<X>_APP := 1` in
  `soc/t23x/target_specific.mk`.
- **The RTOS tick uses TKE `tegra_tke_id_timer0`** (`setup_timer_interrupt()` →
  `tegra_tke_set_up_tick(...)`), reload `configCPU_CLOCK_HZ/configTICK_RATE_HZ` =
  `1000000/1000`. So **timer0 is reserved** and `configCPU_CLOCK_HZ` here is the tick source
  (1 MHz), *not* the 200 MHz core clock. The timer demo uses timer2 to avoid the tick.
- **Static-alloc idiom (copy this):** `main.c` supplies the idle/timer task memory with
  file-scope `static rtosTaskBuffer xTCB; static rtosStackType stack[N];` via
  `vApplicationGetIdleTaskMemory`/`...TimerTaskMemory`. That's exactly how to fill
  `.pxTCB`/`.pcStackBuffer` for your own static tasks.

**Build flag dependencies (`target_specific.mk`, all default 0):** `ENABLE_SPI_APP`,
`ENABLE_SPI_SLV_APP`, `ENABLE_AODMIC_APP` each set `ENABLE_GPCDMA_FUNC := 1` and add their
`ids/aon` + `port/aon` sources; `ENABLE_GTE_APP` forces `ENABLE_GPIO_APP := 1`;
`ENABLE_I2C_APP` also defines `-DI2C_CUSTOM_PORT_INIT`; `ENABLE_SPE_FOR_ORIN_NANO`
(0 = AGX Orin, 1 = Orin Nano). Toolchain: `arm-none-eabi-gcc -std=c99 -pedantic
-mcpu=cortex-r5 -mthumb-interwork -mfloat-abi=softfp -mfpu=vfpv3-d16 -Wall -Werror`
(+`-Wpointer-arith -Wfloat-equal -Wshadow -Wbad-function-cast`).

## CPL layer (CPU abstraction)

Direct hardware access — app code rarely needs these (drivers use them), but they're the
correct primitives when you do.

- **Register access** (`reg-access.h`, wraps `ioread32`/`iowrite32`): `readl(addr)`,
  `readl_base_offset(base, off)`, **`writel(data, addr)`** — note **data first, addr
  second** (Linux-style; easy to reverse), `writel_base_offset(data, base, off)`,
  `writel_base_regoffset(...)`. Never hand-cast `*(volatile uint32_t*)addr`.
- **Barriers** (`barriers.h`, inline asm): `barrier_memory_order()` = `dmb sy`,
  `barrier_memory_complete()` / `barrier_cache_op_complete()` = `dsb sy`,
  `barrier_instruction_synchronization()` = `isb sy`, `barrier_compiler()` = compiler-only.
- **Cache** (`cache.h`; **R5 is L1-only**): `dcache_clean(base, len)` (write dirty→DRAM
  before HW reads your buffer), `dcache_invalidate(base, len)` (drop stale lines before you
  read a HW/DMA-written buffer), `dcache_clean_all()`, `dcache_invalidate_all()`,
  `icache_invalidate[_all]`, `cache_enable()/disable()/enable_ecc()`. **Partial cache lines
  clobber neighbors** — the header warns that a non-line-aligned range invalidates/cleans
  *more* than asked, so **align shared buffers to a cache line**. (`cache_invalidate`/
  `cache_clean` are deprecated → use the `dcache_*` forms.)
- **VIC** (`arm-vic.h`): the R5 has **2 VICs × 32 IRQs = 64** (`MAX_ARM_VICS`,
  `ARM_VIC_IRQ_COUNT`, `MAX_ARM_VIC_IRQS`). `arm_vic_enable/disable(vic, irq)`,
  `arm_vic_disable_all(vic)`, `arm_vic_set_isr_vect_addr(vic, irq, isr)` where ISR is
  `void (*)(void)`, `arm_vic_gen/clear_software_int`, `arm_vic_save_state/restore_state(vic,
  &ctx)` (`struct arm_vic_context` — used across SC7 suspend). App code registers via the
  *driver* (e.g. `tegra_gpio_set_irq_handler`), not these directly.
- **Chip ID** (`chip-id.h`, proprietary): `tegra_get_platform()` → **`0x234` for Orin
  (T234)** (also 0x186/0x194/0x239/0x264), `tegra_get_chipid()` → `0x23` (T23X),
  `tegra_get_major_rev/minor_rev/sub_rev()`, `tegra_get_sku_id()`,
  `tegra_platform_is_silicon()/is_fpga()/is_vdk()`, `tegra_fuse_control_read(off, &val)`.
  Use for runtime platform branching instead of compile-time assumptions.

## GPIO
`<gpio/tegra-gpio.h>`. **All calls return `error_t`** (demos use `int` + nonzero-is-error).
The `gpio` argument is a **packed global id** `((ctrl_id << 16) | pin)` (`gpio_global_id()`
inline; that's why `GPIO_APP_*` are macros, not bare pin numbers). Controllers: max 2.
Precondition: `tegra_gpio_init(&ids, num_ctrls)` (plus `tegra_gpio_suspend/resume(ctrl_id)`
across SC7).
```
tegra_gpio_direction_out(gpio, init_val)      tegra_gpio_direction_in(gpio)
tegra_gpio_get_direction(gpio, &dir)          tegra_gpio_set_value(gpio, val)
tegra_gpio_get_value/get_input_value/get_output_value(gpio, &val)
tegra_gpio_set_debounce(gpio, ms)
tegra_gpio_set_irq_type(gpio, type, level)    tegra_gpio_enable_irq(gpio)/disable_irq(gpio)
tegra_gpio_set_irq_handler(gpio, void(*h)(void *hdata), data)   tegra_gpio_clear_irq_handler(gpio)
tegra_gpio_read_irq_status(gpio, &bool)       tegra_gpio_clear_irq_status(gpio)
tegra_gpio_enable_timestamp(gpio)/disable_timestamp(gpio)       /* GTE hook */
```
IRQ **type** (`tegra_gpio_irq_type`): `NONE`=0, `LEVEL`=1, `SINGLE_EDGE`=2, `DOUBLE_EDGE`=3.
IRQ **level** (`tegra_gpio_irq_level`): `LOW_LEVEL`=`FALLING_EDGE`=0, `HIGH_LEVEL`=
`RISING_EDGE`=1. IRQ dispatch flow: VIC → `tegra_gpio_irq_handler(void*)` → your registered
handler.

## TKE / Timer
`<tke/tke-tegra.h>`, `<processor/tke-tegra-hw.h>`; id `tegra_tke_id_timer2`. This is the
**hardware** Time Keeping Engine (see OSA note re: software timers).
```
tegra_tke_set_up_timer(&tegra_tke_id_timer2, TEGRA_TKE_CLK_SRC_USECCNT,
                       /*periodic*/true, divisor, cb /* void cb(void*) */, /*data*/0)
tegra_tke_stop_timer(&id)   tegra_tke_get_pcv(&id)   tegra_tke_enable/disable/clear_timer_irq(&id)
```
The timer has a **down-counter (PCV)** loaded with `divisor-1`; the IRQ fires when it hits 0.
**Clock sources** (`clk_src_sel`): `USECCNT`=0 (**1 MHz** → divisor is µs), `OSCCNT`=1 (clk_m),
`TSC_BIT0`=2 (31.25 MHz), `TSC_BIT12`=3 (7.63 kHz). `divisor ≤ TEGRA_TKE_MAX_TIMER`
(`0x20000000`). So the demo's `5000000` @ USECCNT = 5 s. `callback` non-NULL auto-enables the
IRQ; it runs in ISR context → `printf_isr`. **Timekeeping/TSC helpers** (separate from the
tick): `tegra_tke_get_usec/osc/tsc32()`, `tegra_tke_get_tsc(&hi,&lo)`/`tsc64()`,
`tegra_tke_get_tsc_ns()`, `tegra_tke_tsc_to_ns/ns_to_tsc`, `tegra_tke_get_elapsed_usec(hi,lo)`,
`tegra_tke_convert_usecs_to_ticks(us)`. The `*_ns` conversions need `tegra_tsc_init()` first
(called in `main`) unless the fixed-ratio TSC file is used. `tegra_tke_set_up_tick(&timer0,
clk, divisor)` is what wires the RTOS tick (main.c).

## I2C
Two layers — both now confirmed:
- **App-facing core `<i2c/i2c.h>`:** `i2c_controller_init(I2C_AON_BUS_NUM, &hi2c)` (or
  `i2c_controller_pre_init`) with `struct i2c_handle`; then the transfer calls (all `error_t`,
  all take a `timeout`):
  - `i2c_reg_read_data(&hi2c, slave_addr, reg_addr, flags, pdata, num_data, timeout)` — the
    BMI160 chip-ID read the demo's `i2c_test()` performs.
  - `i2c_reg_write_data(&hi2c, slave_addr, reg_addr, flags, pdata, num_data, timeout)`
  - `i2c_send_data` / `i2c_receive_data(&hi2c, slave_addr, flags, pdata, num_data, timeout)`
  - `i2c_do_transfer(&hi2c, struct i2c_xfer_msg *msgs, num_msgs, timeout)` — multi-message.
  - `i2c_suspend/resume`. State/bus introspection: `i2c_get_xfer_state` (→
    `I2C_XFER_REQUESTED/STARTED/IN_PROGRESS/COMPLETED` = BIT(0..3)), `i2c_get_bus_state`
    (→ `I2C_SCL_BUSY`/`I2C_SDA_BUSY`/`I2C_BUS_BUSY`). Demo bus clock 100 kHz.
- **Tegra HW layer `<i2c/i2c-tegra.h>` (proprietary):** `struct i2c_tegra_handle`,
  `tegra_i2c_pre_init`/`tegra_i2c_init(ctrl_id, &h, &hw_ops)` (plugs into the core via
  `struct i2c_hw_ops`), ISR `tegra_i2c_irq_handler(ctrl_id)`. Transfer flags (used in `flags`
  above): `I2C_XFER_FLAG_RD`=BIT(0), `_TEN`=BIT(1) (10-bit), `_NOSTART`=BIT(2), `_IGNORE_NAK`=
  BIT(3). Limits: `I2C_PKT_XFER_MAX_BYTES`=500, `I2C_FIFO_MAX_SIZE_W`=128. `ENABLE_I2C_APP`
  compiles with `-DI2C_CUSTOM_PORT_INIT`.

## SPI master
`<spi/spi.h>`; controller `spi_ctlr_spi2` (opaque `struct spi_ctlr`); DMA `gpcdma_id_aon`.
All calls `error_t`: `spi_init(&ctlr, &master_init)` → `spi_client_setup(&ctlr, &client)` →
`spi_transfer(&ctlr, &xfer)` (also `spi_suspend/resume`, ISR `spi_master_isr`).

Structs (verified field order/types):
- `spi_dma_chan { uint32_t tx, rx; }`
- `spi_master_init { void *dma_id; spi_dma_chan dma_chans; uint32_t spi_max_clk_rate; uint8_t dma_slave_req; }`
- `spi_client_setting { uint32_t spi_max_clk_rate; int32_t cs_setup_clk_count, cs_hold_clk_count, cs_inactive_cycles; uint8_t chip_select; bool set_rx_tap_delay, spi_no_dma; }`
  — **correction:** the CS-timing fields live in this shared struct; the *master* demo just
  leaves them 0, the *slave* demo sets them. (Earlier notes wrongly called them slave-only.)
- `spi_xfer { const void *tx_buf; void *rx_buf; uint32_t len; uint32_t spi_clk_rate; uint16_t mode; uint16_t flags; uint8_t tx_nbits, rx_nbits, chip_select, bits_per_word; }`
  — **`tx_buf`/`rx_buf` must be DMA-aligned** (header requirement).

Constant values: `XFER_FIRST_MSG`=3, `LAST_MSG`=4, `HANDLE_CACHE`=5 (used as `BIT(...)` in
`.flags`); `NBITS_SINGLE/DUAL/QUAD`=1/2/4; `CPHA`=1, `CPOL`=2; `MODE_0..3` from those;
`CS_HIGH`=0x04, `LSB_FIRST`=0x08, `3WIRE`=0x10, `LSBYTE_FIRST`=0x1000 (hence `mode` is
16-bit). Demo: 12 MHz, `dma_chans.tx=2/rx=3`, `dma_slave_req =
GPCDMA_AO_CHANNEL_CH0_CSR_0_REQ_SEL_SPI` (`<argpcdma_ao.h>`).

## SPI slave
`<spi-slv/spi-slave.h>` (structs from `spi/spi-priv.h`, shared with the master). Reuses
`spi_client_setting` + `spi_xfer`; adds `struct spi_slave_init_setup { void *dma_id;
spi_dma_chan dma_chans; uint32_t spi_max_clk_rate; uint8_t dma_slave_req; }` and
`SPI_DEFAULT_MAX_CLK` = 10 MHz. Calls `error_t`: `spi_slave_init(&ctlr, &setup)` →
`spi_slave_setup(&ctlr, &client)` → `spi_slave_transfer(&ctlr, &xfer)` (ISR `spi_slave_isr`).
Demo sets CS timing (16/16/32); `xfer.tx_buf = NULL` (rx only). Bring the master up first.

## AODMIC
`<aodmic/tegra-aodmic.h>`; controller `tegra_aodmic_ctlr_aon`; GPCDMA via
`<gpcdma/gpcdma.h>` (`gpcdma_id_aon`). **Precondition: `gpcdma_init()` before
`tegra_aodmic_open`.** All calls `error_t`.

Config struct (exact — the docs showed only 3 of 9 fields):
```c
struct tegra_aodmic_config {
    enum tegra_aodmic_sample_rate    sample_rate;    /* 8000/16000/44100/48000 (literal Hz) */
    enum tegra_aodmic_channel_config channel_config; /* MONO_LEFT=0, MONO_RIGHT=1, STEREO=2 */
    enum tegra_aodmic_sample_width   sample_width;   /* BITS_PER_SAMPLE_16=2, _32=4 (BYTES) */
    uint32_t                         period_size;    /* sample frames; MUST be multiple of 16 */
    uint32_t                         num_periods;    /* 2 .. AODMIC_MAX_NUM_PERIODS(=4) */
    struct gpcdma_id                *dma_id;         /* &gpcdma_id_aon */
    uint32_t                         dma_chan_num;   /* 0..7 */
    uint8_t                         *gpcdma_buf;     /* driver-owned; app allocates, never touches */
    uint32_t                         gpcdma_buf_size;/* >= period_size*num_periods*4*num_channels */
};
```
`tegra_aodmic_open(&ctlr, &cfg)`, `tegra_aodmic_read(&ctlr, data, count_BYTES)`,
`tegra_aodmic_close(&ctlr)`. Native samples are 24-bit, scaled to the chosen width; the
driver uses **4 bytes/sample internally** regardless (hence the `*4` in `gpcdma_buf_size`;
too-small ⇒ `E_AODMIC_MEM_ALLOC_FAILURE`). Error codes include `E_AODMIC_READ_TIMEOUT`,
`E_AODMIC_INVALID_PERIOD`, etc. **The driver owns cache coherency** for `gpcdma_buf` — no
app-side cache ops (only your job if you bypass the driver). Source note: SPE must not enter
low-power states while running. (See "Known demo-code bugs" for the `period_ms` truncation.)

## GTE
`<gte-tegra.h>`, `<gte-tegra-hw.h>`. Instance `tegra_gte_id_aon`. Timestamp struct:
`struct tegra_gte_ts { uint64_t tsc; uint8_t slice; uint8_t bit_index; bool bit_dir; bool
invalid; }` — **`bit_dir`: 0 = rising, 1 = falling** (per header). Raw FIFO entry is
`struct tegra_gte_fifo_msg` (8 words: ctrl, tsc_hi/lo, src, ccv, pcv, encv, cmd).
```
tegra_gte_slice_set_enable_mask(&tegra_gte_id_aon, slice, bitmap)   /* bitmap = BIT(signal) */
error_t tegra_gte_setup(&id, occupancy_threshold,
                        cb /* void cb(void*, struct tegra_gte_ts*) */, irq_data,
                        dma_xfer /* or NULL */, dma_en /* bool */)
tegra_gpio_enable_timestamp(GPIO_APP_IN)
```
Two modes: **no-DMA** (interrupt when FIFO occupancy > `occupancy_threshold` — that field is
only meaningful here) or **DMA** (supply a `tegra_gte_dma_xfer` fn + `tegra_gte_dma_complete_
handler`, `dma_en=1`). **The signal→slice/bit map is in `gte-tegra-hw.h`** (this is the
"hidden GTE column" from the pinmux spreadsheet): slice 2 = GPIO 0–27; slice 1 = LIC0–3, GPIO,
WAKE0, DMIC, **FPUINT** (bit 10 — confirms the FPU), GPIO 28–43; slice 0 = timers 0–3, DMA 0–7,
I2C1–3, SPI, UART1–2, CAN1/2. Callback runs in ISR → `printf_isr`.

## IVC
`<tegra-ivc.h>`, `<ivc-channels.h>`, `<ivc-config.h>`. Channel:
`struct tegra_ivc_channel { void *write_header, *read_header; uint32_t nframes, frame_size;
tegra_ivc_notify_remote notify_remote; uint32_t channel_group, write_count, read_count; }`.
Each app channel is a `struct ivc_task_id { name; struct tegra_ivc_channel *ivc_ch;
struct ivc_channel_ops *ops; void *priv; }`; `ivc_channel_ops` is an init/notify/init_complete/
debug_enable vtable (all `rtosPortBaseType *woken`). `ivc_init_channels_ccplex()` (called in
`main_task`) sets them up; `hsp_ivc_notify_ccplex(ch, is_read)` rings the doorbell.
- TX: `tegra_ivc_tx_get_contiguous_write_space(ch, &buf, &noncontig)` (≥1 ⇒ space) → `memcpy`
  → `tegra_ivc_tx_send_buffers(ch, count)`. Also `tegra_ivc_tx_get_write_space/_frames/
  _get_write_buffer`.
- RX: `tegra_ivc_rx_get_contiguous_read_available(ch, &buf, &noncontig)` → process → 
  `tegra_ivc_rx_notify_buffers_consumed(ch, count)`. Also `_get_read_available/_frame/_frames`.
- Lifecycle: `tegra_ivc_init_channel_in_ram/_from_ram`, `tegra_ivc_channel_reset`,
  `tegra_ivc_channel_notified/_is_synchronized`. Access is **mutex-guarded**; ISR signals work
  via `rtosEventGroupSetBitsFromISR`.
- `ivc-config.h`: **`TEGRA_IVC_ALIGN` = 64** (that's the demo's "64-byte frame":
  `IVC_ECHO_CH_FRAME_SIZE = TEGRA_IVC_ALIGN`), `IVC_ECHO_CH_NFRAMES` = 16, channel size =
  `128 + nframes*frame_size`; carveout at `CCPLEX_CARVEOUT_BASE 0x80000000` via **AST**
  (`AST_IVC_REGION`, `AON_STREAMID`); HSP mailbox `AON_CCPLEX_IVC_HSP_ID = &tegra_hsp_id_top1`.

## Wake (AODMIC voice-wake)
`<wake-tegra.h>`. Setup order: `tegra_wake_route_event(&id, wake_event, tier, sel)` (route
into a tier — this is the AODMIC doc's Tier2/wake83) → `tegra_wake_enable_event(&id,
wake_event)` → `tegra_wake_enable(&id)`. Also `tegra_wake_disable_event/disable/clear_irq`,
and `tegra_wake_trigger_wake_event(&tegra_wake_id_wake)` (what the AODMIC app calls on a loud
sound).

---

## Known demo-code bugs (flag on review)

Two verified defects in the sample apps — good review exemplars, and reasons not to copy the
demos blindly:
- **`aodmic-app.c`:** `period_ms = period_size / sample_rate` = `256 / 16000` = **0** (integer
  truncation; the rate enum really is Hz) — the intended inter-read delay becomes a bare yield.
- **`gte-app.c`:** the callback prints `gte_ts->bit_dir ? "rising" : "falling"`, but
  `gte-tegra.h` documents **`bit_dir` 0 = rising, 1 = falling** — so the edge label is
  **inverted**.

## Power-of-Ten deltas

The demo apps are **sample code, not safety-certified**; the FSP/driver layer beneath them
**is MISRA-C with logged deviations**. Reproduce the driver call sequences faithfully, but
flag/rewrite these app-layer idioms when enforcing Power-of-Ten:

| Reference (demo layer) does | Power-of-Ten wants | Note |
|---|---|---|
| Dynamic alloc: `rtosTaskCreate` w/ `.pxTCB=NULL`, `malloc`/`calloc`, `…Create(NULL,…)` | Static allocation | All at **init**, so meets rule 3's "after init" letter. Harden: set `.pxTCB`+`.pcStackBuffer` (static path is compiled in) and pass static buffers to the create calls. |
| `goto err_exit` / `goto done` (aodmic, i2c) | No `goto` (rule 1) | MISRA also restricts this; refactor to single-exit. |
| `sprintf` (aodmic) | `snprintf` | Fixed buffer, unbounded write. |
| `for (i=1; ; i++)` capture loop (aodmic) | Bounded loops (rule 2) | Legit for a daemon; keep **status-bit polls** bounded — the FSP drivers already encapsulate those. |
| Return checks + `error_hook` | +Assertions (rule 5) | Assertions are an addition; base code is MISRA-style return-checking. |
| `printf_isr` + `*FromISR` + `&higher_prio_woken` | (already correct) | Keep ISRs short. |
| Event-group bits 0–23 only | (respect it) | Bits 24–31 reserved on R5 — using them is a latent correctness bug. |
| Unaligned DMA buffers (SPI xfer bufs on stack) | Cache-line-align + `dcache_*` | SPI header requires DMA-aligned tx/rx; partial cache lines clobber neighbors. |
