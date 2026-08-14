# vendor/BNO055 — what was changed and why

Upstream: **BNO055 v1.2.1**, Robert Bosch GmbH reference driver, packaged for
Arduino as the MKR IMU Shield library. Vendored from the Arduino libraries
folder on 2026-08-13.

Pinned in-repo for the same reason as `vendor/Wire`, `vendor/CANBus` and
`vendor/SdFat`: `BuildAndUpload.cmd` must not resolve a vehicle-facing driver
from a global directory that nothing version-checks. Swapping that directory
silently changes the firmware, which has already happened once on this project.

## Why this library and not Adafruit_BNO055

Measured, not preferred:

| | Adafruit 1.6.4 | this driver |
|---|---|---|
| error propagation | **none on reads** | status returned from every call |
| `double` usage | **68** | **0** |
| blocking `delay()` | **28**, incl. `delay(1000)` in `begin()` | 2, in glue that is discarded |
| transport | fixed to Adafruit_BusIO | **function pointers** |
| extra dependencies | BusIO + Unified_Sensor | none |

Adafruit's `read8()` discards the return of `write_then_read()`, so a failed
read is indistinguishable from data, and `getVector()` returns a bare vector
with nowhere to report failure. That is precisely the fault just diagnosed on
the previous IMU — successful transactions delivering corrupt values — so
adopting it here would have been building the bug back in on purpose.

68 `double`s matter because the SAMD21 has no FPU; the codebase has a standing
rule about this (`* 0.01f`, never `/ 100.0`). And `delay(1000)` in bring-up
would undo the staged non-blocking init that `gpsInitTick()` exists to provide.

The function-pointer transport is the deciding feature: `bus_read`, `bus_write`
and `delay_msec` are hooks, so the driver routes through this project's own I2C
layer — `i2cBusBegin()`, `noteFault()`, the recovery path and the stuck-line
reporting — rather than around it.

## Changes from upstream

**1. Vendoring marker** — `BNO055.h`

```c
#define DASHCAM_BNO055_VENDORED 1
```

The only edit to upstream source. Consumers `#error` on its absence.

**2. `bno055_init()` honours a caller-supplied address** — `BNO055.c`

Upstream did this one line into `bno055_init()`:

```c
p_bno055->dev_addr = BNO055_I2C_ADDR;   /* unconditional */
```

which silently discards whatever the caller put in `dev_addr`. That default is
**0x28** — the address the datasheet calls the *alternative*. The default is
**0x29**, and GY breakouts commonly land there because COM3 is left floating.

On such a board every read in `bno055_init()` went to an address with nothing on
it, `chip_id` came back as whatever the failed read left in the buffer, and any
bus scan that had correctly located the device was overwritten immediately.
Patched to:

```c
if (p_bno055->dev_addr == 0)
    p_bno055->dev_addr = BNO055_I2C_ADDR;
```

Zero still means "caller did not choose", so upstream behaviour is unchanged for
anyone who does not set it.

Worth knowing while you are in there: `bno055_init()`'s return value does not
mean what it looks like. See the not-patched list below.

**3. C linkage guards** — `BNO055.h`

`BNO055.c` is C and the header shipped without `extern "C"` guards, so every
declaration acquired C++ mangling when included from a `.cpp` or `.ino` while
the definitions kept C names. It compiled cleanly and failed at link with

```
undefined reference to `bno055_init(bno055_t*)'
```

— note the argument list in the symbol, which is the tell that the caller was
looking for a mangled name. Wrapped the whole header in the usual
`#ifdef __cplusplus extern "C" {` / `}` pair.

Upstream never hit this because its own `BNO055_support.cpp` was the only
consumer and the examples reached the driver through that. This project calls it
directly, which is the point.

**4. Warning suppression, scoped to `BNO055.c`**

At `--warnings all` this one file emits **307 warnings**: 235
`-Wpointer-compare` and 72 `-Wmisleading-indentation`. Two thousand lines of
console output per build, in the middle of which a real warning from this
project's own code is invisible — which is the whole reason `--warnings all`
is on. Two `#pragma GCC diagnostic ignored` lines at the top of the file, and
nothing else in the build, are affected.

Both classes were read before being silenced:

- `p_bno055 == BNO055_Zero_U8X`, where the macro is `(unsigned char)0`
  (`BNO055.h:367`). In C an integer constant expression of value zero *is* a
  null pointer constant, so the check is correct; GCC only suspects a missing
  dereference.
- The setters indent an **unguarded** `if (status == SUCCESS)` as if the
  preceding one-line `if` covered it. Executing it unconditionally is what the
  code intends — `status` is still meaningful on the path that skips the mode
  change. Only the whitespace lies.

Verified after patching: the production build's warning output is empty, and
none of the 307 ever came from `lib/` or the sketches.

**5. Files deliberately NOT vendored**

| omitted | why |
|---|---|
| `BNO055_support.cpp` / `.h` | The Arduino glue. Drives the bus with raw `Wire` calls, checks no return value, and calls `delay()`. Replaced wholesale by `lib/BNO055Transport.{h,cpp}`. |
| `examples/` | All built on that glue. |

The core driver has **zero** references to the glue (verified by grep), so
removing it is clean — `BNO055.c` includes only `BNO055.h`, which includes only
`<limits.h>`.

## Upstream defects found but NOT patched

Left alone because callers can work around them and a fix would diverge further
from upstream than it is worth. They are recorded because each one has already
cost time, or would have.

**`bno055_set_operation_mode()` returns before the part has switched.** This is
the important one. The datasheet gives 7 ms for CONFIG→operation and 19 ms for
operation→CONFIG, and the driver honours neither — `delay_msec` is a member of
`struct bno055_t` that **`BNO055.c` never calls, not once**. From an operating
mode the function writes CONFIG and then immediately reads and writes the
target mode, inside the 19 ms window in which the part is still switching
(`BNO055.c:5093`).

The consequence is not a returned error. It is a mode change that silently does
not take, or output registers read while the part is still in the old mode.
Every convenience setter that wraps a mode change inherits it —
`bno055_set_accel_unit()`, `set_gyro_unit()`, `set_euler_unit()`,
`set_temperature_unit()`, `set_data_output_format()`, and the `set_*_data_select()`
family all bounce through CONFIG and back with no wait anywhere.

So: **this project's bring-up owns every mode transition itself**, writing
`OPR_MODE` directly, holding the datasheet delay as a state-machine stage
boundary, and reading the register back to confirm. The driver's mode-changing
setters are not used. See `lib/BNO055Init.cpp`.

**`sw_revision_id` is truncated to 8 bits.** The struct field is
`unsigned char` (`BNO055.h:297`) but is assigned a 16-bit composite of the LSB
and MSB registers (`BNO055.c:126`), so the major version is discarded. A part
reporting firmware 3.08 prints as `0x8`. Harmless — nothing reads the field —
but it looks exactly like a failed second byte of a two-byte read, which is a
misdiagnosis worth not repeating.

**`bno055_init()`'s return value describes only its LAST read.** It reassigns
`comres` from each of its several reads in turn, so a success return is not
proof the part answered. Check `chip_id` against `0xA0` instead.

## Notes for anyone touching this

`BNO055.c` is ~16 000 lines because it exposes every register the part has.
Almost none of it is called; the linker's `-ffunction-sections` /
`--gc-sections` drops the rest. Check the flash figure after any change rather
than assuming.

Facts from the datasheet (BST-BNO055-DS000-14 rev 1.4) that the driver does not
enforce and callers must respect:

- **No FIFO.** Zero occurrences in the datasheet. Peaks must come from polling
  at the 100 Hz fusion rate plus the High-G interrupt.
- **The I2C interface uses clock stretching** (§4). Relevant on a bus shared
  with the GNSS.
- **Fusion modes lock the accelerometer at ±4 g** (§3.5), so peaks saturate at
  39.2 m/s². Only non-fusion modes (AMG) can select a wider range.
- **Timings:** 400 ms power-on to CONFIG, 650 ms reset to CONFIG, 7 ms
  CONFIG→operation, 19 ms operation→CONFIG.
- **Default address is 0x29**, alternative 0x28 — but the driver hardcodes
  `BNO055_I2C_ADDR = 0x28`. GY boards commonly land on 0x29. Scan, do not assume.
- **High-G is unavailable in low-power mode** (§3.2); only No-motion and
  Any-motion apply there.
