# Vendored SAMD `Wire` library (patched fork)

## Why this exists

`TwoWire::requestFrom()` in the stock Arduino SAMD core invokes undefined
behaviour on **every single-byte read**:

```cpp
bool busOwner;                                   // never initialised
for (byteRead = 1; byteRead < quantity && (busOwner = sercom->isBusOwnerWIRE()); ++byteRead)
{ ... }
...
if (stopBit && busOwner) { /* emit STOP */ }
if (!busOwner) { byteRead--; }
```

The assignment to `busOwner` is part of the loop **condition**. With
`quantity == 1` the condition short-circuits on `1 < 1` before that assignment
ever runs, so both later uses read an indeterminate value. Depending on what the
compiler happens to leave in that register, a perfectly good one-byte transfer
can return 0 bytes, or complete without emitting a STOP and leave the bus held.

It is not theoretical for this project: the MKR Zero shares one I2C bus between
the u-blox GNSS receiver, the LSM6DSOX and the LIS3MDL, and single-byte register
reads (WHO_AM_I, STATUS) are the most common transaction on it. The
`WireRegression` T5 test measures the GNSS driver issuing **20 one-byte reads to
0x42** in a few seconds of ordinary polling.

It also cannot be worked around library-by-library. The IMU driver could pad a
one-byte read to two bytes because its extra byte is harmless, but the same
trick is **wrong** for the u-blox: its data register `0xFF` is a consuming byte
stream, so an extra read silently eats a byte of a real UBX message. The fix has
to be in `Wire` itself, once, for every client on the bus.

## Provenance

| | |
|---|---|
| Upstream project | `arduino/ArduinoCore-samd` |
| Upstream path | `libraries/Wire` |
| Version | Arduino SAMD Boards core **1.8.14** (`platform.txt`: `version=1.8.14`) |
| Local source | `%LOCALAPPDATA%/Arduino15/packages/arduino/hardware/samd/1.8.14/libraries/Wire` |
| Upstream tree | <https://github.com/arduino/ArduinoCore-samd/tree/1.8.14/libraries/Wire> |
| Licence | LGPL-2.1-or-later — see `LICENSE.LGPL-2.1`, original notices retained verbatim in both sources |

SHA-256 of the **unmodified upstream files** this fork was taken from, so a
future reader can verify exactly what was forked and diff the patch back out:

```
Wire.cpp  71EB440D1969D35229AA704FB05D4FCB40A864D049D8B1EEC3089B50F08CEF7F
Wire.h    9A792E47A8F4FFACD5DD97CC93F15D9EDED867ADE6EA97D00EEEE26F25DDB42B
```

`LICENSE.LGPL-2.1` is copied verbatim from the same core distribution
(`.../samd/1.8.14/LICENSE`, SHA-256
`20C17D8B8C48A600800DFD14F95D5CB9FF47066A9641DDEAB48DC54AEC96E331`). It is
included because the repository root `LICENSE` is MIT, which does not cover this
directory: these files are LGPL and stay LGPL.

The current upstream `master` still contains the uninitialised variable, so this
is not a defect a core upgrade is expected to resolve on its own. Re-check the
patch site when moving off 1.8.14.

## The patches

### 1. `busOwner` initialised

One line, in `Wire.cpp`:

```diff
-    bool busOwner;
+    bool busOwner = sercom->isBusOwnerWIRE();
     for (byteRead = 1; byteRead < quantity && (busOwner = sercom->isBusOwnerWIRE()); ++byteRead)
```

The loop still refreshes `busOwner` on every iteration, so multi-byte
**semantics** are unchanged.

**Cost, measured rather than asserted.** The patch adds one SERCOM status read
at function entry. `requestFrom(uint8_t, size_t, bool)` grows from **0x90 to
0x96 bytes** (arm-none-eabi-gcc 7.2.1, `-Os`, verified with `objdump -t` on both
builds). It adds **no I2C bus traffic**. The generated code is therefore *not*
identical to upstream — an earlier revision of this file claimed "costs nothing"
and "bit-for-bit identical", and both were wrong.

### 2. Bounded master transfers (`DASHCAM_WIRE_BOUNDED`, 2026-09-27)

The core's `SERCOM::startTransmissionWIRE`, `sendDataMasterWIRE` and
`readDataWIRE` spin on `INTFLAG.MB` / `INTFLAG.SB` with no deadline. A slave
that holds SCL low mid-transfer never lets either flag set, so the MKR sat in
that loop until the 8 s watchdog reset it, and the next boot quarantined the IMU
**and** the GNSS (`lib/I2CBus.h`, `bootAfterHang()`). That happened on the car
on 2026-09-26 at 20:47.

`requestFrom()` and `endTransmission()` now call `startBounded()`,
`sendBounded()` and `readBounded()`. These are the same register sequences,
polled against `DASHCAM_WIRE_WAIT_US` (25 ms per bus event).

- **On expiry:** `abortTransfer()` resets the SERCOM (disable, `initMasterWIRE`
  at the last clock, enable) and bumps `timeoutCount()`. `endTransmission()`
  returns 4 and `requestFrom()` returns 0. `lib/I2CBus.cpp` sees the count
  change, and the next `i2cBusBegin()` runs the full GPIO recovery: clock the
  slave out, then STOP.
- **Lost arbitration** in the write address phase is reported as a failure.
  The core recursed on it without bound.
- **Register pointer:** the register block is found by matching the `SERCOM`
  object against `sercom0`…`sercom5`, because the core keeps its pointer
  private. An unknown SERCOM falls back to the core's own unbounded path.

Only the waits that depend on the bus are bounded. The SYNCBUSY waits (enable,
reset, command synchronisation) depend on the peripheral clock alone.

**Cost:** about 0.7 KB of flash (the production image went from 120,780 to
121,492 bytes when this landed). Normal transfers add no bus traffic.

**Proof:** `helper_scripts/BusFaultInjection`, run on the rig on 2026-09-27,
passed 10 of 10, twice.

| Test | Wedge | Result |
|---|---|---|
| T1b | SDA held low by the BNO055 | Fails with 2 in 7 µs, before reaching any wait |
| T5 | SCL held low before a read's START | The address-phase wait expires at 25.02 ms; one abandon |
| T5b | SCL pulled low part-way through a 64-byte read (TC5 one-shot) | A per-byte wait expires at 25.3 ms; one abandon |
| T6 / T6b | The same two wedges, then left to `i2cBusBegin()` | Recovers by itself; BNO055 CHIP_ID 0xA0 and the GNSS both answer |
| `c` control | T5 injection through the core's own waits | Hung until the watchdog reset the board (`RCAUSE=0x20`), 8.06 s later |

T6b also caught a recovery bug. `i2cBusRecover()` stopped clocking at the
first high SDA. A slave part-way through a read presents its data bits on SDA,
and a 1 among them reads exactly like a release, so the BNO055 was left
mid-byte and pulled SDA low again under the STOP. The recovery now always
clocks at least nine times (`I2C_RECOVER_MIN_CLOCKS`).

**Stretch margin, measured:** the longest bounded wait that completed during the
helper's IMU and GNSS traffic was 760 µs, against the 25 ms deadline. Neither
device's datasheet bounds its clock stretching, which is why the deadline is the
SMBus clock-low timeout (tTIMEOUT, 25–35 ms) rather than a device figure.
Transfers the write path makes (`sendBounded`) share the same wait structure
but were not injected on hardware.

## Test-only instrumentation

Under `#ifdef DASHCAM_WIRE_INSTRUMENT` this fork also exposes three counters
(`dashcamWireQty1Total`, `dashcamWireQty1Filtered`, `dashcamWireQty1AddrFilter`)
that record one-byte transfers. They exist so `WireRegression` T5 can *prove* a
`quantity == 1` request occurred rather than inferring it from a chunk-size
calculation that only makes one likely. The same macro exposes
`dashcamWireCoreWaits`, which routes transfers back through the core's
unbounded waits for `BusFaultInjection`'s control experiment. No production
build script defines the macro, and everything it guards compiles out.

## How the project proves it is being used

`Wire.h` defines:

```cpp
#define DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX 1
```

`lib/I2CBus.h` requires that marker with `#error`. A build that misses the
`--library` path for this folder — an Arduino IDE build, say, or a
`BuildAndUpload.cmd` that lost the flag — fails at compile time instead of
quietly linking the stock library and reintroducing the bug.

There is exactly one sanctioned bypass, `DASHCAM_WIRE_STOCK_CONTROL`, set only
by `helper_scripts/WireRegression/BuildAndUpload.cmd /stock` so the stock-versus-
patched control experiment is reproducible from this tree. That sketch prints
`STOCK CONTROL / marker absent` on every run. No production script defines it.

## What the control experiment currently shows

One point remains empirically unobserved, not technically unresolved: with the
current GCC 7.2.1 / `-Os` build, **the stock-core control also passes**. The
generated code happens to carry a nonzero register value into the two uses of
the indeterminate `busOwner`, so this bench did not reproduce an observable
stock-versus-vendored failure for this exact binary. Source inspection
nevertheless proves undefined behaviour on every acknowledged `quantity == 1`
path. The patch is therefore required to make the behaviour *defined*; what
remains unproven is only that this exact stock binary fails on this bench. Its
latent, toolchain-dependent nature is why the fix is justified from source
evidence rather than waiting for a measured failure.

## Coupling to the core version

This copy calls into the core's `SERCOM` class (`initMasterWIRE`,
`startTransmissionWIRE`, `isBusOwnerWIRE`, …). That is core-private API with no
stability guarantee, so this library is only valid against **`arduino:samd@1.8.14`**.
Every `BuildAndUpload.cmd` checks the installed core version and refuses to build
on a different one rather than producing a binary against an API this copy was
never tested with.

## What this does NOT fix

- **Slave mode:** unchanged; this project never uses it.
- **Anything outside Wire:** a hang elsewhere still ends in the watchdog reset,
  and therefore in the quarantine.
- **The wiring:** the patch contains a wedge, it does not prevent one. Soldered
  I2C wiring removes the contact glitches behind the 2026-09-26 corrupt reads.

`DASHCAM_WIRE_BOUNDED` is not `#error`-enforced, unlike the fix-1 marker.
`lib/I2CBus.cpp` compiles its recovery hook only when the marker is present,
so the `/stock` control build still links.
