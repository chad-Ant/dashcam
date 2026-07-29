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

## The patch

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

## Test-only instrumentation

Under `#ifdef DASHCAM_WIRE_INSTRUMENT` this fork also exposes three counters
(`dashcamWireQty1Total`, `dashcamWireQty1Filtered`, `dashcamWireQty1AddrFilter`)
that record one-byte transfers. They exist so `WireRegression` T5 can *prove* a
`quantity == 1` request occurred rather than inferring it from a chunk-size
calculation that only makes one likely. No production build script defines the
macro, and everything it guards compiles out.

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

Only the `busOwner` undefined behaviour. The SAMD I2C driver still waits on its
bus flags in unbounded loops (`SERCOM::startTransmissionWIRE`,
`SERCOM::readDataWIRE`), so a device that wedges mid-transaction can still hang
the CPU. `i2cBusBegin()` clears the common power-on case, but a watchdog remains
the only containment for a wedge that happens at runtime.
