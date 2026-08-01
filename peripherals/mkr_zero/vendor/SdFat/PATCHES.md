# Vendored SdFat — changes against upstream

Upstream: [greiman/SdFat](https://github.com/greiman/SdFat) v2.3.1.

Vendored for the same reason as `vendor/Wire` and `vendor/CANBus`: without an
explicit `--library` flag, arduino-cli resolves `<SdFat.h>` from the user's
global `Documents/Arduino/libraries` folder — an unpinned directory nothing
version-checks, which has already been swapped once mid-project without the
build noticing.

## Source changes

**One line.** `src/SdFat.h` gains `#define DASHCAM_SDFAT_VENDORED 1` beside
`SD_FAT_VERSION`. `lib/SDFunctions.h` `#error`s if it is absent, so losing the
`--library` flag fails the build instead of silently linking the global copy.

No functional patches. Unlike `vendor/CANBus`, this library has no defects this
project needs to work around.

## Files removed

`doc/` (2.9 MB of generated Doxygen HTML) and `images/` (566 KB). Neither is
compiled or referenced; keeping them would have put 3.4 MB of generated output
into a firmware repository. `src/`, `library.properties`, `LICENSE.md` and
`README.md` are unmodified apart from the marker above.

---

## The trap this vendoring exists to document

⚠️ **SdFat does NOT select SPI1 on a SAMD21, despite appearances.**

`src/SdFatConfig.h` contains:

```c
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
...
#ifndef SDCARD_SPI
#define SDCARD_SPI SPI1
...
#endif  // SDCARD_SPI
#endif  // defined(__MK64FX512__) || defined(__MK66FX1M0__)
```

Those guards are **Teensy 3.5 / 3.6**. On the MKR Zero the block never compiles,
so a bare `begin(csPin)` puts the card on the **main SPI bus — the same bus as
the MCP2515 CAN controller at CS 3**.

Grepping for `SDCARD_SPI` finds the define and suggests the opposite. The port
must be passed explicitly:

```cpp
static const SdSpiConfig kSdCfg(SDCARD_SS_PIN, DEDICATED_SPI,
                                SD_SCK_MHZ(12), &SPI1);
gSd.begin(kSdCfg);
```

`SDCARD_SS_PIN` (28) here comes from the **MKR Zero variant.h**, not from SdFat
and not from `DataDictionary.h`'s `SD_CS_PIN` (4) — that constant names an
external module on the CAN-shared bus and must not be used for the onboard slot.

`DEDICATED_SPI` because nothing else on this board uses SPI1.
