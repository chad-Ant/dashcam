# Storage — onboard microSD card & flash persistence (no EEPROM)

The MKR Zero's headline feature is the **onboard microSD slot**, wired to its own
dedicated SPI bus (SERCOM4) so it doesn't tie up the header SPI. Separately, the
SAMD21 has **no EEPROM**, so small persistent settings need a flash-based
approach. Both are below, with the error handling a field device needs.

## Onboard microSD card (`SD` library)

The SD card is on a **dedicated SPI interface** — independent of the D8/D9/D10
header SPI, so you can use an SD card *and* another SPI device at the same time.
The `SD` library auto-recognizes the MKR Zero's chip-select.

```cpp
#include <SD.h>

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }      // bounded — never hang on battery

  // SD.begin() with no arg auto-detects on the MKR Zero; SDCARD_SS_PIN is explicit.
  if (!SD.begin(SDCARD_SS_PIN)) {                   // CHECK the return — card may be absent
    Serial.println("SD init failed (card missing / unformatted?)");
    // defined fallback: blink an error, retry next cycle, or run without logging —
    // do NOT loop forever on SD.begin().
    return;
  }
  Serial.println("SD ready");
}
```

Reading/writing files, with every fallible step checked:

```cpp
File f = SD.open("log.csv", FILE_WRITE);            // append mode
if (f) {                                            // open can fail (full / locked)
  f.print(millis()); f.print(','); f.println(reading);
  f.close();                                        // close to actually flush to card
} else {
  Serial.println("could not open log.csv");
}

// Reading back:
File r = SD.open("log.csv");
if (r) {
  while (r.available()) { Serial.write(r.read()); }
  r.close();
}
```

Practical rules that keep SD logging alive in the field:

- **Always check `SD.begin()` and every `SD.open()`** — a missing/full/corrupt
  card is a normal runtime condition, not an impossible one. Never
  `while(!SD.begin())`.
- **Use short 8.3 filenames** (`DATA001.CSV`) for maximum compatibility;
  long-filename support is limited.
- **`close()` after writes** — data isn't guaranteed on the card until the file
  is closed (or flushed). If power can drop mid-write, keep writes small and
  close often; the last unclosed record can be lost.
- **SD writes draw current spikes.** On a weak LiPo this can brown the board out
  mid-write — budget power, and consider closing/flushing before a sleep.
- **Don't churn the heap per record.** Build each line in a fixed `char` buffer
  with `snprintf`, not growing `String`s — 32 KB SRAM fragments quickly under a
  long-running logger (`mission-critical.md`, Rule 3).
- For audio/larger throughput, the board's I2S + SD combination is the intended
  use case; the same open/close/error-check discipline applies.

## Persistent settings without EEPROM (FlashStorage)

There is **no EEPROM** on the SAMD21. `EEPROM.h` on this core is *emulated in a
flash page*, and — critically — **flash is erased on every sketch upload**, so
anything you "save" is wiped when you next program the board. For small
persistent values (calibration, counters, config), use the **FlashStorage**
library and understand the same caveat.

```cpp
#include <FlashStorage.h>

struct Settings { uint32_t magic; int32_t calibration; uint16_t bootCount; };
FlashStorage(store, Settings);                      // reserves a flash slot

Settings s;
void setup() {
  s = store.read();
  if (s.magic != 0xC0FFEE) {                        // first run / after re-flash: init
    s = { 0xC0FFEE, 0, 0 };
  }
  s.bootCount++;
  store.write(s);                                   // persists across power cycles
}
```

Key points and limits:

- **A `magic`/sentinel field is essential** — after a re-upload the slot reads as
  erased (often 0xFF bytes), so detect "uninitialized" and write defaults rather
  than trusting garbage.
- **Flash endurance is finite — the datasheet rates it at a minimum of 25,000
  erase/write cycles** (≈150k typical; Table 37-43). The plain FlashStorage
  approach rewrites a whole page per `write()`, so those cycles are spent quickly
  if you write often: **never write in `loop()`** at speed — you'll wear out the
  flash. Write only on change, or buffer and write periodically/on shutdown. This
  is a direct case of Rule 3 (no churn after init): treat flash writes as rare
  events. (Arduino's full *EEPROM-emulation* library wear-levels across a region
  for ≈100k-cycle endurance, but it's lost on re-upload just the same.)
- **Only 8 consecutive writes are allowed per flash row before a row erase is
  required** (datasheet NVM note) — another reason to batch, not dribble, writes.
- A flash write briefly stalls the CPU (page program ≈2.5 ms, row erase ≈6 ms;
  Table 37-45) and can disturb timing — don't do it inside time-critical sections
  or ISRs.
- Survives power loss; **does not** survive a new sketch upload (it's the same
  flash being reprogrammed). For data that must outlive reprogramming, log to the
  SD card instead.

## Choosing between SD and flash

- **Bulk/append data, logs, audio, anything large or that must survive
  reprogramming** → SD card.
- **A handful of small settings/counters that change rarely** → FlashStorage.
- Need both robustness and reprogramming-survival for settings? Store the
  authoritative copy as a small file on the SD card and cache it in RAM.
