# Vendored arduino-CAN — patches against upstream

Upstream: [timurrrr/arduino-CAN](https://github.com/timurrrr/arduino-CAN), itself a
fork of sandeepmistry/arduino-CAN. Library version 0.3.1.

Vendored because this firmware transmits on a live vehicle bus and the upstream
copy has defects that are invisible at build time and only appear on a bus that
is not idle. It was previously resolved from the user's global Arduino libraries
folder — an unpinned directory nothing version-checks, which had already been
swapped once mid-project without the build noticing.

Every change is marked `DASHCAM PATCH` in the source so the diff against
upstream stays greppable and the fork stays rebaseable.

`MCP2515.h` defines `DASHCAM_CANBUS_VENDORED_FIXES`. `lib/OBD2Functions.h`
`#error`s if it is absent, so losing the `--library` flag fails the build
instead of silently reintroducing all of the below.

---

## 1. `endPacket()` had no deadline — **hang / reboot loop**

Upstream spins on `TXREQ` with no bound, and arms its abort only once `TXERR`
appears. On a stuck-dominant bus neither happens: the controller never starts
transmitting because the bus never goes idle, so `TXREQ` stays set and `TXERR`
stays clear, forever.

One-Shot Mode does **not** bound this. OSM limits *re*-transmission after an
attempt; it says nothing about waiting for an idle bus. The project's own
`OBD2Probe` helper hung at exactly this line, and `sendS1Command()` carried a
comment asserting OSM made it safe — it did not. In production the watchdog
would have reset the board straight back into the same call: a reboot loop, not
a recovery.

Now: a hard iteration deadline, then an unconditional `ABAT`, then a bounded
wait for the abort to land, then a failure return the caller can act on.

## 2. `parsePacket()` overflowed its RX buffer on DLC > 8 — **memory corruption**

`_rxDlc = regDLC & 0x0f` accepts 0–15, and upstream then copied that many bytes
into `_rxData[8]`.

ISO 11898-1 says any DLC above 8 still means eight data bytes, so a *conforming*
transmitter may legally send DLC 9–15, and some ECUs do. `_rxData` is the last
member of `CANControllerClass`, so `MCP2515Class`'s own `_spiSettings`, `_csPin`
and `_intPin` sit immediately after it — one malformed frame could silently
repoint the driver's chip-select. Clamped to 8; the MCP2515 only stores eight
data bytes anyway, so nothing is lost.

## 3. `setFilterRegisters()` truncated every filter to 8 bits

`uint16_t` parameters were copied into `uint8_t` locals. The compiler emitted six
narrowing warnings at `--warnings all`.

The failure was quiet and asymmetric rather than obvious: mask `0x7FF` became an
effective `0x0FF` and filter `0x158` became `0x058`, so `0x158` was **still
accepted** — along with `0x058`, `0x258`, `0x358` … `0x758`. The filter silently
became eight times more permissive than requested, which on a busy bus is
precisely the RX overrun it was installed to prevent.

## 4. Mode switches were never verified — and destroyed One-Shot Mode

Upstream requested a mode with a full `writeRegister(REG_CANCTRL, …)` and then
checked the `CANCTRL` **readback**. Two bugs in one:

- `CANCTRL` only echoes the *request*. The controller finishes any frame in
  progress before the mode actually changes, so on an active bus the readback
  reports success while the chip is still in the old mode — and register writes
  that are only legal in Configuration mode get silently ignored.
- A full `CANCTRL` write clears `ABAT`, `OSM`, `CLKEN` and `CLKPRE` along with
  the mode. Clearing `OSM` matters most: a mode switch silently disarmed the
  only protection the caller had just enabled.

Added `switchToMode()`: `modifyRegister` on the REQOP bits only, then a bounded
poll of `CANSTAT` OPMOD. Every mode path routes through it.

## 5. `observe()` requested Configuration mode, not Listen-Only

Wrote `0x80` (Configuration) where Listen-Only is `0x60`, with upstream's own
`// TODO: These should probably be 0x60, not 0x80.` beside it.

In Configuration mode the controller is **off the bus and receives nothing**,
which is indistinguishable from "the gateway bridges no broadcast traffic". This
cost the project two sessions chasing a hardware fault that did not exist.

## 6. `sleep()` did not sleep

Wrote `0x01` to `CANCTRL`. Bits 1:0 are `CLKPRE`, the CLKOUT prescaler — not
`REQOP`. Upstream left the controller in **Normal mode** with a divided clock
output, then reported success because the readback matched what it wrote. A node
that believes it is asleep while still ACKing every frame on the bus is a
considerably worse outcome than a failed sleep. Sleep is REQOP 001 = `0x20`.

## 7. `filter()` / `filterExtended()` wrote reserved RXM modes and no rollover

Upstream wrote `FLAG_RXM0` (or `FLAG_RXM1`) to `RXBnCTRL` — twice to the same
register, with its own TODO doubting it. RXM `01`/`10` are documented as
reserved on current silicon; `00` is what makes the filters being written three
lines later actually apply. Standard-vs-extended matching is decided by the
`EXIDE` bit in each filter, which is already set correctly.

Also added `BUKT` on RXB0, doubling usable receive depth from one frame to two.

## 8. Extended filters dropped identifier bit 20

`(((mask >> 18) & 0x03) << 5)` — `SIDL` bits 7:5 carry EID bits 20:18, three
bits. Masking with `0x03` kept only 19:18. Any 29-bit filter or mask differing
solely in bit 20 was programmed wrong. Fixed to `0x07` in all four places.

## 9. `setFilterRegisters()` forced Normal mode on exit

It ended with an unconditional `switchToNormalMode()`, which cancels Listen-Only
— so "install filters, then sniff read-only" could not be expressed, and any
listen-only path that configured filters became bus-active without saying so.

Now takes `uint8_t targetMode = 0x00`, defaulting to Normal so existing callers
are unaffected. Pass `0x60` for Listen-Only.

---

## Known-remaining upstream defects — NOT patched

- **`onReceive()` runs `parsePacket()` inside the ISR** — ~85 µs of SPI at up to
  4 kHz, LOW-level-triggered, and its trampoline hardcodes the global `CAN`
  object. Not fixed because nothing in this project uses it: the census and the
  planned sniffer both poll `CANINTF` and drain with raw `READ RX BUFFER`, which
  is cheaper than the interrupt path anyway.
- **`parsePacket()` returns the DLC**, so a valid DLC-0 frame is
  indistinguishable from "no frame". Callers must use `packetId() != -1`.
  Upstream API contract; left alone.
- **`filter()` still ends in Normal mode.** Upstream's contract, kept so existing
  callers work. It must never appear in a read-only path — use
  `setFilterRegisters()` with an explicit `targetMode`.
