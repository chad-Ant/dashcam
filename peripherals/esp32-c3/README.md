# ESP_Sentinel — telemetry bridge (ESP32-C3 XIAO)

Two framed serial links, one on each side of the C3:

```
 MKR Zero  ──UART1 115200──>  ESP32-C3 XIAO  ──USB-C native──>  Jetson Orin Nano
 OBD2+GPS   CommProtocol.h     (this sketch)   HostProtocol.h     libcommlink
            C3 = initiator                     C3 = responder
```

Both hops use the same wire format, so they fail the same way and can be
debugged with the same reasoning:

```
SOF(0x7E) | VER(0x07) | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16_LE(2)
CRC-16/CCITT-FALSE over VER..last payload byte, low byte first.
```

## Jetson hop message set (`HostProtocol.h`)

| TYPE | Name | Dir | Payload | Meaning |
|------|------|-----|---------|---------|
| 0x01 | `CMD_GET_ONCE` | J→C3 | — | Fetch one fresh snapshot from the MKR and relay it |
| 0x02 | `CMD_GET_STATUS` | J→C3 | — | Send one `MSG_STATUS` now |
| 0x03 | `CMD_HELLO` | J→C3 | — | New session: reset session state, reply `MSG_HELLO`. **Sent on every port open** |
| 0x10 | `CMD_START_STREAM` | J→C3 | — | Forward master telemetry as it arrives |
| 0x11 | `CMD_STOP_STREAM` | J→C3 | — | Stop forwarding (status keeps flowing) |
| 0x12 | `CMD_SET_DECIM` | J→C3 | `uint8 N` | Forward every Nth master frame (N ≥ 1) |
| 0x20 | `CMD_PING` | J→C3 | — | Link check **and** host-liveness keepalive |
| 0x81 | `MSG_TELEMETRY` | C3→J | 180 B `Telemetry` | OBD2 + GPS + IMU + switch snapshot, relayed verbatim |
| 0x82 | `MSG_STATUS` | C3→J | 44 B `BridgeStatus` | Bridge health, 1 Hz |
| 0x83 | `MSG_LOG` | C3→J | `uint8 level` + ASCII | Bridge log line → Jetson liblog |
| 0xA0 | `MSG_PONG` | C3→J | — | Ping ack |
| 0xA1 | `MSG_HELLO` | C3→J | 12 B `Hello` | Identity + reset reason, on every reconnect |
| 0xEE | `MSG_NACK` | C3→J | 2 B `Nack` | Command rejected (type + reason) |

### Session contract

1. Jetson opens the port and sends `CMD_HELLO`, then `CMD_PING` every second as
   a keepalive.
2. The C3 **resets session state** (streaming off, decimation 1) and sends
   `MSG_HELLO`.
3. The Jetson answers `MSG_HELLO` with `CMD_SET_DECIM` + `CMD_START_STREAM`
   (`CommLinkConfig::autoStream`, on by default).
4. Telemetry flows event-driven — one frame per master frame, never a repeat of
   a stale sample. `MSG_STATUS` continues at 1 Hz regardless.
5. If the Jetson goes quiet for 5 s the C3 drops the session and stops
   streaming, so it never pushes into a ring nobody drains.

This is why a C3 reset is invisible to the application: the reboot produces a
new `MSG_HELLO`, which re-establishes streaming with no application code
involved.

`CMD_HELLO` is what makes the *reverse* case safe. The 5 s timeout alone cannot
detect a host that reopens the port faster than the timeout — there is no
silence to observe — so without an explicit announcement a restarted Jetson
would inherit the dead process's streaming state and, if that process had
stopped the stream, wait forever for telemetry that is never sent.

### Telling the three failure modes apart

| Symptom | Meaning |
|---------|---------|
| No frames at all | USB link down — cable, enumeration, or C3 not running |
| `MSG_STATUS` only, `BRIDGE_FLAG_MASTER_LINK` clear, `telemetryAgeMs` climbing | Bridge alive, MKR Zero down |
| `MSG_HELLO` repeating with small `bridgeMillis`, `bootCount` climbing | C3 in a reset loop — check `resetReason` |
| Rising `masterCrcErrors` | Marginal ground or EMI on the MKR UART harness |
| Rising `hostTxDropped` | Jetson not draining the port fast enough |

## Board settings (required)

- **USB CDC On Boot = Enabled.** `Serial` must be the native USB port; the
  sketch fails to compile otherwise. This also frees GPIO20/21 (UART0's default
  pins) for `Serial1`, which is what the MKR hop uses.
- PlatformIO equivalent: `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1`
- Compiler warnings = All (`-Wall -Wextra`).

> `Serial` carries binary frames. **Never `Serial.print()` from this sketch** —
> use `bridgeLog()`, which wraps the text in a `MSG_LOG` frame that lands in the
> Jetson's log file tagged `[c3]`.

## Wiring

| MKR Zero | ESP32-C3 |
|----------|----------|
| Serial1 TX (pin 14) | RX — GPIO20 |
| Serial1 RX (pin 13) | TX — GPIO21 |
| GND | GND |

C3 USB-C → any USB port on the Orin Nano. Both boards are 3.3 V logic; no level
shifter needed. A common ground is mandatory — a floating ground is the usual
cause of a link that shows CRC errors instead of frames.

## Building

```bash
peripherals/esp32-c3/BuildAndUpload.cmd          # compile
peripherals/esp32-c3/BuildAndUpload.cmd COM7     # compile + flash
```

The script drives `arduino-cli` and sets the board flags this firmware depends
on — `CDCOnBoot=cdc` above all, without which `hostLink.h` stops the build with
an `#error`. It passes `lib/commLink` and `lib/hostLink` with `--library`
because Arduino compiles the sketch folder and `src/` but never an arbitrary
`lib/`; `esp32-c3.ino` is an empty marker that exists only because arduino-cli
requires a `.ino` named after the directory.

`platformio.ini` builds the same sources with `pio run` if PlatformIO is
installed, but the arduino-cli path above is the one verified against this
tree.

Last verified build: 318 KB flash (10%), 16 KB RAM (4%), clean at
`--warnings all`.

The MKR Zero half builds the same way:

```bash
peripherals/mkr_zero/BuildAndUpload.cmd
```

## Bring-up

1. Flash this sketch to the C3 with the board settings above.
2. Install the udev rule so the port has a stable name and ModemManager stops
   probing it (this is the most common cause of a link that works on a desktop
   but not on the Jetson):
   ```bash
   sudo cp lib/libcommlink/99-dashcam-bridge.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. Build and run the Jetson-side test:
   ```bash
   make commlink_test && ./bin/build_*/commlink_test
   ```
   It auto-discovers the port, streams, and checks each condition separately:
   `MSG_HELLO` received, 1 Hz status, telemetry, still connected at exit,
   telemetry fresh at exit, and no protocol mismatch. Unplug and replug the
   cable while it runs to also cover reconnect recovery.

## Keeping the contract

`HostProtocol.h` exists in two places and **must stay byte-identical**:

- `peripherals/esp32-c3/lib/hostLink/HostProtocol.h`
- `lib/libcommlink/HostProtocol.h`

```bash
diff peripherals/esp32-c3/lib/hostLink/HostProtocol.h lib/libcommlink/HostProtocol.h
```

`MSG_HELLO` carries `sizeof(Telemetry)` and `sizeof(BridgeStatus)` as the bridge
sees them, so a mismatched pair is reported as a `PROTOCOL MISMATCH` error on
the Jetson rather than silently misreading every field.
