# canstream — the Orin side of the MKR Zero's raw CAN stream

The production firmware boots into `CanMode::DISCOVER`: listen-only, accept-all, every frame the drain
reads goes to the Orin over the MKR's native USB, and **nothing is decoded on the MKR** (the C3
telemetry's CAN fields read unavailable). Decoding happens here, offline, with the firmware's own
decoder. The line contract is in `lib/CANRawStream.h`.

| file | what it is |
|---|---|
| `mkr_stream_log.py` | Records the USB stream: `can_raw.log` (candump `-l`), `can_stats.csv` (the once-a-second `FS` counters, plus `rx_frames`: frame lines received since the previous FS line, which must equal Δ`streamed` — the exact USB/host-side loss count), `can_sync.csv` (timelines, offset changes, clock steps), `mkr_console.txt` (everything else). Its docstring explains the timing. |
| `can_decode` (`can_decode.cpp`) | Offline decoder: candump log + map file → CSV of every field the MKR publishes from CAN, plus a per-ID census. Compiled from the **unmodified** `lib/CANSniffFunctions.cpp`, `lib/CANMap.cpp`, `lib/VehicleSignals.cpp` against the host stand-ins in `../../tests/host`. |
| `canrawlog2candump.py` | Converts a CANRawLog recording (`helper_scripts/CANRawLog`) to candump, timed by its `S ... @epoch` lines. |
| `can_contract_emit` (`can_contract_emit.cpp`) | Test fixture: real F/FS lines from the firmware's formatter (`format`) or from the whole drain → ring → `canStreamService()` path (`stream`). |
| `tests/` | `make check`: pty-driven logger scenarios, converter, decoder, end-to-end contract. `make perf`: logger CPU. |

## Build and test

On the Jetson, in the dev container (python3 and g++):

```bash
docker run --rm --user 1000:1000 -v /home/jetson/dashcam:/user/dashcam \
    -w /user/dashcam/peripherals/mkr_zero/tools/canstream l4t-ml-gpio:latest make check
```

`make check` takes about two minutes, most of it real-time playback: the logger's timing runs on the
real monotonic clock, so its scenarios cannot be fast-forwarded. No hardware, no serial port: a pty
stands in for `/dev/ttyACM0`.

## Recording

```bash
python3 mkr_stream_log.py SECONDS OUTDIR            # finds the MKR by USB id 2341:804f
python3 mkr_stream_log.py SECONDS OUTDIR --port /dev/ttyACM0
```

Needs tty permission (on the Jetson: a `--privileged -v /dev:/dev` container). It opens the port
exclusively and with HUPCL, so DTR drops when it exits and the MKR counts frames as `nohost` rather than
filling its ring; it survives MKR resets (new timeline per boot) and a full or failing disk (it keeps
reading, writes again when it can, loses whole lines only and notes how many bytes of `can_raw.log`
never reached the disk: `output writes failed` in `mkr_console.txt` and `can_sync.csv`); SIGINT/SIGTERM
flush and exit 0. The
drive-session launcher outside the repo runs it for 12 h per session. Holding the port also means a
flash cannot reset the MKR while it runs (the 1200-baud touch is refused, or as root never reaches the
board, since DTR only drops at the last close): stop it first.

Restarting it against a running MKR is not a reset: the first packet it receives is the one the MKR's USB
bank kept from before the previous reader closed (one or two old frame lines, or an FS line). It is not
taken as a timing sample or as the start of the loss count, so it neither splits the timeline nor counts
the previous reader's unread tail as lost; `can_sync.csv` says `stale first packet` only if it was over
35.8 min old (half the `micros()` period), when its frames keep their arrival time.

Timing, in short: each frame carries the MKR's `micros()` from the moment it was read out of the
MCP2515; the logger maps that clock to host monotonic with the minimum delivery delay over 20 s (late
lines cannot move a minimum), holds frames 2 s so the minimum covers both sides of them, and writes epoch
time with the wall clock as of writing. Measured on the pty: ~0.1 ms from the truth, also for frames
delivered a second late or six seconds stale after a stall. Two things it cannot remove: the floor of
the real USB delivery delay (all timestamps late by it, typically well under a millisecond), and relative
drift between the MKR's crystal and CLOCK_MONOTONIC (frames early by up to drift x 20 s: 1.7 ms measured
at -100 ppm). NTP steers CLOCK_MONOTONIC's rate as well, so a jittery NTP link at home can make that
drift hundreds of ppm. Cost: about 11 % of one Orin core (10.8 % at 1300 lines/s over 10 min, 10-12 % at
2500: it goes with the number of USB reads more than with lines); memory flat at ~14 MB
(3 h of a 1300 frame/s bus replayed in-process: 14 M frames, 2.5 `micros()` wraps, every one in order and
on time).

## Decoding

```bash
./can_decode can_raw.log -o decoded.csv --census census.csv     # 10 Hz grid (--rate HZ)
./can_decode --every-frame can_raw.log -o frames.csv            # one row per frame the map decodes
./can_decode --map builtin ...                                  # the compiled-in map instead of the file
python3 canrawlog2candump.py canrawlog.log -o canrawlog.candump # old CANRawLog recordings first
```

- The map defaults to `../../config/canmap.brio.txt` (relative to the binary), read by the firmware's
  `canMapLoad()`; a map the MKR would refuse is refused, rejected lines are named, and its checksum is
  the `id=0x..` the MKR prints at boot.
- A row is the state at its instant, as a `loop()` pass then would see it: every frame up to that time
  through `canDecodeFrame()` (standard data frames only; remote and extended are counted, as SNIFF does),
  the indicator holds evaluated, `expireVehicleSignals()` with the 200 ms sniff window.
- Empty cell = unavailable: never seen, expired, or not in the map. Never a zero.
- `yaw_rate_cdps_uncal` is uncalibrated: the MKR learns the tyre-radius mismatch from GNSS, which a
  recording does not have.
- `segment` starts again after a gap over 60 s or a timestamp more than 1 s backwards (a clock step):
  fresh signals, holds lapsed. Small backward steps are clamped, as `tickCANSniff()` clamps its own.
- Exit status 3 if any input line was malformed (the output is still written).

A decoder change reaches recordings already on disk by rebuilding `can_decode` and running it again.
