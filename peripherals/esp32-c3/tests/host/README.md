# Host tests — ESP32-C3 bridge

| tool | what it checks | needs |
|---|---|---|
| `make check` → `bridge_tests` | session handling in `src/main.cpp` + `lib/commLink`: host reconnects, master reboots | g++ (the Orin Nano has it) |
| `imu_e2e_check.py` | the IMU fields end to end, MKR → C3 → PC | both boards flashed, pyserial |

## bridge_tests

`src/main.cpp` is **included** into the test, not linked, so each case can put
its file-static state back to the initialisers and call `setup()` again. It
still compiles unmodified, from the same file the firmware builds, and its
`loop()` runs exactly as on the chip.

- **MKR side — real bytes.** Frames are built with the production
  `buildFrame()` and fed through `Serial1` into the production decoder, so CRC
  and framing are exercised, not bypassed.
- **Jetson side — scripted.** `hostLink.h` here shadows `lib/hostLink`'s and
  restates its session rules (a HELLO resets streaming, decimation and every
  request not yet taken). The wire protocol to the Jetson is covered by the
  Jetson-side `commlink_test`.

Cases carrying a `REGRESSION` comment are anchored to defects found in review
(2026-09-25). Each was reintroduced into a copy of the source and killed:

| mutation | which cases died |
|---|---|
| a one-shot latched by the old host survives a HELLO | old one-shot vs new session only |
| no master-reboot detection | every reboot case and both "caught" clock cases |
| the old boot's owed frame is cleared without being sent | every case that expects it to arrive |
| the owed frame is cleared on the ATTEMPT, not a confirmed send | full host TX ring only |
| the owed frame keeps its instantaneous readings | "claims nothing live" only |
| the owed frame skipped for one-shot answers | one-shot client only |
| "clock went back" tested unsigned (wrap reads as a reboot) | the millis() wrap case only |
| only the went-back rule, no elapsed-time agreement | young-boot and 24.8-day reboot cases |
| **none — unmutated control** | **none** |

79 checks, currently all passing.
