#!/usr/bin/env python3
"""End-to-end check of the IMU path: MKR Zero -> ESP32-C3 -> this PC over USB.

Decodes the bridge's host protocol from the C3's USB-CDC port and validates the
IMU fields against what the sensor's operating mode says they should be. Nothing
is stubbed: this reads the production firmware on both boards, over the real
link, and the only thing it adds is a decoder that disagrees loudly.

WHAT IT NEEDS

    pip install pyserial

    Both boards flashed with production firmware and cross-wired as usual, the
    C3 plugged into this PC. Nothing else on the C3's port.

USAGE

    python imu_e2e_check.py --port COM7                    check whatever it booted into
    python imu_e2e_check.py --port COM7 --mode amg --set   command AMG, then check it
    python imu_e2e_check.py --port COM7 --both --set       check BOTH modes in one run
    python imu_e2e_check.py --list                         show candidate ports

WHY THE MODE MATTERS

The two modes publish DIFFERENT FIELDS, and each one's silence is the other's
signal. In IMUPLUS the magnetometer is off, so magX/Y/Z are NAN by design and
linear acceleration is present; in AMG there is no fusion, so linear
acceleration and relative yaw are NAN and the magnetometer is live. A checker
that did not know which mode it was looking at would have to accept both, which
means accepting a firmware that published neither.

The mode is not inferred from which fields happen to be NAN — a stale channel
and a mode that does not produce that channel look identical. It is read from
TLM_FLAG_IMU_FUSION_MODE, and with --mode the reported mode is CHECKED against
the expected one rather than merely followed.

--set sends CMD_SET_IMU_MODE and waits out the master's ~700 ms bring-up. Without
it the script only observes, which is the right thing when the master was flashed
for a particular mode with -DDASHCAM_IMU_MODE_AMG.
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is not installed.  Run:  pip install pyserial")

# ─── protocol ────────────────────────────────────────────────────────────────
# Mirrors lib/libcommlink/HostProtocol.h. The frame is
#   SOF(0x7E) | VER | TYPE | LEN | PAYLOAD(LEN) | CRC16_LE(2)
# with the CRC covering VER..PAYLOAD and NOT the SOF.

SOF = 0x7E
VERSION = 0x07

MSG_TELEMETRY = 0x81
MSG_STATUS = 0x82
MSG_LOG = 0x83
MSG_HELLO = 0xA1

CMD_START_STREAM = 0x10
CMD_HELLO = 0x03
CMD_SET_IMU_MODE = 0x15

IMU_MODE_FUSION = 1
IMU_MODE_RAW = 2

TLM_FLAG_OBD2_VALID = 0x0001
TLM_FLAG_GPS_FIX = 0x0002
TLM_FLAG_TIME_VALID = 0x0004
TLM_FLAG_IMU_PRESENT = 0x0008
TLM_FLAG_GPS_PRESENT = 0x0010
TLM_FLAG_IMU_DEGRADED = 0x0020
TLM_FLAG_IMU_DATA_GAP = 0x0040
TLM_FLAG_IMU_LOWPOWER = 0x0080
TLM_FLAG_IMU_HIGH_G = 0x0100
TLM_FLAG_IMU_SATURATED = 0x0200
TLM_FLAG_CANMAP_LOADED = 0x0400
TLM_FLAG_IMU_HIGHG_ARMED = 0x0800
TLM_FLAG_IMU_FUSION_MODE = 0x1000
TLM_FLAG_SWITCHES_PRESENT = 0x2000

# hostproto::Telemetry, packed, 180 bytes. '<' is little-endian AND no padding,
# which is what makes this match a __attribute__((packed)) struct exactly.
TELEMETRY_FMT = (
    "<"
    "I"       # masterMillis
    "17f"     # speed accel rpm coolantTemp fuelLevel fuelRate throttle
              # engineLoad airPressure gear gearRatio odo
              # latitude longitude altitude gpsSpeedKmh heading
    "3B"      # satellites fixType fixValid
    "H"       # year
    "5B"      # month day hour minute second
    "13f"     # imuAccelX/Y/Z imuGyroX/Y/Z imuMagX/Y/Z imuTempC
              # imuAccelPeak imuGyroPeak imuLinAccelPeak
    "3f"      # imuLinAccelX/Y/Z                              (v0x07)
    "2H"      # imuHighGMs imuHighGCount                      (v0x07)
    "f"       # imuYawRelDeg
    "3B"      # imuCalib canMapChecksum canMapFlags
    "H"       # flags
    "5B"      # canMode sigSource gearPos vehFlags pedalGas
    "H"       # steerMotorTorque
    "h"       # yawRateCdps
    "4H"      # wheelRaw
    "2H"      # switchState switchChanged                     (v0x07)
)

TELEMETRY_FIELDS = (
    "masterMillis speed accel rpm coolantTemp fuelLevel fuelRate throttle "
    "engineLoad airPressure gear gearRatio odo latitude longitude altitude "
    "gpsSpeedKmh heading satellites fixType fixValid year month day hour "
    "minute second imuAccelX imuAccelY imuAccelZ imuGyroX imuGyroY imuGyroZ "
    "imuMagX imuMagY imuMagZ imuTempC imuAccelPeak imuGyroPeak "
    "imuLinAccelPeak imuLinAccelX imuLinAccelY imuLinAccelZ "
    "imuHighGMs imuHighGCount imuYawRelDeg "
    "imuCalib canMapChecksum canMapFlags flags "
    "canMode sigSource gearPos vehFlags pedalGas steerMotorTorque "
    "yawRateCdps wheel0 wheel1 wheel2 wheel3 switchState switchChanged"
).split()

TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FMT)
assert TELEMETRY_SIZE == 180, f"layout drift: {TELEMETRY_SIZE} bytes, expected 180"
assert len(TELEMETRY_FIELDS) == len(struct.unpack(TELEMETRY_FMT, b"\0" * TELEMETRY_SIZE))

HELLO_FMT = "<BBBBBBHI"   # protoVersion fwMajor fwMinor resetReason
                          # telemetryBytes statusBytes bootCount bridgeMillis

BRIDGE_FLAG_MASTER_LINK = 0x01
BRIDGE_FLAG_STREAMING = 0x02
BRIDGE_FLAG_PM_PRESENT = 0x20

# hostproto::BridgeStatus, packed, 44 bytes.
#
# DECODED, not merely counted. An earlier version tallied MSG_STATUS frames and
# threw the contents away — so a run where telemetry stopped while status kept
# arriving reported only "no telemetry", when the bridge had been sending the
# answer once a second the whole time. Whether the master link is up, how stale
# its last frame is, and whether forwarding is even enabled are all in here.
STATUS_FMT = "<6I3fHBBBB2s"
STATUS_FIELDS = (
    "bridgeMillis telemetryAgeMs masterFrames masterCrcErrors hostFrames "
    "hostTxDropped batteryVolts batteryPercent tempC freeHeapKb batteryStatus "
    "chargeState flags decimation reserved"
).split()
STATUS_SIZE = struct.calcsize(STATUS_FMT)
assert STATUS_SIZE == 44, f"BridgeStatus layout drift: {STATUS_SIZE} bytes, expected 44"


def unpack_status(payload: bytes) -> dict:
    return dict(zip(STATUS_FIELDS, struct.unpack(STATUS_FMT, payload)))


def describe_bridge(s: dict) -> str:
    on = []
    if s["flags"] & BRIDGE_FLAG_MASTER_LINK:
        on.append("MASTER_LINK")
    if s["flags"] & BRIDGE_FLAG_STREAMING:
        on.append("STREAMING")
    return "|".join(on) if on else "none"


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, the same routine both firmware hops use."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, payload: bytes = b"") -> bytes:
    head = bytes((VERSION, msg_type, len(payload)))
    return bytes((SOF,)) + head + payload + struct.pack("<H", crc16(head + payload))


class Decoder:
    """Incremental frame decoder; resynchronises on SOF after any bad frame."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.version_errors = 0

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)
        while True:
            frame = self._take()
            if frame is None:
                return
            yield frame

    def _take(self):
        # LOOPS on a rejected frame rather than returning. None from here means
        # "nothing more can be decoded until more bytes arrive", and it has to
        # mean ONLY that: an earlier version also returned None after discarding
        # a bad frame, which ended the caller's drain loop and left every
        # complete frame behind it unread until the next read() happened to
        # re-enter. On a clean link that is invisible; on the noisy one this
        # check exists to measure, it silently under-reports the stream.
        while True:
            # Drop anything before a start-of-frame.
            sof = self.buf.find(SOF)
            if sof < 0:
                self.buf.clear()
                return None
            if sof > 0:
                del self.buf[:sof]
            if len(self.buf) < 4:
                return None

            ver, mtype, length = self.buf[1], self.buf[2], self.buf[3]
            total = 4 + length + 2
            if len(self.buf) < total:
                return None

            payload = bytes(self.buf[4:4 + length])
            got = struct.unpack_from("<H", self.buf, 4 + length)[0]
            want = crc16(bytes(self.buf[1:4]) + payload)

            # Consume ONE byte on a bad frame, not the whole thing: a corrupt
            # length byte can make a valid frame start look longer than it is,
            # and skipping the claimed length would step over the real SOF that
            # follows.
            if got != want:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            if ver != VERSION:
                self.version_errors += 1
                del self.buf[:1]
                continue

            del self.buf[:total]
            return mtype, payload


def unpack_telemetry(payload: bytes) -> dict:
    return dict(zip(TELEMETRY_FIELDS, struct.unpack(TELEMETRY_FMT, payload)))


# ─── the checks ──────────────────────────────────────────────────────────────

class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.notes = []

    def check(self, ok: bool, name: str, detail: str = ""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   [{detail}]" if detail else ""))
        if ok:
            self.passed += 1
        else:
            self.failed += 1


def finite(x) -> bool:
    return isinstance(x, float) and math.isfinite(x)


def check_mode_contract(rep: Report, t: dict, mode: str):
    """The fields each mode must publish, and the ones it must NOT invent.

    Both halves matter. NAN is this protocol's "not supplied", and a mode
    publishing a plausible number where it has no measurement is the failure the
    whole driver was rebuilt around — a fabricated reading is worse than a
    missing one.
    """
    axes_live = all(finite(t[k]) for k in ("imuAccelX", "imuAccelY", "imuAccelZ"))
    gyro_live = all(finite(t[k]) for k in ("imuGyroX", "imuGyroY", "imuGyroZ"))
    mag_live = all(finite(t[k]) for k in ("imuMagX", "imuMagY", "imuMagZ"))
    mag_absent = all(not finite(t[k]) for k in ("imuMagX", "imuMagY", "imuMagZ"))

    rep.check(axes_live, "accelerometer axes are published",
              f"{t['imuAccelX']:.2f},{t['imuAccelY']:.2f},{t['imuAccelZ']:.2f}")
    rep.check(gyro_live, "gyroscope axes are published",
              f"{t['imuGyroX']:.2f},{t['imuGyroY']:.2f},{t['imuGyroZ']:.2f}")
    rep.check(finite(t["imuTempC"]), "die temperature is published",
              f"{t['imuTempC']:.0f} C")
    rep.check(finite(t["imuAccelPeak"]), "windowed accel peak is published",
              f"{t['imuAccelPeak']:.2f}")
    rep.check(finite(t["imuGyroPeak"]), "windowed gyro peak is published",
              f"{t['imuGyroPeak']:.2f}")

    if mode == "imuplus":
        rep.check(finite(t["imuLinAccelPeak"]),
                  "IMUPLUS: linear-acceleration peak is published",
                  f"{t['imuLinAccelPeak']:.2f}")
        # RANGE, not just finiteness. The origin is arbitrary and drifts — 270
        # at rest is perfectly valid and must not be treated as a fault — but the
        # encoding is 0..360, so a value outside that is a scaling or sign error
        # rather than a heading this driver declines to interpret.
        yaw = t["imuYawRelDeg"]
        rep.check(finite(yaw) and 0.0 <= yaw < 360.0,
                  "IMUPLUS: relative yaw is published and inside [0,360)",
                  f"{yaw:.1f} deg")
        # The magnetometer is switched off on purpose. Zeros here would be a
        # fabricated field strength, and zero is a plausible reading.
        rep.check(mag_absent,
                  "IMUPLUS: magnetometer is NAN, not a fabricated zero")
        # Gravity is included in the raw peak, so a resting board reads ~9.81.
        rep.check(5.0 < t["imuAccelPeak"] < 45.0,
                  "IMUPLUS: accel peak is in the +/-4 g rail's range",
                  f"{t['imuAccelPeak']:.2f} m/s2")
    else:
        rep.check(mag_live, "AMG: magnetometer is published",
                  f"{t['imuMagX']:.1f},{t['imuMagY']:.1f},{t['imuMagZ']:.1f}")
        # No fusion in AMG, so these have nothing behind them.
        rep.check(not finite(t["imuLinAccelPeak"]),
                  "AMG: linear-acceleration peak is NAN, not invented")
        rep.check(not finite(t["imuYawRelDeg"]),
                  "AMG: relative yaw is NAN, not invented")
        rep.check(5.0 < t["imuAccelPeak"] < 160.0,
                  "AMG: accel peak is in the +/-16 g rail's range",
                  f"{t['imuAccelPeak']:.2f} m/s2")


def describe_flags(flags: int) -> str:
    names = [
        (TLM_FLAG_IMU_PRESENT, "IMU_PRESENT"),
        (TLM_FLAG_IMU_DEGRADED, "IMU_DEGRADED"),
        (TLM_FLAG_IMU_DATA_GAP, "DATA_GAP"),
        (TLM_FLAG_IMU_LOWPOWER, "LOWPOWER"),
        (TLM_FLAG_IMU_HIGH_G, "HIGH_G"),
        (TLM_FLAG_IMU_SATURATED, "SATURATED"),
        (TLM_FLAG_IMU_HIGHG_ARMED, "HIGHG_ARMED"),
        (TLM_FLAG_IMU_FUSION_MODE, "FUSION_MODE"),
        (TLM_FLAG_GPS_FIX, "GPS_FIX"),
        (TLM_FLAG_GPS_PRESENT, "GPS_PRESENT"),
        (TLM_FLAG_OBD2_VALID, "OBD2_VALID"),
        (TLM_FLAG_CANMAP_LOADED, "CANMAP"),
    ]
    on = [n for bit, n in names if flags & bit]
    return "|".join(on) if on else "none"


# ─── one measurement pass ────────────────────────────────────────────────────

MODE_NAMES = {"imuplus": IMU_MODE_FUSION, "amg": IMU_MODE_RAW}

# The master restarts its whole bring-up on a mode change and publishes nothing
# for ~700 ms, then needs another peak window before the peaks mean anything.
# Waiting less than this measures the blind period and calls it a failure.
MODE_SETTLE_S = 2.5

# MUST match IMU_MODE_MIN_INTERVAL_MS in the master's DataDictionary.h.
#
# The master refuses a mode change that arrives sooner than this after the last
# one, because each costs the sensor a ~700 ms blind window and a host looping on
# the command would hold it in permanent re-initialisation. That guard is correct
# and this script has to live with it.
#
# It did not. With --seconds 1.5 the cleanup landed 2.5 + 1.5 = 4.0 s after the
# AMG change and was REFUSED, and nothing checked — so a run reporting OK left
# the rig in AMG, publishing no linear acceleration at all, for the next person
# to find. The two numbers below are why the restore now waits and then verifies.
MODE_GUARD_S = 5.0

# When the script last had a mode change applied, as time.monotonic().
_last_mode_change = 0.0


def wait_out_mode_guard():
    """Sleeps until the master will accept another mode change."""
    remaining = MODE_GUARD_S - (time.monotonic() - _last_mode_change)
    if remaining > 0:
        print(f"  waiting {remaining:.1f} s for the master's mode-change guard")
        time.sleep(remaining)


def collect(ser, dec, seconds, raw=False):
    """Drains frames for @p seconds and returns what arrived."""
    acc = {
        "telemetry": 0, "status": 0, "hello": 0, "log": 0,
        "gap": 0, "highg": 0, "armed": 0, "present": 0, "fusion": 0,
        "newest": None, "hello_data": None, "status_data": None,
        "logs": [], "master_millis": [],
    }
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = ser.read(4096)
        if not chunk:
            continue
        for mtype, payload in dec.feed(chunk):
            if mtype == MSG_TELEMETRY and len(payload) == TELEMETRY_SIZE:
                t = unpack_telemetry(payload)
                acc["telemetry"] += 1
                acc["newest"] = t
                acc["master_millis"].append(t["masterMillis"])
                f = t["flags"]
                acc["gap"] += bool(f & TLM_FLAG_IMU_DATA_GAP)
                acc["highg"] += bool(f & TLM_FLAG_IMU_HIGH_G)
                acc["armed"] += bool(f & TLM_FLAG_IMU_HIGHG_ARMED)
                acc["present"] += bool(f & TLM_FLAG_IMU_PRESENT)
                acc["fusion"] += bool(f & TLM_FLAG_IMU_FUSION_MODE)
                if raw:
                    print(f"    t={t['masterMillis']:>8}  "
                          f"a=({t['imuAccelX']:+7.2f},{t['imuAccelY']:+7.2f},"
                          f"{t['imuAccelZ']:+7.2f})  peak={t['imuAccelPeak']:6.2f}  "
                          f"{describe_flags(f)}")
            elif mtype == MSG_STATUS:
                acc["status"] += 1
                if len(payload) == STATUS_SIZE:
                    acc["status_data"] = unpack_status(payload)
            elif mtype == MSG_HELLO:
                acc["hello"] += 1
                if len(payload) >= struct.calcsize(HELLO_FMT):
                    acc["hello_data"] = struct.unpack(
                        HELLO_FMT, payload[:struct.calcsize(HELLO_FMT)])
            elif mtype == MSG_LOG:
                acc["log"] += 1
                if len(payload) > 4:
                    acc["logs"].append(payload[4:].decode("ascii", "replace"))
    return acc


def run_pass(ser, dec, rep, mode, seconds, do_set, raw):
    """Optionally commands @p mode, then collects and judges one pass."""
    print(f"\n===== pass: {mode.upper()} " + "=" * (46 - len(mode)))

    # RE-ASSERTED EVERY PASS, not once at startup.
    #
    # The bridge resets streaming to its power-on default on any host reconnect,
    # and the master clears its own streaming flag on reset — hostLink.h states
    # plainly that "the host must re-issue CMD_START_STREAM, that is the
    # documented contract". Sending it once at the beginning met that contract
    # only for as long as nothing restarted, and a master that rebooted mid-run
    # therefore went quiet for the rest of the session while MSG_STATUS carried
    # on at 1 Hz. Re-issuing is idempotent and costs one 6-byte frame.
    ser.write(build_frame(CMD_START_STREAM))
    ser.flush()

    if do_set:
        global _last_mode_change
        wait_out_mode_guard()
        print(f"  sending CMD_SET_IMU_MODE({MODE_NAMES[mode]}), "
              f"waiting {MODE_SETTLE_S:.1f} s for bring-up")
        ser.write(build_frame(CMD_SET_IMU_MODE, bytes((MODE_NAMES[mode],))))
        ser.flush()
        _last_mode_change = time.monotonic()
        # Drain and DISCARD across the switch. The frames during a bring-up are
        # a sensor that is legitimately absent, and judging the mode contract
        # against them would fail every check for a firmware doing exactly what
        # it promised.
        collect(ser, dec, MODE_SETTLE_S)

    acc = collect(ser, dec, seconds, raw)

    print("\n-- link")
    print(f"        telemetry {acc['telemetry']}, status {acc['status']}, "
          f"hello {acc['hello']}, log {acc['log']}")
    print(f"        crc errors {dec.crc_errors}, version errors {dec.version_errors}")
    for line in acc["logs"][:10]:
        print(f"        bridge log: {line}")

    if acc["hello_data"]:
        h = acc["hello_data"]
        print(f"        bridge fw {h[1]}.{h[2]}, proto 0x{h[0]:02X}, "
              f"telemetry {h[4]} B, boot #{h[6]}")
        rep.check(h[0] == VERSION, "bridge protocol version matches this decoder",
                  f"0x{h[0]:02X}")
        # The bridge reports its own struct size, so a firmware-side layout
        # change is caught here instead of silently shifting every field.
        rep.check(h[4] == TELEMETRY_SIZE, "bridge telemetry size matches this decoder",
                  f"{h[4]} vs {TELEMETRY_SIZE}")

    # The bridge's own account of itself, which is the discriminator whenever
    # telemetry stops: it separates "the master went away" from "the master is
    # fine and forwarding is off" from "the C3 is dropping frames".
    s = acc["status_data"]
    if s:
        print(f"        bridge: {describe_bridge(s)}  "
              f"telemetryAge={s['telemetryAgeMs']} ms  "
              f"masterFrames={s['masterFrames']}  "
              f"masterCrc={s['masterCrcErrors']}  "
              f"hostTxDropped={s['hostTxDropped']}  "
              f"decim={s['decimation']}  up={s['bridgeMillis']} ms")

    rep.check(acc["telemetry"] > 0, "telemetry arrived from the MKR via the C3",
              f"{acc['telemetry']} frames")
    if not acc["telemetry"]:
        print("\n  No telemetry.")
        if not s:
            print("  No MSG_STATUS either - the C3 itself is not talking. Check the")
            print("  USB cable and that nothing else holds the port.")
        elif not (s["flags"] & BRIDGE_FLAG_MASTER_LINK):
            print(f"  The bridge says the MASTER LINK IS DOWN (last master frame "
                  f"{s['telemetryAgeMs']} ms ago).")
            print("  The C3 is fine; the MKR is not talking to it. Check the MKR's own")
            print("  console for a reset banner, and note that the MKR clears its")
            print("  streaming flag on reset - which this script now re-asserts.")
        elif not (s["flags"] & BRIDGE_FLAG_STREAMING):
            print("  The master link is UP but FORWARDING IS OFF. The bridge reset its")
            print("  streaming state, most likely on a host reconnect.")
        elif s["hostTxDropped"]:
            print(f"  The master link is up and forwarding is on, but the bridge has")
            print(f"  dropped {s['hostTxDropped']} frames to a full host TX ring.")
        else:
            print("  Master link up, forwarding on, nothing dropped - so the MKR is")
            print("  connected but producing no telemetry. Check its console.")
        return

    # A live link is not the same as a healthy one: the master can be forwarding
    # stale frames after it stopped producing new ones.
    if s:
        rep.check((s["flags"] & BRIDGE_FLAG_MASTER_LINK) != 0,
                  "the bridge reports the master link up",
                  describe_bridge(s))

    rep.check(dec.crc_errors == 0, "no CRC errors on the USB link", str(dec.crc_errors))

    # masterMillis must advance: identical values on every frame would mean the
    # bridge is republishing one stale snapshot rather than relaying a living
    # stream, and every other check here would happily pass on that.
    rep.check(len(set(acc["master_millis"])) > 1,
              "master timestamps advance (a live stream, not a stuck snapshot)",
              f"{len(set(acc['master_millis']))} distinct")
    print(f"        telemetry rate: {acc['telemetry'] / seconds:.1f} frames/s")

    t = acc["newest"]
    print(f"\n-- IMU contract ({mode.upper()})")
    print(f"        flags: {describe_flags(t['flags'])}")
    c = t["imuCalib"]
    print(f"        calib 0x{c:02X}  (mag {c & 3}, accel {(c >> 2) & 3}, "
          f"gyro {(c >> 4) & 3}, sys {(c >> 6) & 3})")

    rep.check(acc["present"] == acc["telemetry"], "IMU_PRESENT set on every frame",
              f"{acc['present']}/{acc['telemetry']}")

    # THE MODE IS READ FROM THE WIRE, not inferred and not assumed. Checking the
    # reported mode against the expected one is what makes the field checks below
    # meaningful: without it, a master stuck in the other mode would simply be
    # judged against the contract it happens to satisfy.
    want_fusion = (mode == "imuplus")
    reported_fusion = acc["fusion"] == acc["telemetry"]
    reported_raw = acc["fusion"] == 0
    rep.check(reported_fusion or reported_raw,
              "the reported mode is stable across the pass",
              f"FUSION_MODE on {acc['fusion']}/{acc['telemetry']}")
    rep.check(reported_fusion == want_fusion,
              f"the master reports {mode.upper()} as asked",
              "FUSION_MODE " + ("set" if reported_fusion else "clear"))

    check_mode_contract(rep, t, mode)

    print(f"        HIGHG_ARMED on {acc['armed']}/{acc['telemetry']}, "
          f"HIGH_G on {acc['highg']}, DATA_GAP on {acc['gap']}")
    rep.check(acc["armed"] == acc["telemetry"],
              "High-G backstop reports armed on every frame",
              f"{acc['armed']}/{acc['telemetry']}")


def restore_imuplus(ser, dec, rep):
    """Puts the master back in IMUPLUS and CONFIRMS it, rather than assuming.

    Assuming was the bug. The old cleanup sent the command and slept, so a
    request the master refused — which is what happened, every time, because it
    arrived inside the 5 s guard — left the rig in AMG under a report that said
    OK. A cleanup nobody checks is a cleanup that silently does not happen.
    """
    global _last_mode_change
    print("\n  restoring IMUPLUS")

    # RE-ASSERTED HERE TOO, and forgetting it here was a real bug. run_pass()
    # re-issues CMD_START_STREAM for the documented reason — the bridge resets
    # streaming to its power-on default on any host reconnect — and this path
    # needs it for exactly the same reason but did not have it. The result was a
    # restore that WAS applied (the master's console said so every time) and then
    # could not be confirmed, so a working rig reported FAILURES PRESENT.
    ser.write(build_frame(CMD_START_STREAM))
    ser.flush()

    wait_out_mode_guard()
    ser.write(build_frame(CMD_SET_IMU_MODE, bytes((IMU_MODE_FUSION,))))
    ser.flush()
    _last_mode_change = time.monotonic()

    acc = collect(ser, dec, MODE_SETTLE_S)          # ride out the bring-up
    acc = collect(ser, dec, 1.0)                    # then judge settled frames

    if acc["telemetry"] == 0:
        # Say what WAS seen. "No frames" on its own cannot distinguish a master
        # that stopped from a bridge that stopped forwarding, and this check
        # failing while the mode change plainly worked is the case that needs
        # telling apart.
        s = acc["status_data"]
        print(f"        saw status={acc['status']} hello={acc['hello']}"
              + (f" bridge={describe_bridge(s)}" if s else " (no status decoded)"))
        rep.check(False, "IMUPLUS restored after the run", "no frames to confirm with")
        print("  *** The mode change may still have been applied - check the master's")
        print("  *** console for 'host requested IMUPLUS'. This check only reports")
        print("  *** what the host could OBSERVE.")
        return
    restored = acc["fusion"] == acc["telemetry"]
    rep.check(restored, "IMUPLUS restored after the run",
              f"FUSION_MODE on {acc['fusion']}/{acc['telemetry']}")
    if not restored:
        print("  *** The rig is still in AMG. Re-run with:")
        print("  ***   --mode imuplus --set")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="End-to-end IMU check: MKR Zero -> ESP32-C3 -> this PC.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Without --set the master is only observed, so it must already be\n"
               "in the expected mode (flash it with BuildAndUpload.cmd COM5 amg).\n"
               "With --set the mode is commanded over the wire and no reflash is\n"
               "needed.",
    )
    ap.add_argument("--port", help="C3 USB-CDC port, e.g. COM7 or /dev/ttyACM0")
    ap.add_argument("--mode", choices=tuple(MODE_NAMES), default="imuplus",
                    help="mode to expect (default: imuplus)")
    ap.add_argument("--both", action="store_true",
                    help="check IMUPLUS then AMG in one run, restoring IMUPLUS after")
    ap.add_argument("--set", dest="do_set", action="store_true",
                    help="command the mode with CMD_SET_IMU_MODE before checking")
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="collection time per pass (default: 8)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--raw", action="store_true",
                    help="print every telemetry frame as it arrives")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"  {p.device}   {p.description}")
        return 0
    if not args.port:
        ap.error("--port is required (use --list to find it)")
    if args.both and not args.do_set:
        ap.error("--both needs --set: with no way to command the mode, one run can "
                 "only observe whichever mode the master booted into")

    print("=== IMU end-to-end check =====================================")
    print(f"  port {args.port}, {args.seconds:.0f} s per pass, "
          f"{'commanding' if args.do_set else 'observing'} the mode")

    rep = Report()
    dec = Decoder()
    modes = ["imuplus", "amg"] if args.both else [args.mode]

    with serial.Serial(args.port, 115200, timeout=0.1) as ser:
        # A fresh host connection: the bridge answers CMD_HELLO with its identity
        # and resets its own streaming state, so the run starts from a known
        # point rather than from whatever the last client left behind.
        ser.reset_input_buffer()
        ser.write(build_frame(CMD_HELLO))
        time.sleep(0.2)
        ser.write(build_frame(CMD_START_STREAM))
        time.sleep(0.2)

        try:
            for m in modes:
                run_pass(ser, dec, rep, m, args.seconds, args.do_set, args.raw)
        finally:
            # IN A finally, AND VERIFIED. Leaving the rig in AMG is not a tidy-up
            # detail: that mode publishes no linear acceleration at all, so the
            # next person to look is debugging a mode change they never made.
            # An exception or Ctrl-C part-way through is exactly when the restore
            # matters most, which is why it no longer sits on the success path.
            if args.do_set and (args.both or args.mode != "imuplus"):
                restore_imuplus(ser, dec, rep)

    print("\n=== summary ==================================================")
    print(f"  passed {rep.passed}, failed {rep.failed}")
    print("  RESULT: OK" if rep.failed == 0 else "  RESULT: FAILURES PRESENT")
    return 0 if rep.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())


