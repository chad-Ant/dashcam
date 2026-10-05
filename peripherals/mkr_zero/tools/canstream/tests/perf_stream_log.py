#!/usr/bin/env python3
"""The logger's CPU cost at 2500 lines/s for 60 s — twice the measured bus (~1230 frames/s).

    perf_stream_log.py [--rate 2500] [--seconds 60]

The synthetic MKR writes as the firmware does: whole lines, at most 63 bytes per write, every
millisecond whatever has fallen due (so the logger sees the many small reads a real CDC port gives it,
not a few big ones). Mostly 8-byte frames (34-byte lines), an FS line a second, a console line a second.
Reports the logger process's CPU (all threads, from /proc/<pid>/stat) as a share of one core, and checks
that every frame reached can_raw.log.
"""
import argparse
import os
import random
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from streamsim import PACKET, Logger, Pty, read_candump  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=2500.0)
    ap.add_argument("--seconds", type=float, default=60.0)
    a = ap.parse_args()

    rng = random.Random(9)
    tmp = tempfile.mkdtemp(prefix="canstream_perf_")
    port = os.path.join(tmp, "ttyMKR")
    out = os.path.join(tmp, "out")
    pty = Pty(port)
    lg = Logger(out, port)
    if not lg.wait_open():
        sys.exit("logger did not open the port")
    ids = [0x158, 0x17C, 0x191, 0x1AB, 0x1D0, 0x294, 0x100, 0x2A0, 0x3C0, 0x055, 0x0F0, 0x4E0, 0x7E0]
    sent = 0
    us0 = 0x7FFFFFFF
    t0 = time.monotonic()
    cpu0 = lg.cpu_seconds()
    next_fs = next_con = 1.0
    due = 0.0
    while True:
        now = time.monotonic() - t0
        if now >= a.seconds:
            break
        lines = []
        while due <= now:
            us = (us0 + int(due * 1e6)) & 0xFFFFFFFF
            if due >= next_fs:
                lines.append("FS %08X %d %d 0 0 0\n" % (us, sent, sent))
                next_fs += 1.0
            ident = rng.choice(ids)
            dlc = 8 if rng.random() < 0.85 else rng.randrange(9)
            data = "".join("%02X" % rng.randrange(256) for _ in range(dlc))
            lines.append("F %08X %03X %d%s\n" % (us, ident, dlc, (" " + data) if dlc else ""))
            sent += 1
            due += 1.0 / a.rate
        if now >= next_con:
            lines.append("MKR status can=discover rx=%d ovf=0 drop=0\n" % sent)
            next_con += 1.0
        buf = b""
        for line in lines:
            b = line.encode()
            if len(buf) + len(b) > PACKET:
                pty.write(buf)
                buf = b""
            buf += b
        if buf:
            pty.write(buf)
        time.sleep(0.001)
    wall = time.monotonic() - t0
    cpu = lg.cpu_seconds() - cpu0
    rc = lg.stop()
    pty.close()
    got = len(read_candump(os.path.join(out, "can_raw.log")))
    print("rate %.0f lines/s for %.1f s: %d frames sent, %d logged, logger rc %d" % (a.rate, wall, sent, got, rc))
    print("logger CPU %.2f s = %.1f %% of one core (writer blocked %.3f s in total, worst %.3f s)"
          % (cpu, 100.0 * cpu / wall, pty.blocked, pty.worst_block))
    shutil.rmtree(tmp, ignore_errors=True)
    return 0 if got == sent and rc == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
