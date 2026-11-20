#!/usr/bin/env python3
"""Guided microphone measurement with bench/mic_capture (MIC_LEVEL_PLAN.md, P1).

The board shows the prompts; the person only follows them. For each gain step
the run records a quiet phase, a speech phase and a clap phase, then prints a
per-phase summary and writes every 200 ms window to a CSV file.

    python3 mic_session.py /dev/dashcam-display --csv mic_session.csv

Standard library only (termios), like host/es3_display.py.
"""
import argparse
import csv
import os
import select
import statistics
import sys
import termios
import time

# Gain steps of mic_capture.ino: index -> total dB (PGA + ADC scale).
GAIN_DB = [0, 12, 24, 30, 36, 42, 48, 54]
DEFAULT_STEPS = [1, 2, 3, 5]          # 12, 24, 30, 42 dB
PHASES = [("quiet", "STAY QUIET", 6.0), ("talk", "TALK NORMALLY", 7.0), ("clap", "CLAP 3 TIMES", 6.0)]
SETTLE_S = 1.0                        # windows during the first second are ignored


class Board:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                  # iflag: raw
        attrs[1] = 0                                  # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                  # lflag: no echo, non-canonical
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buffer = b""

    def send(self, command):
        os.write(self.fd, (command + "\n").encode("ascii"))

    def lines(self, timeout):
        """Yields complete lines until timeout seconds pass."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise ConnectionError("board disconnected")
            self.buffer += chunk
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                yield line.decode("ascii", "replace").strip()


def parse_window(line):
    """W seq gN dB L rms pk dc clip R rms pk dc clip D diff ovf n err n draw last max"""
    f = line.split()
    if len(f) < 21 or f[0] != "W" or f[4] != "L" or f[9] != "R" or f[14] != "D":
        return None
    return {
        "seq": int(f[1]), "step": int(f[2][1:]), "gain_db": int(f[3]),
        "l_rms": float(f[5]), "l_peak": float(f[6]), "l_dc": int(f[7]), "l_clip": int(f[8]),
        "r_rms": float(f[10]), "r_peak": float(f[11]), "r_dc": int(f[12]), "r_clip": int(f[13]),
        "diff": float(f[15]), "ovf": int(f[17]), "err": int(f[19]),
        "draw_us": int(f[21]), "draw_max_us": int(f[22]) if len(f) > 22 else 0,
    }


def record(board, step, phase, prompt, seconds, rows):
    board.send("p" + prompt)
    start = time.monotonic()
    taken = []
    for line in board.lines(seconds):
        w = parse_window(line)
        if w is None or w["step"] != step or time.monotonic() - start < SETTLE_S:
            continue
        w["phase"] = phase
        rows.append(w)
        taken.append(w)
    return taken


def summarize(rows, steps):
    print("\nstep gain  phase  slot   RMS median  RMS p90  peak max  clips  DC median")
    for step in steps:
        for phase, _, _ in PHASES:
            sel = [r for r in rows if r["step"] == step and r["phase"] == phase]
            if not sel:
                print(f"{step:>4} {GAIN_DB[step]:>3} dB {phase:<6} (no windows)")
                continue
            for slot in ("l", "r"):
                rms = sorted(r[slot + "_rms"] for r in sel)
                p90 = rms[min(len(rms) - 1, int(0.9 * len(rms)))]
                print(f"{step:>4} {GAIN_DB[step]:>3} dB {phase:<6} {slot.upper():<4} "
                      f"{statistics.median(rms):>10.1f} {p90:>8.1f} "
                      f"{max(r[slot + '_peak'] for r in sel):>9.1f} "
                      f"{sum(r[slot + '_clip'] for r in sel):>6} "
                      f"{statistics.median(r[slot + '_dc'] for r in sel):>10.0f}")
    if rows:
        diff = statistics.median(r["diff"] for r in rows)
        print(f"\nL-R difference, median: {diff:.1f} dBFS (about -99 = the slots are copies)")
        print(f"overflows during run: {rows[-1]['ovf'] - rows[0]['ovf']}, read errors: {rows[-1]['err']}, "
              f"longest screen update: {max(r['draw_max_us'] for r in rows) / 1000:.0f} ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device")
    parser.add_argument("--steps", default=",".join(map(str, DEFAULT_STEPS)),
                        help="gain step indices to measure (default %(default)s)")
    parser.add_argument("--csv", help="write every window here")
    parser.add_argument("--wait", type=float, default=300.0, help="seconds to wait for the start tap")
    args = parser.parse_args()
    steps = [int(s) for s in args.steps.split(",")]

    board = Board(args.device)
    board.send("g" + str(steps[0]))
    board.send("pTAP SCREEN TO START")
    print("Waiting for a tap on the board screen ...", flush=True)
    if not any(line == "[tap]" for line in board.lines(args.wait)):
        board.send("c")
        sys.exit("no tap within %.0f s" % args.wait)

    rows = []
    for step in steps:
        board.send("g" + str(step))
        for phase, prompt, seconds in PHASES:
            print(f"gain {GAIN_DB[step]} dB: {phase}", flush=True)
            record(board, step, phase, prompt, seconds, rows)
    board.send("pDONE - THANK YOU")
    summarize(rows, steps)
    if args.csv:
        with open(args.csv, "w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=list(rows[0].keys()) if rows else ["phase"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"windows written to {args.csv}")


if __name__ == "__main__":
    main()
