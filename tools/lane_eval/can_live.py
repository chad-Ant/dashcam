#!/usr/bin/env python3
"""can_live.py — live CAN state for a lane test run (plan P5b, sections 12.4, 12.7).

    python3 tools/lane_eval/can_live.py --run RUN_DIR [--decoder PATH] [--map FILE] [--min-lc-kmh 30]

Runs on the HOST beside lane_run.sh (stdlib only).  It follows the MKR
logger's RUN_DIR/can/can_raw.log (and can/partN/can_raw.log after a logger
restart), pipes every new line into the UNMODIFIED offline decoder
(`stdbuf -oL can_decode --every-frame -`; there is no second decoder
anywhere) and keeps the newest value of each signal.  Writes:

  can/live_state.json  5 Hz, atomic: mono_ns of the newest decoded frame (its
                       log time mapped to CLOCK_MONOTONIC), speed_kmh, gear
                       (the letter), turn_l, turn_r, brake, age_ms, frames,
                       decoder_alive, part
  can/live_events.csv  stalk_on / stalk_off / stop_go episodes
  triggers.csv         the same episodes as clip triggers for lane_drive:
                       mono_ns,source,type,priority,detail (priority 1 =
                       lane change: a stalk episode at >= --min-lc-kmh; 2 =
                       stop->go; 3 = a stalk at low speed)

Live data carries the logger's 2-3 s hold: frames are timed by their log
time, so a trigger's mono_ns is when the frame was on the bus, not when it
arrived here.  Log time (epoch) -> mono: minus (CLOCK_REALTIME -
CLOCK_MONOTONIC) sampled now (a clock step mid-run moves it; lane_drive's
clock.csv records steps).  SIGTERM / SIGINT: exit 0.
"""
import argparse
import csv
import glob
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DECODER = os.path.join(REPO, "peripherals", "mkr_zero", "tools", "canstream", "can_decode")

stop = False


def on_signal(*_):
    global stop
    stop = True


def wall_minus_mono_ns():
    return time.time_ns() - time.monotonic_ns()


def newest_log(run):
    """(path, part) of the newest can_raw.log: can/partN beats can/."""
    best = (os.path.join(run, "can", "can_raw.log"), 0)
    for d in glob.glob(os.path.join(run, "can", "part*")):
        m = re.search(r"part(\d+)$", d)
        if m and int(m.group(1)) > best[1]:
            best = (os.path.join(d, "can_raw.log"), int(m.group(1)))
    return best


def write_atomic(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.mono_ns = None
        self.speed = None
        self.gear = None
        self.turn_l = 0
        self.turn_r = 0
        self.brake = None
        self.frames = 0


class Episodes:
    """Stalk and stop->go episodes from the decoded rows (log time)."""

    def __init__(self, run, min_lc_kmh):
        self.run, self.min_lc = run, min_lc_kmh
        self.stalk = None            # (side, on_mono, speed_at_on)
        self.stopped_since = None
        self.was_stopped = False
        self.ev = open(os.path.join(run, "can", "live_events.csv"), "a")
        if self.ev.tell() == 0:
            self.ev.write("mono_ns,type,side,speed_kmh,detail\n")
        self.trig = open(os.path.join(run, "triggers.csv"), "a")

    def emit(self, mono, typ, side, speed, prio, detail=""):
        sp = "" if speed is None else "%.1f" % speed
        self.ev.write("%d,%s,%s,%s,%s\n" % (mono, typ, side, sp, detail))
        self.ev.flush()
        if prio is not None:
            self.trig.write("%d,can_live,%s,%d,%s %s km/h %s\n" % (mono, typ, prio, side, sp, detail))
            self.trig.flush()

    def feed(self, mono, speed, turn_l, turn_r):
        side = "L" if turn_l and not turn_r else "R" if turn_r and not turn_l else ("H" if turn_l and turn_r else None)
        if self.stalk is None and side in ("L", "R"):
            fast = speed is not None and speed >= self.min_lc
            self.stalk = (side, mono, speed)
            self.emit(mono, "stalk_on", side, speed, 1 if fast else 3)
        elif self.stalk is not None and side != self.stalk[0]:
            dur = (mono - self.stalk[1]) / 1e9
            self.emit(mono, "stalk_off", self.stalk[0], speed, 1 if (self.stalk[2] or 0) >= self.min_lc else 3,
                      "%.1f s" % dur)
            self.stalk = None
            if side in ("L", "R"):
                self.feed(mono, speed, turn_l, turn_r)
        if speed is None:
            return
        if speed < 2.0:
            if self.stopped_since is None:
                self.stopped_since = mono
            if mono - self.stopped_since >= 2e9:
                self.was_stopped = True
        else:
            self.stopped_since = None
            if self.was_stopped and speed > 10.0:
                self.was_stopped = False
                self.emit(mono, "stop_go", "", speed, 2)


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--decoder", default=DECODER)
    ap.add_argument("--map", help="can_decode --map (default: the decoder's own default)")
    ap.add_argument("--min-lc-kmh", type=float, default=30.0)
    ap.add_argument("--interval", type=float, default=0.2)
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    os.makedirs(os.path.join(a.run, "can"), exist_ok=True)

    cmd = ["stdbuf", "-oL", a.decoder, "--every-frame"] + (["--map", a.map] if a.map else []) + ["-"]
    dec = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           text=True, bufsize=1)
    st = State()
    eps = Episodes(a.run, a.min_lc_kmh)

    def reader():
        rd = csv.reader(dec.stdout)
        header = None
        for row in rd:
            if header is None:
                header = {k: i for i, k in enumerate(row)}
                continue
            try:
                t = float(row[header["time"]])
            except (ValueError, KeyError, IndexError):
                continue

            def get(k):
                i = header.get(k)
                return row[i] if i is not None and i < len(row) else ""
            mono = int(t * 1e9) - wall_minus_mono_ns()
            speed = num(get("speed_kmh"))
            tl = get("turn_left") == "1"
            tr = get("turn_right") == "1"
            with st.lock:
                st.mono_ns = mono
                st.frames += 1
                if speed is not None:
                    st.speed = speed
                g = get("gear")
                st.gear = g if g else st.gear
                st.turn_l, st.turn_r = int(tl), int(tr)
                b = get("brake_pressed")
                st.brake = int(b) if b in ("0", "1") else st.brake
                sp = st.speed
            eps.feed(mono, sp, tl, tr)

    threading.Thread(target=reader, daemon=True).start()

    path, part, f = None, -1, None
    last_state = 0.0
    buf = ""
    while not stop:
        npath, npart = newest_log(a.run)
        if npart != part and os.path.exists(npath):
            if f:
                f.close()
            f = open(npath, "r", errors="replace")
            # Part 0 (the run's first log) is read from its start; a file that
            # was already there when can_live started is followed from its end.
            if part == -1 and npart == 0 and os.path.getsize(npath) > 0 and time.time() - os.path.getmtime(npath) > 5:
                f.seek(0, os.SEEK_END)
            path, part, buf = npath, npart, ""
        idle = True
        if f:
            chunk = f.read(1 << 20)
            if chunk:
                idle = False
                buf += chunk
                lines = buf.split("\n")
                buf = lines.pop()
                try:
                    dec.stdin.write("\n".join(lines) + ("\n" if lines else ""))
                    dec.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
        now = time.monotonic()
        if now - last_state >= a.interval:
            last_state = now
            with st.lock:
                d = {"mono_ns": st.mono_ns, "speed_kmh": st.speed, "gear": st.gear, "turn_l": st.turn_l,
                     "turn_r": st.turn_r, "brake": st.brake,
                     "age_ms": None if st.mono_ns is None else (time.monotonic_ns() - st.mono_ns) // 1000000,
                     "frames": st.frames, "decoder_alive": dec.poll() is None, "part": part,
                     "log": path, "updated_mono_ns": time.monotonic_ns()}
            write_atomic(os.path.join(a.run, "can", "live_state.json"), json.dumps(d) + "\n")
        if idle:
            time.sleep(0.05)
    try:
        dec.stdin.close()
    except OSError:
        pass
    dec.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
