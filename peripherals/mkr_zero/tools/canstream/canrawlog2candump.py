#!/usr/bin/env python3
"""Convert a CANRawLog recording to candump -l, timed by its own S lines.

    canrawlog2candump.py IN.log [-o OUT.log] [--iface can0]

CANRawLog (helper_scripts/CANRawLog, recorded with its can_log.py) writes
  F <micros hex8> <id hex3> <dlc> <data hex>     one standard data frame
  S <micros hex8> <frames> <ovf> <rtr> <ext> @<host epoch>   once a second
  # ...                                          comments, banners, host notes
The S line's micros() is taken as it is written and can_log.py appends the
host's wall clock as the line arrives, so each S line is one delivery sample
tying the board's clock to the host's — exactly what an FS line is to
mkr_stream_log.py, and handled the same way:

  - micros() is unwrapped by mkr_stream_log.Timeline: signed modular distance
    from the newest value, F lines in drain order, S lines (like FS) never
    behind the frames before them. A step back, a '# host start' line (a second
    recording appended), a '# CANRawLog' banner (the board restarted) or a
    '# port lost' note starts a new timeline.
  - The MKR -> epoch offset is the MINIMUM of (epoch_received - micros) over
    the S lines within WINDOW_S of each frame: the delivery delay is only ever
    positive, so its floor is the truest sample. Offline, the window is centred
    on the frame (+-WINDOW_S/2) instead of trailing it, since the whole file is
    at hand. can_log.py stamps to the millisecond, so the result is good to
    about a millisecond plus that floor.

Frames come out in file order, in mkr_stream_log.py's candump format, so
can_decode reads either. Extended and remote frames never appear in CANRawLog
v1 (it counts them in the S line without logging them); the parser accepts the
production stream's extended and remote F shapes anyway, at no cost. Only S
lines carrying '@epoch' anchor time — the production stream's FS lines carry no
host time, so a raw capture of it belongs to mkr_stream_log.py, not here. A
timeline with no anchored S line cannot be placed and its frames are dropped
(counted on stderr; exit status 1, as for malformed lines). A wall-clock step during the
recording would show as the anchors' spread; more than STEP_WARN_S is warned
about, since the minimum would then mix the two clocks.
"""
import argparse
import bisect
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mkr_stream_log as msl  # noqa: E402  (the unwrap rules and the candump format live there)

WINDOW_S = msl.WINDOW_S
STEP_WARN_S = 1.0
HEX = frozenset("0123456789ABCDEFabcdef")


def is_hex(s, n=None):
    return bool(s) and (n is None or len(s) == n) and all(c in HEX for c in s)


def parse_f(tokens):
    """F tokens -> (us, id, ext, rtr, dlc, data_hex) or None.

    CANRawLog v1: 'F us iii d data' (a DLC 0 frame keeps the space before its empty data). Also the
    production stream's extended ('F us iiiiiiii d data') and remote ('F us iii R d') shapes."""
    if len(tokens) < 4 or not is_hex(tokens[1], 8):
        return None
    if is_hex(tokens[2], 3):
        ext = False
    elif is_hex(tokens[2], 8):
        ext = True
    else:
        return None
    ident = int(tokens[2], 16)
    if ident > (0x1FFFFFFF if ext else 0x7FF):
        return None
    us = int(tokens[1], 16)
    if tokens[3] == "R":
        if len(tokens) != 5 or len(tokens[4]) != 1 or tokens[4] not in "012345678":
            return None
        return us, ident, ext, True, int(tokens[4]), ""
    if len(tokens[3]) != 1 or tokens[3] not in "012345678":
        return None
    dlc = int(tokens[3])
    data = tokens[4] if len(tokens) > 4 else ""
    if len(tokens) > 5 or len(data) != 2 * dlc or (dlc and not is_hex(data)):
        return None
    return us, ident, ext, False, dlc, data.upper()


def parse_s(tokens):
    """S tokens -> (us, epoch) or None. An S line without '@epoch' carries no host time: None."""
    if len(tokens) < 3 or not is_hex(tokens[1], 8) or not tokens[-1].startswith("@"):
        return None
    try:
        return int(tokens[1], 16), float(tokens[-1][1:])
    except ValueError:
        return None


def read(path):
    """Pass 1: frames with their timeline and unwrapped MKR seconds, anchors per timeline."""
    frames = []            # (timeline, mkr_s, (id, ext, rtr, dlc, data))
    anchors = [[]]         # per timeline: [(mkr_s, epoch)]
    stats = {"lines": 0, "malformed": 0, "comments": 0, "other": 0, "s_without_epoch": 0}
    tl = msl.Timeline()
    tl_no = 0

    def new_timeline():
        nonlocal tl, tl_no
        if tl.newest is None:              # nothing on this one yet: keep it
            return
        tl, tl_no = msl.Timeline(), tl_no + 1
        anchors.append([])

    def place(raw, frame):
        us = tl.unwrap(raw, frame)
        if us is None:
            print("canrawlog2candump: new timeline at line %d: %s" % (stats["lines"], tl.why), file=sys.stderr)
            new_timeline()
            us = tl.unwrap(raw, frame)
        return us / 1e6

    with open(path, "r", encoding="ascii", errors="replace") as f:
        for line in f:
            stats["lines"] += 1
            line = line.rstrip("\r\n")
            if line.startswith("#"):
                stats["comments"] += 1
                low = line.lower()
                if low.startswith("# host start") or low.startswith("# canrawlog") or "port lost" in low:
                    new_timeline()
                continue
            tokens = line.split(" ")
            if tokens[0] == "F":
                fr = parse_f(tokens)
                if fr is None:
                    stats["malformed"] += 1
                    continue
                mkr_s = place(fr[0], True)
                frames.append((tl_no, mkr_s, fr[1:]))
            elif tokens[0] == "S":
                an = parse_s(tokens)
                if an is None:
                    stats["s_without_epoch" if len(tokens) > 1 and is_hex(tokens[1], 8) else "malformed"] += 1
                    continue
                mkr_s = place(an[0], False)
                anchors[tl_no].append((mkr_s, an[1]))
            elif line.strip():
                stats["other"] += 1
    return frames, anchors, stats


class Anchor:
    """Centred windowed minimum of (epoch - mkr_s) over one timeline's S lines.

    Evaluated AT each S line and interpolated linearly between them, rather than over a window
    centred on every frame: the latter jumps by the full difference whenever the window takes in or
    lets go of the lowest sample, and a jump of a few ms puts frames 0.8 ms apart out of order. The
    interpolated mapping is continuous, and monotonic as long as the minimum moves less than the
    ~1 s between S lines — by four orders of magnitude, on a recording with tens of ms of spread."""

    def __init__(self, samples):
        self.mk = [m for m, _ in samples]
        c = [e - m for m, e in samples]
        self.spread = (max(c) - min(c)) if c else 0.0
        self.off = []
        for m in self.mk:
            lo = bisect.bisect_left(self.mk, m - WINDOW_S / 2)
            hi = bisect.bisect_right(self.mk, m + WINDOW_S / 2)
            self.off.append(min(c[lo:hi]))

    def offset(self, t):
        if not self.mk:
            return None
        i = bisect.bisect_right(self.mk, t)
        if i == 0:
            return self.off[0]
        if i == len(self.mk):
            return self.off[-1]
        m0, m1 = self.mk[i - 1], self.mk[i]
        if m1 <= m0:
            return self.off[i]
        return self.off[i - 1] + (self.off[i] - self.off[i - 1]) * (t - m0) / (m1 - m0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("infile")
    ap.add_argument("-o", "--out", help="candump output (default stdout)")
    ap.add_argument("--iface", default="can0", help="interface name written into the output")
    a = ap.parse_args(argv)

    frames, anchors, stats = read(a.infile)
    fits = [Anchor(s) for s in anchors]
    out = open(a.out, "w") if a.out else sys.stdout
    written = unanchored = 0
    try:
        for tl_no, mkr_s, (ident, ext, rtr, dlc, data) in frames:
            off = fits[tl_no].offset(mkr_s)
            if off is None:
                unanchored += 1
                continue
            out.write(msl.candump_line(mkr_s + off, ident, ext, rtr, dlc, data, a.iface))
            written += 1
    finally:
        if out is not sys.stdout:
            out.close()

    for i, fit in enumerate(fits):
        if fit.mk:
            print("canrawlog2candump: timeline %d: %d S anchors over %.1f s, anchor spread %.1f ms"
                  % (i, len(fit.mk), fit.mk[-1] - fit.mk[0], fit.spread * 1e3), file=sys.stderr)
            if fit.spread > STEP_WARN_S:
                print("canrawlog2candump: WARNING: timeline %d anchors spread %.3f s - a wall-clock step during "
                      "the recording? Timestamps near it mix the two clocks." % (i, fit.spread), file=sys.stderr)
    print("canrawlog2candump: %d frames written, %d unanchored (timeline without S lines) dropped; %d lines, "
          "%d malformed, %d S without @epoch, %d other, %d comments, %d timeline(s)"
          % (written, unanchored, stats["lines"], stats["malformed"], stats["s_without_epoch"], stats["other"],
             stats["comments"], len(fits)), file=sys.stderr)
    return 1 if (stats["malformed"] or unanchored) else 0


if __name__ == "__main__":
    sys.exit(main())
