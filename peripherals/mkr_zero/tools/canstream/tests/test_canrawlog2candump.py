"""canrawlog2candump.py: CANRawLog v1 recordings to candump, timed by their own S lines.

Synthetic recordings with a known truth: each frame's true wall time, S lines stamped with the board's
micros() and the host's receive time (true time + a delivery delay with a 1 ms floor, rounded to the
millisecond as can_log.py does). The converted timestamps must sit on the truth plus that floor.
"""
import os
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

from streamsim import read_candump

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONVERT = os.path.join(TOOLS, "canrawlog2candump.py")
WRAP = 1 << 32


def recording(rng, epoch0, start_us, duration, rate=800.0, header=True, banner=True, s_lines=True, step=None,
              ppm=0.0):
    """One CANRawLog session. Returns (text, truth [(epoch, id, dlc, data)]). @ppm: the host's clock runs
    that much fast (negative: slow) against the board's micros()."""
    k = 1.0 + ppm * 1e-6
    out, truth = [], []
    if header:
        out.append("# host start %.3f device /dev/ttyACM0" % epoch0)
    if banner:
        out.append("# CANRawLog v1  listen-only  500 kbps  accept-all  (F us id dlc data | S us frames ovf rtr ext)")
    t, next_s, frames = 0.0, 1.0, 0
    while t < duration:
        if s_lines and t >= next_s:
            us = (start_us + int(next_s * 1e6)) % WRAP
            recv = epoch0 + next_s * k + 0.001 + rng.expovariate(1 / 0.003)
            if step and next_s >= step[0]:
                recv += step[1]
            out.append("S %08X %d %d 0 0 @%.3f" % (us, frames, frames // 300, recv))
            next_s += 1.0
        ident = rng.choice([0x158, 0x17C, 0x191, 0x1D0, 0x055, 0x7FF, 0x000])
        dlc = rng.randrange(9)
        data = "".join("%02X" % rng.randrange(256) for _ in range(dlc))
        us = (start_us + int(round(t * 1e6))) % WRAP
        out.append("F %08X %03X %X %s" % (us, ident, dlc, data))     # DLC 0 keeps the space before no data
        truth.append((epoch0 + int(round(t * 1e6)) / 1e6 * k, ident, dlc, data))
        frames += 1
        t += rng.expovariate(rate)
    return "\n".join(out) + "\n", truth


class ConvertCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="canstream_convert_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def convert(self, text):
        src = os.path.join(self.tmp, "in.log")
        dst = os.path.join(self.tmp, "out.log")
        with open(src, "w") as f:
            f.write(text)
        p = subprocess.run([sys.executable, CONVERT, src, "-o", dst], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
        return p, (read_candump(dst) if os.path.exists(dst) else [])

    def assert_matches(self, got, truth, early=0.0004, late=0.0030):
        self.assertEqual(len(got), len(truth))
        errs = []
        for g, (epoch, ident, dlc, data) in zip(got, truth):
            self.assertEqual((g[2], g[3], g[4], g[5], g[6]), (ident, False, False, dlc, data))
            errs.append(g[0] / 1e6 - epoch)
        # The floor of the delivery delay (1 ms) plus can_log.py's millisecond rounding.
        self.assertGreaterEqual(min(errs), early, "early by %.3f ms" % (-min(errs) * 1e3))
        self.assertLessEqual(max(errs), late, "late by %.3f ms" % (max(errs) * 1e3))
        ts = [g[0] for g in got]
        self.assertTrue(all(b >= a for a, b in zip(ts, ts[1:])), "output out of time order")


class TestConvert(ConvertCase):
    def test_wrap_and_dlc0(self):
        rng = random.Random(11)
        text, truth = recording(rng, 1790000000.0, WRAP - 5_000_000, 30.0)
        p, got = self.convert(text)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertRegex(text, r"(?m)^F [0-9A-F]{8} [0-9A-F]{3} 0 $")      # CANRawLog's DLC 0 shape
        self.assertTrue(any(t[2] == 0 for t in truth))
        self.assert_matches(got, truth)
        self.assertIn("1 timeline(s)", p.stderr)

    def test_appended_recordings_and_board_restart(self):
        rng = random.Random(12)
        a, ta = recording(rng, 1791000000.0, 0x10000000, 12.0)
        b, tb = recording(rng, 1791000100.0, 0x00200000, 12.0)          # '# host start': a second recording
        c, tc = recording(rng, 1791000200.0, 0x00100000, 12.0, header=False)   # banner only: the board restarted
        p, got = self.convert(a + b + c)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("3 timeline(s)", p.stderr)
        self.assert_matches(got, ta + tb + tc)

    def test_appended_recording_one_wrap_later(self):
        """A second recording, '# host start' only: the board kept running for 71.6 min + 12.5 s, so its
        micros() resume 0.5 s past where the first recording's ended — they read as one continuous run. Only
        the header separates them; merged, the first 10 s of the second recording would take their offset
        from the first's anchors, 71.6 min away."""
        rng = random.Random(17)
        a, ta = recording(rng, 1791000000.0, 0x10000000, 12.0)
        b, tb = recording(rng, 1791000000.0 + WRAP / 1e6 + 12.5, 0x10000000 + 12_500_000, 12.0, banner=False)
        p, got = self.convert(a + b)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("2 timeline(s)", p.stderr)
        self.assert_matches(got, ta + tb)

    def test_host_clock_drift(self):
        """The host clock 500 ppm fast, then slow, against the board's (an NTP slew at full rate). The window
        minimum is biased by drift x WINDOW_S/2 = 5 ms at most (early, either sign); output stays in time
        order, which a per-anchor step function would not (the offset falls 0.5 ms at every S line when the
        host is slow), and the window stays a window (a whole-file minimum drifts 30 ms in 60 s)."""
        for seed, ppm in ((18, 500.0), (19, -500.0)):
            rng = random.Random(seed)
            text, truth = recording(rng, 1791000000.0, 0x50000000, 60.0, ppm=ppm)
            p, got = self.convert(text)
            self.assertEqual(p.returncode, 0, p.stderr)
            self.assert_matches(got, truth, early=-0.0065, late=0.0035)

    def test_unanchored_timeline_dropped(self):
        rng = random.Random(13)
        a, ta = recording(rng, 1791000000.0, 0x10000000, 6.0)
        b, _tb = recording(rng, 1791000100.0, 0x00200000, 0.9, s_lines=False)   # no S line: no host time
        p, got = self.convert(a + b)
        self.assertEqual(p.returncode, 1)
        self.assertIn("unanchored", p.stderr)
        self.assert_matches(got, ta)

    def test_malformed_and_production_shapes(self):
        rng = random.Random(14)
        text, truth = recording(rng, 1791000000.0, 0x20000000, 5.0)
        lines = text.splitlines()
        # Production-stream shapes are accepted too (extended, remote), placed in time order.
        k = len(lines) // 2
        us = int(lines[k].split()[1], 16)
        lines.insert(k + 1, "F %08X 18DAF110 2 ABCD" % us)
        lines.insert(k + 2, "F %08X 123 R 4" % us)
        lines.insert(k + 3, "F %08X 17C 9 00" % us)                       # malformed: DLC 9
        lines.insert(k + 4, "F %08X 17C 2 ABC" % us)                      # malformed: odd data
        lines.insert(k + 5, "F %08X 6A5 0" % us)                          # production DLC 0: no trailing space
        p, got = self.convert("\n".join(lines) + "\n")
        self.assertEqual(p.returncode, 1)
        self.assertIn("2 malformed", p.stderr)
        self.assertEqual(len(got), len(truth) + 3)
        shapes = [(g[2], g[3], g[4], g[5], g[6]) for g in got if g[3] or g[4] or g[2] == 0x6A5]
        self.assertEqual(shapes, [(0x18DAF110, True, False, 2, "ABCD"), (0x123, False, True, 4, ""),
                                  (0x6A5, False, False, 0, "")])

    def test_stale_frames_after_s_line_are_one_timeline(self):
        """A stall as the production stream shows it: an S anchor 8 s AHEAD of the frames that follow it
        (the ring's oldest, still in drain order), then the stream resuming from the anchor's time. One
        boot: the fixed '5 s backwards = reset' rule would have split it."""
        rng = random.Random(15)
        e0, u0 = 1791000000.0, 0x30000000
        text, truth = recording(rng, e0, u0, 14.0)
        t_of = lambda line: ((int(line.split()[1], 16) - u0) % WRAP) / 1e6
        out, kept, fi = [], [], iter(truth)
        inserted = False
        for line in text.splitlines():
            if line.startswith("#"):
                out.append(line)
                continue
            t = t_of(line)
            is_f = line.startswith("F ")
            tr = next(fi) if is_f else None
            if 3.0 <= t and not inserted:
                out.append("S %08X 1 0 0 0 @%.3f" % ((u0 + 11_000_000) % WRAP, e0 + 11.0 + 0.001))
                inserted = True
            if 3.5 <= t < 11.0 or (3.0 <= t < 3.5 and not is_f):
                continue                       # dropped in the full ring, or no S while stalled
            out.append(line)
            if is_f:
                kept.append(tr)
        out.insert(3, "S %08X 1 0 0 0" % u0)    # no @epoch: not an anchor, just counted
        p, got = self.convert("\n".join(out) + "\n")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("1 timeline(s)", p.stderr)
        self.assertIn("1 S without @epoch", p.stderr)
        self.assert_matches(got, kept)

    def test_clock_step_warned(self):
        rng = random.Random(16)
        text, _truth = recording(rng, 1791000000.0, 0x40000000, 25.0, step=(12.0, 3600.0))
        p, _got = self.convert(text)
        self.assertIn("wall-clock step", p.stderr)


if __name__ == "__main__":
    unittest.main()
