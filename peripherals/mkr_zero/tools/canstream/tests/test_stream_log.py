"""mkr_stream_log.py, driven through a pty by a synthetic MKR (streamsim.py).

Each class plays one scenario in real time — the logger's timing logic runs on
the real monotonic clock, so it cannot be fast-forwarded — then checks what the
logger wrote against the scenario's truth:
  - can_raw.log parses back EXACTLY: every frame, in order, ids, flags, DLC,
    data; nothing extra (malformed lines, console text) and nothing missing;
  - every timestamp is within tolerance of the frame's TRUE host time (the
    moment the synthetic drain read it), however late its line was delivered;
  - can_stats.csv, can_sync.csv and mkr_console.txt say what happened.

Tolerances: the logger maps MKR time to host time through the minimum delivery
delay, so its error is that floor (pty + thread wake-up, well under 1 ms on an
idle Jetson) plus scheduling noise; anything early is a bug. TOL_MAX is loose
enough for a busy machine, TOL_P99 is where a correct mapping sits.
"""
import os
import re
import resource
import shutil
import signal
import statistics
import tempfile
import time
import unittest

from streamsim import WRAP, Logger, Pty, build, console_body, play, read_candump, read_csv

TOL_EARLY = -0.0015
TOL_P99 = 0.006
TOL_MAX = 0.025


def run_scenario(sch, patch=None, seconds=600):
    tmp = tempfile.mkdtemp(prefix="canstream_test_")
    port = os.path.join(tmp, "ttyMKR")
    out = os.path.join(tmp, "out")
    pty = Pty(port)
    lg = Logger(out, port, seconds=seconds, patch=patch)
    try:
        if not lg.wait_open():
            raise AssertionError("logger never opened the port:\n" + lg.console_text())
        base = play(sch, pty, lg)
        time.sleep(0.3)
    finally:
        rc = lg.stop()
        pty_blocked, pty_worst = pty.blocked, pty.worst_block
        pty.close()
    e_m = lg.K                  # the logger's wall clock is monotonic + K: see streamsim.Logger
    with open(out + ".stderr") as f:
        stderr = f.read()
    return dict(tmp=tmp, out=out, base=base, rc=rc, e_m=e_m, blocked=pty_blocked, worst_block=pty_worst,
                stderr=stderr, console=lg.console_text())


class ScenarioCase(unittest.TestCase):
    """Common checks; subclasses build `sch` and may pass a patch to the logger."""

    sch = None
    r = None
    patch = None

    @classmethod
    def tearDownClass(cls):
        if cls.r and os.environ.get("CANSTREAM_KEEP") is None:
            shutil.rmtree(cls.r["tmp"], ignore_errors=True)

    def frames(self):
        return read_candump(os.path.join(self.r["out"], "can_raw.log"))

    def assert_frames_exact(self):
        got = self.frames()
        want = self.sch.frames
        for i, (g, f) in enumerate(zip(got, want)):
            if g[2:] != f.key():
                self.fail("frame %d differs: logged %r, sent %r" % (i, g[2:], f.key()))
        self.assertEqual(len(got), len(want), "frame count")
        self.assertTrue(all(g[1] == "can0" for g in got))
        return got

    def errors(self, got=None, shift=None):
        """Logged host time minus true host time, per frame (seconds)."""
        got = got or self.frames()
        base, e_m = self.r["base"], self.r["e_m"]
        shift = shift or (lambda i: 0.0)
        return [(g[0] / 1e6 - e_m - shift(i)) - (base + f.s) for i, (g, f) in enumerate(zip(got, self.sch.frames))]

    def assert_timestamps(self, errs, label=""):
        errs_sorted = sorted(errs)
        p50 = errs_sorted[len(errs) // 2]
        p99 = errs_sorted[int(len(errs) * 0.99)]
        msg = "%s timestamp error ms: min %.3f p50 %.3f p99 %.3f max %.3f over %d frames" % (
            label, errs_sorted[0] * 1e3, p50 * 1e3, p99 * 1e3, errs_sorted[-1] * 1e3, len(errs))
        print("\n    " + msg)
        self.assertGreaterEqual(errs_sorted[0], TOL_EARLY, msg)
        self.assertLessEqual(p99, TOL_P99, msg)
        self.assertLessEqual(errs_sorted[-1], TOL_MAX, msg)

    def sync_rows(self):
        return read_csv(os.path.join(self.r["out"], "can_sync.csv"))

    def new_timelines(self):
        return [row["event"] for row in self.sync_rows() if row["event"].startswith("new timeline")]

    def assert_clean_exit(self):
        self.assertEqual(self.r["rc"], 0, self.r["stderr"])
        self.assertFalse(os.path.exists(os.path.join(self.r["out"], "mkr_stream_log.err")), self.r["stderr"])


class TestMainStream(ScenarioCase):
    """24 s at 600 frames/s: every frame shape, a micros() wrap 3 s in with the FS just after it ahead of
    older frames, latency jitter and 0.03-0.25 s ring waits, a burst of a full second of frames delivered
    late behind an FS line, lines split across reads, console lines with LF and CRLF, malformed lines."""

    @classmethod
    def setUpClass(cls):
        cls.sch = build(seed=1, duration=24.0, rate=600.0, start_us=WRAP - 3_000_000 + 20_000,
                        burst=(10.0, 11.0), wrap_hold=0.06, malformed_at=[1.3 + 1.37 * i for i in range(15)])
        cls.r = run_scenario(cls.sch)

    def test_clean_exit(self):
        self.assert_clean_exit()

    def test_frames_exact(self):
        got = self.assert_frames_exact()
        kinds = {(f.ext, f.rtr, f.dlc == 0) for f in self.sch.frames}
        self.assertTrue({(True, False, False), (False, True, False), (True, True, False),
                         (False, False, True)} <= kinds, "the scenario must carry ext, RTR and DLC 0 frames")
        self.assertGreater(len(got), 10000)

    def test_timestamps(self):
        self.assert_timestamps(self.errors(), "all")

    def test_late_burst_timed_by_drain_not_delivery(self):
        errs = self.errors()
        burst = [e for e, f in zip(errs, self.sch.frames) if 10.0 <= f.s < 11.0]
        self.assertGreater(len(burst), 300)
        self.assert_timestamps(burst, "burst (delivered up to 1 s late)")

    def test_burst_does_not_move_offset(self):
        base = self.r["base"]
        rows = [(float(r["host_mono"]) - base, float(r["offset_s"])) for r in self.sync_rows() if r["event"] == "offset"]
        before = [o for s, o in rows if s < 9.5]
        self.assertTrue(before, "no offset before the burst")
        during = [o for s, o in rows if 9.5 <= s <= 14.0]
        for o in during:
            self.assertLess(abs(o - before[-1]), 0.001, "offset moved by %.3f ms around the burst" % ((o - before[-1]) * 1e3))

    def test_wrap_does_not_jump(self):
        got = self.frames()
        ts = [g[0] for g in got]
        steps = [b - a for a, b in zip(ts, ts[1:])]
        self.assertLess(max(steps), 500000, "a gap of %.1f s: a wrap misplaced" % (max(steps) / 1e6))
        self.assertGreater(min(steps), -2000, "frames out of order by %.1f ms" % (-min(steps) / 1e3))
        wrapped = [f for f in self.sch.frames if f.us < 1_000_000]
        self.assertTrue(wrapped, "the scenario must cross the wrap")
        # The FS just after the wrap did arrive ahead of frames read before it.
        fs_after = [s for s, us, _, _ in self.sch.fs if us < 100_000]
        self.assertTrue(fs_after)
        older_after = [f for f in self.sch.frames if f.us > WRAP - 100_000 and f.s < fs_after[0]]
        self.assertTrue(older_after)

    def test_stats_rows(self):
        rows = read_csv(os.path.join(self.r["out"], "can_stats.csv"))
        self.assertEqual(len(rows), len(self.sch.fs))
        for row, (s, us, counters, _k) in zip(rows, self.sch.fs):
            self.assertEqual(int(row["mkr_us"]), us)
            self.assertEqual([int(row[k]) for k in ("drained", "streamed", "ovf", "ringdrop", "nohost")], counters)
            self.assertEqual(row["timeline"], "0")
        s0, m0 = self.sch.fs[0][0], float(rows[0]["mkr_s"])
        for row, (s, *_rest) in zip(rows, self.sch.fs):
            self.assertAlmostEqual(float(row["mkr_s"]) - m0, s - s0, delta=2e-6)   # unwrapped across the wrap

    def test_console(self):
        self.assertEqual(console_body(self.r["console"]), self.sch.console)
        # Half the console lines came CRLF (println): the CR goes, or every line of the file ends in one —
        # which grep '...$' would notice, and neither splitlines() nor a text-mode read (universal newlines).
        with open(os.path.join(self.r["out"], "mkr_console.txt"), "rb") as f:
            self.assertNotIn(b"\r", f.read())

    def test_malformed_counted_not_logged(self):
        notes = [l for l in self.r["console"].splitlines() if l.startswith("#") and " malformed: " in l]
        self.assertEqual(len(notes), self.sch.malformed)
        self.assertIn("%d malformed" % self.sch.malformed, self.r["console"])

    def test_single_timeline(self):
        self.assertEqual(self.new_timelines(), [])


class TestStall(ScenarioCase):
    """Nothing is delivered for 6 s; the ring keeps the oldest 1024 frames and drops the rest, and the
    first packet afterwards leads with a fresh FS line followed by frames 4-6 s OLDER than it. That is a
    stall, not a reset: frames stay in drain order."""

    @classmethod
    def setUpClass(cls):
        cls.sch = build(seed=2, duration=14.0, rate=600.0, start_us=0x12345678, stall=(4.0, 10.0),
                        console_rate=0.5, split_frac=0.0)
        cls.r = run_scenario(cls.sch)

    def test_no_new_timeline(self):
        self.assertEqual(self.new_timelines(), [])

    def test_frames_exact(self):
        self.assert_frames_exact()

    def test_timestamps(self):
        errs = self.errors()
        self.assert_timestamps(errs, "all")
        stale = [e for e, f in zip(errs, self.sch.frames) if 4.0 <= f.s < 10.0]
        self.assertGreater(len(stale), 1000)
        self.assert_timestamps(stale, "stale ring frames (4-6 s behind the FS)")


class TestResets(ScenarioCase):
    """Four MKR resets, each detected once, each a new timeline with its own offset:
       4 s  after >35.8 min of uptime, no banner: micros() reads as jumping AHEAD modulo 2^32
       8 s  banner on the same port; the new boot's first lines 0.15 s before the micros() wrap
      12 s  no banner, micros() backwards on the same port
      16 s  the port is lost and a new one appears (every real reset); its banner must not add one more."""

    @classmethod
    def setUpClass(cls):
        events = [(4.0, "ahead", 2_000_000), (8.0, "banner", WRAP - 1_650_000), (12.0, "back", 0),
                  (16.0, "hangup", 2_500_000)]
        cls.sch = build(seed=3, duration=20.0, rate=500.0, start_us=0xC0000000, events=events,
                        malformed_at=[2.2, 6.6, 10.4, 14.3, 18.7])
        cls.r = run_scenario(cls.sch)

    def test_clean_exit(self):
        self.assert_clean_exit()

    def test_one_timeline_per_reset(self):
        tl = self.new_timelines()
        self.assertEqual(len(tl), 4, tl)
        self.assertIn("ahead", tl[0])
        self.assertIn("MKR boot banner", tl[1])
        self.assertIn("backwards", tl[2])
        self.assertIn("port reopened", tl[3])

    def test_frames_exact(self):
        self.assert_frames_exact()

    def test_timestamps(self):
        self.assert_timestamps(self.errors(), "all timelines")

    def test_stats_follow_timelines(self):
        rows = read_csv(os.path.join(self.r["out"], "can_stats.csv"))
        self.assertEqual(len(rows), len(self.sch.fs))
        for row, (s, us, counters, k) in zip(rows, self.sch.fs):
            self.assertEqual(int(row["timeline"]), k)
            self.assertEqual(int(row["mkr_us"]), us)
        # Timeline 2 began 0.15 s before the wrap: unwrapped, its FS lines are exactly 1 s apart.
        t2 = [(s, float(row["mkr_s"])) for row, (s, _us, _c, k) in zip(rows, self.sch.fs) if k == 2]
        self.assertGreaterEqual(len(t2), 2)
        for (s0, m0), (s1, m1) in zip(t2, t2[1:]):
            self.assertAlmostEqual(m1 - m0, s1 - s0, delta=2e-6)

    def test_console_and_port_notes(self):
        self.assertEqual(console_body(self.r["console"]), self.sch.console)
        self.assertEqual(self.r["console"].count("port lost"), 1)
        self.assertEqual(self.r["console"].count("port open"), 2)


class TestClockStep(ScenarioCase):
    """The wall clock steps +1 h mid-run (v0.4 setting it from GPS). Frames written after the step carry
    the new time, frames written before it the old; written = arrived + HOLD_S, so the switch lands on
    frames that arrived up to 2 s before the step. can_sync.csv and the console record it once."""

    patch = {"clock_step": [5.0, 3600.0]}

    @classmethod
    def setUpClass(cls):
        cls.sch = build(seed=4, duration=10.0, rate=500.0, start_us=0x00ABCDEF)
        cls.r = run_scenario(cls.sch, patch=cls.patch)

    def test_clean_exit(self):
        self.assert_clean_exit()

    def test_step_recorded_once(self):
        steps = [r for r in self.sync_rows() if r["event"].startswith("clock step")]
        self.assertEqual(len(steps), 1, steps)
        self.assertAlmostEqual(float(steps[0]["event"].split()[2]), 3600.0, delta=0.01)
        self.assertIn("wall clock stepped +3600", self.r["console"])

    def test_frames_on_one_side_or_the_other(self):
        got = self.assert_frames_exact()
        step_mono = float([r for r in self.sync_rows() if r["event"].startswith("clock step")][0]["host_mono"])
        errs = self.errors(got)
        shifted = [abs(e - 3600.0) < 0.1 for e in errs]
        # Exactly two populations, unshifted then shifted, with nothing in between.
        first = shifted.index(True)
        self.assertTrue(all(shifted[first:]), "a frame after the switch carries the old clock")
        self.assertFalse(any(shifted[:first]))
        self.assert_timestamps([e - 3600.0 if s else e for e, s in zip(errs, shifted)], "both clocks")
        # The switch is by WRITE time: the first shifted frame arrived within HOLD_S (+ margin) of the step.
        base = self.r["base"]
        arrive_first = base + self.sch.frames[first].s
        self.assertGreater(arrive_first, step_mono - 2.0 - 0.6)
        self.assertLess(arrive_first, step_mono + 0.3)

    def test_stats_epoch_steps(self):
        rows = read_csv(os.path.join(self.r["out"], "can_stats.csv"))
        d = [float(r["host_epoch"]) - float(r["host_mono"]) for r in rows]
        jumps = [b - a for a, b in zip(d, d[1:]) if abs(b - a) > 1.0]
        self.assertEqual(len(jumps), 1)
        self.assertAlmostEqual(jumps[0], 3600.0, delta=0.01)


class TestDrift(ScenarioCase):
    """The MKR's crystal 100 ppm SLOW against the host clock — the costly direction: the windowed minimum is
    then its oldest sample, so frames map early by up to 100 ppm x 20 s = 2 ms. Bounded, and documented in
    the logger; a fast crystal costs only 100 ppm x HOLD_S."""

    @classmethod
    def setUpClass(cls):
        cls.sch = build(seed=6, duration=26.0, rate=400.0, start_us=0x01000000, ppm=-100.0, console_rate=0.5)
        cls.r = run_scenario(cls.sch)

    def test_bias_within_drift_bound(self):
        self.assert_frames_exact()
        errs = self.errors()
        late = [e for e, f in zip(errs, self.sch.frames) if f.s >= 21.0]   # the window is full by now
        lo, hi = min(errs), max(errs)
        print("\n    drift -100 ppm: error ms min %.3f max %.3f; after 21 s min %.3f" % (lo * 1e3, hi * 1e3, min(late) * 1e3))
        self.assertGreaterEqual(lo, -0.0020 - 0.0005)
        self.assertLessEqual(hi, TOL_MAX)
        self.assertLess(min(late), -0.0012, "the bias should have built up to about -2 ms")


class TestSlowDisk(ScenarioCase):
    """The first fsync batch takes 6 s (two 3 s calls): an SD card or USB disk stalling. The port must
    keep being read meanwhile, or cdc-acm throttles and the MKR's ring overflows; here the pty would fill
    and the synthetic MKR's writes would block."""

    patch = {"fsync_delay": 3.0, "fsync_slow_calls": 2}

    @classmethod
    def setUpClass(cls):
        cls.sch = build(seed=5, duration=14.0, rate=600.0, start_us=0x7FFF0000)
        cls.r = run_scenario(cls.sch, patch=cls.patch)

    def test_reading_never_stalls(self):
        self.assertLess(self.r["worst_block"], 0.5, "the MKR side blocked %.2f s (total %.2f s)"
                        % (self.r["worst_block"], self.r["blocked"]))

    def test_frames_exact_and_timed(self):
        self.assert_frames_exact()
        self.assert_timestamps(self.errors(), "through a 6 s disk stall")

    def test_clean_exit(self):
        self.assert_clean_exit()


class LiveStream:
    """A hand-driven synthetic MKR for the scenarios below, which turn on WHEN the logger reads, not on
    what the firmware writes: frames at a fixed rate, an FS line every second, micros() = the host's
    clock + us0 (no drift), so every frame's true host time is known exactly. Each FS line's `streamed`
    counts every frame line the MKR handed to USB, including ones the logger never receives."""

    PERIOD = 0.002

    def __init__(self, test, us0, streamed0=0, preseed=None, **logger_kw):
        """@preseed: text already in OUTDIR/can_raw.log when the logger starts (it appends)."""
        self.test = test
        self.tmp = tempfile.mkdtemp(prefix="canstream_live_")
        self.port = os.path.join(self.tmp, "ttyMKR")
        self.out = os.path.join(self.tmp, "out")
        if preseed is not None:
            os.makedirs(self.out)
            with open(os.path.join(self.out, "can_raw.log"), "w") as f:
                f.write(preseed)
        self.pty = Pty(self.port)
        self.lg = Logger(self.out, self.port, **logger_kw)
        if not self.lg.wait_open():
            raise AssertionError("logger never opened the port:\n" + self.lg.console_text())
        self.base = time.monotonic()
        self.us0 = us0
        self.streamed = streamed0
        self.truth = []          # (true host_mono, id, data) per frame line written, in stream order
        self.t = 0.0             # sim seconds since base: the next frame's drain time

    def us(self, t):
        return (self.us0 + int(round(t * 1e6))) % WRAP

    def frame(self, t, ident=0x17C):
        data = "%016X" % len(self.truth)           # a sequence number: every line is distinguishable
        self.truth.append((self.base + t, ident, data))
        self.streamed += 1
        return "F %08X %03X 8 %s\n" % (self.us(t), ident, data)

    def fs(self, t):
        return "FS %08X %d %d 0 0 0\n" % (self.us(t), self.streamed + 7, self.streamed)

    def run(self, until, write=None):
        """Streams in real time up to sim time @until. @write(bytes) -> bool (False: the bank is armed,
        nothing more is written until it returns True again; frames meanwhile are ringdrop)."""
        write = write or (lambda b: self.pty.write(b) or True)
        while self.t < until:
            now = time.monotonic() - self.base
            if now < self.t:
                time.sleep(self.t - now)
            pkt = self.fs(self.t) if abs(self.t - round(self.t)) < 1e-9 else ""
            n_before, s_before = len(self.truth), self.streamed
            pkt += self.frame(self.t)
            if not write(pkt.encode()):
                del self.truth[n_before:]
                self.streamed = s_before
            self.t = round(self.t + self.PERIOD, 6)

    def finish(self):
        time.sleep(0.3)
        rc = self.lg.stop()
        self.pty.close()
        got = read_candump(os.path.join(self.out, "can_raw.log"))
        self.test.assertEqual(rc, 0)
        self.test.assertEqual([(g[2], g[6]) for g in got], [(i, d) for _t, i, d in self.truth],
                              "every frame line logged once, in order")
        return got

    def errors(self, got):
        return [(g[0] / 1e6 - self.lg.K) - truth for g, (truth, _i, _d) in zip(got, self.truth)]

    def sync_events(self):
        return [r["event"] for r in read_csv(os.path.join(self.out, "can_sync.csv"))]

    def console(self):
        return self.lg.console_text()

    def cleanup(self):
        if os.environ.get("CANSTREAM_KEEP") is None:
            shutil.rmtree(self.tmp, ignore_errors=True)


class TestRestartAgainstRunningMkr(unittest.TestCase):
    """This logger restarted while the MKR kept running: the MKR's bulk-IN bank still holds the last packet
    written before the previous reader closed the port, and the open receives it first — an FS line and a
    frame line from long before. Neither is a new MKR boot, neither may set the offset, and the frames the
    previous reader left unread may not count as lost here. Twice: 30 s stale (the frame is then placed on
    the same timeline and timed exactly) and 40 min stale (past half the micros() period, so the fresh lines
    read as going backwards: the timeline is re-seeded, not split, and the stale frame keeps its arrival)."""

    def stale_then_live(self, stale_s):
        ls = LiveStream(self, us0=0x50000000)
        try:
            # The previous reader got 900 frames, then 100 more after this FS line, then closed with the
            # packet below still in the bank.
            ls.streamed = 900
            pkt = ls.fs(-stale_s)
            ls.streamed += 100
            pkt += ls.frame(-stale_s - 0.0005, ident=0x158)
            ls.pty.write(pkt.encode())
            time.sleep(0.01)
            ls.t = 0.05
            ls.run(4.0)
            got = ls.finish()
            return ls, got
        except BaseException:
            ls.cleanup()
            raise

    def test_30_s_stale_packet(self):
        ls, got = self.stale_then_live(30.0)
        try:
            self.assertEqual([e for e in ls.sync_events() if "timeline" in e or "stale" in e], [])
            errs = ls.errors(got)
            self.assertLess(abs(errs[0]), 0.002, "stale frame timed %.3f s off" % errs[0])
            self.assertLess(max(abs(e) for e in errs[1:]), TOL_MAX)
            self.assertIn(" 0 not received here", ls.console())
        finally:
            ls.cleanup()

    def test_40_min_stale_packet(self):
        ls, got = self.stale_then_live(2400.0)
        try:
            self.assertEqual([e for e in ls.sync_events() if "timeline" in e], [])
            self.assertEqual(len([e for e in ls.sync_events() if e.startswith("stale first packet")]), 1)
            self.assertIn("first packet after the port opened was stale", ls.console())
            errs = ls.errors(got)
            self.assertLess(abs(errs[0] - 2400.0), 0.1, "the stale frame keeps its arrival time")
            self.assertLess(max(abs(e) for e in errs[1:]), TOL_MAX)
            self.assertIn(" 0 not received here", ls.console())
        finally:
            ls.cleanup()


class TestReaderFrozen(unittest.TestCase):
    """The logger frozen for 25 s (SIGSTOP; a stalled bus behaves alike) — longer than the offset window.
    The pty fills with lines from the start of the freeze and then holds the writer back, as the kernel's
    tty buffer and an armed bank hold the MKR back. On resume the first reads carry data ~20 s older than
    their arrival: alone in an emptied window that would become the offset, map the next fresh line 20 s
    into the future and split the timeline, and time the buffered frames 20 s late."""

    def test_freeze_longer_than_window(self):
        ls = LiveStream(self, us0=0x01234567)
        try:
            ls.run(4.0)
            os.kill(ls.lg.p.pid, signal.SIGSTOP)
            carry = [None]

            def frozen_write(b):
                if carry[0] is not None:
                    return False                     # bank armed: nothing more until the host reads
                try:
                    n = os.write(ls.pty.master, b)
                except BlockingIOError:
                    n = 0
                if n < len(b):
                    carry[0] = b[n:]                 # the armed packet, collected whole on resume
                return True

            try:
                ls.run(29.0, frozen_write)
            finally:
                os.kill(ls.lg.p.pid, signal.SIGCONT)
            self.assertIsNotNone(carry[0], "the pty never filled: the freeze tested nothing")
            ls.pty.write(carry[0])
            ls.run(34.0)
            got = ls.finish()
            self.assertEqual([e for e in ls.sync_events() if "timeline" in e], [])
            errs = ls.errors(got)
            buffered = [e for e, (t, _i, _d) in zip(errs, ls.truth) if 4.0 <= t - ls.base < 29.0]
            self.assertGreater(len(buffered), 500)
            worst = max(errs, key=abs)
            self.assertLess(abs(worst), TOL_MAX, "a frame timed %.3f s off" % worst)
        finally:
            ls.cleanup()


class TestDiskFull(unittest.TestCase):
    """can_raw.log stops taking writes for 4 s (RLIMIT_FSIZE: EFBIG, as ENOSPC would on a full backup disk),
    then takes them again. The logger must neither die (nothing restarts it in a drive session) nor stop
    reading the port, must keep the file parseable line by line, must resume writing, and must say exactly
    how much it lost. Before the fix: one traceback at the first EFBIG and nothing more for the session."""

    LIMIT = 100_000

    def test_survives_and_reports_the_gap(self):
        # A previous run's frames already in the file (an append): the loss count must not take them in.
        before = "".join("(1789999000.%06d) can0 7FF#\n" % i for i in range(100))
        ls = LiveStream(self, us0=0x0ABCDEF0, preseed=before, fsize=self.LIMIT)
        try:
            raw = os.path.join(ls.out, "can_raw.log")
            end = 15.0
            while os.path.getsize(raw) < self.LIMIT - 100 and ls.t < end:
                ls.run(ls.t + 0.5)
            self.assertLess(ls.t, end, "can_raw.log never reached the limit")
            ls.run(ls.t + 4.0)
            self.assertIsNone(ls.lg.p.poll(), "the logger died at the full disk:\n" + ls.console())
            resource.prlimit(ls.lg.p.pid, resource.RLIMIT_FSIZE, (resource.RLIM_INFINITY, resource.RLIM_INFINITY))
            ls.run(ls.t + 5.0)
            self.assertIn("output writes failed for", ls.console(), "recovery reported while running, not at exit")
            n_sent = len(ls.truth)
            time.sleep(HOLD_WAIT)
            rc = ls.lg.stop()
            ls.pty.close()
            self.assertEqual(rc, 0)
            self.assertFalse(os.path.exists(os.path.join(ls.out, "mkr_stream_log.err")))
            self.assertLess(ls.pty.worst_block, 0.5, "the logger stopped reading while it could not write")
            got = read_candump(raw)                         # raises on any torn or merged line
            self.assertEqual([g[2] for g in got[:100]], [0x7FF] * 100, "the earlier run's lines kept")
            got = got[100:]
            seqs = [int(g[6], 16) for g in got]
            self.assertEqual(seqs, sorted(set(seqs)), "frames out of order or duplicated")
            gaps = [(a, b) for a, b in zip(seqs, seqs[1:]) if b != a + 1]
            self.assertEqual(len(gaps), 1, "one gap expected, got %r" % gaps)
            self.assertEqual((seqs[0], seqs[-1]), (0, n_sent - 1), "frames missing at the start or the end")
            lost = gaps[0][1] - gaps[0][0] - 1
            self.assertGreater(lost, 500)
            m = re.search(r"output writes failed for [0-9.]+ s \(first: can_raw.log: .*File too large.*\); "
                          r"can_raw.log lost (\d+) bytes, about (\d+) frame lines", ls.console())
            self.assertIsNotNone(m, ls.console()[-2000:])
            with open(raw) as f:
                line_len = len(f.readlines()[100])          # every line of this run is the same length
            self.assertEqual(int(m.group(1)), lost * line_len, "lost bytes reported")
            self.assertEqual(int(m.group(2)), lost)
            self.assertTrue(any(e.startswith("output writes failed") for e in ls.sync_events()))
            with open(ls.out + ".stderr") as f:
                self.assertIn("frames are dropped until it can write again", f.read())
            errs = [(g[0] / 1e6 - ls.lg.K) - ls.truth[q][0] for g, q in zip(got, seqs)]
            self.assertLess(max(abs(e) for e in errs), TOL_MAX)
        finally:
            ls.cleanup()


HOLD_WAIT = 2.5     # past the logger's 2 s hold, so every frame read has been handed to can_raw.log


if __name__ == "__main__":
    unittest.main()
