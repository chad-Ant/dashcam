"""mkr_stream_log.py's pieces that need no clock: the port setup and the line/unwrap rules.

Fast, in-process, on a pty slave standing in for /dev/ttyACM0.
"""
import bisect
import importlib.util
import os
import queue
import random
import re
import shutil
import tempfile
import termios
import threading
import time as time_mod
import tty
import unittest

from streamsim import LOGGER, WRAP, Pty, console_body, read_candump, read_csv

spec = importlib.util.spec_from_file_location("msl_under_test", LOGGER)
msl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(msl)


class TestOpenPort(unittest.TestCase):
    def setUp(self):
        self.master, s = os.openpty()
        tty.setraw(s)
        self.path = os.ttyname(s)
        os.close(s)

    def tearDown(self):
        os.close(self.master)

    def test_raw_hupcl_vmin(self):
        fd = msl.open_port(self.path)
        try:
            a = termios.tcgetattr(fd)
            # HUPCL: DTR drops when the logger exits, so the MKR counts 'nohost' instead of filling its ring.
            self.assertTrue(a[2] & termios.HUPCL)
            self.assertTrue(a[2] & termios.CLOCAL)
            self.assertEqual(a[3], 0, "no echo, no canonical mode")
            self.assertEqual(a[6][termios.VMIN], 1)
        finally:
            os.close(fd)

    def test_exclusive(self):
        fd = msl.open_port(self.path)
        try:
            with self.assertRaises(OSError):
                msl.open_port(self.path)          # a second reader would take a share of the lines
        finally:
            os.close(fd)
        # Not re-opened here: on a pty the TIOCEXCL flag outlives the fd, because the slave tty lives as
        # long as its master. A ttyACM's tty is released at its last close, flag and all.


class TestParse(unittest.TestCase):
    def test_frame_shapes(self):
        p = msl.parse_frame
        self.assertEqual(p(b"F 0001ABCD 158 8 123456789ABCDEF0"), (0x1ABCD, 0x158, False, False, 8, "123456789ABCDEF0"))
        self.assertEqual(p(b"F 00000000 7FF 0"), (0, 0x7FF, False, False, 0, ""))
        self.assertEqual(p(b"F 00C0FFEE 18DAF110 2 1234"), (0xC0FFEE, 0x18DAF110, True, False, 2, "1234"))
        self.assertEqual(p(b"F 00000010 123 R 4"), (0x10, 0x123, False, True, 4, ""))
        self.assertEqual(p(b"F 00000010 00ABCDEF R 0"), (0x10, 0xABCDEF, True, True, 0, ""))
        for bad in (b"F 00000000 7FF 0 ", b"F 00000000 800 0", b"F 0000000 7FF 0", b"F 00000000 7ff 0",
                    b"F 00000000 20000000 0", b"F 00000000 123 9 00", b"F 00000000 123 2 001",
                    b"F 00000000 123 R 9", b"F 00000000 123 R", b"F 00000000 1234 1 00", b"F 00000000 123 1 00 ",
                    b"F 00000000 0123 1 00", b"F 00000000 0000123 1 00"):   # 4 / 7 digits: neither width
            self.assertIsNone(p(bad), bad)

    def test_stats_shapes(self):
        p = msl.parse_stats
        self.assertEqual(p(b"FS FFFFFFFF 4294967295 0 1 22 333"), (0xFFFFFFFF, [4294967295, 0, 1, 22, 333]))
        for bad in (b"FS FFFFFFFF 4294967296 0 1 22 333", b"FS FFFFFFFF 1 2 3 4", b"FS FFFFFFFF 1 2 3 4 5 6",
                    b"FS FFFFFFFF 01234567890 1 2 3 4", b"FS FFFFFFFF  1 2 3 4 5"):
            self.assertIsNone(p(bad), bad)


class TestUnwrap(unittest.TestCase):
    def test_wrap_with_older_frames_after_fs(self):
        tl = msl.Timeline()
        self.assertEqual(tl.unwrap(WRAP - 1000, True), WRAP - 1000)
        self.assertEqual(tl.unwrap(500, False), WRAP + 500)             # FS just past the wrap
        self.assertEqual(tl.unwrap(WRAP - 800, True), WRAP - 800)       # an older frame after it
        self.assertEqual(tl.unwrap(700, True), WRAP + 700)

    def test_stale_frames_after_a_stall_are_not_a_reset(self):
        tl = msl.Timeline()
        tl.unwrap(1_000_000, True)
        tl.unwrap(30_000_000, False)                                     # FS after a 29 s stall
        self.assertEqual(tl.unwrap(1_000_100, True), 1_000_100)          # the ring's oldest frame

    def test_resets(self):
        tl = msl.Timeline()
        tl.unwrap(5_000_000, True)
        self.assertIsNone(tl.unwrap(4_800_000, True))                    # frame behind a frame: reset
        tl = msl.Timeline()
        tl.unwrap(5_000_000, True)
        tl.unwrap(5_000_100, False)
        self.assertIsNone(tl.unwrap(4_000_000, False))                   # FS behind a frame: reset
        # An FS line is formatted after every frame already streamed was drained, so behind the last
        # FRAME is a reset too — with no FS before it, and with an older FS before that frame.
        tl = msl.Timeline()
        tl.unwrap(5_000_000, True)
        self.assertIsNone(tl.unwrap(3_000_000, False))
        tl = msl.Timeline()
        tl.unwrap(1_000_000, False)
        tl.unwrap(5_000_000, True)
        self.assertIsNone(tl.unwrap(3_000_000, False))
        tl = msl.Timeline()
        tl.unwrap(5_000_000, True)
        tl.observe(100.0, 5.0)
        self.assertIsNone(tl.unwrap(5_000_000 + 60_000_000, True, host_mono=100.0))   # a minute in the future
        self.assertIn("ahead", tl.why)

    def test_offset_is_windowed_minimum(self):
        tl = msl.Timeline()
        tl.observe(10.0, 9.0)          # candidate 1.0
        tl.observe(11.0, 10.5)         # 0.5
        tl.observe(12.0, 10.0)         # 2.0: a late chunk cannot raise it
        self.assertEqual(tl.offset, 0.5)
        tl.observe(11.0 + msl.WINDOW_S + 0.1, 30.0)   # the 0.5 sample has aged out
        self.assertAlmostEqual(tl.offset, 11.0 + msl.WINDOW_S + 0.1 - 30.0)

    def test_silence_longer_than_window_keeps_the_minimum(self):
        tl = msl.Timeline()
        tl.observe(10.0, 9.0)                          # offset 1.0
        tl.observe(11.0, 10.0)
        # 25 s of nothing, then a read of data written at the start of the silence (candidate 21.0): carried.
        self.assertFalse(tl.observe(36.0, 15.0))
        self.assertEqual(tl.offset, 1.0)
        tl.observe(36.5, 35.0)                         # still stale, but the carried minimum is fresh now
        self.assertEqual(tl.offset, 1.0)
        tl.observe(37.0, 36.0 - 0.0002)                # a fresh line: 0.2 ms of drift, accepted from here on
        tl.observe(37.0 + msl.WINDOW_S + 0.5, 37.0 + msl.WINDOW_S + 0.5 - 1.0002)
        self.assertAlmostEqual(tl.offset, 1.0002)
        # Drift in a long silence is not stale data: 30 ms over 30 s (1000 ppm) is taken as it comes.
        tl = msl.Timeline()
        tl.observe(10.0, 9.0)
        tl.observe(40.0, 39.0 - 0.030)
        self.assertAlmostEqual(tl.offset, 1.030)


class TestReadSample(unittest.TestCase):
    """One read is one delivery sample, taken with the NEWEST MKR time in it: every line of the read had
    arrived by then, and the newest — usually the FS line leading its packet — waited least."""

    def test_sample_is_the_newest_line_of_the_read(self):
        out = tempfile.mkdtemp(prefix="canstream_sample_")
        try:
            lg = msl.Logger(out, "can0")
            lg.port_open("/dev/null", None)
            lg.data(99.0, b"F 00895440 17C 8 0011223344556677\n")          # first read after the open: no sample
            self.assertIsNone(lg.tl.offset)
            # FS stamped 10.000000 s as written, then a frame drained 50 ms earlier that waited in the ring.
            lg.data(100.0, b"FS 00989680 2 2 0 0 0\nF 0097D330 17C 8 0011223344556677\n")
            self.assertAlmostEqual(lg.tl.offset, 90.0, places=6)
            lg.close()
        finally:
            shutil.rmtree(out, ignore_errors=True)


class TestLossAccounting(unittest.TestCase):
    """rx_frames against the change in `streamed`, FS line to FS line — within ONE boot only. A new boot
    restarts the MKR's counters at 0; carried across, the first FS of the new boot would be measured
    against the old boot's count and book ~2^32 frames as lost."""

    def test_new_boot_restarts_the_accounting(self):
        out = tempfile.mkdtemp(prefix="canstream_loss_")
        try:
            lg = msl.Logger(out, "can0")
            lg.port_open("/dev/null", None)
            f = b"F %08X 17C 8 0011223344556677\n"
            lg.data(50.0, b"MKR Zero telemetry master\n")              # the first read since the open
            lg.data(51.0, b"FS 05F5E100 1000 1000 0 0 0\n" + f % 0x05F5E200 + f % 0x05F5E300)
            lg.data(52.0, f % 0x05F5E400 + b"FS 05F5E500 1003 1003 0 0 0\n")
            self.assertEqual((lg.usb_checked, lg.usb_lost), (3, 0))
            # Unreported reset (banner on the same port), then one shown only by micros() going back 0.9 s.
            for banner, u in ((b"BOOT: reset cause 0x40 - external reset\n", 0x00100000), (b"", 0x00020000)):
                lg.data(53.0, banner + f % u + f % (u + 0x100))
                lg.data(54.0, b"FS %08X 2 2 0 0 0\n" % (u + 0x200) + f % (u + 0x300))
                lg.data(55.0, b"FS %08X 3 3 0 0 0\n" % (u + 0x400))
                self.assertEqual((lg.usb_checked, lg.usb_lost), (3 + (1 if banner else 2), 0))
            self.assertEqual(lg.timeline_no, 2)
            lg.close()
        finally:
            shutil.rmtree(out, ignore_errors=True)


class TestCheckClock(unittest.TestCase):
    """Logger.check_clock() on a scripted clock: a preemption between its two clock reads is not a step,
    a slew under STEP_S is followed silently, a step is recorded once."""

    class Clock:
        K = 1790000000.0

        def __init__(self):
            self.now, self.wall_off, self.preempt = 100.0, 0.0, 0.0

        def monotonic(self):
            return self.now

        def time(self):
            w = self.K + self.now + self.wall_off
            self.now += self.preempt          # the scheduler takes the CPU right after this read
            self.preempt = 0.0
            return w

        strftime = staticmethod(time_mod.strftime)
        localtime = staticmethod(time_mod.localtime)

    def test_preemption_slew_and_step(self):
        out = tempfile.mkdtemp(prefix="canstream_clock_")
        real = msl.time
        clock = self.Clock()
        msl.time = clock
        try:
            lg = msl.Logger(out, "can0")
            self.assertAlmostEqual(lg.wall_minus_mono, clock.K, places=6)
            clock.preempt = 0.2               # 200 ms between reading the wall clock and the monotonic one
            lg.check_clock()
            self.assertAlmostEqual(lg.wall_minus_mono, clock.K, places=6)
            clock.wall_off = 0.010            # an NTP slew: 10 ms since the last check, under STEP_S
            lg.check_clock()
            self.assertAlmostEqual(lg.wall_minus_mono, clock.K + 0.010, places=6)
            clock.wall_off = 3600.010         # v0.4 sets the clock
            lg.check_clock()
            self.assertAlmostEqual(lg.wall_minus_mono, clock.K + 3600.010, places=6)
            lg.close()
        finally:
            msl.time = real
        with open(os.path.join(out, "can_sync.csv")) as f:
            steps = [l for l in f if "clock step" in l]
        shutil.rmtree(out, ignore_errors=True)
        self.assertEqual(len(steps), 1, steps)
        self.assertIn("clock step +3600.000000 s", steps[0])


class TestWallMinusMono(unittest.TestCase):
    def test_preemption_between_the_two_reads_is_rejected(self):
        """A 300 ms stall between reading the wall clock and the monotonic one must not reach the result."""
        K = 1790000000.0
        seq = iter([100.0, 100.3,                     # first try: preempted for 300 ms around time.time()
                    100.4, 100.40001])                # second try: tight
        walls = iter([K + 100.0, K + 100.400005])

        class FakeTime:
            monotonic = staticmethod(lambda: next(seq))
            time = staticmethod(lambda: next(walls))

        real = msl.time
        msl.time = FakeTime
        try:
            self.assertAlmostEqual(msl.wall_minus_mono(), K, delta=1e-5)
        finally:
            msl.time = real
        self.assertAlmostEqual(msl.wall_minus_mono(), real.time() - real.monotonic(), delta=0.005)


# ─── the reader's bounded hand-over queue (HUNG DISK in the logger's docstring) ──


def queue_stream(n_frames, us0=0x20000000, period_us=800, fs_every=40, con_every=25):
    """[(us, kind, line)] in stream order: frame lines carrying their sequence number as data, an FS line
    every @fs_every frames (`streamed` = frame lines before it, as the firmware counts them), and a CRLF
    console line every @con_every frames."""
    out = []
    for i in range(n_frames):
        us = (us0 + i * period_us) % WRAP
        if i and i % fs_every == 0:
            out.append((us, "fs", b"FS %08X %d %d 0 0 0\n" % (us, i + 5, i)))
        if i % con_every == 7:
            out.append((us, "con", b"MKR status n=%d\r\n" % i))
        out.append((us, "f", b"F %08X 17C 8 %016X\n" % (us, i)))
    return out


def queue_reads(lines, seed, base=100.0):
    """The stream cut into reads of 1-150 bytes at random — lines split across reads included — each
    stamped with a host time just after the newest MKR time it carries."""
    blob = b"".join(line for _us, _k, line in lines)
    starts, pos = [], 0
    for _us, _k, line in lines:
        starts.append(pos)
        pos += len(line)
    rng = random.Random(seed)
    out, pos = [], 0
    while pos < len(blob):
        end = min(len(blob), pos + rng.randint(1, 150))
        k = bisect.bisect_right(starts, end - 1) - 1         # the line holding the read's last byte
        out.append((base + lines[k][0] / 1e6 + 0.001, blob[pos:end]))
        pos = end
    return out


def runs_of(indices):
    """[(first, last)] of the contiguous runs in a sorted index list."""
    out = []
    for i in indices:
        if out and i == out[-1][1] + 1:
            out[-1][1] = i
        else:
            out.append([i, i])
    return [tuple(r) for r in out]


NOTE_RE = re.compile(r"queue full for [0-9.]+ s: .*?can_raw\.log (\d+) frame lines \((\d+) bytes\), can_stats\.csv "
                     r"(\d+) FS lines \((\d+) bytes\), mkr_console\.txt (\d+) lines \((\d+) bytes\)")


class TestHandoverQueue(unittest.TestCase):
    """The reader -> writer queue under a writer that stops dead, in-process and deterministic: a PortReader
    that is never started is fed reads (received()), and the test plays the main thread — dispatching what
    was queued into a real Logger after every read — or a hung disk, by not dispatching at all. The reader
    must stay within the bound, drop whole lines only, count exactly what it dropped (per file, lines and
    bytes as received), keep the USB check exact, and resume by itself once the writer catches up."""

    LIMIT = 40 * (msl.EVENT_COST + 60)       # ~9 KB: full after a few dozen reads

    def setUp(self):
        self.out = tempfile.mkdtemp(prefix="canstream_queue_")
        self.lg = msl.Logger(self.out, "can0")
        self.q = msl.Handover(self.LIMIT)
        self.lg.handover = self.q
        self.said = []
        self.rd = msl.PortReader(None, self.q, threading.Event(), self.said.append)
        self.q.force(("open", 0.0, "/dev/ttyACM0", None))

    def tearDown(self):
        shutil.rmtree(self.out, ignore_errors=True)

    def write(self):
        while True:
            try:
                msl.dispatch(self.lg, self.q.get_nowait())
            except queue.Empty:
                return

    def play(self, lines, hung, seed=11, slow=lambda k: False):
        """Feeds every read; while hung(read index), nothing is dispatched, while slow(read index) one event
        after every other read, and in both the bound is checked after each read. Returns the highest level
        seen then."""
        worst = 0
        self.slow_took = {"data": 0, "drop": 0}
        for k, (mono, data) in enumerate(queue_reads(lines, seed)):
            self.rd.received(mono, data)
            if hung(k) or slow(k):
                self.assertLessEqual(self.q.level, self.q.limit, "read %d: the queue grew past its bound" % k)
                worst = max(worst, self.q.level)
                if slow(k) and k % 2:
                    ev = self.q.get_nowait()
                    self.slow_took[ev[0]] += 1
                    msl.dispatch(self.lg, ev)
            else:
                self.write()
        self.write()
        self.lg.close()
        return worst

    def check(self, lines, episodes, gap_per_episode=True):
        """Everything the logger wrote against the stream; returns the frame gaps."""
        f_lines = [line for _us, k, line in lines if k == "f"]
        fs_lines = [line for _us, k, line in lines if k == "fs"]
        con_lines = [line for _us, k, line in lines if k == "con"]
        got = read_candump(os.path.join(self.out, "can_raw.log"))       # raises on a torn or merged line
        seqs = [int(g[6], 16) for g in got]
        self.assertEqual(seqs, sorted(set(seqs)), "frames out of order or duplicated")
        self.assertEqual(seqs[-1], len(f_lines) - 1, "writing did not resume to the end")
        kept = set(seqs)
        miss_f = [i for i in range(len(f_lines)) if i not in kept]
        rows = read_csv(os.path.join(self.out, "can_stats.csv"))
        kept_fs = {int(r["streamed"]) for r in rows}
        miss_s = [j for j, line in enumerate(fs_lines) if int(line.split()[3]) not in kept_fs]
        with open(os.path.join(self.out, "mkr_console.txt")) as f:
            console = f.read()
        body = console_body(console)
        for b in body:
            self.assertRegex(b, r"^MKR status n=\d+$", "a torn or merged console line")
        kept_c = set(body)
        miss_c = [j for j, line in enumerate(con_lines) if line.decode().rstrip("\r\n") not in kept_c]
        gaps = runs_of(miss_f)
        if gap_per_episode:
            self.assertEqual(len(gaps), episodes, "one contiguous gap per hang: %r" % gaps)
        # Exact, per file, lines and bytes as received (CRLF included).
        self.assertEqual(self.lg.drops, [len(miss_f), sum(len(f_lines[i]) for i in miss_f),
                                         len(miss_s), sum(len(fs_lines[j]) for j in miss_s),
                                         len(miss_c), sum(len(con_lines[j]) for j in miss_c)])
        self.assertGreater(len(miss_f), 500, "the hang dropped too little to test anything")
        self.assertGreater(len(miss_s), 3)
        self.assertGreater(len(miss_c), 3)
        # Every missing FS and console line lies inside a frame gap: dropped with its neighbours, not apart.
        for j in miss_s:
            s = int(fs_lines[j].split()[3])
            self.assertTrue(any(a <= s <= b + 1 for a, b in gaps), "FS line %d dropped outside the gap" % j)
        # can_stats.csv: rx_frames still counts every frame line received, dropped or not, so it equals the
        # change in `streamed` row to row; rx_dropped / fs_dropped say what of it is missing.
        prev = 0
        for r in rows:
            self.assertEqual(int(r["rx_frames"]), int(r["streamed"]) - prev, r)
            prev = int(r["streamed"])
        self.assertEqual(sum(int(r["rx_dropped"]) for r in rows), len(miss_f))
        self.assertEqual(sum(int(r["fs_dropped"]) for r in rows), len(miss_s))
        self.assertEqual(self.lg.usb_lost, 0)
        self.assertGreater(self.lg.usb_checked, 0)
        self.assertEqual(self.lg.malformed, 0)
        # Reported once per episode: stderr at the start, mkr_console.txt / can_sync.csv at the end.
        self.assertEqual(len(self.said), episodes, self.said)
        self.assertEqual(self.lg.drop_episodes, episodes)
        notes = [tuple(int(x) for x in m.groups()) for m in NOTE_RE.finditer(console)]
        self.assertEqual(len(notes), episodes, console[-3000:])
        if gap_per_episode:
            self.assertEqual([n[0] for n in notes], [b - a + 1 for a, b in gaps])
        self.assertEqual([sum(n[i] for n in notes) for i in range(6)], self.lg.drops)
        self.assertNotIn("still under way", console)
        sync = [r["event"] for r in read_csv(os.path.join(self.out, "can_sync.csv"))]
        self.assertEqual(len([e for e in sync if e.startswith("queue full ")]), episodes, sync)
        self.assertIn("queue full %d time(s): dropped %d frame lines (%d bytes)" % (episodes, self.lg.drops[0],
                                                                                    self.lg.drops[1]), console)
        self.assertLessEqual(self.q.peak, self.q.limit)
        return gaps

    def test_one_hang(self):
        lines = queue_stream(6000)
        worst = self.play(lines, hung=lambda k: 800 <= k < 1800)
        self.check(lines, episodes=1)
        self.assertGreater(worst, self.LIMIT - (msl.EVENT_COST + 200), "the queue never filled")

    def test_two_hangs_two_episodes(self):
        lines = queue_stream(8000)
        self.play(lines, hung=lambda k: 600 <= k < 1300 or 2500 <= k < 3300, seed=12)
        self.check(lines, episodes=2)

    def test_slow_writer_gets_data_not_drop_records(self):
        """A writer that takes one event per two reads for 2000 reads (a slow disk, not a hung one): the
        queue stays full and the reader drops and queues by turns. It queues again only once the writer has
        freed 1/8 of the bound, so what the writer finds is mostly data in long runs. Resuming as soon as
        anything fits would fill the queue with drop records instead — one per read that fit between two
        that did not (889 of the writer's 1000 events, measured) — and lose twice the frames. One episode,
        counted exactly all the same."""
        lines = queue_stream(8000)
        self.play(lines, hung=lambda k: False, seed=13, slow=lambda k: 600 <= k < 2600)
        gaps = self.check(lines, episodes=1, gap_per_episode=False)
        took = self.slow_took
        print("\n    slow writer: took %d data events and %d drop records; %d gaps, %d frame lines dropped"
              % (took["data"], took["drop"], len(gaps), self.lg.drops[0]))
        self.assertGreater(len(gaps), 10, "the writer never freed anything: not a slow writer")
        self.assertLess(took["drop"] * 5, took["data"] + took["drop"], "the writer got mostly drop records")


class TestDropCounting(unittest.TestCase):
    """What a dropped read is counted as: each line by the file Logger.handle would have put it in."""

    def test_count_lines_by_destination(self):
        parts = [(0, b"F 00000010 17C 8 0011223344556677\n"), (2, b"FS 00000020 1 1 0 0 0\r\n"),
                 (4, b"MKR status\r\n"), (4, b"\n"), (4, b"F 0000ABCD 17C 9 00\n"),    # malformed: a console note
                 (4, b"FS 0000ABCD 1 2 3\n"), (0, b"F 00000030 123 R 4\n"), (0, b"F 00000040 18DAF110 0\r\n")]
        want = [0] * 6
        for k, line in parts:
            want[k] += 1
            want[k + 1] += len(line)
        acc = [0] * 6
        msl.count_lines(b"".join(line for _k, line in parts), acc)
        self.assertEqual(acc, want)
        self.assertEqual(acc[1] + acc[3] + acc[5], sum(len(line) for _k, line in parts))

    def test_overlong_fragment_dropped_counts_as_its_console_note(self):
        q = msl.Handover(1)                                  # nothing fits
        rd = msl.PortReader(None, q, threading.Event())
        rd.received(1.0, b"x" * (msl.MAX_PARTIAL + 10))    # no newline, past MAX_PARTIAL
        rd.received(1.1, b"F 00000010 17C 0\n")
        self.assertEqual(rd.dropping, [1, 17, 0, 0, 1, msl.MAX_PARTIAL + 10])
        self.assertEqual(rd.drop_span, [1.0, 1.1])
        self.assertEqual(rd.drop_record(), ("drop", 1.0, 1.1, 1, 17, 0, 0, 1, msl.MAX_PARTIAL + 10))
        self.assertEqual(len(q.items), 0)


class TestStatsColumns(unittest.TestCase):
    """rx_dropped and fs_dropped are ADDED at the end of can_stats.csv. A file an older logger started keeps
    its header and its row width when appended to, so a reader of it never sees a ragged file."""

    FS = b"FS 05F5E100 1000 1000 0 0 0\n"

    def run_logger(self, out):
        lg = msl.Logger(out, "can0")
        lg.port_open("/dev/null", None)
        lg.data(50.0, self.FS)
        lg.close()
        with open(os.path.join(out, "can_stats.csv")) as f:
            return [line.rstrip("\n").split(",") for line in f]

    def test_new_file_has_the_new_columns(self):
        out = tempfile.mkdtemp(prefix="canstream_cols_")
        try:
            rows = self.run_logger(out)
            self.assertEqual(",".join(rows[0]) + "\n", msl.STATS_HEADER)
            self.assertEqual(rows[0][:11], ["host_epoch", "host_mono", "timeline", "mkr_us", "mkr_s", "drained",
                                            "streamed", "ovf", "ringdrop", "nohost", "rx_frames"])
            self.assertEqual(rows[0][11:], ["rx_dropped", "fs_dropped"])
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1][11:], ["0", "0"])
        finally:
            shutil.rmtree(out, ignore_errors=True)

    def test_older_file_keeps_its_width(self):
        out = tempfile.mkdtemp(prefix="canstream_cols_")
        try:
            old = "host_epoch,host_mono,timeline,mkr_us,mkr_s,drained,streamed,ovf,ringdrop,nohost,rx_frames\n"
            with open(os.path.join(out, "can_stats.csv"), "w") as f:
                f.write(old + "1.0,2.0,0,3,4.0,5,6,7,8,9,10\n")
            rows = self.run_logger(out)
            self.assertEqual(",".join(rows[0]) + "\n", old)
            self.assertEqual([len(r) for r in rows], [11, 11, 11])
            with open(os.path.join(out, "mkr_console.txt")) as f:
                self.assertIn("older header", f.read())
        finally:
            shutil.rmtree(out, ignore_errors=True)


class TestReaderDropsAcrossPortLoss(unittest.TestCase):
    """The real read loop on a pty, nobody consuming its queue (the main thread hung): the reader keeps
    reading and dropping, and when the port is then lost the drop record goes into the queue AHEAD of the
    loss — the gap stays where it happened, on the timeline it happened on. Every line read is accounted
    for exactly once: queued, or counted in the record."""

    def test_record_before_loss_and_every_line_accounted(self):
        tmp = tempfile.mkdtemp(prefix="canstream_qport_")
        link = os.path.join(tmp, "ttyMKR")
        pty = Pty(link)
        q = msl.Handover(20 * (msl.EVENT_COST + 40))
        stop = threading.Event()
        said = []
        rd = msl.PortReader(link, q, stop, said.append)
        lines = [b"F %08X 17C 8 %016X\n" % (0x100000 + i * 800, i) for i in range(3000)]

        def items():
            with q.cond:
                return list(q.items)

        def wait_for(cond, what):
            end = time_mod.monotonic() + 10.0
            while not cond():
                self.assertLess(time_mod.monotonic(), end, what)
                time_mod.sleep(0.01)

        rd.start()
        try:
            wait_for(lambda: any(ev[0] == "open" for ev in items()), "the reader never opened the pty")
            for i, line in enumerate(lines):
                pty.write(line)
                if i % 8 == 7:
                    time_mod.sleep(0.0005)                  # many small reads, as from the MKR
            queued = lambda: sum(ev[2].count(b"\n") for ev in items() if ev[0] == "data")
            dropped = lambda: rd.dropping[0] if rd.dropping is not None else 0
            wait_for(lambda: queued() + dropped() == len(lines), "lines read != lines written")
            self.assertLess(pty.worst_block, 0.5, "the reader stopped reading while its queue was full")
            pty.hangup()
            wait_for(lambda: [ev[0] for ev in items()].count("open") == 2, "the reader never reopened")
        finally:
            stop.set()
            rd.join(2.0)
            pty.close()
            shutil.rmtree(tmp, ignore_errors=True)
        evs = items()
        kinds = [ev[0] for ev in evs]
        self.assertEqual(kinds[0], "open")
        self.assertEqual(kinds[-3:], ["drop", "lost", "open"], kinds[-6:])
        self.assertEqual(set(kinds[1:-3]), {"data"})
        data = b"".join(ev[2] for ev in evs if ev[0] == "data")
        rec = evs[-3]
        self.assertEqual(data + b"".join(lines[data.count(b"\n"):]), b"".join(lines), "queued data is not a prefix")
        self.assertEqual(rec[3:], (len(lines) - data.count(b"\n"), len(b"".join(lines)) - len(data), 0, 0, 0, 0))
        self.assertEqual(evs[-2][3], b"", "no partial line at the loss")
        self.assertEqual(len(said), 1, said)
        self.assertLessEqual(q.peak, q.limit + 3 * msl.EVENT_COST)   # + the forced record, loss and open


if __name__ == "__main__":
    unittest.main()
