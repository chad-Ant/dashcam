"""mkr_stream_log.py's pieces that need no clock: the port setup and the line/unwrap rules.

Fast, in-process, on a pty slave standing in for /dev/ttyACM0.
"""
import importlib.util
import os
import shutil
import tempfile
import termios
import time as time_mod
import tty
import unittest

from streamsim import LOGGER, WRAP

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


if __name__ == "__main__":
    unittest.main()
