"""can_decode: the firmware decoder replayed over candump logs.

What is under test here is the REPLAY — time, rows, segments, the map loading, the census, the candump
parser. The decoding itself is the firmware's (canDecodeFrame() and friends, compiled unmodified), and
its bit-level results are checked end to end against independent payload readings in test_contract.py.
"""
import os
import random
import shutil
import subprocess
import tempfile
import unittest

from streamsim import CAN_DECODE, read_csv

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIO = os.path.join(TOOLS, "..", "..", "config", "canmap.brio.txt")


def line(t, ident, data, iface="can0"):
    return "(%.6f) %s %s#%s\n" % (t, iface, ident, data)


class DecodeCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not os.path.exists(CAN_DECODE):
            raise unittest.SkipTest("build first: make")
        cls.tmp = tempfile.mkdtemp(prefix="canstream_decode_")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def decode(self, text, *args, map_path=BRIO):
        log = os.path.join(self.tmp, "in.log")
        out = os.path.join(self.tmp, "out.csv")
        census = os.path.join(self.tmp, "census.csv")
        with open(log, "w") as f:
            f.write(text)
        p = subprocess.run([CAN_DECODE, "--map", map_path, "-o", out, "--census", census] + list(args) + [log],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        rows = read_csv(out) if os.path.exists(out) and p.returncode in (0, 3) else []
        cen = {r["id"]: r for r in read_csv(census)} if os.path.exists(census) and p.returncode in (0, 3) else {}
        return p, rows, cen

    def at(self, rows, t):
        for r in rows:
            if abs(float(r["time"]) - t) < 1e-4:
                return r
        self.fail("no row at %.3f" % t)


class TestReplay(DecodeCase):
    def test_brio_file_equals_builtin_map(self):
        """config/canmap.brio.txt is documented as byte-for-byte the compiled-in map: same decode."""
        rng = random.Random(7)
        text, t = "", 5000.0
        for _ in range(4000):
            ident = rng.choice(["158", "17C", "191", "1AB", "1D0", "294", "100", "39A"])
            n = 3 if ident == "1AB" else 8
            text += line(t, ident, "".join("%02X" % rng.randrange(256) for _ in range(n)))
            t += rng.expovariate(1 / 0.004)
        for extra in ([], ["--every-frame"]):
            p1, r1, _ = self.decode(text, *extra)
            p2, r2, _ = self.decode(text, *extra, map_path="builtin")
            self.assertEqual(p1.returncode, 0, p1.stderr)
            self.assertEqual(p2.returncode, 0, p2.stderr)
            self.assertGreater(len(r1), 100)
            self.assertEqual(r1, r2)

    def test_rows_are_state_at_their_instant(self):
        text = (line(1000.000, "17C", "1000034C00002000") +      # rpm 844, pedal 0x10, brake pressed (53|1)
                line(1000.050, "158", "0457000000000000") +        # 11.11 km/h
                line(1000.350, "100", "0000000000000000"))         # not in the map: only extends the grid
        p, rows, _ = self.decode(text, "--rate", "10")
        self.assertEqual(p.returncode, 0, p.stderr)
        # A frame AT a grid instant is in that row; nothing is in a row before it arrives.
        r = self.at(rows, 1000.0)
        self.assertEqual((r["rpm"], r["pedal_raw"], r["brake_pressed"], r["speed_kmh"]), ("844", "16", "1", ""))
        self.assertEqual(self.at(rows, 1000.1)["speed_kmh"], "11.11")
        # VEH_FRESH_SNIFF_MS = 200: 200 ms old is still fresh, older is expired — empty, never zero.
        self.assertEqual(self.at(rows, 1000.2)["rpm"], "844")
        self.assertEqual(self.at(rows, 1000.2)["speed_kmh"], "11.11")
        r = self.at(rows, 1000.3)
        self.assertEqual((r["rpm"], r["pedal_raw"], r["brake_pressed"], r["speed_kmh"]), ("", "", "", ""))
        self.assertEqual(len(rows), 4, "rows stop at the last frame's instant")

    def test_indicator_hold_and_exclusivity(self):
        text = line(10.000, "294", "2000000000000000")              # left lit
        t = 10.040
        while t < 11.5:
            text += line(t, "294", "0000000000000000")              # dark: the off half of a blink
            t += 0.040
        text += line(11.600, "294", "4000000000000000")             # right lit
        text += line(11.650, "294", "2000000000000000")             # left lit, right dark: right ends NOW
        p, rows, _ = self.decode(text, "--rate", "10")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual(self.at(rows, 10.9)["turn_left"], "1")      # 900 ms hold
        self.assertEqual(self.at(rows, 11.0)["turn_left"], "0")
        p, rows, _ = self.decode(text, "--every-frame")
        last = rows[-2:]
        self.assertEqual((last[0]["turn_left"], last[0]["turn_right"]), ("0", "1"))
        self.assertEqual((last[1]["turn_left"], last[1]["turn_right"]), ("1", "0"))

    def test_grid_row_reevaluates_the_hold_at_its_instant(self):
        """A grid row is a loop() pass at its instant: the holds are evaluated THEN, not as of the last
        frame. Left lit at 10.000, dark 0x294 every 150 ms (fresh: under 200 ms apart) up to 10.900, when the
        900 ms hold still stands; at 11.000 it has lapsed, though no frame arrived since to say so."""
        text = line(10.000, "294", "2000000000000000")
        for i in range(1, 7):
            text += line(10.000 + 0.150 * i, "294", "0000000000000000")
        text += line(11.050, "100", "0000000000000000")         # not in the map: only carries the grid on
        p, rows, _ = self.decode(text, "--rate", "10")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual(self.at(rows, 10.9)["turn_left"], "1")
        self.assertEqual(self.at(rows, 11.0)["turn_left"], "0")

    def test_every_frame_rows_expire_and_never_go_back(self):
        """--every-frame rows are the state as each decoded frame leaves it: other signals expired by then
        (rpm 300 ms old at the speed frame), and the time clamped forward as tickCANSniff() clamps."""
        text = (line(1.000, "17C", "1000034C00002000") + line(1.300, "158", "0457000000000000") +
                line(1.400, "17C", "1000034C00002000") + line(1.350, "158", "0457000000000000"))
        p, rows, _ = self.decode(text, "--every-frame")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual([(r["id"], r["time"], r["rpm"], r["speed_kmh"]) for r in rows],
                         [("17C", "1.000000", "844", ""), ("158", "1.300000", "", "11.11"),
                          ("17C", "1.400000", "844", "11.11"), ("158", "1.400000", "844", "11.11")])
        self.assertIn("1 clamped back-step(s)", p.stderr)

    def test_remote_and_extended_counted_not_decoded(self):
        text = (line(1.0, "17C", "R8") + line(1.1, "0000017C", "0000034C00000000") +
                line(1.2, "17C", "R") + line(1.25, "123", ""))
        p, rows, cen = self.decode(text, "--every-frame")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual(rows, [])
        self.assertEqual((cen["17C"]["count"], cen["17C"]["remote"], cen["17C"]["decoded"], cen["17C"]["dlcs"]),
                         ("2", "2", "0", "0|8"))
        self.assertEqual((cen["0000017C"]["ext"], cen["0000017C"]["decoded"]), ("1", "0"))
        self.assertEqual(cen["123"]["dlcs"], "0")
        self.assertIn("2 remote, 1 extended", p.stderr)

    def test_segments_and_clamps(self):
        text = (line(1000.00, "17C", "0000034C00000000") + line(1000.10, "17C", "0000034C00000000") +
                line(1000.05, "17C", "0000034C00000000") +      # 50 ms back: clamped, same segment
                line(997.00, "158", "0457000000000000") +        # 3 s back: a clock step, new segment
                line(997.05, "158", "0457000000000000") +
                line(1200.0, "191", "0000000000040000"))         # 200 s gap: new segment
        p, rows, _ = self.decode(text, "--rate", "10")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("3 segment(s), 1 clamped back-step(s)", p.stderr)
        segs = [(r["segment"], r["time"]) for r in rows]
        self.assertEqual(segs[0], ("1", "1000.000"))
        s2 = [r for r in rows if r["segment"] == "2"]
        self.assertEqual(s2[0]["time"], "997.000")
        self.assertEqual(s2[0]["rpm"], "", "a new segment starts from a fresh template")
        s3 = [r for r in rows if r["segment"] == "3"]
        self.assertEqual([r["gear"] for r in s3], ["D"])

    def test_census_rate_and_window(self):
        text = "".join(line(50.0 + 0.010 * i, "1D0", "0000000000000000") for i in range(101))
        p, rows, cen = self.decode(text)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual(cen["1D0"]["count"], "101")
        self.assertEqual(cen["1D0"]["mean_rate_hz"], "100.000")
        self.assertEqual((cen["1D0"]["first_time"], cen["1D0"]["last_time"]), ("50.000000", "51.000000"))
        self.assertEqual(cen["1D0"]["in_map"], "1")
        # Map ids never seen are named, so a mis-keyed map shows itself.
        self.assertIn("map id 158 never appeared", p.stderr)

    def test_candump_variants_and_iface(self):
        text = (line(1.0, "17C", "00.00.03.4C.00.00.00.00") +            # can-utils dot separators
                "(1.100000) can0 158#0457000000000000 R\n" +              # candump -L direction flag
                line(1.2, "17c", "0000034d00000000") +                    # lowercase hex
                "(1.300000) can0 17C##10011223344\n" +                    # CAN FD: skipped, counted
                line(1.4, "17C", "0000034E00000000", iface="can1"))
        p, rows, _ = self.decode(text, "--every-frame", "--iface", "can0")
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual([r["rpm"] for r in rows if r["id"] == "17C"], ["844", "845"])
        self.assertIn("1 CAN FD skipped, 1 other-interface", p.stderr)
        p, rows, _ = self.decode(text, "--rate", "1")
        self.assertEqual([r["time"] for r in rows], ["1.000"])

    def test_malformed_lines_exit_3(self):
        text = line(1.0, "17C", "0000034C00000000") + "garbage\n" + "(1.1) can0 1234#00\n" + "(1.2) can0 17C#001\n"
        p, rows, _ = self.decode(text, "--every-frame")
        self.assertEqual(p.returncode, 3)
        self.assertIn("3 malformed", p.stderr)
        self.assertEqual(len(rows), 1, "the good frames are still decoded")


class TestMapLoading(DecodeCase):
    def write_map(self, text):
        path = os.path.join(self.tmp, "canmap.test.txt")
        with open(path, "w") as f:
            f.write(text)
        return path

    def test_reports_checksum_like_the_boot_log(self):
        p, _, _ = self.decode(line(1.0, "17C", "0000034C00000000"))
        self.assertRegex(p.stderr, r"ok, 13 signals across 6 ids, id=0x[0-9A-F]{2}, yaw ok")

    def test_refuses_what_the_mkr_refuses(self):
        with open(BRIO) as f:
            good = f.read()
        p, _, _ = self.decode("", map_path=self.write_map(good + "#" * 9000 + "\n"))
        self.assertEqual(p.returncode, 1)
        self.assertIn("map too large", p.stderr)
        p, _, _ = self.decode("", map_path=self.write_map("rpm,0x17C,23,16,u,1\n"))
        self.assertEqual(p.returncode, 1)
        self.assertIn("map has no speed source", p.stderr)
        p, _, _ = self.decode("", map_path=os.path.join(self.tmp, "absent.txt"))
        self.assertEqual(p.returncode, 1)

    def test_names_rejected_lines(self):
        p, _, _ = self.decode(line(1.0, "158", "0457000000000000"),
                              map_path=self.write_map("speed,0x158,7,16,u,0.01\r\nbogus,line\nrpm,0x17C,99,16,u,1\n"))
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("map line 2 rejected: bogus,line", p.stderr)
        self.assertIn("map line 3 rejected: rpm,0x17C,99,16,u,1", p.stderr)


if __name__ == "__main__":
    unittest.main()
