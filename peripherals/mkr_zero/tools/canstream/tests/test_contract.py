"""End to end: the FIRMWARE's own lines -> pty -> mkr_stream_log.py -> can_raw.log -> can_decode.

can_contract_emit (built from lib/CANRawStream.cpp and, in `stream` mode, the whole drain -> ring ->
canStreamService() path) writes the lines and a TRUTH file; this plays them into the logger and requires:
  - can_raw.log to reproduce every frame exactly: ids, extended/remote flags, DLC, data, order;
  - its timestamps to keep the firmware's micros() spacing to the microsecond, across the wrap (one
    offset maps a whole run, so logged - truth must be the same number for every frame);
  - can_stats.csv to carry every FS line's counters exactly, unwrapped mkr_s included;
  - can_decode to decode the logger's output into the values the payloads encode — checked here against
    an independent bit-level reading of config/canmap.brio.txt, not against the firmware's extractor —
    and to count every identifier, remote and extended frames counted but not decoded.
"""
import os
import shutil
import struct
import subprocess
import tempfile
import time
import unittest

from streamsim import CAN_DECODE, EMIT, PACKET, WRAP, Logger, Pty, read_candump, read_csv

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRIO = os.path.join(TOOLS, "..", "..", "config", "canmap.brio.txt")


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def read_truth(path):
    frames, stats = [], []
    with open(path) as f:
        for line in f:
            p = line.split()
            if p[0] == "F":
                data = "" if p[6] == "-" else p[6]
                frames.append((int(p[1]), int(p[2], 16), p[3] == "1", p[4] == "1", int(p[5]), data))
            else:
                stats.append((int(p[1]), [int(x) for x in p[2:7]]))
    return frames, stats


def unwrap(values):
    out, prev, acc = [], None, 0
    for v in values:
        if prev is not None:
            acc += ((v - prev + (1 << 31)) % WRAP) - (1 << 31)
        else:
            acc = v
        out.append(acc)
        prev = v
    return out


def motorola(data, start, length):
    """DBC Motorola: MSB at `start`, walking down, wrapping to bit 7 of the next byte."""
    v, pos = 0, start
    for _ in range(length):
        v = (v << 1) | ((data[pos // 8] >> (pos % 8)) & 1)
        pos = pos + 15 if pos % 8 == 0 else pos - 1
    return v


GEARS = {1: "P", 2: "R", 3: "N", 4: "D", 7: "L", 10: "S"}


class ContractCase(unittest.TestCase):
    mode = None

    @classmethod
    def setUpClass(cls):
        if cls.mode is None:
            raise unittest.SkipTest("abstract: see the two subclasses")
        if not (os.path.exists(EMIT) and os.path.exists(CAN_DECODE)):
            raise unittest.SkipTest("build first: make")
        cls.tmp = tempfile.mkdtemp(prefix="canstream_contract_")
        truth = os.path.join(cls.tmp, "truth.txt")
        out = subprocess.run([EMIT, cls.mode, truth], check=True, stdout=subprocess.PIPE).stdout
        cls.frames, cls.stats = read_truth(truth)
        if cls.mode == "format":
            # Whole lines, at most 63 bytes per write, as the firmware packs them.
            cls.packets, buf = [], b""
            for line in out.splitlines(keepends=True):
                if len(buf) + len(line) > PACKET:
                    cls.packets.append(buf)
                    buf = b""
                buf += line
            cls.packets.append(buf)
        else:
            cls.packets = [bytes.fromhex(l) for l in out.decode().split()]
            # The FS lines are whatever the firmware emitted: the truth for can_stats.csv.
            for p in cls.packets:
                for line in p.decode().splitlines():
                    if line.startswith("FS "):
                        q = line.split()
                        cls.stats.append((int(q[1], 16), [int(x) for x in q[2:]]))
        port = os.path.join(cls.tmp, "ttyMKR")
        cls.out = os.path.join(cls.tmp, "out")
        pty = Pty(port)
        lg = Logger(cls.out, port)
        try:
            assert lg.wait_open(), lg.console_text()
            # Delivered within well under HOLD_S, so every frame is written with the same final offset.
            for p in cls.packets:
                pty.write(p)
                time.sleep(0.001)
            time.sleep(0.3)
        finally:
            cls.rc = lg.stop()
            pty.close()
        cls.logged = read_candump(os.path.join(cls.out, "can_raw.log"))
        cls.decoded = os.path.join(cls.tmp, "decoded.csv")
        cls.census = os.path.join(cls.tmp, "census.csv")
        cls.dec = subprocess.run([CAN_DECODE, "--every-frame", "--map", BRIO, "-o", cls.decoded, "--census", cls.census,
                                  os.path.join(cls.out, "can_raw.log")], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 universal_newlines=True)

    @classmethod
    def tearDownClass(cls):
        if os.environ.get("CANSTREAM_KEEP") is None:
            shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_logger_clean(self):
        self.assertEqual(self.rc, 0)
        with open(os.path.join(self.out, "mkr_console.txt")) as f:
            con = f.read()
        self.assertIn(" 0 malformed", con)
        self.assertNotIn("new timeline", con)

    def test_frames_exact(self):
        self.assertEqual(len(self.logged), len(self.frames))
        for i, (g, t) in enumerate(zip(self.logged, self.frames)):
            self.assertEqual(g[2:], t[1:], "frame %d" % i)

    def test_spacing_exact_across_wrap(self):
        truth_us = unwrap([t[0] for t in self.frames])
        self.assertGreater(truth_us[-1], WRAP, "the run must cross the micros() wrap")
        d = [g[0] - u for g, u in zip(self.logged, truth_us)]
        self.assertLessEqual(max(d) - min(d), 2, "one offset must map the whole run (spread %d us)" % (max(d) - min(d)))

    def test_stats_exact(self):
        rows = read_csv(os.path.join(self.out, "can_stats.csv"))
        self.assertEqual(len(rows), len(self.stats))
        for row, (us, counters) in zip(rows, self.stats):
            self.assertEqual(int(row["mkr_us"]), us)
            self.assertEqual([int(row[k]) for k in ("drained", "streamed", "ovf", "ringdrop", "nohost")], counters)
        mk = [float(r["mkr_s"]) for r in rows]
        tu = unwrap([us for us, _ in self.stats])
        for a, b in zip(mk, tu):
            self.assertAlmostEqual(a - mk[0], (b - tu[0]) / 1e6, delta=1e-6)

    def test_rx_frames_account_for_every_frame(self):
        """rx_frames = F lines since the previous FS line; with the firmware's real counters (stream mode)
        it must equal the change in `streamed`, which is how the logger counts USB/host loss."""
        rows = read_csv(os.path.join(self.out, "can_stats.csv"))
        expected, n = [], 0
        for p in self.packets:
            for line in p.decode().splitlines():
                if line.startswith("FS "):
                    expected.append(n)
                    n = 0
                elif line.startswith("F "):
                    n += 1
        self.assertEqual([int(r["rx_frames"]) for r in rows], expected)
        if self.mode == "stream":
            st = [int(r["streamed"]) for r in rows]
            for i in range(1, len(rows)):
                self.assertEqual((st[i] - st[i - 1]) % WRAP, int(rows[i]["rx_frames"]))
            with open(os.path.join(self.out, "mkr_console.txt")) as f:
                self.assertIn(" 0 not received here", f.read())

    def test_decode_matches_payloads(self):
        self.assertEqual(self.dec.returncode, 0, self.dec.stderr)
        rows = read_csv(self.decoded)
        decodable = [t for t in self.frames if not t[2] and not t[3] and t[1] in (0x158, 0x17C, 0x191, 0x1AB, 0x1D0, 0x294)]
        self.assertEqual(len(rows), len(decodable))
        left = right = 0
        for row, (us, ident, ext, rtr, dlc, data) in zip(rows, decodable):
            self.assertEqual(int(row["id"], 16), ident)
            b = bytes.fromhex(data) + bytes(8 - dlc)
            if ident == 0x158:
                self.assertEqual(row["speed_kmh"], "%.2f" % f32(f32(motorola(b, 7, 16)) * f32(0.01)))
            elif ident == 0x17C:
                self.assertEqual(int(row["rpm"]), motorola(b, 23, 16))
                self.assertEqual(int(row["pedal_raw"]), motorola(b, 7, 8))
                self.assertEqual(int(row["brake_switch"]), motorola(b, 32, 1))
                self.assertEqual(int(row["brake_pressed"]), motorola(b, 53, 1))
            elif ident == 0x191:
                self.assertEqual(row["gear"], GEARS.get(motorola(b, 44, 5), "UNKNOWN"))
            elif ident == 0x1AB:
                self.assertEqual(int(row["steer_torque_raw"]), motorola(b, 1, 10))
            elif ident == 0x1D0:
                w = [motorola(b, s, 15) for s in (7, 8, 25, 42)]
                self.assertEqual([int(row[k]) for k in ("wheel_fl_raw", "wheel_fr_raw", "wheel_rl_raw", "wheel_rr_raw")], w)
                diff = f32(f32(w[3]) - f32(w[2]))
                cdps = f32(diff * f32(10.83))           # uncalibrated: no GNSS, no tyre correction
                self.assertEqual(int(row["yaw_rate_cdps_uncal"]), int(cdps + 0.5) if cdps >= 0 else -int(-cdps + 0.5))
            elif ident == 0x294:
                # Held lamps with the exclusivity rule: one lit and the other dark cancels the other's hold;
                # a dark frame within 900 ms of a flash cancels nothing.
                l, r = (b[0] >> 5) & 1, (b[0] >> 6) & 1
                if l and not r:
                    left, right = 1, 0
                elif r and not l:
                    left, right = 0, 1
                self.assertEqual((int(row["turn_left"]), int(row["turn_right"])), (left, right))

    def test_census(self):
        rows = {(r["id"], r["ext"]): r for r in read_csv(self.census)}
        want = {}
        for us, ident, ext, rtr, dlc, data in self.frames:
            k = (("%08X" if ext else "%03X") % ident, "1" if ext else "0")
            c = want.setdefault(k, [0, 0, set()])
            c[0] += 1
            c[1] += rtr
            c[2].add(dlc)
        self.assertEqual(set(rows), set(want))
        for k, (n, nr, dlcs) in want.items():
            self.assertEqual(int(rows[k]["count"]), n, k)
            self.assertEqual(int(rows[k]["remote"]), nr, k)
            self.assertEqual(rows[k]["dlcs"], "|".join(str(d) for d in sorted(dlcs)), k)
            if k[1] == "1" or nr == n:
                self.assertEqual(int(rows[k]["decoded"]), 0, "extended/remote frames are never decoded: %s" % (k,))


class TestContractFormat(ContractCase):
    """Lines from canFormatFrameLine()/canFormatStatsLine(), FS stamped ahead of the frames after it."""
    mode = "format"


class TestContractStream(ContractCase):
    """Packets from the full production path: MCP2515 model -> drain ISR -> ring -> canStreamService()."""
    mode = "stream"

    def test_packets_are_whole_lines_fs_first(self):
        for p in self.packets:
            self.assertLessEqual(len(p), PACKET)
            self.assertTrue(p.endswith(b"\n"))
            # A packet may open with a bare '\n' (the firmware ends console text that a print may have left
            # half-sent before it streams again): an empty console line, not a stream line.
            lines = [l for l in p.decode().splitlines() if l]
            for i, line in enumerate(lines):
                if line.startswith("FS "):
                    self.assertEqual(i, 0, "FS goes first in its packet")


if __name__ == "__main__":
    unittest.main()
