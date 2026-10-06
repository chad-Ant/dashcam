"""No devices; independently constructed wheel packets and reference geometry."""
import csv
import dataclasses
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "turning"))
from calibrate import fit
from demo import Display, lines, pack_fields
from dr_binding import DeadReckoning
from model import Decoder, Profile, estimate, geometry, motorola, parse_line


class TurningTests(unittest.TestCase):
    def setUp(self):
        self.p = Profile(filter_tau_s=0)

    def decode(self, rl=1000, rr=1100, gear=4):
        decoder = Decoder(self.p)
        decoder.feed(10, 0x191, bytes.fromhex(pack_fields([(44, 5, gear)])))
        return decoder.feed(10.01, 0x1D0, bytes.fromhex(pack_fields([(7, 15, 999), (8, 15, 1001), (25, 15, rl), (42, 15, rr)])))

    def test_left_positive(self):
        r = self.decode()
        self.assertEqual(r["status"], "ok")
        self.assertAlmostEqual(r["yaw_dps"], 10.83)
        self.assertGreater(r["angle_deg"], 0)
        self.assertEqual(r["direction"], "left")

    def test_right_negative(self):
        self.assertLess(self.decode(1100, 1000)["yaw_dps"], 0)

    def test_bicycle_geometry(self):
        r = geometry(7.2, math.degrees(2 / 10), self.p)
        self.assertAlmostEqual(r["radius_m"], 10)
        self.assertAlmostEqual(r["angle_deg"], math.degrees(math.atan(2.405 / 10)))

    def test_straight_is_not_zero_radius(self):
        self.assertIsNone(self.decode(1000, 1000)["radius_m"])

    def test_either_wheel_cutoff(self):
        for rl, rr in [(299, 700), (700, 299), (0, 0)]:
            self.assertNotEqual(self.decode(rl, rr)["status"], "ok")

    def test_reverse_park_neutral_unknown(self):
        for g in [0, 1, 2, 3, 31]:
            self.assertNotEqual(self.decode(gear=g)["status"], "ok")

    def test_stale_gear(self):
        d = Decoder(self.p)
        d.feed(10, 0x191, bytes.fromhex(pack_fields([(44, 5, 4)])))
        r = d.feed(10.3, 0x1D0, bytes.fromhex(pack_fields([(25, 15, 1000), (42, 15, 1100)])))
        self.assertEqual(r["status"], "gear missing/stale")

    def test_time_step_resets_gear(self):
        for stamp in [9, 20]:
            d = Decoder(self.p)
            d.feed(10, 0x191, bytes.fromhex(pack_fields([(44, 5, 4)])))
            r = d.feed(stamp, 0x1D0, b"\xff" * 8)
            self.assertEqual(r["segment"], 1)
            self.assertEqual(r["status"], "gear missing/stale")

    def test_ratio_correction_scales_with_speed(self):
        p = dataclasses.replace(self.p, mismatch_ratio=.02)
        for mean in [500, 1000, 2000]:
            r = estimate(mean * .99, mean * 1.01, p)
            self.assertAlmostEqual(r["yaw_raw_dps"], 0)

    def test_profile_validation(self):
        for kwargs in [{"filter_tau_s": -1}, {"wheelbase_m": float('nan')},
                       {"yaw_dps_per_count": float('inf')}, {"wheel_kmh_per_count": 0},
                       {"mismatch_ratio": .5}, {"max_speed_kmh": True}]:
            with self.assertRaises(ValueError):
                Profile(**kwargs)

    def test_short_frame(self):
        with self.assertRaises(ValueError):
            Decoder(self.p).feed(0, 0x1D0, b"\0" * 6)

    def test_parser_rejects_malformed(self):
        for line in ["(NaN) can0 1D0#", "(1) can0 1D0#ABC", "(1) can0 FFF#FF", "garbage"]:
            with self.assertRaises(ValueError):
                parse_line(line)

    def test_parser_ignores_extended_remote_other_interface(self):
        for line in ["(1) can0 000001D0#1122", "(1) can0 1D0#R8", "(1) can1 1D0#1122"]:
            self.assertIsNone(parse_line(line))

    def test_independent_bit_layout(self):
        # Known monotonically increasing bytes; independently specified values.
        b = bytes.fromhex("123456789ABCDEF0")
        self.assertEqual([motorola(b, s, 15) for s in [7, 8, 25, 42]],
                         [0x1234 >> 1, ((0x34 & 1) << 14) | (0x5678 >> 2),
                          ((0x78 & 3) << 13) | (0x9ABC >> 3),
                          ((0xBC & 7) << 12) | (0xDEF0 >> 4)])

    def test_stale_display_hides_estimates(self):
        d = Display(False, False)
        d.publish(self.decode())
        d.updated -= 2
        self.assertNotIn("yaw_dps", d.snapshot()["latest"])

    def test_backlog_display_hides_estimates(self):
        d = Display(True, False)
        d.publish(self.decode())
        self.assertNotIn("yaw_dps", d.snapshot()["latest"])

    def test_history_bounded(self):
        d = Display(False, False)
        for t in range(2000):
            d.publish({"time": t, "status": "ok"})
        self.assertEqual(len(d.history), 300)

    def test_filter_resets_after_invalid(self):
        p = dataclasses.replace(self.p, filter_tau_s=1)
        d = Decoder(p)
        gear = bytes.fromhex(pack_fields([(44, 5, 4)]))
        wheel = lambda a, b: bytes.fromhex(pack_fields([(25, 15, a), (42, 15, b)]))
        d.feed(1, 0x191, gear)
        d.feed(1, 0x1D0, wheel(1000, 1100))
        d.feed(1.01, 0x1D0, wheel(0, 0))
        r = d.feed(1.02, 0x1D0, wheel(1100, 1000))
        self.assertAlmostEqual(r["yaw_dps"], -10.83)

    def test_partial_replay_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'can.log'
            path.write_text('(1) can0 191#0000')
            with self.assertRaises(ValueError):
                list(lines(path, False, threading.Event()))

    def test_follow_ignores_preexisting_partial(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'can.log'
            path.write_text('old partial')
            def append():
                time.sleep(.1)
                with path.open('a') as f:
                    f.write(' tail\nnew line\n')
            thread = threading.Thread(target=append)
            thread.start()
            stop = threading.Event()
            gen = lines(path, True, stop)
            self.assertEqual(next(gen), 'new line\n')
            stop.set()
            gen.close()
            thread.join()


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        def invoke(*args):
            return subprocess.run([sys.executable, str(ROOT / "turning/demo.py"), *map(str, args)],
                                  check=True, capture_output=True, text=True, timeout=20)
        invoke('synthetic', '--out', cls.root / 'source')
        invoke('run', '--input', cls.root / 'source/can_raw.log', '--out', cls.root / 'run', '--speed', 0)
        with (cls.root / 'run/samples.csv').open() as f:
            cls.rows = list(csv.DictReader(f))
        cls.refs = json.loads((cls.root / 'source/references.json').read_text())['runs']

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_recover_parameters_and_holdout(self):
        p, report = fit(self.rows, self.refs, Profile())
        self.assertAlmostEqual(p.mismatch_ratio, .012, delta=.001)
        self.assertAlmostEqual(p.wheel_kmh_per_count, .0101, delta=.00002)
        self.assertAlmostEqual(p.yaw_dps_per_count, .1106, delta=.001)
        for row in report['runs']:
            self.assertLess(abs(row['mean_yaw_error_dps']), .12)
            self.assertLess(abs(row['distance_error_pct']), .2)

    def test_requires_heldout_both_directions(self):
        with self.assertRaises(ValueError):
            fit(self.rows, self.refs[:-1], Profile())

    def test_prevents_overlap(self):
        refs = [dict(r) for r in self.refs]
        refs[-1].update(start=refs[-2]['start'], end=refs[-2]['end'])
        with self.assertRaisesRegex(ValueError, 'overlap'):
            fit(self.rows, refs, Profile())

    def test_rejects_wrong_turn_sign(self):
        refs = [dict(r) for r in self.refs]
        refs[1]['kind'] = 'right'
        refs[2]['kind'] = 'left'
        with self.assertRaisesRegex(ValueError, 'sign'):
            fit(self.rows, refs, Profile())

    def test_rejects_gap(self):
        rows = self.rows[:100] + self.rows[125:]
        with self.assertRaisesRegex(ValueError, 'gap'):
            fit(rows, self.refs, Profile())

    def test_rejects_invalid_sample(self):
        rows = [dict(r) for r in self.rows]
        rows[100]['status'] = 'gear missing/stale'
        with self.assertRaisesRegex(ValueError, 'invalid sample'):
            fit(rows, self.refs, Profile())

    def test_validation_does_not_change_fit(self):
        refs = [dict(r) for r in self.refs]
        refs[-1]['radius_m'] *= 1.1
        p1, _ = fit(self.rows, self.refs, Profile())
        p2, report = fit(self.rows, refs, Profile())
        self.assertEqual(p1, p2)
        self.assertGreater(abs(report['runs'][-1]['distance_error_pct']), 8)

    def test_rejects_nan_reference(self):
        refs = [dict(r) for r in self.refs]
        refs[1]['radius_m'] = float('nan')
        with self.assertRaises(ValueError):
            fit(self.rows, refs, Profile())


@unittest.skipUnless((ROOT.parents[3] / 'lib/libdeadreckoning/build/libdeadreckoning.so').exists(),
                     'build libdeadreckoning first for binding tests')
class BindingTests(unittest.TestCase):
    def test_real_cpp_library_and_gap(self):
        path = ROOT.parents[3] / 'lib/libdeadreckoning/build/libdeadreckoning.so'
        dr = DeadReckoning(path, Profile(filter_tau_s=0))
        try:
            row = dict(time=10, segment=0, gear_raw=4, wheel_rl=720, wheel_rr=720, status='ok')
            dr.update(row)
            row['time'] = 10.1
            result = dr.update(row)
            self.assertAlmostEqual(result['dr_x_m'], .2, places=6)
            row['time'] = 11
            result = dr.update(row)
            self.assertEqual(result['dr_status'], 'gap')
            self.assertFalse(result['dr_continuous'])
            row.update(time=12, segment=1)
            result = dr.update(row)
            self.assertEqual(result['dr_x_m'], 0)
            self.assertTrue(result['dr_continuous'])
        finally:
            dr.close()


if __name__ == '__main__':
    unittest.main()
