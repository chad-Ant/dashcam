"""Analytic Jacobians, independent geometry, adversarial inputs and real demo CLI."""
import csv
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest
import urllib.request

HERE = Path(__file__).resolve().parents[1] / 'turning'
sys.path.insert(0, str(HERE))
from fusion import FusionConfig, TurningEKF, course_model, wheel_model, wrap
from fusion_input import FusionSession, TelemetryInput
from model import Profile


def config(**kw):
    return FusionConfig(gyro_up=(0, 0, 1), front_track_m=1.475, rear_track_m=1.459, **kw)


def wheels(t, v=3., r=.2):
    # Independently use each tyre's path radius and distance / time, not wheel_model.
    values = []
    for longitudinal, lateral in [(2.405, .7375), (2.405, -.7375), (0, .7295), (0, -.7295)]:
        speed = math.sqrt((v - r * lateral) ** 2 + (r * longitudinal) ** 2)
        values.append(round(speed * 360))
    return dict(time=t, segment=0, status='ok', gear_raw=4,
                **dict(zip(('wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr'), values)))


def telemetry_row(t=10, master=10000, **kw):
    row = dict(host_ms=str(round(t * 1000)), master_ms=str(master), flags='0x101e', imu_calib='0x30',
               imu_gx='0', imu_gy='0', imu_gz='12', heading_deg='355', gps_speed_kmh='10.8',
               fix_valid='1', fix_type='3', sats='8', utc='2026-10-07T00:00:10Z')
    row.update(kw)
    return row


class EKFTests(unittest.TestCase):
    def make(self, **kw):
        return TurningEKF(Profile(), config(**kw))

    def test_invalid_config(self):
        for extra in [dict(gyro_sigma_dps=0), dict(nis_gate=float('nan')), dict(gnss_time_offset_s=1)]:
            with self.assertRaises(ValueError):
                config(**extra)
        with self.assertRaises(ValueError):
            FusionConfig(gyro_up=(0, 0, 2), front_track_m=1.4, rear_track_m=1.4)

    def test_wheel_jacobians(self):
        for r in (-.3, 0, .3):
            x = [.5, 2., r, .01]
            for i in range(4):
                _, H = wheel_model(x, i, 2.405, 1.475, 1.459)
                for j in range(4):
                    xp, xm = list(x), list(x)
                    xp[j] += 1e-6
                    xm[j] -= 1e-6
                    actual = (wheel_model(xp, i, 2.405, 1.475, 1.459)[0] - wheel_model(xm, i, 2.405, 1.475, 1.459)[0]) / 2e-6
                    self.assertAlmostEqual(H[j], actual, places=7)

    def test_course_jacobian_and_lever_arm(self):
        x = [.1, 2.5, -.2, 0]
        h, H = course_model(x, 2.1, -.1)
        self.assertNotEqual(h, x[0])
        for j in range(4):
            xp, xm = list(x), list(x)
            xp[j] += 1e-6
            xm[j] -= 1e-6
            actual = wrap(course_model(xp, 2.1, -.1)[0] - course_model(xm, 2.1, -.1)[0]) / 2e-6
            self.assertAlmostEqual(H[j], actual, places=7)

    def test_circle_bias_and_all_sensors(self):
        f = self.make(antenna_x_m=2., course_sigma_deg=2)
        for i in range(1001):
            t = 10 + i * .05
            f.wheels(wheels(t))
            f.sensor(dict(kind='gyro', time=t, value=math.degrees(.2) + 2.))
            if i % 20 == 0:
                course = (90 - math.degrees(.2 * (t - 10) + math.atan2(.2 * 2, 3))) % 360
                f.sensor(dict(kind='course', time=t, value=course, speed_mps=3))
        result = f.snapshot(t)
        self.assertEqual(result['ekf_status'], 'fused')
        self.assertAlmostEqual(result['ekf_bias_dps'], 2, delta=.15)
        self.assertAlmostEqual(result['ekf_yaw_dps'], math.degrees(.2), delta=.12)
        self.assertAlmostEqual(result['ekf_radius_m'], 15, delta=.2)
        self.assertAlmostEqual(result['ekf_angle_deg'], math.degrees(math.atan(2.405 / 15)), delta=.12)

    def test_wrap_north(self):
        f = self.make()
        for i in range(40):
            t = 10 + i * .1
            f.wheels(wheels(t, r=.03))
            if i % 10 == 0:
                f.sensor(dict(kind='course', time=t, value=(1 - math.degrees(.03 * i * .1)) % 360, speed_mps=3))
        self.assertLess(abs(wrap(f.x[0] - math.radians(90 - (1 - math.degrees(.03 * 3.9))))), .03)

    def test_straight_radius_not_infinity(self):
        f = self.make()
        f.wheels(wheels(10, r=0))
        self.assertIsNone(f.snapshot(10)['ekf_radius_m'])

    def test_all_wheels_cutoff_hides_cached_motion(self):
        f = self.make()
        f.wheels(wheels(10))
        f.wheels(wheels(10.1, v=0, r=0))
        self.assertEqual(f.snapshot(10.1)['ekf_status'], 'unavailable')

    def test_low_speed_gnss_rejected(self):
        f = self.make()
        f.wheels(wheels(10))
        self.assertFalse(f.sensor(dict(kind='course', time=10, value=0, speed_mps=.2)))
        self.assertFalse(f.aligned)

    def test_stale_and_dropout(self):
        f = self.make()
        f.wheels(wheels(10))
        self.assertEqual(f.snapshot(11)['ekf_status'], 'unavailable')
        f.wheels(wheels(11))
        self.assertGreater(f.resets, 1)
        self.assertEqual(f.snapshot(11)['ekf_status'], 'degraded')

    def test_reverse_resets(self):
        f = self.make()
        row = wheels(10)
        row['gear_raw'] = 2
        f.wheels(row)
        self.assertEqual(f.snapshot(10)['ekf_status'], 'unavailable')

    def test_duplicate_and_out_of_order(self):
        f = self.make()
        f.wheels(wheels(10))
        e = dict(kind='gyro', time=10, value=12)
        self.assertTrue(f.sensor(e))
        before = [list(row) for row in f.P]
        self.assertFalse(f.sensor(e))
        self.assertFalse(f.sensor(dict(e, time=9)))
        self.assertEqual(before, f.P)

    def test_nonfinite_and_saturated(self):
        f = self.make()
        f.wheels(wheels(10))
        for i, val in enumerate((float('nan'), float('inf'), 2000)):
            self.assertFalse(f.sensor(dict(kind='gyro', time=10 + .1 * i, value=val)))

    def test_one_wheel_slip_rejected(self):
        f = self.make()
        for i in range(30):
            f.wheels(wheels(10 + .1 * i))
        row = wheels(13)
        row['wheel_fl'] += 600
        f.wheels(row)
        self.assertEqual(f.wheels_used, 3)
        self.assertGreater(f.rejected, 0)
        self.assertAlmostEqual(f.x[2], .2, delta=.01)

    def test_covariance_psd_long_noisy_run(self):
        f, rng = self.make(), random.Random(51)
        for i in range(2000):
            t = 10 + i * .05
            row = wheels(t, r=.2 * math.sin(i / 200))
            for k in ('wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr'):
                row[k] += rng.randrange(-12, 13)
            f.wheels(row)
            f.sensor(dict(kind='gyro', time=t, value=math.degrees(.2 * math.sin(i / 200)) + rng.gauss(1, .7)))
            # Cholesky pivots prove positive definiteness more strongly than diagonals.
            a = [[0.] * 4 for _ in range(4)]
            for j in range(4):
                for k in range(j + 1):
                    v = f.P[j][k] - sum(a[j][n] * a[k][n] for n in range(k))
                    if j == k:
                        self.assertGreater(v, 0)
                        a[j][k] = math.sqrt(v)
                    else:
                        a[j][k] = v / a[k][k]
                    self.assertAlmostEqual(f.P[j][k], f.P[k][j], places=12)


class InputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'telemetry.csv'

    def tearDown(self):
        self.temp.cleanup()

    def source(self, rows, **kw):
        with self.path.open('w') as stream:
            writer = csv.DictWriter(stream, list(telemetry_row()))
            writer.writeheader()
            writer.writerows(rows)
        src = TelemetryInput(self.path, config(**kw))
        self.addCleanup(src.close)
        return src

    def test_deduplicates_gnss_not_gyro(self):
        src = self.source([telemetry_row(10 + i * .1, 10000 + i * 100) for i in range(10)])
        ev = list(src.until(12))
        self.assertEqual(sum(e['kind'] == 'course' for e in ev), 1)
        self.assertEqual(sum(e['kind'] == 'gyro' for e in ev), 10)

    def test_imu_quality_gates(self):
        for kw in [dict(flags='0x123e'), dict(imu_calib='0x10'), dict(imu_gz=''), dict(imu_gz='NaN')]:
            src = self.source([telemetry_row(**kw)])
            self.assertFalse(any(e['kind'] == 'gyro' for e in src.until(11)))
            src.close()

    def test_gnss_quality_gates(self):
        for kw in [dict(fix_valid='0'), dict(sats='3'), dict(heading_deg=''), dict(utc='')]:
            src = self.source([telemetry_row(**kw)])
            self.assertFalse(any(e['kind'] == 'course' for e in src.until(11)))
            src.close()

    def test_regressing_utc_is_not_a_new_fix(self):
        src = self.source([telemetry_row(), telemetry_row(10.1, 10100, utc='2026-10-07T00:00:09Z'),
                           telemetry_row(10.2, 10200), telemetry_row(10.3, 10300, utc='invalid')])
        ev = list(src.until(11))
        self.assertEqual(sum(e['kind'] == 'course' for e in ev), 1)
        self.assertEqual(sum(e['kind'] == 'gyro' for e in ev), 4)

    def test_offsets_sorted_no_future(self):
        src = self.source([telemetry_row(10 + i * .1, 10000 + i * 100) for i in range(10)], gnss_time_offset_s=-.5)
        ev = list(src.until(10.2))
        self.assertEqual(ev[0]['time'], 9.5)
        self.assertEqual([e['time'] for e in ev], sorted(e['time'] for e in ev))
        self.assertTrue(all(e['time'] <= 10.2 for e in ev))

    def test_clock_reset_refused(self):
        src = self.source([telemetry_row(), telemetry_row(11, 10)])
        with self.assertRaisesRegex(ValueError, 'clock'):
            list(src.until(12))

    def test_segment_reset_refused(self):
        src = self.source([])
        session = FusionSession(Profile(), config(), src)
        session.update(wheels(10))
        with self.assertRaisesRegex(ValueError, 'segment'):
            session.update(dict(wheels(11), segment=1))

    def test_late_events_counted_not_retimed(self):
        src = self.source([telemetry_row(t=10)])
        session = FusionSession(Profile(), config(), src)
        session.filter.wheels(wheels(10.1))
        session.last_wheel = 10.1
        result = session.update(wheels(10.2))
        self.assertEqual(result['ekf_late_events'], 2)
        self.assertEqual(result['ekf_sources'], 'wheels')

    def test_truncation_refused(self):
        src = self.source([telemetry_row()])
        list(src.until(11))
        self.path.write_text('')
        with self.assertRaisesRegex(ValueError, 'truncated'):
            list(src.until(12))

    def test_live_only_new_complete_rows(self):
        self.source([telemetry_row()]).close()
        src = TelemetryInput(self.path, config(), follow=True)
        self.addCleanup(src.close)
        self.assertEqual(list(src.until(12)), [])
        row = telemetry_row(10.1, 10100)
        line = ','.join(row[k] for k in src.header) + '\n'
        with self.path.open('a') as f:
            f.write(line[:30])
        self.assertEqual(list(src.until(12)), [])
        with self.path.open('a') as f:
            f.write(line[30:])
        self.assertEqual(len(list(src.until(12))), 2)


class FusionCLITests(unittest.TestCase):
    def test_http_serves_fusion_state(self):
        from demo import Display, start_server
        f = TurningEKF(Profile(), config())
        row = wheels(10)
        f.wheels(row)
        row.update(f.snapshot(10))
        display = Display(False, True)
        display.publish(row)
        server = start_server(0, display)
        try:
            url = 'http://127.0.0.1:%d' % server.server_address[1]
            with urllib.request.urlopen(url + '/state', timeout=2) as response:
                data = json.load(response)
            self.assertEqual(data['latest']['ekf_status'], 'degraded')
            with urllib.request.urlopen(url, timeout=2) as response:
                self.assertIn('ekf-angle', response.read().decode())
        finally:
            server.shutdown()
            server.server_close()

    def test_multisensor_replay_improves_noisy_wheels(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            def run(*args):
                return subprocess.run([sys.executable, str(HERE / 'demo.py'), *map(str, args)],
                                      capture_output=True, text=True, check=True, timeout=30)
            run('synthetic-fusion', '--out', root / 'source')
            run('run', '--input', root / 'source/can_raw.log', '--telemetry', root / 'source/telemetry.csv',
                '--fusion-config', root / 'source/fusion.json', '--out', root / 'result', '--speed', 0)
            with (root / 'source/truth.csv').open() as f:
                truth = {float(r['time']): float(r['yaw_dps']) for r in csv.DictReader(f)}
            with (root / 'result/samples.csv').open() as f:
                rows = list(csv.DictReader(f))
            selected = [r for r in rows if float(r['time']) > 1800000010 and r['ekf_yaw_dps']]
            def rmse(field):
                return math.sqrt(sum((float(r[field]) - truth[float(r['time'])]) ** 2 for r in selected) / len(selected))
            self.assertLess(rmse('ekf_yaw_dps'), rmse('yaw_raw_dps'))
            self.assertLess(rmse('ekf_yaw_dps'), 1)
            self.assertIn('fused', {r['ekf_status'] for r in selected})
            self.assertIn('degraded', {r['ekf_status'] for r in selected})
            self.assertGreater(int(rows[-1]['ekf_rejected']), 0)
            self.assertEqual(int(rows[-1]['ekf_late_events']), 0)

    def test_mount_template_refuses_unmeasured_antenna(self):
        with self.assertRaisesRegex(ValueError, 'antenna_x'):
            FusionConfig(**json.loads((HERE / 'fusion.brio_rs.example.json').read_text()))


if __name__ == '__main__':
    unittest.main()
