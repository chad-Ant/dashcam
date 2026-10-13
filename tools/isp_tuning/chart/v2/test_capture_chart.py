#!/usr/bin/env python3
"""capture_chart.py without cameras: --simulate (videotestsrc) plus a stub v4l2-ctl.

Run on the host like the tool itself:  /usr/bin/python3 test_capture_chart.py
"""
import contextlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import capture_chart as cc

HERE = Path(__file__).resolve().parent

# The UGREEN's real --list-ctrls (2026-10-12), so parsing is tested on the true format.
LIST_CTRLS = """
User Controls

                     brightness 0x00980900 (int)    : min=1 max=64 step=1 default=32 value={brightness}
                           gain 0x00980913 (int)    : min=0 max=15 step=1 default=0 value={gain}
        white_balance_automatic 0x0098090c (bool)   : default=1 value={white_balance_automatic}
      white_balance_temperature 0x0098091a (int)    : min=1800 max=10000 step=1 default=5000 value={white_balance_temperature} flags=inactive

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value={auto_exposure} (Aperture Priority Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=100 value={exposure_time_absolute} flags=inactive
                   pan_absolute 0x009a0908 (int)    : min=-576000 max=576000 step=3600 default=0 value=0
"""
INITIAL = dict(brightness=32, gain=0, white_balance_automatic=1, white_balance_temperature=5000,
               auto_exposure=3, exposure_time_absolute=100)

STUB = r'''#!/usr/bin/env python3
import json, os, sys
state_path, log_path = os.environ['STUB_STATE'], os.environ['STUB_LOG']
state = json.load(open(state_path))
args = sys.argv[1:]
open(log_path, 'a').write(' '.join(args) + '\n')
op = args[2]
if op == '--list-ctrls':
    print(open(os.environ['STUB_LIST']).read().format(**state)); sys.exit(0)
if op.startswith('--get-ctrl='):
    name = op.split('=', 1)[1]
    suffix = ' (Manual Mode)' if name == 'auto_exposure' else ''
    print(f'{name}: {state[name]}{suffix}'); sys.exit(0)
if op.startswith('--set-ctrl='):
    name, value = op.split('=', 1)[1].split('=')
    if os.environ.get('STUB_SLOW_SET'):
        import time; time.sleep(float(os.environ['STUB_SLOW_SET']))
    if os.environ.get('STUB_FAIL_SET') == f'{name}={value}':
        print('VIDIOC_S_EXT_CTRLS: failed: Input/output error', file=sys.stderr); sys.exit(1)
    state[name] = int(value)
    json.dump(state, open(state_path, 'w')); sys.exit(0)
sys.exit(2)
'''


class CaptureChartTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='capture-chart-test-')
        self.root = Path(self.temp.name)
        self.state = self.root / 'state.json'
        self.log = self.root / 'v4l2.log'
        self.state.write_text(json.dumps(INITIAL))
        (self.root / 'list.txt').write_text(LIST_CTRLS)
        stub = self.root / 'v4l2-ctl'
        stub.write_text(STUB)
        stub.chmod(0o755)
        self.env = dict(STUB_STATE=str(self.state), STUB_LOG=str(self.log), STUB_LIST=str(self.root / 'list.txt'))
        self.saved_env = {k: os.environ.get(k) for k in list(self.env) + ['STUB_FAIL_SET', 'STUB_SLOW_SET']}
        os.environ.update(self.env)
        os.environ.pop('STUB_FAIL_SET', None)
        os.environ.pop('STUB_SLOW_SET', None)
        self.saved_v4l2ctl, cc.V4L2CTL = cc.V4L2CTL, str(stub)

    def tearDown(self):
        cc.V4L2CTL = self.saved_v4l2ctl
        for k, v in self.saved_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        self.temp.cleanup()

    def run_main(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = cc.main(list(argv))
        return rc, out.getvalue(), err.getvalue()

    def session(self, name):
        return json.loads((self.root / name / 'session.json').read_text())

    def sets(self):
        return [line.split()[2] for line in self.log.read_text().splitlines()
                if line.split()[2].startswith('--set-ctrl=')] if self.log.exists() else []

    def stub_state(self):
        return json.loads(self.state.read_text())

    # ── captures ──────────────────────────────────────────────────────────────
    def test_colour_both_cameras_spaced(self):
        rc, out, _ = self.run_main('colour', '--simulate', '--settle', '0.3', '--out', str(self.root / 'c'))
        self.assertEqual(rc, 0)
        m = self.session('c')
        self.assertTrue(m['complete'])
        for cam in ('csi', 'ug'):
            names = [f['file'] for f in m['frames'][cam]]
            self.assertEqual(names, [f'colour_{i:03d}.{cam}.jpg' for i in range(5)])
            self.assertEqual(len({f['sha256'] for f in m['frames'][cam]}), 5, 'frames must differ')
            arrivals = [f['arrival_monotonic_ns'] for f in m['frames'][cam]]
            self.assertTrue(all(b - a >= 0.19e9 for a, b in zip(arrivals, arrivals[1:])), arrivals)
            for name in names:
                data = (self.root / 'c' / name).read_bytes()
                self.assertTrue(data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'))
        self.assertIn("--images '" + str(self.root / 'c') + "/colour_*.csi.jpg'", out)
        self.assertIn("--reference '" + str(self.root / 'c') + "/colour_*.ug.jpg'", out)
        self.assertEqual(self.sets(), [], 'no control is written unless --ug-set asks')

    def test_noise_burst_is_consecutive(self):
        rc, out, _ = self.run_main('noise', '--simulate', '--settle', '0', '--frames', '32',
                                   '--out', str(self.root / 'n'))
        self.assertEqual(rc, 0)
        m = self.session('n')
        iv = m['intervals']['csi']
        self.assertEqual(len(m['frames']['csi']), 32)
        self.assertTrue(iv['consecutive'])
        self.assertEqual(iv['gaps_over_1p5_frames'], 0)
        self.assertTrue(all(30 < d < 37 for d in iv['pts_deltas_ms']), iv['pts_deltas_ms'])
        self.assertIn('noise_*.csi.jpg', out)
        self.assertNotIn('ug', m['frames'])

    def test_detail_single_frame_name(self):
        rc, out, _ = self.run_main('detail', '--simulate', '--settle', '0', '--out', str(self.root / 'd'))
        self.assertEqual(rc, 0)
        self.assertTrue((self.root / 'd' / 'detail.csi.jpg').is_file())
        self.assertIn("/detail.csi.jpg'", out)

    def test_ug_only_analysis_uses_standard_decoder(self):
        rc, out, _ = self.run_main('noise', '--simulate', '--cameras', 'ug', '--settle', '0', '--frames', '32',
                                   '--out', str(self.root / 'u'))
        self.assertEqual(rc, 0)
        self.assertIn("noise_*.ug.jpg' --decoder standard", out)

    def test_gap_detection(self):
        none = cc.OFFSET_NONE
        frames = [(b'', int(i * 33.333e6), 0, none) for i in (0, 1, 2, 4, 5)]
        stats = cc.interval_stats(frames, 30, 0)
        self.assertEqual(stats['gaps_over_1p5_frames'], 1)
        self.assertFalse(stats['consecutive'])
        self.assertNotIn('offset_gaps', stats, 'no offsets from the source: PTS only')
        self.assertNotIn('consecutive', cc.interval_stats(frames, 30, 0.2), 'spaced captures have no gap verdict')
        # arrival-stamped PTS can hide one drop (step 37 ms < 50 ms); the buffer offsets show it
        late = [(b'', t, 0, o) for t, o in ((0, 10), (33_000_000, 11), (70_000_000, 13), (103_000_000, 14))]
        stats = cc.interval_stats(late, 30, 0)
        self.assertEqual((stats['gaps_over_1p5_frames'], stats['offset_gaps']), (0, 1))
        self.assertFalse(stats['consecutive'])

    def test_csi_props_are_read_back(self):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        Gst.init(None)
        element = Gst.ElementFactory.make('videobalance')          # saturation 0..2, like nvarguscamerasrc
        cc.apply_props(element, {'saturation': 1.5})
        self.assertAlmostEqual(element.get_property('saturation'), 1.5)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(cc.CaptureFailed):
            cc.apply_props(element, {'saturation': 5.0})           # GObject keeps 1.5 with just a warning

    def test_imx296_state_reads_the_running_device_tree(self):
        mode = self.root / 'dt' / 'i2c@1' / 'rbpcv3_imx296_c@1a' / 'mode0'
        mode.mkdir(parents=True)
        (mode / 'max_gain_val').write_bytes(b'200\0')
        (mode / 'max_exp_time').write_bytes(b'1000\0')
        saved, cc.DT_MODE_GLOB = cc.DT_MODE_GLOB, str(self.root / 'dt' / 'i2c@*' / 'rbpcv3_imx296_*@1a' / 'mode0')
        try:
            state = cc.imx296_state()
        finally:
            cc.DT_MODE_GLOB = saved
        self.assertEqual((state['live_dt_mode0']['max_gain_val'], state['live_dt_mode0']['max_exp_time']),
                         ('200', '1000'))
        self.assertIn('changed_since_boot', state['overlay_on_disk'])

    def test_outside_drive_logs_note(self):
        rc, out, _ = self.run_main('detail', '--simulate', '--settle', '0', '--out', str(self.root / 'o'))
        self.assertEqual(rc, 0)
        self.assertIn('run calibrate.py directly', out)

    # ── refusals: nothing captured, nothing written ──────────────────────────
    def test_existing_directory_refused(self):
        (self.root / 'x').mkdir()
        rc, _, err = self.run_main('colour', '--simulate', '--out', str(self.root / 'x'))
        self.assertEqual(rc, 1)
        self.assertIn('exists', err)
        self.assertEqual(list((self.root / 'x').iterdir()), [])

    def test_argument_refusals(self):
        cases = [
            ('noise', '--frames', '31'), ('colour', '--frames', '17'), ('detail', '--frames', '0'),
            ('colour', '--cameras', 'csi,csi'), ('colour', '--cameras', 'usb'),
            ('colour', '--label', 'bad/label'), ('colour', '--ug-size', '1920'),
            ('colour', '--csi-prop', 'not-a-prop=1'), ('colour', '--csi-prop', 'ee-mode=x'),
            ('colour', '--csi-prop', 'gainrange=8 2'), ('colour', '--csi-prop', 'gainrange=8'),
            ('noise', '--ug-set', 'gain=4'),                                   # ug not among the cameras
            ('colour', '--ug-set', 'zoom_absolute=1'), ('colour', '--ug-set', 'gain=99'),
            ('colour', '--ug-set', 'gain=4', '--ug-set', 'gain=5'), ('colour', '--ug-set', 'gain'),
            ('colour', '--spacing', '-1'), ('detail', '--frames', '2'), ('noise', '--spacing', '0.1'),
        ]
        for i, case in enumerate(cases):
            with self.subTest(case=case):
                out = self.root / f'r{i}'
                rc, _, err = self.run_main(*case, '--simulate', '--out', str(out))
                self.assertEqual(rc, 1, err)
                self.assertIn('REFUSED', err)
                self.assertFalse(out.exists())
        self.assertEqual(self.sets(), [])

    # ── UGREEN controls ───────────────────────────────────────────────────────
    def test_ug_set_order_and_restore(self):
        rc, _, err = self.run_main('colour', '--simulate', '--settle', '0', '--out', str(self.root / 's'),
                                   '--ug-set', 'gain=4', '--ug-set', 'exposure_time_absolute=150',
                                   '--ug-set', 'auto_exposure=1', '--ug-set', 'white_balance_automatic=0',
                                   '--ug-set', 'white_balance_temperature=4500')
        self.assertEqual(rc, 0, err)
        self.assertEqual(self.sets(), [
            # switches first, then values ...
            '--set-ctrl=auto_exposure=1', '--set-ctrl=white_balance_automatic=0',
            '--set-ctrl=exposure_time_absolute=150', '--set-ctrl=gain=4', '--set-ctrl=white_balance_temperature=4500',
            # ... put back in reverse: values while manual is still in force, switches last
            '--set-ctrl=white_balance_temperature=5000', '--set-ctrl=gain=0', '--set-ctrl=exposure_time_absolute=100',
            '--set-ctrl=white_balance_automatic=1', '--set-ctrl=auto_exposure=3'])
        self.assertEqual(self.stub_state(), INITIAL)
        m = self.session('s')
        self.assertEqual([w['control'] for w in m['ug_written']][:2], ['auto_exposure', 'white_balance_automatic'])
        self.assertEqual(m['ug_restore_failed'], [])

    def test_failed_write_aborts_and_restores(self):
        os.environ['STUB_FAIL_SET'] = 'gain=4'
        rc, _, err = self.run_main('colour', '--simulate', '--settle', '0', '--out', str(self.root / 'f'),
                                   '--ug-set', 'auto_exposure=1', '--ug-set', 'gain=4')
        self.assertEqual(rc, 2)
        self.assertIn('cannot set gain=4', err)
        self.assertEqual(self.stub_state(), INITIAL)
        m = self.session('f')
        self.assertFalse(m['complete'])
        self.assertFalse(any(p.suffix == '.jpg' for p in (self.root / 'f').iterdir()))

    def test_simulate_never_writes_a_real_camera(self):
        saved, cc.V4L2CTL = cc.V4L2CTL, 'v4l2-ctl'
        try:
            rc, _, err = self.run_main('colour', '--simulate', '--out', str(self.root / 'real'), '--ug-set', 'gain=4')
        finally:
            cc.V4L2CTL = saved
        self.assertEqual(rc, 1)
        self.assertIn('never writes a real camera', err)
        self.assertFalse((self.root / 'real').exists())

    def test_failed_restore_and_unwritable_session(self):
        # the fix command must come out even when session.json cannot be written afterwards
        os.environ['STUB_FAIL_SET'] = 'gain=0'
        real_write = Path.write_text

        def write_text(path, *a, **k):
            if path.name == 'session.json':
                raise OSError(28, 'No space left on device')
            return real_write(path, *a, **k)
        Path.write_text = write_text
        try:
            rc, _, err = self.run_main('colour', '--simulate', '--settle', '0', '--out', str(self.root / 'h'),
                                       '--ug-set', 'auto_exposure=1', '--ug-set', 'gain=4')
        finally:
            Path.write_text = real_write
        self.assertEqual(rc, 3)
        self.assertIn('--set-ctrl=gain=0', err)
        self.assertIn('could not write', err)
        self.assertLess(err.index('could NOT put back'), err.index('could not write'))

    def test_failed_restore_prints_the_fix(self):
        os.environ['STUB_FAIL_SET'] = 'gain=0'
        rc, _, err = self.run_main('colour', '--simulate', '--settle', '0', '--out', str(self.root / 'g'),
                                   '--ug-set', 'auto_exposure=1', '--ug-set', 'gain=4')
        self.assertEqual(rc, 3)
        self.assertIn('could NOT put back gain', err)
        self.assertIn('--set-ctrl=gain=0', err)
        self.assertEqual(self.stub_state()['auto_exposure'], 3, 'the others are still put back')
        self.assertEqual(self.session('g')['ug_restore_failed'][0][:2], ['gain', 0])

    def sigterm_run(self, out, sig=signal.SIGTERM):
        proc = subprocess.Popen([sys.executable, str(HERE / 'capture_chart.py'), 'colour', '--simulate',
                                 '--settle', '30', '--out', str(out), '--ug-set', 'auto_exposure=1',
                                 '--ug-set', 'gain=4'],
                                env=dict(os.environ, CAPTURE_CHART_V4L2CTL=cc.V4L2CTL, PYTHONDONTWRITEBYTECODE='1'),
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic() + 20
        while not (self.stub_state()['gain'] == 4 and 'gain=4' in self.log.read_text()):
            self.assertLess(time.monotonic(), deadline, 'never wrote the controls')
            time.sleep(0.05)
        time.sleep(1.0)                                  # streaming, inside the settle
        proc.send_signal(sig)
        return proc

    def test_hangup_restores(self):
        out = self.root / 'hup'
        proc = self.sigterm_run(out, signal.SIGHUP)      # the SSH session dropped
        _, err = proc.communicate(timeout=20)
        self.assertEqual(proc.returncode, 129, err)
        self.assertEqual(self.stub_state(), INITIAL)
        self.assertEqual(json.loads((out / 'session.json').read_text())['error'], 'interrupted')

    def test_second_sigterm_does_not_cut_the_restore(self):
        os.environ['STUB_SLOW_SET'] = '0.6'              # each write (and put-back) takes 0.6 s
        out = self.root / 't2'
        proc = self.sigterm_run(out)
        deadline = time.monotonic() + 20
        while 'gain=0' not in self.log.read_text():      # the restore has started
            self.assertLess(time.monotonic(), deadline, 'restore never started')
            time.sleep(0.02)
        proc.send_signal(signal.SIGTERM)                 # the second stop, mid-restore
        proc.send_signal(signal.SIGINT)
        _, err = proc.communicate(timeout=20)
        self.assertEqual(proc.returncode, 143, err)
        self.assertEqual(self.stub_state(), INITIAL)

    def test_sigterm_restores_and_records(self):
        out = self.root / 't'
        proc = self.sigterm_run(out)
        _, err = proc.communicate(timeout=20)
        self.assertEqual(proc.returncode, 143, err)
        self.assertEqual(self.stub_state(), INITIAL)
        m = json.loads((out / 'session.json').read_text())
        self.assertFalse(m['complete'])
        self.assertEqual(m['error'], 'interrupted')


if __name__ == '__main__':
    unittest.main(verbosity=2)
