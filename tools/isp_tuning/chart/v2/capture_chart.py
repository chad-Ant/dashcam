#!/usr/bin/env python3
"""Capture the v2 chart pack for calibrate.py: one continuous session per camera, then session.json.

Run on the Jetson HOST with /usr/bin/python3 (Argus and the cameras are local; no container), with
dashcam-v04 stopped, one sheet mounted as README.md describes:

  /usr/bin/python3 capture_chart.py colour      # 5 frames from the CSI and 5 from the UGREEN
  /usr/bin/python3 capture_chart.py noise       # 64 consecutive CSI frames
  /usr/bin/python3 capture_chart.py detail      # 1 CSI frame, saved as detail.csi.jpg

It writes a NEW directory (default /home/jetson/drive_logs/chart_captures/<sheet>_<stamp>[_<label>])
with the names CALIBRATION.md's commands expect (colour_000.csi.jpg, colour_000.ug.jpg, noise_000.csi.jpg,
detail.csi.jpg) and session.json: pipelines, settings, the UGREEN's controls before and after, the
installed ISP override / IMX296 overlay / module hashes, and every frame's PTS, arrival time and size.
Then it prints the run_calibration.sh command for the directory.

CSI: nvarguscamerasrc -> nvjpegenc quality 95 (the rig JPEGs calibrate.py's decoder recognises) at 30 fps,
the recorder's rate (with a staged IMX296 candidate overlay installed, stay at 30 fps: see its README).
It settles, then locks AE and AWB as tune_session.py does, unless --auto.  It sets no other property
(the recorder sets none) unless --csi-prop asks.
UGREEN: the camera's own MJPEG, not re-encoded, 1920x1080 at 30 fps (the recording mode).  Its controls
are NOT written unless --ug-set asks; whatever is written is put back afterwards and read back.  Standard
UVC controls only, never the vendor extension unit.  Under the camera's auto-exposure the exposure/gain
readback is stale, so a fixed setting has to be given (--ug-set auto_exposure=1 exposure_time_absolute=..
gain=.. white_balance_automatic=0 white_balance_temperature=..); noise bursts need one.

The two cameras run in the same window but are not synchronised.  Lag units in calibrate.py are file
intervals.  A noise burst must be consecutive (no PTS step over 1.5 frame times and, where the source
numbers its buffers, no offset gap), otherwise the run fails with exit 2 and the files are kept.
session.json records the running IMX296 device tree next to the on-disk overlay/module hashes.

Exit: 0 complete; 1 refused (arguments, recorder running, existing directory); 2 capture failed;
3 a UGREEN control could not be put back (the fix command is printed); 130 Ctrl-C, 129/131/143 SIGHUP/
SIGQUIT/SIGTERM (controls are put back first; a second signal does not cut that short).
--simulate swaps both cameras for videotestsrc and skips the recorder check (tests, no hardware); it
refuses --ug-set unless CAPTURE_CHART_V4L2CTL names a stub.
"""
import argparse
import glob
import hashlib
import json
import os
import pathlib
import re
import signal
import subprocess
import sys
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent
UG_DEVICE = '/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0'
DRIVE_LOGS = pathlib.Path('/home/jetson/drive_logs')       # the only captures run_calibration.sh can see
CAPTURE_ROOT = DRIVE_LOGS / 'chart_captures'
ISP = pathlib.Path('/var/nvidia/nvcam/settings/camera_overrides.isp')
DTBO = pathlib.Path('/boot/tegra234-p3767-camera-p3768-imx296-cam1.dtbo')
DT_MODE_GLOB = '/proc/device-tree/bus@0/cam_i2cmux/i2c@*/rbpcv3_imx296_*@1a/mode0'   # the running device tree
DT_KEYS = ('max_gain_val', 'min_gain_val', 'max_exp_time', 'min_exp_time', 'default_framerate', 'max_framerate',
           'gain_factor', 'exposure_factor')
V4L2CTL = os.environ.get('CAPTURE_CHART_V4L2CTL', 'v4l2-ctl')   # tests point this at a stub
OFFSET_NONE = 2**64 - 1                                          # GST_BUFFER_OFFSET_NONE

SHEETS = {   # default frame count, the range calibrate.py accepts, default cameras, spacing between frames (s)
    'colour': dict(frames=5, lo=1, hi=16, cameras='csi,ug', spacing=0.2),
    'noise': dict(frames=64, lo=32, hi=256, cameras='csi', spacing=0.0),
    'detail': dict(frames=1, lo=1, hi=1, cameras='csi', spacing=0.0),      # calibrate.py: one image per run
}
CSI_PROPS = {   # nvarguscamerasrc properties --csi-prop accepts; 'range' = "LOW HIGH"
    'ee-mode': int, 'ee-strength': float, 'tnr-mode': int, 'tnr-strength': float,
    'saturation': float, 'wbmode': int, 'exposurecompensation': float, 'aeantibanding': int,
    'exposuretimerange': 'range', 'gainrange': 'range', 'ispdigitalgainrange': 'range',
}
# UGREEN controls --ug-set may write.  Mode switches are written first and put back last, so a value
# control is always written and restored while its manual mode is in force.
UG_SWITCHES = ('auto_exposure', 'white_balance_automatic', 'focus_automatic_continuous')
UG_VALUES = ('exposure_time_absolute', 'gain', 'white_balance_temperature', 'focus_absolute', 'brightness',
             'contrast', 'saturation', 'hue', 'gamma', 'sharpness', 'backlight_compensation',
             'power_line_frequency')


class Refused(Exception):
    """Bad arguments or an unsafe state: nothing was captured or written to a camera."""


class CaptureFailed(Exception):
    """The capture started but did not complete."""


def sha256(path):
    try:
        return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
    except OSError as error:
        return f'unreadable: {error.strerror}'


def run(cmd, timeout=5):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def boot_time():
    try:
        with open('/proc/stat') as stat:
            for line in stat:
                if line.startswith('btime '):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


def file_state(path, btime):
    """A file's sha256, and whether it changed after boot (then it may not be what is running)."""
    p = pathlib.Path(path)
    try:
        changed = btime is not None and p.stat().st_mtime > btime
    except OSError:
        changed = None
    return dict(path=str(p), sha256=sha256(p), changed_since_boot=changed)


def imx296_state():
    """What the CSI runs: the live device tree (fixed at boot) and the overlay/module files on disk.

    install_candidate.sh replaces the files, which take effect only at the next boot (or module reload), so
    the on-disk hashes alone can name the wrong candidate; changed_since_boot flags that case."""
    btime = boot_time()
    live = {}
    for mode in sorted(glob.glob(DT_MODE_GLOB))[:1]:
        live['node'] = mode
        for key in DT_KEYS:
            try:
                live[key] = pathlib.Path(mode, key).read_bytes().rstrip(b'\0').decode()
            except (OSError, UnicodeDecodeError):
                pass
    module = run(['modinfo', '-n', 'imx296'])
    return dict(live_dt_mode0=live or 'not found', overlay_on_disk=file_state(DTBO, btime),
                module_on_disk=file_state(module.stdout.strip(), btime) if module.returncode == 0 else 'not found',
                module_loaded=os.path.isdir('/sys/module/imx296'))


# ─── UGREEN controls (v4l2-ctl) ──────────────────────────────────────────────

def list_ctrls(device):
    r = run([V4L2CTL, '-d', device, '--list-ctrls'])
    if r.returncode != 0:
        raise Refused(f'cannot read controls of {device}: {r.stderr.strip()}')
    ctrls = {}
    for line in r.stdout.splitlines():
        m = re.match(r'\s*(\w+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:\s*(.*)$', line)
        if m:
            fields = dict(re.findall(r'(\w+)=(-?\d+)', m.group(3)))
            ctrls[m.group(1)] = dict(type=m.group(2), **{k: int(v) for k, v in fields.items()})
    return r.stdout, ctrls


def get_ctrl(device, name):
    r = run([V4L2CTL, '-d', device, f'--get-ctrl={name}'])
    m = re.match(rf'\s*{name}:\s*(-?\d+)(?:\s+\(.*\))?\s*$', r.stdout)     # menus add "(label)"
    if r.returncode != 0 or not m:
        raise CaptureFailed(f'cannot read {name}: {r.stderr.strip() or r.stdout.strip()}')
    return int(m.group(1))


def set_ctrl(device, name, value):
    r = run([V4L2CTL, '-d', device, f'--set-ctrl={name}={value}'])
    if r.returncode != 0:
        raise CaptureFailed(f'cannot set {name}={value}: {r.stderr.strip()}')
    got = get_ctrl(device, name)
    if got != value:
        raise CaptureFailed(f'{name} reads {got} after writing {value}')


def parse_ug_set(items, ctrls):
    """['gain=5', ...] -> [(name, value)] in write order (switches first); checked against the device."""
    wanted = {}
    for item in items:
        name, sep, value = item.partition('=')
        if not sep or not re.fullmatch(r'-?\d+', value):
            raise Refused(f'--ug-set {item!r}: expected CONTROL=INTEGER')
        if name not in UG_SWITCHES + UG_VALUES:
            raise Refused(f'--ug-set {name}: not one of {", ".join(UG_SWITCHES + UG_VALUES)}')
        if name in wanted:
            raise Refused(f'--ug-set {name} given twice')
        info = ctrls.get(name)
        if info is None:
            raise Refused(f'--ug-set {name}: the camera has no such control')
        v = int(value)
        if 'min' in info and not info['min'] <= v <= info['max']:
            raise Refused(f'--ug-set {name}={v}: outside {info["min"]}..{info["max"]}')
        wanted[name] = v
    return [(n, wanted[n]) for n in UG_SWITCHES + UG_VALUES if n in wanted]


def restore_ug(device, written):
    """Put back [(name, before)] in reverse write order; returns the controls that did not come back."""
    failed = []
    for name, before in reversed(written):
        try:
            set_ctrl(device, name, before)
        except (CaptureFailed, subprocess.SubprocessError, OSError) as error:
            failed.append((name, before, str(error)))
    return failed


# ─── capture ─────────────────────────────────────────────────────────────────

class Camera:
    """One pipeline ending in an appsink; its streaming thread keeps the frames asked for."""

    def __init__(self, Gst, name, desc, fps):
        self.Gst, self.name, self.desc, self.fps = Gst, name, desc, fps
        self.pipe = Gst.parse_launch(desc)
        self.src = self.pipe.get_by_name('cam')
        self.lock = threading.Lock()
        self.seen = self.bad = 0
        self.caps = None
        self.want, self.spacing_ns, self.last_take, self.frames = 0, 0, None, []
        sink = self.pipe.get_by_name('sink')
        sink.set_property('emit-signals', True)
        sink.connect('new-sample', self._on_sample)

    def _on_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return self.Gst.FlowReturn.ERROR
        now = time.monotonic_ns()
        buf = sample.get_buffer()
        with self.lock:
            self.seen += 1
            if self.caps is None:
                self.caps = sample.get_caps().to_string()
            due = self.spacing_ns == 0 or self.last_take is None or now - self.last_take >= self.spacing_ns
            if len(self.frames) < self.want and due:
                data = buf.extract_dup(0, buf.get_size())
                if data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'):
                    self.frames.append((data, buf.pts, now, buf.offset))
                    self.last_take = now
                else:
                    self.bad += 1
        return self.Gst.FlowReturn.OK

    def record(self, n, spacing):
        with self.lock:
            self.frames, self.last_take = [], None
            self.spacing_ns = int(spacing * 1e9)
            self.want = n

    def taken(self):
        with self.lock:
            return len(self.frames)

    def check_bus(self):
        msg = self.pipe.get_bus().pop_filtered(self.Gst.MessageType.ERROR | self.Gst.MessageType.EOS)
        if msg is not None:
            if msg.type == self.Gst.MessageType.EOS:
                raise CaptureFailed(f'{self.name}: stream ended')
            error, debug = msg.parse_error()
            raise CaptureFailed(f'{self.name}: {error.message} ({debug})')


def wait_for(cameras, done, seconds, what):
    deadline = time.monotonic() + seconds
    while not all(done(c) for c in cameras):
        for c in cameras:
            c.check_bus()
        if time.monotonic() > deadline:
            raise CaptureFailed(f'{what}: ' + ', '.join(f'{c.name} {c.taken()}/{c.want} frames, '
                                                        f'{c.seen} seen' for c in cameras))
        time.sleep(0.02)


def pipelines(args, cams):
    out = {}
    tail = 'appsink name=sink max-buffers=2 drop=true sync=false'
    if args.simulate:
        for name, size in (('csi', '320,height=240'), ('ug', '384,height=216')):
            if name in cams:
                out[name] = (f'videotestsrc name=cam is-live=true pattern=ball ! video/x-raw,width={size},'
                             f'framerate=30/1 ! jpegenc quality=95 ! {tail}', 30)
        return out
    if 'csi' in cams:
        out['csi'] = (f'nvarguscamerasrc name=cam sensor-id={args.csi_sensor_id} ! video/x-raw(memory:NVMM),'
                      f'width=1456,height=1088,format=NV12,framerate={args.csi_fps}/1 ! nvjpegenc quality=95 ! '
                      f'{tail}', args.csi_fps)
    if 'ug' in cams:
        w, h = args.ug_size.split('x')
        out['ug'] = (f'v4l2src name=cam device={args.ug_device} ! image/jpeg,width={w},height={h},'
                     f'framerate={args.ug_fps}/1 ! {tail}', args.ug_fps)
    return out


def parse_csi_props(items):
    props = {}
    for item in items:
        key, sep, value = item.partition('=')
        kind = CSI_PROPS.get(key)
        if not sep or kind is None:
            raise Refused(f'--csi-prop {item!r}: expected one of {", ".join(CSI_PROPS)} =VALUE')
        try:
            if kind == 'range':
                low, high = (float(x) for x in value.split())
                if low > high:
                    raise ValueError
                props[key] = ' '.join(value.split())
            else:
                props[key] = kind(value)
        except ValueError:
            raise Refused(f'--csi-prop {item!r}: bad value') from None
    return props


def apply_props(element, props):
    """Set each property and read it back: GObject keeps the old value of an out-of-range one, with only a
    warning on stderr."""
    for key, value in props.items():
        element.set_property(key, value)
        got = element.get_property(key)
        if isinstance(value, float):
            ok = abs(float(got) - value) <= 1e-4 * max(1.0, abs(value))
        elif isinstance(value, int):
            ok = int(got) == value
        else:
            ok = str(got) == value
        if not ok:
            raise CaptureFailed(f'--csi-prop {key}={value}: the element kept {got} (out of range?)')


def interval_stats(frames, fps, spacing):
    """PTS steps; for a burst (spacing 0) whether it is consecutive: no PTS step over 1.5 frame times and,
    where the source numbers its buffers (v4l2src: the V4L2 sequence), offset steps of exactly 1.  A PTS
    stamped on arrival (nvarguscamerasrc do-timestamp) can hide a single drop; the offsets cannot."""
    pts = [f[1] for f in frames]
    offsets = [f[3] for f in frames]
    deltas = [(b - a) / 1e6 for a, b in zip(pts, pts[1:])]
    stats = dict(pts_deltas_ms=[round(d, 3) for d in deltas])
    if deltas:
        stats.update(min_ms=round(min(deltas), 3), max_ms=round(max(deltas), 3))
        if spacing == 0:
            frame_ms = 1000 / fps
            stats['gaps_over_1p5_frames'] = sum(d > 1.5 * frame_ms for d in deltas)
            consecutive = stats['gaps_over_1p5_frames'] == 0 and min(deltas) > 0
            if all(o != OFFSET_NONE for o in offsets):
                stats['offset_gaps'] = sum(b - a != 1 for a, b in zip(offsets, offsets[1:]))
                consecutive = consecutive and stats['offset_gaps'] == 0
            stats['consecutive'] = consecutive
    return stats


def analyse_commands(out, sheet, files):
    script = HERE / 'run_calibration.sh'
    cmds = []
    have = {cam for cam in files if files[cam]}
    if sheet == 'colour' and 'csi' in have:
        ref = f" \\\n  --reference '{out}/colour_*.ug.jpg'" if 'ug' in have else ''
        cmds.append(f"bash {script} colour \\\n  --images '{out}/colour_*.csi.jpg'{ref}")
    else:
        for cam in sorted(have):
            pattern = 'detail.' + cam + '.jpg' if files[cam] == [f'detail.{cam}.jpg'] else f'{sheet}_*.{cam}.jpg'
            decoder = ' --decoder standard' if cam == 'ug' else ''
            cmds.append(f"bash {script} {sheet} \\\n  --images '{out}/{pattern}'{decoder}")
    if not out.is_relative_to(DRIVE_LOGS):
        cmds.append(f'# {out} is outside {DRIVE_LOGS}, which is all run_calibration.sh mounts: run calibrate.py '
                    'directly with --out (CALIBRATION.md, "Direct invocation")')
    return cmds


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('sheet', choices=SHEETS)
    p.add_argument('--cameras', help='csi, ug or csi,ug (default per sheet: colour csi,ug; noise/detail csi)')
    p.add_argument('--frames', type=int, help='frames per camera (default colour 5, noise 64; detail 1 only)')
    p.add_argument('--spacing', type=float, help='seconds between saved colour frames (default 0.2; noise '
                                                 'bursts are always consecutive)')
    p.add_argument('--settle', type=float, default=7.0, help='seconds of streaming before the lock (default 7)')
    p.add_argument('--out', type=pathlib.Path, help='NEW directory (default under ' f'{CAPTURE_ROOT})')
    p.add_argument('--label', help='appended to the default directory name ([A-Za-z0-9._-], up to 40)')
    p.add_argument('--auto', action='store_true', help='CSI: no AE/AWB lock (production-like)')
    p.add_argument('--csi-prop', action='append', default=[], metavar='KEY=VALUE',
                   help='an nvarguscamerasrc property (repeatable): ' + ', '.join(CSI_PROPS))
    p.add_argument('--csi-fps', type=int, default=30, choices=(15, 30, 60))
    p.add_argument('--csi-sensor-id', type=int, default=0)
    p.add_argument('--ug-set', action='append', default=[], metavar='CONTROL=VALUE',
                   help='write a UGREEN control for the capture, put back afterwards (repeatable)')
    p.add_argument('--ug-size', default='1920x1080', help='UGREEN MJPEG mode WxH (default 1920x1080)')
    p.add_argument('--ug-fps', type=int, default=30, choices=(30, 60))
    p.add_argument('--ug-device', default=UG_DEVICE)
    p.add_argument('--simulate', action='store_true', help='videotestsrc instead of the cameras (tests)')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return capture(args)
    except Refused as error:
        print(f'capture_chart: REFUSED: {error}', file=sys.stderr)
        return 1


def capture(args):
    sheet = SHEETS[args.sheet]
    cams = (args.cameras or sheet['cameras']).split(',')
    if not cams or len(set(cams)) != len(cams) or not set(cams) <= {'csi', 'ug'}:
        raise Refused('--cameras: csi, ug or csi,ug')
    n = sheet['frames'] if args.frames is None else args.frames
    if not sheet['lo'] <= n <= sheet['hi']:
        raise Refused(f'--frames {n}: calibrate.py takes {sheet["lo"]}..{sheet["hi"]} for {args.sheet}')
    spacing = sheet['spacing'] if args.spacing is None else args.spacing
    if not 0 <= spacing <= 10 or not 0 <= args.settle <= 120:
        raise Refused('--spacing must be 0..10 s and --settle 0..120 s')
    if args.sheet == 'noise' and spacing != 0:
        raise Refused('--spacing: a noise burst is consecutive frames (spacing 0)')
    if not re.fullmatch(r'\d{3,4}x\d{3,4}', args.ug_size):
        raise Refused('--ug-size: WxH, e.g. 1920x1080')
    if args.label is not None and not re.fullmatch(r'[A-Za-z0-9._-]{1,40}', args.label):
        raise Refused('--label: up to 40 of [A-Za-z0-9._-]')
    props = parse_csi_props(args.csi_prop)
    if args.ug_set and 'ug' not in cams:
        raise Refused('--ug-set needs the ug camera in --cameras')
    if args.simulate and args.ug_set and V4L2CTL == 'v4l2-ctl':
        raise Refused('--simulate never writes a real camera: --ug-set needs CAPTURE_CHART_V4L2CTL (a stub)')
    if not args.simulate:
        if run(['systemctl', 'is-active', '--quiet', 'dashcam-v04']).returncode == 0:
            raise Refused('dashcam-v04 is running: stop it first (sudo systemctl stop dashcam-v04)')
        if 'ug' in cams and not os.path.exists(args.ug_device):
            raise Refused(f'no UGREEN at {args.ug_device}')
    ctrls_text, ctrls = list_ctrls(args.ug_device) if 'ug' in cams and (args.ug_set or not args.simulate) \
        else (None, {})
    ug_plan = parse_ug_set(args.ug_set, ctrls)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    out = args.out or CAPTURE_ROOT / (f'{args.sheet}_{stamp}' + (f'_{args.label}' if args.label else ''))
    out = out.resolve()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        raise Refused(f'{out} exists: captures never go into an existing directory') from None

    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import GLib, Gst
    Gst.init(None)

    meta = dict(schema='imx296-chart-v2-capture', sheet=args.sheet, cameras=cams, frames_per_camera=n,
                spacing_s=spacing, settle_s=args.settle, simulate=args.simulate, args=vars(args),
                started=time.time(), kernel=os.uname().release,
                isp_override_sha256=sha256(ISP), imx296=imx296_state(),
                csi_ae_awb_lock=('csi' in cams and not args.auto and not args.simulate),
                csi_props=props, ug_controls_before=ctrls_text, ug_written=[], complete=False)
    cameras, written, rc = [], [], 0

    # SIGTERM, a dropped SSH session (SIGHUP) and SIGQUIT stop like Ctrl-C: the finally puts controls back.
    stop_signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT)
    saved_handlers = {s: signal.getsignal(s) for s in stop_signals}

    def on_stop(signum, _frame):
        raise SystemExit(128 + signum)
    for s in stop_signals[1:]:
        signal.signal(s, on_stop)
    try:
        for name, value in ug_plan:                       # switches first, then values
            written.append((name, get_ctrl(args.ug_device, name)))
            set_ctrl(args.ug_device, name, value)
            meta['ug_written'].append(dict(control=name, before=written[-1][1], value=value))
        for name, (desc, fps) in pipelines(args, cams).items():
            cam = Camera(Gst, name, desc, fps)
            if name == 'csi' and not args.simulate:
                apply_props(cam.src, props)
            cameras.append(cam)
        meta['pipelines'] = {c.name: c.desc for c in cameras}
        for c in cameras:
            if c.pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise CaptureFailed(f'{c.name}: cannot start the pipeline')
        print(f'capture_chart: streaming {", ".join(c.name for c in cameras)}; settling {args.settle:g} s',
              flush=True)
        wait_for(cameras, lambda c: c.seen > 0, 15, 'no first frame')
        t_end = time.monotonic() + args.settle
        while time.monotonic() < t_end:
            for c in cameras:
                c.check_bus()
            time.sleep(0.05)
        if meta['csi_ae_awb_lock']:
            src = next(c.src for c in cameras if c.name == 'csi')
            src.set_property('aelock', True)
            src.set_property('awblock', True)
            time.sleep(1.5)
        if 'ug' in cams and not args.simulate:
            try:
                meta['ug_controls_during'] = list_ctrls(args.ug_device)[0]
            except Refused as error:
                meta['ug_controls_during'] = str(error)
        for c in cameras:
            c.record(n, spacing)
        expected = max(n / min(c.fps for c in cameras), n * spacing)
        wait_for(cameras, lambda c: c.taken() >= n, 2 * expected + 10, 'burst incomplete')
        meta['caps'] = {c.name: c.caps for c in cameras}
        for c in cameras:
            c.pipe.set_state(Gst.State.NULL)
        files = {}
        for c in cameras:
            with c.lock:
                frames = list(c.frames[:n])
            names = [f'detail.{c.name}.jpg'] if args.sheet == 'detail' and n == 1 else \
                [f'{args.sheet}_{i:03d}.{c.name}.jpg' for i in range(n)]
            entries = []
            for fname, (data, pts, arrival, offset) in zip(names, frames):
                (out / fname).write_bytes(data)
                entries.append(dict(file=fname, pts_ns=pts, arrival_monotonic_ns=arrival, bytes=len(data),
                                    offset=None if offset == OFFSET_NONE else offset,
                                    sha256=hashlib.sha256(data).hexdigest()))
            files[c.name] = names
            meta.setdefault('frames', {})[c.name] = entries
            meta.setdefault('intervals', {})[c.name] = dict(interval_stats(frames, c.fps, spacing),
                                                            seen=c.seen, rejected_incomplete_jpeg=c.bad)
        if args.sheet == 'noise':
            broken = [c.name for c in cameras if not meta['intervals'][c.name].get('consecutive')]
            if broken:
                raise CaptureFailed(f'the noise burst is not consecutive for {", ".join(broken)} (a dropped frame or '
                                    'a slower camera; session.json "intervals"): capture again')
        meta['analyse'] = analyse_commands(out, args.sheet, files)
        meta['complete'] = True
    except (CaptureFailed, Refused, subprocess.SubprocessError, OSError, GLib.Error, TypeError,
            ValueError) as error:
        meta['error'] = str(error)
        print(f'capture_chart: FAILED: {error}', file=sys.stderr)
        rc = 2
    except (KeyboardInterrupt, SystemExit) as stop:
        rc = 130 if isinstance(stop, KeyboardInterrupt) else stop.code if isinstance(stop.code, int) else 143
        meta['error'] = 'interrupted'
    finally:
        # A second Ctrl-C, SIGTERM or hangup must not cut the restore short.
        for s in stop_signals:
            signal.signal(s, signal.SIG_IGN)
        # The UGREEN first, and its message at once: UVC controls can be written while it streams, while an
        # Argus teardown can block for seconds, and nothing below may stand between a failure and its fix.
        failed = restore_ug(args.ug_device, written) if written else []
        if failed:
            rc = 3
            fix = ','.join(f'{name}={before}' for name, before, _ in reversed(failed))
            print(f'capture_chart: could NOT put back {", ".join(f[0] for f in failed)}; run:\n'
                  f'  v4l2-ctl -d {args.ug_device} --set-ctrl={fix}', file=sys.stderr, flush=True)
        for c in cameras:
            c.pipe.set_state(Gst.State.NULL)
        meta['ug_restore_failed'] = failed
        if 'ug' in cams and not args.simulate:
            try:
                meta['ug_controls_after'] = list_ctrls(args.ug_device)[0]
            except (Refused, subprocess.SubprocessError, OSError) as error:
                meta['ug_controls_after'] = str(error)
        meta['isp_override_unchanged'] = sha256(ISP) == meta['isp_override_sha256']
        meta['finished'] = time.time()
        try:
            (out / 'session.json').write_text(json.dumps(meta, indent=2, default=str) + '\n')
        except OSError as error:
            print(f'capture_chart: could not write {out}/session.json: {error}', file=sys.stderr)
            rc = rc or 2
        for s, handler in saved_handlers.items():
            signal.signal(s, handler)
    if rc == 0:
        print(f'capture_chart: complete: {out}')
        for c in cameras:
            iv = meta['intervals'][c.name]
            gaps = ', consecutive' if iv.get('consecutive') else ''
            size = 'x'.join(re.findall(r'(?:width|height)=\(int\)(\d+)', c.caps or ''))
            print(f'  {c.name}: {len(meta["frames"][c.name])} frames, {size}{gaps}')
        print('analyse with:\n' + '\n'.join(meta['analyse']))
    return rc


if __name__ == '__main__':
    sys.exit(main())
