"""Bounded live runtime sweep; no ISP installation or USB control writes.

Run with /usr/bin/python3 tune_session.py NEW_OUTPUT_DIRECTORY.
Uses one continuous Argus session, locks AE/AWB after settling (not with --auto), retains paired
USB references for drift checks, and never reuses an existing output directory.
Each setting has five frames separated by at least 73 ms. Not hardware synchronized.
--manual selects bench-only driver-specific exposure/gain; do not use on the road.
--auto: no fixed exposure/gain and no AE/AWB lock (production-like, e.g. daylight); not with --manual.
--with-unset: first capture 5 frames before any saturation property is set ('unset': ee-mode=0 from the
launch string, ISP saturation), i.e. what an app that sets no saturation gets.
"""
import hashlib
import json
import pathlib
import signal
import subprocess
import sys
import time

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

UG = '/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0'
ISP = pathlib.Path('/var/nvidia/nvcam/settings/camera_overrides.isp')


def controls(device):
    r = subprocess.run(['v4l2-ctl', '-d', device, '--list-ctrls'],
                       capture_output=True, text=True, timeout=5, check=True)
    return r.stdout


def pull(pipe):
    sample = pipe.get_by_name('sink').emit('try-pull-sample', 5 * Gst.SECOND)
    msg = pipe.get_bus().pop_filtered(Gst.MessageType.ERROR)
    if msg:
        raise RuntimeError(str(msg.parse_error()))
    if sample is None:
        raise RuntimeError('No fresh frame within 5 seconds')
    buf = sample.get_buffer()
    data = buf.extract_dup(0, buf.get_size())
    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise RuntimeError('Incomplete JPEG')
    return data, int(buf.pts)


def settle(pipe, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        pull(pipe)


def main():
    if subprocess.run(['systemctl', 'is-active', '--quiet', 'dashcam-v04']).returncode == 0:
        raise RuntimeError('Recorder is active; refusing to contend for cameras')
    manual = '--manual' in sys.argv[2:]
    auto = '--auto' in sys.argv[2:]
    with_unset = '--with-unset' in sys.argv[2:]
    if manual and auto:
        raise RuntimeError('--manual and --auto exclude each other')
    dest = pathlib.Path(sys.argv[1]); dest.mkdir(exist_ok=False)
    isp = ISP.read_bytes(); (dest / 'installed_before.isp').write_bytes(isp)
    meta = dict(isp_sha256=hashlib.sha256(isp).hexdigest(), started=time.time(), manual=manual,
                auto=auto, with_unset=with_unset, usb_before=controls(UG), captures=[], complete=False)
    Gst.init(None)
    pipes = []
    try:
        # This rig's vendor driver treats exposure as lines (14.815 us) and gain
        # as 0.1 dB codes with DT gain_factor=16. These are NOT generic Argus units:
        # 675000 ns -> 675 lines ~= 10 ms; 11.5 -> code 184 (18.4 dB).
        fixed = 'exposuretimerange="675000 675000" gainrange="11.5 11.5" ispdigitalgainrange="1 1" ' if manual else ''
        csi = Gst.parse_launch('nvarguscamerasrc name=cam sensor-id=0 ee-mode=0 tnr-mode=1 ' + fixed + '! '
              'video/x-raw(memory:NVMM),width=1456,height=1088,format=NV12,framerate=60/1 ! '
              'nvjpegenc quality=95 ! appsink name=sink max-buffers=1 drop=true sync=false')
        pipes.append(csi)
        usb = Gst.parse_launch(f'v4l2src device={UG} ! image/jpeg,width=1920,height=1080,framerate=30/1 ! '
                               'appsink name=sink max-buffers=1 drop=true sync=false')
        pipes.append(usb)
        for pipe in pipes:
            if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError('Cannot start capture')
        settle(csi, 7)
        pull(usb)
        cam = csi.get_by_name('cam')
        if not auto:
            cam.set_property('aelock', True); cam.set_property('awblock', True)
        meta['csi_locked_controls'] = controls('/dev/video0')

        def shoot(label, **settings):
            for repeat in range(5):
                settle(csi, .073)
                name = f'{label}_{repeat}'
                entry = dict(name=name, **settings, ae_lock=not auto, awb_lock=not auto, time=time.time())
                for kind, pipe in [('csi', csi), ('ug', usb)]:
                    data, pts = pull(pipe)
                    (dest / f'{name}.{kind}.jpg').write_bytes(data)
                    entry[kind + '_pts_ns'] = pts
                meta['captures'].append(entry)
            meta.setdefault('controls_per_variant', {})[label] = controls('/dev/video0')
            print('captured', label, flush=True)

        if with_unset:                      # before any saturation/ee-strength property is set
            settle(csi, 1.5)
            shoot('unset', ee_mode=0, ee_strength=None, saturation=None)
        variants = [('base', 0, -1., 1.), ('ee005', 1, .05, 1.),
                    ('ee010', 1, .10, 1.), ('ee020', 1, .20, 1.),
                    ('ee_default', 1, -1., 1.), ('sat090', 0, -1., .90), ('sat110', 0, -1., 1.10),
                    ('sat120', 0, -1., 1.20), ('sat130', 0, -1., 1.30),
                    ('base_end', 0, -1., 1.)]
        for label, mode, strength, saturation in variants:
            cam.set_property('ee-mode', mode)
            cam.set_property('ee-strength', strength)
            cam.set_property('saturation', saturation)
            settle(csi, 1.5)
            shoot(label, ee_mode=mode, ee_strength=strength, saturation=saturation)
        meta['complete'] = True
    finally:
        for pipe in pipes:
            pipe.set_state(Gst.State.NULL)
        meta['finished'] = time.time()
        try:
            meta['usb_after'] = controls(UG)
            meta['isp_unchanged'] = ISP.read_bytes() == isp
        except Exception as error:
            meta['cleanup_check_error'] = str(error)
            meta['complete'] = False
        (dest / 'session.json').write_text(json.dumps(meta, indent=2))
    print('Complete:', dest, flush=True)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    main()
