"""Offline session metrics and a restrained, uninstalled colour-match candidate.
Run in l4t-ml-gpio with tuning workspace at /w, session directory argument.
Uses measured USB rendering, not the nominal RGB values of the printed chart.
Since 2026-10-01 23:59 the CSI stills are decoded with their limited range (chartcmp/chartsharp use
csi_decode.read_bgr). It refuses to overwrite an existing metrics.json or c7_usb_half.isp (evidence;
trial_c7.sh pins the candidate's hash) unless --force is given.
"""
import json
import pathlib
import re
import sys
import cv2
import numpy as np
from chartcmp import measure, lab, homography, GEO
from chartsharp import analyse
from fitccm import fit, read_ccm

d = pathlib.Path(sys.argv[1])
for f in ('metrics.json',) + (() if '--no-fit' in sys.argv[2:] else ('c7_usb_half.isp',)):
    if (d / f).exists() and '--force' not in sys.argv[2:]:
        sys.exit(f'{d / f} exists; refusing to overwrite it (use --force, or analyse a copy)')
session = json.loads((d / 'session.json').read_text())
assert session['complete'] and session['isp_unchanged']
groups = {}
for cap in session['captures']:
    name = cap['name']; key = name.rsplit('_', 1)[0]
    a, b = [measure(str(d / f'{name}.{k}.jpg')) for k in ('csi', 'ug')]
    if a is None or b is None or a[2] != '4m' or b[2] != '4m':
        print('Reject incomplete chart:', name, flush=True); continue
    pa, wa, _, xa, _ = a; pb, wb, _, xb, _ = b
    groups.setdefault(key, []).append(dict(xa=xa, xb=xb,
        color_de=float(np.linalg.norm(pa[:18] - pb[:18], axis=1).mean()),
        gray_de=float(np.linalg.norm(pa[18:22] - pb[18:22], axis=1).mean()),
        paper_L=float(wa[0]), usb_L=float(wb[0]), gray_a=float(pa[18:22,1].mean()),
        gray_b=float(pa[18:22,2].mean()), name=name))

report = {}
print('setting n colorDE grayDE paperL(range) usbL(range) gray_a gray_b', flush=True)
for key, vals in groups.items():
    avg = {k: float(np.mean([v[k] for v in vals])) for k in
           ('color_de', 'gray_de', 'paper_L', 'usb_L', 'gray_a', 'gray_b')}
    avg['n'] = len(vals)
    avg['paper_L_range'] = [min(v['paper_L'] for v in vals), max(v['paper_L'] for v in vals)]
    avg['usb_L_range'] = [min(v['usb_L'] for v in vals), max(v['usb_L'] for v in vals)]
    # One representative sharpness measurement. Spatial variation includes print/JPEG texture.
    sharp = analyse(str(d / (vals[0]['name'] + '.csi.jpg')))
    avg['spatial_luma_std'] = float(sharp['nz']['sd'])
    avg['low_edge_overshoot_pct'] = float(sharp['lo']['metr'][1])
    report[key] = avg
    print(key, json.dumps(avg), flush=True)

# Candidate from all base captures, cross-validate across chart colours (not
# repeated frames of the same patch). Half-strength to avoid overfitting the lamp.
if '--no-fit' not in sys.argv[2:] and 'base' in groups and 'base_end' in groups:
    base = groups['base'] + groups['base_end']
    xa = np.mean([v['xa'] for v in base], axis=0)
    xb = np.mean([v['xb'] for v in base], axis=0)
    # Normalize brightness to leave exposure tuning separate from chromatic correction.
    brightness = xa[18:22,1].mean() / xb[18:22,1].mean()
    target = xb * brightness
    valid = (xa.max(1) < .9) & (xa.min(1) > .008) & (target.max(1) < .9)
    ids = np.where(valid)[0]
    errs_before, errs_after = [], []
    for i in ids:
        train = ids[ids != i]
        M = .5 * np.eye(3) + .5 * fit(xa[train], target[train])
        errs_before.append(float(np.linalg.norm(lab(xa[i]) - lab(target[i]))))
        errs_after.append(float(np.linalg.norm(lab(np.clip(M @ xa[i], 0, 1)) - lab(target[i]))))
    M = .5 * np.eye(3) + .5 * fit(xa[ids], target[ids])
    A = read_ccm(str(d / 'installed_before.isp')).T
    F = (M @ A).T
    src = (d / 'installed_before.isp').read_text()
    for i in range(3):
        src, count = re.subn(rf'^(colorCorrection\.srgbMatrix\[{i}\]\s*=\s*)\{{[^}}]*\}}',
            lambda m: m[1] + '{' + ', '.join(f'{v:.8f}' for v in F[i]) + '}', src, flags=re.M)
        assert count == 1
    candidate = dict(matrix=M.tolist(), valid_patches=ids.tolist(), brightness_normalization=float(brightness),
        heldout_de_before=float(np.mean(errs_before)), heldout_de_after=float(np.mean(errs_after)),
        status='OFFLINE PREDICTION ONLY; requires live A/B, USB rendering under this room light is target')
    report['candidate'] = candidate
    print('Candidate:', json.dumps(candidate), flush=True)
    (d / 'c7_usb_half.isp').write_text('# Experimental half-strength USB match under 2026-10-01 room lighting; not daylight calibrated.\n' + src)
(d / 'metrics.json').write_text(json.dumps(report, indent=2))
