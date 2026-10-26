#!/usr/bin/env python3
"""roi_calib.py — ROI / mount calibration of the lane cameras (plan P5b, section 12.1).

    python3 tools/lane_eval/roi_calib.py CAL_DIR [--cams csi,usb]
                                         [--hood-row csi=812] [--vp csi=728,560] [--no-lanes-ok]

Runs in the container (numpy, cv2) on the output of `lane_run.sh calibrate`
(lane_drive --stills: CAL_DIR/<cam>/stills/still_*.jpg, frames.csv, run.json,
mount_ref/ref.pgm), parked on a straight, marked road.  Per camera:

  hood edge   auto-detected: the strongest long horizontal edge (Sobel-y) in
              the bottom 40 % of the median still, located in the left, centre
              and right thirds -> hood row h (the topmost of the three) and roll;
              --hood-row CAM=ROW overrides (the "click" of the plan)
  VP          the median intersection of the model's two ego boundaries
              (frames.csv slot 1 / slot 2 lines, source px), valid only above
              the band top; --vp CAM=X,Y overrides
  verdict     CSI (plan 12.1, band [top, H - bottom)): car-body fraction
              c = max(0, band_bottom - h) / band_h; FAIL if c > 3 %, |roll| >
              1.5 deg, VP row outside [444, 604] or |VP x - W/2| > 10 % W; WARN
              if 0 < c <= 3 %, roll in (0.75, 1.5], VP row outside [524, 574]
              or |VP x - W/2| in (5 %, 10 %] W; PASS otherwise.  No VP (no lanes
              seen) = INCONCLUSIVE for the VP terms (at most WARN, or FAIL from
              the others).  UGREEN: the same numbers, REPORT only.
  crops       proposal: top = VP row - 10, bottom = min(h - 8, top + 320) ->
              --crop T,B fractions for lane_run.sh (--csi-crop / --usb-crop)

Writes CAL_DIR/calib.json, CAL_DIR/<cam>_overlay.jpg (band, 72 row anchors,
reference row, centre column, hood edge, VP, model lanes) and the mount
template for the drive: CAL_DIR/mount/ref_<cam>.pgm + roi_<cam>.txt (the
car-fixed rows below the hood edge; no roi file when no hood edge was found,
which leaves that camera's mount monitor report-only), for `lane_run.sh start
--mount-ref CAL_DIR/mount`.  Exit 0 = no FAIL.
"""
import argparse
import csv
import glob
import json
import math
import os
import shutil
import sys

import cv2
import numpy as np


def load_run(d):
    with open(os.path.join(d, "run.json")) as f:
        return json.load(f)


def median_still(d):
    files = sorted(glob.glob(os.path.join(d, "stills", "still_*.jpg")))
    imgs = [cv2.imread(f) for f in files]
    imgs = [i for i in imgs if i is not None]
    if not imgs:
        return None, None
    return np.median(np.stack(imgs), axis=0).astype(np.uint8), imgs[0]


def hood_edge(gray):
    """(h, roll_deg, rows[3]) of the hood/dash top edge, or (None, None, None)."""
    H, W = gray.shape
    g = cv2.GaussianBlur(gray.astype(np.float32), (5, 5), 0)
    gy = np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    y0 = int(H * 0.6)
    rows, xs = [], []
    for k in range(3):
        x0, x1 = int(W * (0.1 + 0.27 * k)), int(W * (0.1 + 0.27 * (k + 1)))
        prof = gy[y0:H - 4, x0:x1].mean(axis=1)
        prof = np.convolve(prof, np.ones(5) / 5, mode="same")
        if prof.max() < 4.0:              # no edge worth the name (night, featureless)
            return None, None, None
        rows.append(y0 + int(np.argmax(prof)))
        xs.append((x0 + x1) / 2)
    a, b = np.polyfit(xs, rows, 1)
    if max(rows) - min(rows) > 0.08 * H:  # the three thirds disagree: not one edge
        return None, None, rows
    return min(rows), math.degrees(math.atan(a)), rows


def model_lines(d, geo):
    """Median (slope, icpt) of slots 1 and 2 over frames with both, and the VP list."""
    s1, s2, vps = [], [], []
    try:
        with open(os.path.join(d, "frames.csv")) as f:
            for r in csv.DictReader(f):
                if r.get("s1_det") == "1" and r.get("s2_det") == "1":
                    a1, b1 = float(r["s1_slope"]), float(r["s1_icpt"])
                    a2, b2 = float(r["s2_slope"]), float(r["s2_icpt"])
                    s1.append((a1, b1))
                    s2.append((a2, b2))
                    if abs(a1 - a2) > 1e-6:
                        fy = (b2 - b1) / (a1 - a2)
                        row = geo["implied_top_src"] + fy * geo["implied_h"]
                        vps.append((a1 * fy + b1, row))
    except OSError:
        pass
    med = lambda v: tuple(np.median(np.array(v), axis=0)) if v else None
    return med(s1), med(s2), vps


def verdict_csi(c, roll, vp, W):
    fails, warns = [], []
    if c is not None and c > 0.03:
        fails.append("car body %.1f %% of the band" % (100 * c))
    elif c is not None and c > 0:
        warns.append("car body %.1f %% of the band" % (100 * c))
    if roll is not None and abs(roll) > 1.5:
        fails.append("roll %.2f deg" % roll)
    elif roll is not None and abs(roll) > 0.75:
        warns.append("roll %.2f deg" % roll)
    if vp is None:
        warns.append("no VP (no ego lanes seen; --vp X,Y to give one)")
    else:
        x, y = vp
        if not 444 <= y <= 604:
            fails.append("VP row %.0f outside [444, 604]" % y)
        elif not 524 <= y <= 574:
            warns.append("VP row %.0f outside [524, 574]" % y)
        dx = abs(x - W / 2) / W
        if dx > 0.10:
            fails.append("VP x off centre by %.1f %% W" % (100 * dx))
        elif dx > 0.05:
            warns.append("VP x off centre by %.1f %% W" % (100 * dx))
    return ("FAIL" if fails else "WARN" if warns else "PASS"), fails + warns


def parse_overrides(items, n):
    out = {}
    for it in items or []:
        cam, _, v = it.partition("=")
        vals = [float(x) for x in v.split(",")]
        if len(vals) != n:
            raise SystemExit("bad override %s" % it)
        out[cam] = vals if n > 1 else vals[0]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cal")
    ap.add_argument("--cams", default="csi,usb")
    ap.add_argument("--hood-row", action="append")
    ap.add_argument("--vp", action="append")
    a = ap.parse_args()
    hood_ov = parse_overrides(a.hood_row, 1)
    vp_ov = parse_overrides(a.vp, 2)
    out = {"cal": os.path.basename(os.path.normpath(a.cal)), "cams": {}}
    os.makedirs(os.path.join(a.cal, "mount"), exist_ok=True)
    any_fail = False
    for cam in [c for c in a.cams.split(",") if c]:
        d = os.path.join(a.cal, cam)
        if not os.path.isdir(d):
            print("[%s] no data" % cam)
            continue
        run = load_run(d)
        geo = run["geometry"]
        W, H = run["src"]
        med, first = median_still(d)
        if med is None:
            print("[%s] no stills" % cam)
            out["cams"][cam] = {"verdict": "FAIL", "notes": ["no stills"]}
            any_fail = True
            continue
        gray = cv2.cvtColor(med, cv2.COLOR_BGR2GRAY)
        h, roll, rows = hood_edge(gray)
        if cam in hood_ov:
            h, roll = int(hood_ov[cam]), roll if roll is not None else 0.0
        l1, l2, vps = model_lines(d, geo)
        top, bot = geo["crop_top_px"], H - geo["crop_bottom_px"]
        vp = None
        good = [v for v in vps if v[1] < top]
        if cam in vp_ov:
            vp = tuple(vp_ov[cam])
        elif len(good) >= 5:
            vp = tuple(np.median(np.array(good), axis=0))
        c = None if h is None else max(0, bot - h) / float(bot - top)
        if cam == "csi":
            verdict, notes = verdict_csi(c, roll, vp, W)
        else:
            v, notes = verdict_csi(c, roll, None if vp is None else (vp[0], vp[1] * 1088.0 / H), W)
            verdict = "REPORT (" + v + ")"
        if h is None:
            notes.append("hood edge not found (night / featureless: --hood-row %s=ROW)" % cam)
        prop = None
        if vp is not None:
            ptop = int(round(vp[1] - 10))
            pbot = int(min((h - 8) if h is not None else H, ptop + 320))
            prop = {"top_px": ptop, "bottom_px": pbot, "crop": "%.4f,%.4f" % (ptop / H, (H - pbot) / H)}
        # Mount template for the drive: car-fixed rows below the hood edge.  No
        # hood edge = no roi file: lane_drive then reports the mount only (a
        # region of road would read as a moved view once the car moves).
        ref = os.path.join(d, "mount_ref", "ref.pgm")
        if os.path.exists(ref):
            shutil.copy(ref, os.path.join(a.cal, "mount", "ref_%s.pgm" % cam))
            roi_file = os.path.join(a.cal, "mount", "roi_%s.txt" % cam)
            if h is not None and (H - h - 8) / H >= 0.06:
                with open(roi_file, "w") as f:
                    f.write("%.4f,1.0,0.05,0.95\n" % ((h + 8) / H))
            else:
                if os.path.exists(roi_file):
                    os.remove(roi_file)
                notes.append("mount monitor: no car-fixed region found -> report only")
        # Overlay.
        ov = med.copy()
        for r in (top, bot):
            cv2.line(ov, (0, r), (W, r), (0, 255, 0), 2)
        for k in range(72):
            r = int(geo["implied_top_src"] + (0.42 + (1 - 0.42) * k / 71) * geo["implied_h"])
            if 0 <= r < H:
                cv2.line(ov, (W - 30, r), (W, r), (0, 160, 0), 1)
        rr = int(geo["ref_row_src"])
        cv2.line(ov, (0, rr), (W, rr), (0, 255, 255), 1)
        cv2.line(ov, (W // 2, 0), (W // 2, H), (255, 255, 255), 1)
        if h is not None:
            cv2.line(ov, (0, h), (W, h), (0, 0, 255), 2)
        for ln in (l1, l2):
            if ln is None:
                continue
            pts = []
            for fy in np.linspace(0.42, 1.0, 20):
                pts.append((int(ln[0] * fy + ln[1]), int(geo["implied_top_src"] + fy * geo["implied_h"])))
            cv2.polylines(ov, [np.array(pts, np.int32)], False, (255, 0, 255), 2)
        if vp is not None:
            cv2.drawMarker(ov, (int(vp[0]), int(vp[1])), (255, 255, 0), cv2.MARKER_CROSS, 40, 2)
        cv2.putText(ov, "%s %s" % (cam.upper(), verdict), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(a.cal, "%s_overlay.jpg" % cam), ov, [cv2.IMWRITE_JPEG_QUALITY, 90])
        res = {"src": [W, H], "band": [top, bot], "hood_row": h, "hood_rows_thirds": rows, "roll_deg": roll,
               "car_body_fraction": c, "vp": vp, "vp_samples": len(good), "verdict": verdict, "notes": notes,
               "proposed": prop, "upright_luma_top_bottom": [float(gray[: H // 3].mean()), float(gray[2 * H // 3:].mean())]}
        out["cams"][cam] = res
        any_fail = any_fail or verdict == "FAIL"
        print("[%s] %s  hood %s roll %s car-body %s VP %s (%d samples)  proposal %s" % (
            cam, verdict, h, "%.2f" % roll if roll is not None else "-", "%.1f%%" % (100 * c) if c is not None else "-",
            "(%.0f, %.0f)" % vp if vp is not None else "-", len(good), prop["crop"] if prop else "-"))
        for n in notes:
            print("       " + n)
    with open(os.path.join(a.cal, "calib.json"), "w") as f:
        json.dump(out, f, indent=1)
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
