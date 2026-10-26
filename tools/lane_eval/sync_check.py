#!/usr/bin/env python3
"""sync_check.py — cross-camera time alignment, bench test B5 / pre-flight P13 (plan P5b).

    python3 tools/lane_eval/sync_check.py RUN_DIR [--min-step 25] [--window-s 0.5]

Stdlib only.  Someone flashes a phone torch at both lenses (bench), or passes
a hand in front of both (car; at night a torch is allowed), three or more
times.  Each lane result's band_luma (lane_drive frames.csv, the mean luma of
the lane input band) jumps at once in both cameras; this finds those steps
(|change between consecutive results| >= --min-step and >= 6 x the camera's
median absolute change), times each at the 50 % crossing interpolated on
capture_mono_ns, pairs CSI and UGREEN steps of the same sign within
--window-s, and reports the UGREEN - CSI offset per event, its mean and SD.

Gate (plan B5): SD <= 33 ms over >= 3 events.  The mean is REPORT only: it is
the M10 pairing correction for cross-camera analysis.  Writes
RUN_DIR/post/sync.json.  Exit 0 = PASS (or not enough events: INCONCLUSIVE, 2).
"""
import argparse
import csv
import json
import os
import statistics as st
import sys


def series(path):
    out = []
    try:
        with open(path) as f:
            for r in csv.DictReader(f):
                t, l = r.get("capture_mono_ns"), r.get("band_luma")
                if t and l:
                    out.append((int(t), float(l)))
    except OSError:
        pass
    out.sort()
    return out


def steps(s, min_step):
    """[(time_ns, sign, size)] of luma steps between consecutive results."""
    if len(s) < 10:
        return []
    d = [abs(b[1] - a[1]) for a, b in zip(s, s[1:])]
    mad = st.median(d) or 0.5
    thr = max(min_step, 6 * mad)
    out = []
    i = 0
    while i < len(s) - 1:
        (t0, l0), (t1, l1) = s[i], s[i + 1]
        if abs(l1 - l0) >= thr:
            # extend over a step spread across a few results (rolling shutter, exposure)
            j = i + 1
            while j < len(s) - 1 and (s[j + 1][1] - s[j][1]) * (l1 - l0) > 0 and abs(s[j + 1][1] - s[j][1]) >= thr / 3:
                j += 1
            lo, hi = s[i][1], s[j][1]
            mid = (lo + hi) / 2
            tc = None
            for k in range(i, j):
                (ta, la), (tb, lb) = s[k], s[k + 1]
                if (la - mid) * (lb - mid) <= 0 and lb != la:
                    tc = ta + (mid - la) / (lb - la) * (tb - ta)
                    break
            out.append((tc if tc is not None else t1, 1 if hi > lo else -1, hi - lo))
            i = j + 1
        else:
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--min-step", type=float, default=25.0)
    ap.add_argument("--window-s", type=float, default=0.5)
    a = ap.parse_args()
    cs = steps(series(os.path.join(a.run, "csi", "frames.csv")), a.min_step)
    us = steps(series(os.path.join(a.run, "usb", "frames.csv")), a.min_step)
    pairs = []
    used = set()
    for tc, sc, zc in cs:
        best = None
        for k, (tu, su, zu) in enumerate(us):
            if k in used or su != sc or abs(tu - tc) > a.window_s * 1e9:
                continue
            if best is None or abs(tu - tc) < abs(us[best][0] - tc):
                best = k
        if best is not None:
            used.add(best)
            pairs.append({"csi_mono_ns": int(tc), "usb_mono_ns": int(us[best][0]), "sign": sc,
                          "offset_ms": (us[best][0] - tc) / 1e6, "csi_step": zc, "usb_step": us[best][2]})
    offs = [p["offset_ms"] for p in pairs]
    res = {"csi_steps": len(cs), "usb_steps": len(us), "pairs": pairs,
           "mean_offset_ms": st.mean(offs) if offs else None,
           "sd_ms": st.stdev(offs) if len(offs) >= 2 else None}
    if len(offs) < 3:
        res["verdict"] = "INCONCLUSIVE"
    else:
        res["verdict"] = "PASS" if res["sd_ms"] <= 33 else "FAIL"
    os.makedirs(os.path.join(a.run, "post"), exist_ok=True)
    with open(os.path.join(a.run, "post", "sync.json"), "w") as f:
        json.dump(res, f, indent=1)
    print("sync_check %s: %d CSI / %d UGREEN steps, %d pairs" % (a.run, len(cs), len(us), len(pairs)))
    for p in pairs:
        print("  %s step %+5.0f / %+5.0f  UGREEN - CSI %+7.1f ms" % ("up  " if p["sign"] > 0 else "down",
                                                                  p["csi_step"], p["usb_step"], p["offset_ms"]))
    if offs:
        print("  mean %+.1f ms, SD %s ms -> %s" % (res["mean_offset_ms"], "%.1f" % res["sd_ms"] if res["sd_ms"] is not None
                                                  else "-", res["verdict"]))
    return 0 if res["verdict"] == "PASS" else 2 if res["verdict"] == "INCONCLUSIVE" else 1


if __name__ == "__main__":
    sys.exit(main())
