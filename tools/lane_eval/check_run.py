#!/usr/bin/env python3
"""check_run.py — health / resource gates of a lane test run (plan 9.2, 9.3, 11).

    python3 tools/lane_eval/check_run.py RUN_DIR [--dual] [--bench]

Reads RUN_DIR/<cam>/{frames,counters,events}.csv, RUN_DIR/sys/{sysmon,proc}.csv
and RUN_DIR/*/summary.json (lane_drive + lane_run.sh output), prints one line
per gate (PASS / FAIL / WARN / REPORT) and writes RUN_DIR/post/check.json.
Exit 0 = no FAIL.  Stdlib only (runs on the host).

Gates (lanes-on period only; thresholds from the plan, dual = both cameras):
  eff_hz       mean >= 0.9 x min(target, delivered source fps)
  latency      capture->done p95 <= 60 / 90 (dual) ms, p99 <= 100 / 150 ms
  trt          TensorRT stage p99 <= 25 / 40 (dual) ms
  errors       0 (map/size/h2d/pre/trt/d2h/sync/nonfinite)
  completeness frames.csv rows == processed, CSV drops 0
  restarts     0 (bench)
  rss_slope    <= 20 MB/h after the first 5 min (REPORT when shorter)
  leaks        threads and fds unchanged from the first to the last lanes-on row
  mem          MemAvailable min >= 1.5 GB (bench); swap growth <= 100 MB
  thermal      tj max < 85 C (WARN), < 95 C (FAIL)
  storage      run GB/h <= 20
"""
import csv
import json
import os
import statistics as st
import sys


def rows(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def pct(vals, p):
    vals = sorted(v for v in vals if v is not None)
    if not vals:
        return None
    return vals[min(len(vals) - 1, int(p / 100.0 * len(vals)))]


def slope_per_hour(ts_ns, vals):
    pts = [(t / 3.6e12, v) for t, v in zip(ts_ns, vals) if v is not None]
    if len(pts) < 3:
        return None
    mt = st.mean(p[0] for p in pts)
    mv = st.mean(p[1] for p in pts)
    den = sum((p[0] - mt) ** 2 for p in pts)
    return sum((p[0] - mt) * (p[1] - mv) for p in pts) / den if den else None


class Report:
    def __init__(self):
        self.items, self.fails = [], 0

    def gate(self, name, verdict, detail):
        self.items.append({"gate": name, "verdict": verdict, "detail": detail})
        self.fails += verdict == "FAIL"
        print("  %-6s %-24s %s" % (verdict, name, detail))


def check_cam(run, cam, dual, bench, rep):
    d = os.path.join(run, cam)
    fr, ct = rows(os.path.join(d, "frames.csv")), rows(os.path.join(d, "counters.csv"))
    try:
        summ = json.load(open(os.path.join(d, "summary.json")))
    except (OSError, ValueError):
        summ = {}
    try:
        cfg = json.load(open(os.path.join(d, "run.json")))
    except (OSError, ValueError):
        cfg = {}
    print("[%s]" % cam)
    on = [r for r in ct if r.get("lanes_on") == "1" and r.get("cam_state") == "RUNNING"]
    if not on:
        rep.gate(cam + " lanes_on", "FAIL", "no RUNNING lanes-on counter rows")
        return
    target = num(cfg.get("target_hz")) or 20
    eff = [num(r["eff_hz"]) for r in on[2:]] or [num(r["eff_hz"]) for r in on]
    src = [num(r["src_fps"]) for r in on[2:]] or [num(r["src_fps"]) for r in on]
    want = 0.9 * min(target, st.mean(v for v in src if v is not None))
    me = st.mean(v for v in eff if v is not None)
    rep.gate(cam + " eff_hz", "PASS" if me >= want else "FAIL",
             "mean %.2f Hz (want >= %.2f = 0.9 x min(%g, source %.1f fps)); min 1 s window %.1f" %
             (me, want, target, st.mean(v for v in src if v is not None), min(v for v in eff if v is not None)))

    lat = [num(r["lat_cap_done_ms"]) for r in fr]
    lat = [v for v in lat if v is not None]
    p95, p99 = pct(lat, 95), pct(lat, 99)
    lim95, lim99 = (90, 150) if dual else (60, 100)
    if lat:
        rep.gate(cam + " latency", "PASS" if p95 <= lim95 and p99 <= lim99 and min(lat) >= 0 else "FAIL",
                 "capture->done p50 %.1f p95 %.1f p99 %.1f max %.1f ms (limits %d / %d); min %.1f" %
                 (pct(lat, 50), p95, p99, max(lat), lim95, lim99, min(lat)))
    else:
        rep.gate(cam + " latency", "REPORT", "no capture timestamps")
    trt = [num(r["trt_us"]) for r in fr]
    trt = [v / 1000 for v in trt if v is not None]
    if trt:
        lim = 40 if dual else 25
        rep.gate(cam + " trt", "PASS" if pct(trt, 99) <= lim else "FAIL",
                 "TensorRT p50 %.1f p99 %.1f ms (limit %d)" % (pct(trt, 50), pct(trt, 99), lim))
    errs = summ.get("errors", {})
    total = sum(v for v in errs.values() if isinstance(v, (int, float)))
    rep.gate(cam + " errors", "PASS" if total == 0 else "FAIL", json.dumps(errs))
    complete = summ.get("frames_rows") == summ.get("processed") and not summ.get("frames_csv_drops")
    rep.gate(cam + " completeness", "PASS" if complete else "FAIL",
             "rows %s processed %s csv drops %s" % (summ.get("frames_rows"), summ.get("processed"),
                                                   summ.get("frames_csv_drops")))
    rs = summ.get("restarts", 0)
    rep.gate(cam + " restarts", ("PASS" if rs == 0 else "FAIL") if bench else "REPORT",
             "%s restarts, cam_lost %s, stop_hang %s" % (rs, summ.get("cam_lost"), summ.get("stop_hang")))

    t = [int(r["mono_ns"]) for r in on]
    span_min = (t[-1] - t[0]) / 6e10
    late = [r for r in on if int(r["mono_ns"]) - t[0] >= 3e11]
    sl = slope_per_hour([int(r["mono_ns"]) for r in late], [num(r["rss_kb"]) / 1024 for r in late]) if late else None
    if sl is None or span_min < 10:
        rep.gate(cam + " rss_slope", "REPORT", "run too short for a slope (%.1f min lanes-on); RSS %s -> %s MiB" %
                 (span_min, int(num(on[0]["rss_kb"]) / 1024), int(num(on[-1]["rss_kb"]) / 1024)))
    else:
        rep.gate(cam + " rss_slope", "PASS" if sl <= 20 else "FAIL", "%.1f MB/h after minute 5" % sl)
    leak = (on[0]["threads"], on[-1]["threads"], on[0]["fds"], on[-1]["fds"])
    rep.gate(cam + " leaks", "PASS" if leak[0] == leak[1] and leak[2] == leak[3] else "WARN",
             "threads %s -> %s, fds %s -> %s" % leak)
    rep.gate(cam + " volume", "REPORT", "%d results over %.1f min lanes-on, %s review segments" %
             (len(fr), span_min, summ.get("review_segments")))


def check_sys(run, bench, rep):
    sm = rows(os.path.join(run, "sys", "sysmon.csv"))
    print("[system]")
    if not sm:
        rep.gate("sysmon", "FAIL", "no sys/sysmon.csv")
        return
    mem = [num(r["mem_avail_kb"]) for r in sm]
    mmin = min(v for v in mem if v is not None) / 1048576
    rep.gate("mem_available", ("PASS" if mmin >= 1.5 else "FAIL") if bench else ("PASS" if mmin >= 1.0 else "WARN"),
             "min %.2f GB" % mmin)
    sw = [num(r["swap_used_kb"]) for r in sm]
    growth = (sw[-1] - sw[0]) / 1024 if sw and sw[0] is not None and sw[-1] is not None else 0
    rep.gate("swap_growth", "PASS" if growth <= 100 else "FAIL", "%.0f MB" % growth)
    tj = [num(r.get("t_tj_thermal_c")) for r in sm]
    tj = [v for v in tj if v is not None]
    if tj:
        m = max(tj)
        rep.gate("thermal", "FAIL" if m >= 95 else "WARN" if m >= 85 else "PASS", "tj max %.1f C, mean %.1f C" % (m, st.mean(tj)))
    gbh = [num(r["run_gbph_5min"]) for r in sm]
    gbh = [v for v in gbh if v is not None]
    if gbh:
        rep.gate("storage", "PASS" if max(gbh) <= 20 else "FAIL", "trailing-5-min max %.1f GB/h, final run size %s GB" %
                 (max(gbh), sm[-1].get("run_gb")))
    gpu = [num(r["gpu_load_pct"]) for r in sm]
    pin = [num(r.get("p_vdd_in_mw")) for r in sm]
    cpu = [num(r["cpu_pct"]) for r in sm]
    rep.gate("load", "REPORT", "GPU load mean %.1f%% max %.1f%%, GPU clock median %s MHz, CPU mean %.0f%% of one core, VDD_IN mean %.2f W" %
             (st.mean(v for v in gpu if v is not None), max(v for v in gpu if v is not None),
              st.median(int(r["gpu_mhz"]) for r in sm if r["gpu_mhz"]),
              st.mean(v for v in cpu if v is not None), st.mean(v for v in pin if v is not None) / 1000))
    pr = rows(os.path.join(run, "sys", "proc.csv"))
    by = {}
    for r in pr:
        if r["cpu_pct"] and num(r["cpu_pct"]) >= 0:   # older sysmon: an exit gave one negative sample
            by.setdefault(r["name"], []).append((num(r["cpu_pct"]), num(r["rss_kb"])))
    rep.gate("processes", "REPORT", "; ".join("%s CPU %.0f%% RSS %.0f MiB" % (n, st.mean(c for c, _ in v), max(m for _, m in v) / 1024)
                                              for n, v in sorted(by.items())))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    run = sys.argv[1]
    cams = [c for c in ("csi", "usb") if os.path.isdir(os.path.join(run, c))]
    dual = "--dual" in sys.argv or len(cams) > 1
    bench = "--bench" in sys.argv
    rep = Report()
    print("check_run %s (%s, %s)" % (run, "dual" if dual else "single", "bench gates" if bench else "drive gates"))
    for c in cams:
        check_cam(run, c, dual, bench, rep)
    check_sys(run, bench, rep)
    os.makedirs(os.path.join(run, "post"), exist_ok=True)
    with open(os.path.join(run, "post", "check.json"), "w") as f:
        json.dump({"run": run, "dual": dual, "bench": bench, "fails": rep.fails, "items": rep.items}, f, indent=1)
    print("RESULT: %s (%d FAIL)" % ("PASS" if rep.fails == 0 else "FAIL", rep.fails))
    return 0 if rep.fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
