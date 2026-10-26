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
  rss_slope    <= 20 MB/h after the first 5 min, on 5-min RSS minima (REPORT when shorter)
  leaks        threads and fds unchanged from the first to the last lanes-on row
  mem          MemAvailable min >= 1.5 GB (bench); swap growth <= 100 MB
  thermal      tj max < 85 C (WARN), < 95 C (FAIL)
  storage      run GB/h <= 20
P5b additions (per camera, then the run):
  ring         clip ring fed at its nominal rate (>= 90 % of ring_fps x lanes-on time)
  v3_losses    sig / clip-overflow / clip-queue / dump-queue drops = 0 (plan V3)
  clips        kept / refused, every clip MKV ffprobe-clean
  dumps        written; periodic dumps paired with an exact-PTS frame (UGREEN 100 %,
               CSI >= 80 %, REPORT otherwise)
  review       every review MKV ffprobe-clean (the newest 30), .ass banner present
  mount        reference taken, measured fraction, FAIL onsets (REPORT; bench: 0)
  supervisor   container / logger restarts, degrade steps, alerts (REPORT)
"""
import csv
import json
import os
import glob
import statistics as st
import subprocess
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


def load_summary(d):
    """summary.json plus every restarted instance's summary_i<K>.json: counts
    added up, cam_lost / exit from the last instance, stop_hang if any."""
    parts = []
    for f in [os.path.join(d, "summary.json")] + sorted(glob.glob(os.path.join(d, "summary_i*.json")),
                                                        key=lambda p: int(p.rsplit("_i", 1)[1].split(".")[0])):
        try:
            parts.append(json.load(open(f)))
        except (OSError, ValueError):
            pass
    if not parts:
        return {}
    out = dict(parts[-1])
    for k in ("processed", "pulled", "gated", "frames_rows", "frames_csv_drops", "restarts", "clips_written",
              "clips_refused", "clip_overflow_frames", "clip_queue_drops", "clip_bytes", "dumps_written",
              "dump_queue_drops", "dump_bytes", "sig_drops", "ring_frames"):
        out[k] = sum(p.get(k) or 0 for p in parts)
    errs = {}
    for p in parts:
        for k, v in (p.get("errors") or {}).items():
            errs[k] = errs.get(k, 0) + (v or 0)
    out["errors"] = errs
    out["stop_hang"] = any(p.get("stop_hang") for p in parts)
    out["review_segments"] = parts[-1].get("review_segments")
    out["instances"] = len(parts)
    return out


def ffprobe_clean(path):
    try:
        r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path],
                           capture_output=True, text=True, timeout=30)
        return r.returncode == 0 and not r.stderr.strip() and num(r.stdout.strip()) is not None
    except (OSError, subprocess.TimeoutExpired):
        return None


def check_p5b(run, cam, summ, cfg, on, bench, rep):
    d = os.path.join(run, cam)
    if "ring_frames" not in summ:
        return                                   # a P5a run
    span_s = (int(on[-1]["mono_ns"]) - int(on[0]["mono_ns"])) / 1e9 if len(on) > 1 else 0
    ct = rows(os.path.join(d, "counters.csv"))
    if ct and "ring_frames" in ct[0] and len(ct) > 2:
        # Per running stretch: a restarted instance starts its counters again.
        frames = secs = 0.0
        for a, b in zip(ct, ct[1:]):
            df = int(b["ring_frames"]) - int(a["ring_frames"])
            if df >= 0 and a["cam_state"] == "RUNNING" and b["cam_state"] == "RUNNING":
                frames += df
                secs += (int(b["mono_ns"]) - int(a["mono_ns"])) / 1e9
        rate = frames / secs if secs > 0 else 0
        want = 0.9 * (num(cfg.get("ring_fps")) or 30)
        degr = max(int(r.get("degrade") or 0) for r in ct)
        rep.gate(cam + " ring", "PASS" if rate >= want or degr >= 1 else "FAIL",
                 "clip ring fed at %.1f fps (want >= %.1f); ring span at the end %s s" %
                 (rate, want, ct[-1].get("ring_span_s")))
    losses = {k: summ.get(k, 0) for k in ("sig_drops", "clip_overflow_frames", "clip_queue_drops", "dump_queue_drops")}
    rep.gate(cam + " v3_losses", "PASS" if sum(losses.values()) == 0 else ("FAIL" if bench else "WARN"), json.dumps(losses))
    clips = sorted(glob.glob(os.path.join(d, "clips", "*.mkv")))
    bad = [os.path.basename(c) for c in clips if ffprobe_clean(c) is False]
    rep.gate(cam + " clips", "PASS" if not bad else "FAIL", "%d written (%.0f MB), %s refused by the budget%s" %
             (len(clips), (summ.get("clip_bytes") or 0) / 1e6, summ.get("clips_refused"),
              ("; NOT ffprobe-clean: " + " ".join(bad[:5])) if bad else "; all ffprobe-clean"))
    di = rows(os.path.join(d, "dumps_index.csv"))
    per = [r for r in di if r["trigger"] == "periodic"]
    exact = [r for r in per if r.get("dpts_ns") == "0"]
    ev = [r for r in di if r["trigger"].startswith("event:")]
    frac = len(exact) / len(per) if per else None
    want = 1.0 if cam == "usb" else 0.8
    v = "REPORT" if frac is None else ("PASS" if frac >= want else "WARN")
    rep.gate(cam + " dumps", v, "%d periodic (%s with the exact-PTS frame), %d event dumps; %.0f MB" %
             (len(per), "%.0f %%" % (100 * frac) if frac is not None else "-", len(ev), (summ.get("dump_bytes") or 0) / 1e6))
    mk = sorted(glob.glob(os.path.join(d, "review", "*.mkv")))[-30:]
    badm = [os.path.basename(c) for c in mk if ffprobe_clean(c) is False]
    ass = glob.glob(os.path.join(d, "review", "*.ass"))
    nl = 0
    for a in ass:
        with open(a) as f:
            nl += sum(1 for line in f if line.startswith("Dialogue:"))
    evs0 = rows(os.path.join(d, "events.csv"))
    killed = any(e["type"] == "start" and "instance 0" not in e["detail"] for e in evs0)
    rep.gate(cam + " review", "PASS" if not badm and nl > 0 else ("WARN" if killed and nl > 0 else "FAIL"),
             "%d segments checked%s; %d .ass file(s), %d banner lines" %
             (len(mk), (", NOT clean: " + " ".join(badm[:5]) + (" (a killed instance's open segment)" if killed else ""))
              if badm else ", all clean", len(ass), nl))
    mt = rows(os.path.join(d, "mount.csv"))
    evs = rows(os.path.join(d, "events.csv"))
    onsets = [e for e in evs if e["type"] == "mount_shift_on"]
    refs = [e["value"] + " " + e["detail"][:40] for e in evs if e["type"] == "mount_ref"]
    measured = sum(1 for r in mt if r["dx_px"] != "")
    auth = summ.get("mount_authoritative")
    if auth is False:
        onsets = [o for o in onsets if o.get("value") != "unverified_roi"] or []
    rep.gate(cam + " mount", ("PASS" if not onsets else "FAIL") if bench and auth else "REPORT",
             "ref %s; roi %s; %d of %d samples measured; %d FAIL onsets%s" %
             (refs[0] if refs else "none", "car-fixed" if auth else "unverified (report only)", measured, len(mt), len(onsets),
              (" (first at %s)" % onsets[0]["mono_ns"]) if onsets else ""))
    kinds = {}
    for e in evs:
        kinds[e["type"]] = kinds.get(e["type"], 0) + 1
    keys = ("cam_restart", "stall", "latency_spike", "counter_inc", "flicker_burst", "missed_lc", "trigger",
            "clip_kept", "clip_refused", "event_dump", "degrade_1", "degrade_gpu", "lib_error", "lib_warn")
    rep.gate(cam + " events", "REPORT", ", ".join("%s %d" % (k, kinds[k]) for k in keys if k in kinds) or "none")


def check_cam(run, cam, dual, bench, rep):
    d = os.path.join(run, cam)
    fr, ct = rows(os.path.join(d, "frames.csv")), rows(os.path.join(d, "counters.csv"))
    summ = load_summary(d)
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
    complete = summ.get("frames_rows") == summ.get("processed") == len(fr) and not summ.get("frames_csv_drops")
    started = sum(1 for e in rows(os.path.join(d, "events.csv")) if e["type"] == "start")
    unsummarised = max(0, started - summ.get("instances", 1))
    # An instance that was killed (SIGKILL, OOM) wrote rows but no summary.
    verdict = "PASS" if complete else ("WARN" if unsummarised and len(fr) >= (summ.get("processed") or 0)
                                       and summ.get("frames_rows") == summ.get("processed") else "FAIL")
    rep.gate(cam + " completeness", verdict,
             "frames.csv %d rows; summaries: rows %s processed %s csv drops %s%s" %
             (len(fr), summ.get("frames_rows"), summ.get("processed"), summ.get("frames_csv_drops"),
              "; %d of %d instances left no summary (killed)" % (unsummarised, started) if unsummarised else ""))
    rs = summ.get("restarts", 0)
    rep.gate(cam + " restarts", ("PASS" if rs == 0 and summ.get("instances", 1) == 1 else "FAIL") if bench else "REPORT",
             "%s in-process restarts, %s instance(s), cam_lost %s, stop_hang %s" %
             (rs, summ.get("instances", 1), summ.get("cam_lost"), summ.get("stop_hang")))

    t = [int(r["mono_ns"]) for r in on]
    span_min = (t[-1] - t[0]) / 6e10
    late = [r for r in on if int(r["mono_ns"]) - t[0] >= 3e11]
    # Slope of the 5-min RSS minima: clip jobs hold 50-100 MB for ~20 s at a
    # time (a sawtooth an OLS over raw samples reads as growth); a leak raises
    # the floor.
    mins = {}
    for r in late:
        k = (int(r["mono_ns"]) - t[0]) // int(3e11)
        v = num(r["rss_kb"]) / 1024
        if k not in mins or v < mins[k][1]:
            mins[k] = (int(r["mono_ns"]), v)
    pts = [mins[k] for k in sorted(mins)][:-1] if len(mins) > 3 else []   # the last window may be partial
    sl = slope_per_hour([p[0] for p in pts], [p[1] for p in pts]) if len(pts) >= 3 else None
    if sl is None or span_min < 10:
        rep.gate(cam + " rss_slope", "REPORT", "run too short for a slope (%.1f min lanes-on); RSS %s -> %s MiB" %
                 (span_min, int(num(on[0]["rss_kb"]) / 1024), int(num(on[-1]["rss_kb"]) / 1024)))
    else:
        rep.gate(cam + " rss_slope", "PASS" if sl <= 20 else "FAIL", "%.1f MB/h after minute 5 (5-min minima, %d windows)" %
                 (sl, len(pts)))
    leak = (on[0]["threads"], on[-1]["threads"], on[0]["fds"], on[-1]["fds"])
    rep.gate(cam + " leaks", "PASS" if leak[0] == leak[1] and leak[2] == leak[3] else "WARN",
             "threads %s -> %s, fds %s -> %s" % leak)
    rep.gate(cam + " volume", "REPORT", "%d results over %.1f min lanes-on, %s review segments" %
             (len(fr), span_min, summ.get("review_segments")))
    check_p5b(run, cam, summ, cfg, on, bench, rep)


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
    eh = rows(os.path.join(run, "events_host.csv"))
    sup = {}
    for e in eh:
        if e.get("source") == "supervisor":
            sup[e["type"]] = sup.get(e["type"], 0) + 1
    if sup:
        alerts = [e["detail"] for e in eh if e.get("type") == "alert"]
        rep.gate("supervisor", "REPORT", ", ".join("%s %d" % kv for kv in sorted(sup.items())) +
                 ("; alerts: " + " | ".join(alerts[:8]) if alerts else ""))
    try:
        ls = json.load(open(os.path.join(run, "can", "live_state.json")))
        rep.gate("can_live", "REPORT", "%s frames decoded live, decoder alive %s, part %s" %
                 (ls.get("frames"), ls.get("decoder_alive"), ls.get("part")))
    except (OSError, ValueError):
        pass


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
