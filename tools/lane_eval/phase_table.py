#!/usr/bin/env python3
"""phase_table.py — resource deltas per run phase, bench test B0 (plan P5b, section 11).

    python3 tools/lane_eval/phase_table.py RUN_DIR [--skip-s 15]

Stdlib only.  Splits RUN_DIR/sys/sysmon.csv and proc.csv into phases by what
was running at each sample: idle (before the first container started; from
`lane_run.sh start --idle-lead S`), then by each camera's lanes_on (from
<cam>/counters.csv): "cams" (cameras + recording, lanes off), "csi lanes",
"usb lanes", "both lanes".  The first --skip-s seconds of every phase are
dropped (engine warm-up, DVFS settling).  Prints per phase: seconds, total CPU
(100 = one core), GPU load and clock, VDD_IN, MemAvailable, per-process CPU,
inference rate; then the deltas against idle / "cams" and the energy per
inference ((VDD_IN with lanes − VDD_IN cams) / inferences per s).  Writes
RUN_DIR/post/phases.json.
"""
import argparse
import bisect
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


def mean(v):
    v = [x for x in v if x is not None]
    return st.mean(v) if v else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--skip-s", type=float, default=15)
    a = ap.parse_args()
    cams = [c for c in ("csi", "usb") if os.path.isdir(os.path.join(a.run, c))]
    ctr = {c: rows(os.path.join(a.run, c, "counters.csv")) for c in cams}
    ct_t = {c: [int(r["mono_ns"]) for r in ctr[c]] for c in cams}
    eh = rows(os.path.join(a.run, "events_host.csv"))
    first_start = min((int(e["mono_ns"]) for e in eh if e["type"] == "container_start"), default=None)

    def state(t):
        if first_start is not None and t < first_start:
            return "idle"
        on = {}
        for c in cams:
            i = bisect.bisect_right(ct_t[c], t) - 1
            if i < 0 or ctr[c][i]["cam_state"] != "RUNNING":
                return None                       # starting / restarting: not a clean phase
            on[c] = ctr[c][i]["lanes_on"] == "1"
        if all(on.values()) and len(on) > 1:
            return "both lanes"
        for c in cams:
            if on[c]:
                return "%s lanes" % c
        return "cams"

    sm = rows(os.path.join(a.run, "sys", "sysmon.csv"))
    pr = rows(os.path.join(a.run, "sys", "proc.csv"))
    phases, since, last = {}, {}, None
    for r in sm:
        t = int(r["mono_ns"])
        s = state(t)
        if s != last:
            since[s] = t
            last = s
        if s is None or (t - since[s]) / 1e9 < a.skip_s:
            continue
        phases.setdefault(s, {"t": [], "cpu": [], "gpu": [], "mhz": [], "vin": [], "mem": []})
        p = phases[s]
        p["t"].append(t)
        p["cpu"].append(num(r["cpu_pct"]))
        p["gpu"].append(num(r["gpu_load_pct"]))
        p["mhz"].append(num(r["gpu_mhz"]))
        p["vin"].append(num(r.get("p_vdd_in_mw")))
        p["mem"].append(num(r["mem_avail_kb"]))
    out = {}
    for s, p in phases.items():
        ts = set(p["t"])
        lo, hi = min(p["t"]), max(p["t"])
        procs = {}
        for r in pr:
            t = int(r["mono_ns"])
            if lo <= t <= hi and state(t) == s and r["cpu_pct"]:
                procs.setdefault(r["name"], []).append(num(r["cpu_pct"]))
        rate = 0.0
        for c in cams:
            vals = [num(r["eff_hz"]) for r in ctr[c] if lo <= int(r["mono_ns"]) <= hi and r["lanes_on"] == "1"]
            rate += mean(vals) or 0.0
        out[s] = {"seconds": (hi - lo) / 1e9, "samples": len(ts), "cpu_pct": mean(p["cpu"]), "gpu_load_pct": mean(p["gpu"]),
                  "gpu_mhz": mean(p["mhz"]), "vdd_in_w": (mean(p["vin"]) or 0) / 1000, "mem_avail_gb": (mean(p["mem"]) or 0) / 1048576,
                  "inferences_per_s": rate, "procs_cpu_pct": {k: mean(v) for k, v in sorted(procs.items())}}
    order = [s for s in ("idle", "cams", "csi lanes", "usb lanes", "both lanes") if s in out]
    print("%-11s %6s %7s %6s %6s %7s %7s %6s  processes (CPU %%)" % ("phase", "s", "CPU%", "GPU%", "MHz", "VDD_IN", "MemAv", "inf/s"))
    for s in order:
        o = out[s]
        print("%-11s %6.0f %7.0f %6.1f %6.0f %6.2fW %6.2fG %6.1f  %s" % (
            s, o["seconds"], o["cpu_pct"] or 0, o["gpu_load_pct"] or 0, o["gpu_mhz"] or 0, o["vdd_in_w"], o["mem_avail_gb"],
            o["inferences_per_s"], ", ".join("%s %.0f" % (k, v) for k, v in o["procs_cpu_pct"].items() if v is not None)))
    base = out.get("cams")
    if base:
        print("deltas vs cams (cameras + recording, lanes off):")
        for s in order:
            if s in ("idle", "cams"):
                continue
            o = out[s]
            dw = o["vdd_in_w"] - base["vdd_in_w"]
            o["delta_vs_cams"] = {"cpu_pts": (o["cpu_pct"] or 0) - (base["cpu_pct"] or 0), "vdd_in_w": dw,
                                  "mem_gb": o["mem_avail_gb"] - base["mem_avail_gb"],
                                  "mj_per_inference": 1000 * dw / o["inferences_per_s"] if o["inferences_per_s"] else None}
            d = o["delta_vs_cams"]
            print("  %-11s CPU %+5.0f pts  VDD_IN %+5.2f W  MemAvailable %+5.2f GB  energy %s mJ/inference" % (
                s, d["cpu_pts"], d["vdd_in_w"], d["mem_gb"], "%.0f" % d["mj_per_inference"] if d["mj_per_inference"] else "-"))
    if "idle" in out and base:
        print("cams vs idle: CPU %+.0f pts, VDD_IN %+.2f W, MemAvailable %+.2f GB (engines loaded, lanes off)" % (
            (base["cpu_pct"] or 0) - (out["idle"]["cpu_pct"] or 0), base["vdd_in_w"] - out["idle"]["vdd_in_w"],
            base["mem_avail_gb"] - out["idle"]["mem_avail_gb"]))
    os.makedirs(os.path.join(a.run, "post"), exist_ok=True)
    with open(os.path.join(a.run, "post", "phases.json"), "w") as f:
        json.dump(out, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
