#!/usr/bin/env python3
"""sysmon.py — 1 Hz system sampler for lane test runs (host side, stdlib only).

    python3 tools/lane_eval/sysmon.py --out RUN_DIR/sys [--containers lane_csi,lane_usb,lane_mkr]
                                      [--run-dir RUN_DIR] [--interval 1.0]

Runs on the HOST (not in a container) as the jetson user, beside the
lane_drive / logger containers that lane_run.sh starts.  Writes:

  sysmon.csv  mono_ns, realtime_ns, mem_avail_kb, swap_used_kb, cpu_pct (all
              cores, 100 = one core), gpu_load_pct, gpu_mhz, vic_mhz, nvjpg_mhz,
              nvjpg1_mhz, nvdec_mhz, then one column per thermal zone
              (t_<type>_c) and per INA3221 rail (p_<label>_mw), disk_free_gb,
              run_gb, run_gbph_5min (trailing 5 min rate of RUN_DIR growth)
  proc.csv    mono_ns, name, pids, cpu_pct (100 = one core), rss_kb, threads —
              one row per component per sample: each container's whole process
              tree (host PIDs via docker inspect, re-resolved every 10 s) and
              nvargus-daemon.  CUDA / nvmap memory is not in RSS on Jetson:
              MemAvailable in sysmon.csv is the system-level measure.

SIGTERM / SIGINT: flush and exit 0.  Missing sensors give empty cells.
"""
import argparse
import glob
import os
import shutil
import signal
import subprocess
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")
DEVFREQ = {
    "gpu_mhz":    "/sys/class/devfreq/17000000.gpu/cur_freq",
    "vic_mhz":    "/sys/class/devfreq/15340000.vic/cur_freq",
    "nvjpg_mhz":  "/sys/class/devfreq/15380000.nvjpg/cur_freq",
    "nvjpg1_mhz": "/sys/class/devfreq/15540000.nvjpg/cur_freq",
    "nvdec_mhz":  "/sys/class/devfreq/15480000.nvdec/cur_freq",
}
GPU_LOAD = "/sys/devices/platform/gpu.0/load"

stop = False


def on_signal(*_):
    global stop
    stop = True


def read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def meminfo():
    vals = {}
    for line in (read("/proc/meminfo", "") or "").splitlines():
        k, _, v = line.partition(":")
        vals[k] = int(v.split()[0]) if v.split() else 0
    return vals.get("MemAvailable"), vals.get("SwapTotal", 0) - vals.get("SwapFree", 0)


def cpu_ticks():
    f = (read("/proc/stat", "") or "").splitlines()[0].split()[1:]
    vals = [int(x) for x in f]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


def thermal_zones():
    zones = []
    for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        t = read(z + "/type")
        if t:
            zones.append(("t_%s_c" % t.replace("-", "_").lower(), z + "/temp"))
    return zones


def rails():
    """(column, voltage path, current path) for every labelled INA3221 channel."""
    out = []
    for hw in sorted(glob.glob("/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*")):
        for lab in sorted(glob.glob(hw + "/in*_label")):
            n = os.path.basename(lab)[2:].split("_")[0]
            label = read(lab)
            if not label or label.upper().startswith("SUM"):
                continue
            out.append(("p_%s_mw" % label.lower(), hw + "/in%s_input" % n, hw + "/curr%s_input" % n))
    return out


def all_procs():
    """pid -> (ppid, utime+stime ticks, rss_kb, threads, comm)."""
    procs = {}
    page_kb = os.sysconf("SC_PAGE_SIZE") // 1024
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        s = read("/proc/%s/stat" % d)
        if not s:
            continue
        rp = s.rfind(")")
        comm = s[s.find("(") + 1:rp]
        f = s[rp + 2:].split()
        # f[0] = state (field 3); ppid field 4, utime 14, stime 15, threads 20, rss 24
        try:
            procs[int(d)] = (int(f[1]), int(f[11]) + int(f[12]), int(f[21]) * page_kb, int(f[17]), comm)
        except (IndexError, ValueError):
            pass
    return procs


def tree(root, procs):
    kids = {}
    for pid, p in procs.items():
        kids.setdefault(p[0], []).append(pid)
    out, stack = [], [root]
    while stack:
        p = stack.pop()
        if p in procs:
            out.append(p)
            stack.extend(kids.get(p, []))
    return out


def container_pid(name):
    try:
        r = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}} {{.State.Pid}}", name],
                           capture_output=True, text=True, timeout=5)
        running, pid = r.stdout.split()
        return int(pid) if running == "true" and int(pid) > 0 else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--run-dir")
    ap.add_argument("--containers", default="lane_csi,lane_usb,lane_mkr")
    ap.add_argument("--interval", type=float, default=1.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    zones, rl = thermal_zones(), rails()
    cols = (["mono_ns", "realtime_ns", "mem_avail_kb", "swap_used_kb", "cpu_pct", "gpu_load_pct"] +
            list(DEVFREQ) + [z[0] for z in zones] + [r[0] for r in rl] +
            ["disk_free_gb", "run_gb", "run_gbph_5min"])
    sysf = open(os.path.join(a.out, "sysmon.csv"), "w")
    sysf.write(",".join(cols) + "\n")
    procf = open(os.path.join(a.out, "proc.csv"), "w")
    procf.write("mono_ns,name,pids,cpu_pct,rss_kb,threads\n")

    names = [c for c in a.containers.split(",") if c]
    roots, last_resolve = {}, 0.0
    prev_cpu = cpu_ticks()
    prev_ticks = {}
    prev_t = time.monotonic()
    run_hist = []                       # (mono s, bytes) for the trailing GB/h
    run_bytes, last_walk = 0, 0.0
    next_t = time.monotonic()

    while not stop:
        next_t += a.interval
        time.sleep(max(0.0, next_t - time.monotonic()))
        now = time.monotonic()
        dt = max(1e-3, now - prev_t)
        prev_t = now
        mono_ns, real_ns = time.monotonic_ns(), time.time_ns()

        if now - last_resolve >= 10:
            last_resolve = now
            roots = {n: container_pid(n) for n in names}
            try:
                argus = subprocess.run(["pgrep", "-x", "nvargus-daemon"], capture_output=True, text=True).stdout.split()
                roots["nvargus-daemon"] = int(argus[0]) if argus else None
            except (OSError, ValueError):
                roots["nvargus-daemon"] = None

        avail, swap = meminfo()
        tot, idle = cpu_ticks()
        dtot = max(1, tot - prev_cpu[0])
        cpu_pct = 100.0 * os.cpu_count() * (1 - (idle - prev_cpu[1]) / dtot)
        prev_cpu = (tot, idle)
        load = read(GPU_LOAD)
        row = [mono_ns, real_ns, avail, swap, "%.1f" % cpu_pct, "%.1f" % (int(load) / 10) if load else ""]
        for p in DEVFREQ.values():
            v = read(p)
            row.append(int(v) // 1000000 if v else "")
        for _, p in zones:
            v = read(p)
            row.append("%.1f" % (int(v) / 1000) if v and v.lstrip("-").isdigit() else "")
        for _, vp, cp in rl:
            v, c = read(vp), read(cp)
            row.append(int(v) * int(c) // 1000 if v and c else "")
        disk = shutil.disk_usage(a.run_dir or a.out)
        if a.run_dir and now - last_walk >= 10:
            last_walk = now
            run_bytes = dir_bytes(a.run_dir)
            run_hist.append((now, run_bytes))
            while run_hist and now - run_hist[0][0] > 300:
                run_hist.pop(0)
        rate = ""
        if len(run_hist) >= 2 and run_hist[-1][0] > run_hist[0][0]:
            rate = "%.2f" % ((run_hist[-1][1] - run_hist[0][1]) / 1e9 / ((run_hist[-1][0] - run_hist[0][0]) / 3600))
        row += ["%.2f" % (disk.free / 1e9), "%.3f" % (run_bytes / 1e9) if a.run_dir else "", rate]
        sysf.write(",".join(str(x) for x in row) + "\n")

        procs = all_procs()
        for name, root in roots.items():
            if not root:
                procf.write("%d,%s,,,,\n" % (mono_ns, name))
                continue
            pids = tree(root, procs)
            ticks = sum(procs[p][1] for p in pids)
            key = (name, root)
            # Ticks going backwards = processes left the tree (exit): no sample.
            cpu = ("" if key not in prev_ticks or ticks < prev_ticks[key]
                   else "%.1f" % (100.0 * (ticks - prev_ticks[key]) / CLK_TCK / dt))
            prev_ticks[key] = ticks
            procf.write("%d,%s,%d,%s,%d,%d\n" % (mono_ns, name, len(pids), cpu,
                                                 sum(procs[p][2] for p in pids), sum(procs[p][3] for p in pids)))
        sysf.flush()
        procf.flush()

    sysf.close()
    procf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
