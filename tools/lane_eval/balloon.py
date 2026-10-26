#!/usr/bin/env python3
"""balloon.py — memory-pressure stage for bench test B3m (plan P5b, section 8 T4).

    python3 tools/lane_eval/balloon.py --stages 1000:180,600:120 [--floor-mb 450] [--step-mb 64]
                                       [--log RUN_DIR/sys/balloon.csv]

Runs on the HOST (stdlib only) beside a lane test run.  For each stage
TARGET:SECONDS it allocates (and touches) anonymous memory in --step-mb steps
until /proc/meminfo MemAvailable is at or below TARGET MB, keeps it there for
SECONDS (topping up or giving back as the system moves), then goes on to the
next stage; at the end, or on SIGTERM / SIGINT, it frees everything and exits
0.  It never pushes MemAvailable below --floor-mb (the OOM killer's
territory: the plan wants the degrade ladder to act first, at 1.0 GB and
600 MB).  Logs 1 Hz rows: mono_ns, stage_target_mb, held_mb, mem_avail_mb.
"""
import argparse
import signal
import sys
import time

stop = False


def on_signal(*_):
    global stop
    stop = True


def mem_avail_mb():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    return -1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stages", required=True, help="TARGET_MB:SECONDS,...")
    ap.add_argument("--floor-mb", type=int, default=450)
    ap.add_argument("--step-mb", type=int, default=64)
    ap.add_argument("--log")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    stages = [tuple(int(x) for x in s.split(":")) for s in a.stages.split(",")]
    log = open(a.log, "a") if a.log else sys.stdout
    if a.log and log.tell() == 0:
        log.write("mono_ns,stage_target_mb,held_mb,mem_avail_mb\n")
    held = []
    step = a.step_mb << 20
    for target, secs in stages:
        target = max(target, a.floor_mb)
        t_end = None
        last = 0.0
        while not stop:
            avail = mem_avail_mb()
            if avail > target + a.step_mb and avail - a.step_mb >= a.floor_mb:
                b = bytearray(step)
                for i in range(0, step, 4096):     # touch every page: resident, not just reserved
                    b[i] = 1
                held.append(b)
                continue
            if avail < target - a.step_mb and held:
                held.pop()
            if t_end is None:
                t_end = time.monotonic() + secs
                print("[balloon] stage %d MB reached: holding %d MB for %d s" % (target, len(held) * a.step_mb, secs),
                      file=sys.stderr, flush=True)
            now = time.monotonic()
            if now - last >= 1.0:
                last = now
                log.write("%d,%d,%d,%d\n" % (time.monotonic_ns(), target, len(held) * a.step_mb, avail))
                log.flush()
            if now >= t_end:
                break
            time.sleep(0.2)
    held.clear()
    print("[balloon] released", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
