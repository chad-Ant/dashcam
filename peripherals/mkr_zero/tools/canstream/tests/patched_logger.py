#!/usr/bin/env python3
"""Runs the logger with test patches that no real system call can be asked for.

    patched_logger.py JSON LOGGER_PATH LOGGER_ARGS...

JSON keys:
  fixed_wall: K                   time.time() is time.monotonic() + K: a wall clock nothing steps, so
                                  a test's truth is exact even while the host's NTP client steps the
                                  real clock (timesyncd on a jittery link does, by ~0.5 s, every few
                                  polls)
  fsync_delay, fsync_slow_calls   the first N os.fsync() calls sleep this long first: a slow SD card
                                  or USB disk, without needing one
  clock_step: [after_s, step_s]   time.time() jumps by step_s once after_s seconds have passed: the
                                  rig's clock being set by v0.4 (NTP/GPS), without root or touching the
                                  real clock
The logger module is imported from LOGGER_PATH and its main() called, so it runs unmodified; only the
two functions above are replaced in this process.
"""
import importlib.util
import json
import os
import sys
import time

cfg = json.loads(sys.argv[1])
path = sys.argv[2]
argv = sys.argv[3:]

if "fsync_delay" in cfg:
    real_fsync = os.fsync
    slow_left = [int(cfg.get("fsync_slow_calls", 1))]

    def slow_fsync(fd):
        if slow_left[0] > 0:
            slow_left[0] -= 1
            time.sleep(float(cfg["fsync_delay"]))
        return real_fsync(fd)

    os.fsync = slow_fsync

if "fixed_wall" in cfg:
    k = float(cfg["fixed_wall"])
    time.time = lambda: time.monotonic() + k

if "clock_step" in cfg:
    base_time = time.time
    t0 = time.monotonic()
    after_s, step_s = float(cfg["clock_step"][0]), float(cfg["clock_step"][1])
    time.time = lambda: base_time() + (step_s if time.monotonic() - t0 >= after_s else 0.0)

spec = importlib.util.spec_from_file_location("mkr_stream_log_under_test", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
sys.argv = [path] + argv
mod.main()
