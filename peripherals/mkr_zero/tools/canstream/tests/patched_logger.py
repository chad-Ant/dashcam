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
  hang_gate: PATH                 a disk that hangs WITHOUT erroring: while PATH exists, every write
                                  that reaches the kernel for one of the logger's output files, and
                                  every os.fsync(), waits (in time.sleep, so the GIL is released, as in a
                                  blocked system call), then completes normally. The first wait creates
                                  PATH + ".hit", so a test knows the writer really was held.
  set: {NAME: value}              module constants replaced before main() runs (e.g. QUEUE_BYTES: a
                                  small bound a test can fill in seconds)
The logger module is imported from LOGGER_PATH and its main() called, so it runs unmodified; only the
functions above are replaced in this process (hang_gate: the module's open() for its output files).
"""
import importlib.util
import io
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

gate = cfg.get("hang_gate")


def wait_gate():
    if gate is None or not os.path.exists(gate):
        return
    if not os.path.exists(gate + ".hit"):
        with open(gate + ".hit", "w"):
            pass
    while os.path.exists(gate):
        time.sleep(0.005)


if gate is not None:
    real_fsync_g = os.fsync

    def gated_fsync(fd):
        wait_gate()
        return real_fsync_g(fd)

    os.fsync = gated_fsync

    class GatedRaw(io.RawIOBase):
        """FileIO whose write() hangs while the gate exists: what a blocked write(2) looks like."""

        def __init__(self, raw):
            self.raw = raw

        def writable(self):
            return True

        def seekable(self):
            return self.raw.seekable()

        def seek(self, pos, whence=0):
            return self.raw.seek(pos, whence)

        def tell(self):
            return self.raw.tell()

        def fileno(self):
            return self.raw.fileno()

        def write(self, b):
            wait_gate()
            return self.raw.write(b)

        def close(self):
            if not self.closed:
                try:
                    super().close()
                finally:
                    self.raw.close()

    def gated_open(file, mode="r", buffering=-1, encoding=None, errors=None, **kw):
        if mode != "a":
            return open(file, mode, buffering, encoding, errors, **kw)
        buf = io.BufferedWriter(GatedRaw(io.FileIO(file, "a")), buffering if buffering > 1 else io.DEFAULT_BUFFER_SIZE)
        return io.TextIOWrapper(buf, encoding=encoding, errors=errors, line_buffering=(buffering == 1))

spec = importlib.util.spec_from_file_location("mkr_stream_log_under_test", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
for name, value in cfg.get("set", {}).items():
    if not hasattr(mod, name):
        sys.exit("patched_logger: the logger has no %s to set" % name)
    setattr(mod, name, value)
if gate is not None:
    mod.open = gated_open
sys.argv = [path] + argv
mod.main()
