#!/usr/bin/env python3
"""Record the MKR Zero's USB stream: raw CAN frames to candump, the console to text.

    mkr_stream_log.py SECONDS OUTDIR [--port /dev/ttyACMx] [--iface can0]

The production firmware (CanMode::DISCOVER) streams every CAN frame it drains
over its native USB, interleaved with its human console (CANRawStream.h holds
the line contract). This splits them:

  OUTDIR/can_raw.log      candump -l format, one frame per line, arrival order:
                            (1790000000.123456) can0 17C#0011223344556677
                          3 hex digits for a standard id, 8 for an extended one,
                          "<id>#R" / "<id>#R<len>" for a remote frame (can-utils /
                          python-can CanutilsLogReader conventions).
  OUTDIR/can_stats.csv    one row per "FS" line: host epoch and monotonic time,
                          the timeline, the MKR's raw and unwrapped micros(), its
                          five counters (drained, streamed, ovf, ringdrop, nohost;
                          uint32, wrapping, cumulative since MKR boot), and
                          rx_frames: frame lines received here since the previous
                          FS line of the same timeline. Lines leave in order and
                          an FS line is formatted after every earlier packet was
                          written, so rx_frames must equal the change in
                          `streamed`; any shortfall was lost between the MKR's USB
                          write and this logger (totalled in the end note).
  OUTDIR/can_sync.csv     every event that changes how MKR time becomes host
                          time: logger start, new timeline, offset change, wall
                          clock step (and 'output writes failed N s', see
                          DISK below). Each row carries host epoch, host
                          monotonic and their difference, so any timestamp in
                          can_raw.log can be recomputed on either clock.
  OUTDIR/mkr_console.txt  every other line, prefixed with local wall time, plus
                          '# port lost/open' notes, malformed stream lines, and a
                          '# mark' line every 30 s with the full date and
                          CLOCK_MONOTONIC (what mkr_console_log.py always wrote).
  OUTDIR/mkr_stream_log.err  only if the logger itself dies: its traceback.

── TIMESTAMPS ──────────────────────────────────────────────────────────────────
A frame line carries micros() from the moment the drain read it out of the
MCP2515, which is far more precise than its arrival here: frames can wait in the
MKR's RAM ring for hundreds of milliseconds while loop() is busy. So each frame
is timed on the MKR's clock and mapped to the host's:

  host_mono = mkr_seconds + offset,   offset = min(host_receive_mono - mkr_seconds)

The minimum over a sliding window, because the delay between the two clocks is
only ever positive (ring wait, USB, this process) and its floor is what the
fastest-delivered line sees; a late chunk can only raise a candidate, never
lower the minimum. Each read is one sample, taken with the newest MKR time it
carried — usually the FS line, which is stamped as it is written and so carries
no ring wait at all. The window is short enough (20 s) that relative drift costs
little: if the MKR's clock runs slow against the host's, the minimum is the
window's OLDEST sample and maps frames early by drift x 20 s — 0.4 ms at 20 ppm,
2 ms at 100 ppm (a fast MKR costs only drift x HOLD_S). Note that NTP steers
CLOCK_MONOTONIC's rate too: timesyncd on a jittery link can hold it hundreds of
ppm off (-464 ppm on this Jetson over WiFi, 2026-10-05: up to ~9 ms early). In
the car, without NTP, it runs at the crystal's rate. Frames
are written 2 s after they arrive, by which time the minimum includes samples on
both sides of them, so the first frames of a run are timed as well as the rest.

Two reads are NOT samples, because what they carry can have waited out a gap of
any length — and a sample seconds too high, alone in the window, would become
the offset, map the next fresh line seconds into the future and split the
timeline over nothing:
  - the first read after the port opens. The MKR's bulk-IN bank holds the last
    packet written before the previous reader closed the port (cdc-acm stops
    polling at close; nothing on either side clears the bank), and it is the
    first thing the next open receives: on any restart of this logger against a
    running MKR, one or two frame lines, or an FS line, from however long ago.
    Its lines are also PROVISIONAL: should a later line not fit after them (a
    gap of more than 35.8 min reads as micros() going backwards), the timeline
    is re-seeded rather than split, and those frames keep their arrival time.
    An FS line in that read does not start the loss accounting either, or the
    previous reader's unread tail would count as lost here.
  - the first read after a silence longer than the window, if it would RAISE
    the minimum by more than the clocks can drift in that time. The reader was
    frozen (SIGSTOP, a stalled bus) while the kernel's tty buffer or the bank
    kept data from the start of the silence; the old minimum is carried over
    instead, at the cost of drift x silence until fresh samples replace it.

Epoch time is host monotonic plus (epoch - monotonic) AS OF THE MOMENT OF
WRITING. After the rig's clock is stepped (v0.4 sets it from NTP/GPS; at boot in
the car it can start in 1970) every frame written later carries the corrected
time and every frame written before carries the boot clock — so the step shows
in can_raw.log as one jump, at most HOLD_S of arrivals away from the step
itself, and can_sync.csv records it ('clock step +N s') with both clocks.

── micros() ACROSS WRAPS, STALLS AND RESETS ─────────────────────────────────────
micros() wraps every 2^32 us (~71.6 min). Each value is placed by its SIGNED
distance modulo 2^32 from the newest one seen, never by comparing raw values: an
FS line is stamped when written and the frames after it when they were READ, so
near the wrap a raw comparison would put such an older frame 71 minutes ahead.

How far behind the newest an honest line can be is unbounded, though: after a
stall (this host stopped reading; the kernel throttles the CDC endpoint within a
few KB) the ring holds frames from the START of the stall, and the first packet
afterwards leads with a fresh FS line. A fixed "N seconds backwards = reset"
rule calls that a reset. What does hold is order: the ring is FIFO, so F lines
arrive in drain order, and an FS line is formatted after every frame already
streamed was drained. A new timeline (an MKR reset the port did not report) is
therefore declared only when
  - an F line steps back from the previous F line, or an FS line from the
    previous F or FS line, by more than BACK_TOL_US;  or
  - a line maps more than FUTURE_S past host now (a reset after > 35.8 min of
    uptime reads as micros() jumping AHEAD modulo 2^32);  or
  - the port is lost and reopened (every real MKR reset: its native USB
    re-enumerates), or its 'BOOT: reset cause' banner arrives mid-timeline.
Each new timeline restarts the unwrapping and the offset; frames already waiting
keep the offset of the timeline they arrived in.

── READING ─────────────────────────────────────────────────────────────────────
The port is read by a thread of its own that does nothing else. Disk writes and
fsync happen in the main thread and can stall for seconds on an SD card or a
busy USB disk; had they shared the reading thread, the n_tty buffer (4 KB) would
fill, cdc-acm would throttle the bulk-IN endpoint, and the MKR's ring (1024
frames, ~0.85 s of this bus) would overflow — frame loss caused by the logger.

── DISK ────────────────────────────────────────────────────────────────────────
A failing write, flush or fsync (the backup disk full, or gone) is counted, not
fatal: the logger keeps reading the port and writes again as soon as the disk
takes it, losing whole lines only, and then notes in mkr_console.txt for how
long writes failed and exactly how many bytes of can_raw.log (about how many
frames) never reached the disk. The first failure also goes to stderr (docker
logs), since the files may be what cannot be written. See class Out.

── THE PORT ────────────────────────────────────────────────────────────────────
Opened exclusively (flock + TIOCEXCL, as CANRawLog's can_log.py does): a second
reader would silently take a share of the lines. HUPCL is SET, so DTR drops when
this process closes the port or dies. The firmware gates the stream on DTR: low,
it counts frames as 'nohost' and keeps its USB bank free; left high with nobody
reading, the bank stays armed, frames are counted as ringdrop instead, and in
SNIFF the stuck ring would starve the decoder. (mkr_console_log.py and
can_log.py clear HUPCL, which leaves DTR high after they exit.)

Needs tty permission: on the Jetson run it in the dev container
(docker run --privileged -v /dev:/dev ...).
"""
import argparse
import collections
import errno
import fcntl
import glob
import os
import queue
import re
import select
import signal
import sys
import termios
import threading
import time
import traceback

WRAP = 1 << 32
HALF = 1 << 31
WINDOW_S = 20.0          # offset minimum over this much host time
HOLD_S = 2.0             # frames are written this long after arrival
FLUSH_S = 1.0
FSYNC_S = 5.0
MARK_S = 30.0
BACK_TOL_US = 100_000    # out-of-order tolerance; drain order makes anything more a new MKR boot
FUTURE_S = 5.0           # a line mapping this far past host now is a new MKR boot
STEP_S = 0.05            # wall-minus-monotonic moving this much between two checks is a step, not a slew
SILENCE_SLACK_S = 0.05   # after a silence > WINDOW_S, a first sample this much (+ drift) above the minimum is stale
DRIFT_MAX = 0.001        # 1000 ppm: a crystal's +-100 plus the +-500 an NTP slew puts on CLOCK_MONOTONIC, doubled
OFFSET_NOTE_S = 0.0005   # can_sync.csv gets a row when the offset moves more than this
MAX_PARTIAL = 4096       # bytes without a newline before the partial line is dropped
READ_SIZE = 65536

# The contract (CANRawStream.h), matched on bytes so a frame line is never decoded to str:
#   F tttttttt iii d hh..    F tttttttt iiiiiiii d hh..    F tttttttt iii R d
#   FS tttttttt drained streamed ovf ringdrop nohost
# Uppercase hex only, single spaces, DLC 0 ends after the digit. Anything else is malformed.
FRAME_RE = re.compile(rb"F ([0-9A-F]{8}) ([0-9A-F]{3}|[0-9A-F]{8}) (?:([0-8])(?: ([0-9A-F]+))?|R ([0-8]))")
STATS_RE = re.compile(rb"FS ([0-9A-F]{8}) ([0-9]{1,10}) ([0-9]{1,10}) ([0-9]{1,10}) ([0-9]{1,10}) ([0-9]{1,10})")


def find_port():
    """The MKR Zero's tty, by USB identity (Arduino 2341:804f), or None."""
    for t in sorted(glob.glob("/sys/class/tty/ttyACM*")):
        d = os.path.realpath(t + "/device/..")
        try:
            with open(d + "/idVendor") as v, open(d + "/idProduct") as p:
                if v.read().strip() == "2341" and p.read().strip() == "804f":
                    return "/dev/" + os.path.basename(t)
        except OSError:
            pass
    return None


def open_port(dev):
    """Opens @dev raw, exclusively, with HUPCL (see THE PORT above). Raises OSError/termios.error."""
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.HUPCL
        a[4] = a[5] = termios.B115200    # anything but 1200: a 1200-baud close resets a SAMD into its bootloader
        # Pinned, not inherited: a tool that left VMIN=0 behind makes an empty read return b'',
        # which the reader would take for a disconnect on a live port.
        a[6][termios.VMIN] = 1
        a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
    except BaseException:
        os.close(fd)
        raise
    return fd


def parse_frame(line):
    """b'F t id d data' / b'F t id R d' -> (us, id, ext, rtr, dlc, data_hex_str), or None if malformed."""
    m = FRAME_RE.fullmatch(line)
    if m is None:
        return None
    ts, idt, dlc, data, rdlc = m.groups()
    ident = int(idt, 16)
    ext = len(idt) == 8
    if ident > (0x1FFFFFFF if ext else 0x7FF):
        return None
    if rdlc is not None:
        return int(ts, 16), ident, ext, True, int(rdlc), ""
    n = int(dlc)
    data = data or b""
    if len(data) != 2 * n:
        return None
    return int(ts, 16), ident, ext, False, n, data.decode("ascii")


def parse_stats(line):
    """b'FS t drained streamed ovf ringdrop nohost' -> (us, [5 counters]), or None."""
    m = STATS_RE.fullmatch(line)
    if m is None:
        return None
    counters = [int(x) for x in m.groups()[1:]]
    if max(counters) >= WRAP:
        return None
    return int(m.group(1), 16), counters


def wall_minus_mono():
    """time.time() - time.monotonic() at one instant.

    Two separate calls: whatever runs between them — the reader thread taking the GIL, the scheduler
    preempting this one — lands in the difference whole, and from there in every frame written before
    the next check (seen on the pty: a batch 0.26 ms early); past STEP_S it would even be recorded as
    a clock step and its reversal. So the wall clock is read BETWEEN two monotonic readings and set
    against their midpoint, and the tightest of up to three tries is kept: error <= half that gap."""
    best = None
    for _ in range(3):
        m0 = time.monotonic()
        w = time.time()
        m1 = time.monotonic()
        if best is None or m1 - m0 < best[0]:
            best = (m1 - m0, w - (m0 + m1) / 2)
        if m1 - m0 < 0.0001:
            break
    return best[1]


def candump_line(epoch, ident, ext, rtr, dlc, data_hex, iface):
    idtxt = ("%08X" % ident) if ext else ("%03X" % ident)
    body = ("R%d" % dlc if dlc else "R") if rtr else data_hex
    return "(%.6f) %s %s#%s\n" % (epoch, iface, idtxt, body)


class Timeline:
    """One MKR boot as seen by this host: micros() unwrapping and the windowed-minimum offset."""

    def __init__(self):
        self.newest = None       # unwrapped micros() of the newest line of either kind (int, can pass 2^32)
        self.last_frame = None   # ... of the last F line: frames arrive in drain order
        self.last_fs = None      # ... of the last FS line
        self.mins = collections.deque()   # (host_mono, candidate), candidates increasing: sliding minimum
        self.offset = None
        self.why = ""
        self.provisional = True  # every line so far came from the first read after an open (Logger.place)
        self.frames = 0          # F lines placed on it

    def unwrap(self, raw, frame, host_mono=None):
        """Unwrapped micros() of @raw, or None when it cannot belong to this boot (self.why says why).

        Placed by signed modular distance from the newest value — see the module docstring for why
        neither raw comparison nor a fixed backwards limit is right — then checked against ORDER."""
        if self.newest is None:
            t = raw
        else:
            t = self.newest + ((raw - self.newest + HALF) % WRAP) - HALF
            if frame:
                floor = self.last_frame
            elif self.last_frame is None or (self.last_fs is not None and self.last_fs > self.last_frame):
                floor = self.last_fs
            else:
                floor = self.last_frame
            if floor is not None and t < floor - BACK_TOL_US:
                self.why = "micros() went backwards %.3f s" % ((floor - t) / 1e6)
                return None
            if self.offset is not None and host_mono is not None and t / 1e6 + self.offset > host_mono + FUTURE_S:
                self.why = "micros() jumped %.1f s ahead of the host" % (t / 1e6 + self.offset - host_mono)
                return None
        if self.newest is None or t > self.newest:
            self.newest = t
        if frame:
            self.frames += 1
            if self.last_frame is None or t > self.last_frame:
                self.last_frame = t
        elif self.last_fs is None or t > self.last_fs:
            self.last_fs = t
        return t

    def observe(self, host_mono, mkr_s):
        """Adds one delivery sample (host receive time vs the newest MKR time it carried).

        Returns True when the offset moved. The delay between the clocks is only ever positive, so the
        offset is the MINIMUM candidate over the window; a monotonic deque keeps that O(1)."""
        c = host_mono - mkr_s
        if self.mins and host_mono - self.mins[-1][0] > WINDOW_S:
            # Every sample has aged out: nothing was read for longer than the window. A first sample that
            # would raise the minimum by more than the clocks can drift in that time is data that waited
            # out the silence (the module docstring); carry the old minimum over, re-stamped, until fresh
            # samples have filled the window around it.
            floor = self.mins[0][1]
            if c > floor + SILENCE_SLACK_S + DRIFT_MAX * (host_mono - self.mins[-1][0]):
                self.mins.clear()
                self.mins.append((host_mono, floor))
                return False
        while self.mins and self.mins[-1][1] >= c:
            self.mins.pop()
        self.mins.append((host_mono, c))
        while host_mono - self.mins[0][0] > WINDOW_S:
            self.mins.popleft()
        new = self.mins[0][1]
        changed = self.offset is None or abs(new - self.offset) > OFFSET_NOTE_S
        self.offset = new
        return changed


class Out:
    """One output file that outlives a full or failing disk.

    A write, flush or fsync that fails — ENOSPC when the backup disk fills, EIO when it drops off the USB
    bus — must not end the logger: under drive_session.sh nothing restarts it (docker --rm), so one full
    disk for a few seconds, before v0.4's loop recording frees space again, would cost the raw CAN of the
    rest of a 12 h session (seen on the pty under RLIMIT_FSIZE: one EFBIG, traceback, exit). The error is
    counted (Logger.out_failed) and the logger carries on reading the port. What is lost is lost in whole
    lines, so the file stays parseable: every write() here is one complete line; a failed hand-over from
    TextIOWrapper discards its pending chunk whole, and BufferedWriter keeps the unwritten tail of a short
    write and completes it at the next flush that succeeds (checked under EFBIG, CPython 3.10: no torn or
    merged lines across the gap). Explicitly UTF-8: console text can carry U+FFFD from a garbled byte, which
    a non-UTF-8 locale could not encode."""

    __slots__ = ("f", "name", "lg", "start", "handed")

    def __init__(self, lg, path, buffering):
        self.f = open(path, "a", buffering=buffering, encoding="utf-8", errors="replace")
        self.name = os.path.basename(path)
        self.lg = lg
        self.start = self.f.tell()   # size at open (append mode): what the file held before this run
        self.handed = 0              # characters given to write(); can_raw.log is pure ASCII, so bytes

    def write(self, s):
        self.handed += len(s)
        try:
            self.f.write(s)
        except OSError as e:
            self.lg.out_failed(self, e)

    def flush(self):
        try:
            self.f.flush()
            return True
        except OSError as e:
            self.lg.out_failed(self, e)
            return False

    def fsync(self):
        try:
            os.fsync(self.f.fileno())
        except OSError as e:
            if e.errno != errno.EINVAL:  # EINVAL: not a file that can be synced (a pipe, /dev/null)
                self.lg.out_failed(self, e)

    def lost(self):
        """Characters handed over that are not in the file. Exact right after a flush that succeeded."""
        try:
            return self.start + self.handed - os.fstat(self.f.fileno()).st_size
        except OSError:
            return 0

    def close(self):
        try:
            self.f.close()               # flushes first; the fd is closed even if that flush fails
        except OSError as e:
            self.lg.out_failed(self, e)


class Logger:
    """Everything but the port: line splitting, timelines, the hold queue and the four files."""

    def __init__(self, outdir, iface):
        os.makedirs(outdir, exist_ok=True)
        self.iface = iface
        self.out_errors = 0          # failed writes/flushes/fsyncs (Out)
        self.out_err = None          # (monotonic, first error text) of the failure episode under way
        self.raw_lost = 0            # can_raw.log characters already reported lost
        self.written = 0             # frame lines handed to can_raw.log
        self.raw = Out(self, os.path.join(outdir, "can_raw.log"), 1 << 16)
        self.stats = Out(self, os.path.join(outdir, "can_stats.csv"), 1 << 14)
        self.sync = Out(self, os.path.join(outdir, "can_sync.csv"), 1)
        self.con = Out(self, os.path.join(outdir, "mkr_console.txt"), 1)
        if self.stats.start == 0:
            self.stats.write("host_epoch,host_mono,timeline,mkr_us,mkr_s,drained,streamed,ovf,ringdrop,nohost,"
                             "rx_frames\n")
        if self.sync.start == 0:
            self.sync.write("host_epoch,host_mono,wall_minus_mono,timeline,event,offset_s,mkr_s\n")
        self.timeline_no = 0
        self.tl = Timeline()
        self.pending = collections.deque()   # (host_mono_arrival, timeline obj, mkr_s, frame tuple)
        self.buf = b""
        self.frames = self.stat_lines = self.console_lines = self.malformed = 0
        self.rx_since_fs = 0         # F lines on this timeline since its last FS line
        self.prev_streamed = None    # that FS line's `streamed`, or None before the first
        self.usb_checked = self.usb_lost = 0
        self.after_open = False      # the next read that completes a line is the first since the port opened
        self.wall_minus_mono = wall_minus_mono()
        self.last_flush = self.last_fsync = time.monotonic()
        self.sync_row("logger start")

    # ── output helpers ──────────────────────────────────────────────────────────

    def stamp(self):
        now = time.time()
        return time.strftime("%H:%M:%S", time.localtime(now)) + ".%03d" % int((now % 1) * 1000)

    def console(self, text):
        self.con.write(self.stamp() + " " + text + "\n")

    def note(self, text):
        self.con.write("# " + self.stamp() + " " + text + "\n")

    def out_failed(self, out, e):
        """An output write/flush/fsync failed (Out). The episode is reported in the files once they take
        writes again (out_recovered), and at once on stderr — docker logs drive_mkr — since the files may
        be exactly what cannot be written."""
        self.out_errors += 1
        if self.out_err is None:
            self.out_err = (time.monotonic(), "%s: %s" % (out.name, e))
            try:
                sys.stderr.write("mkr_stream_log: %s %s - frames are dropped until it can write again\n"
                                 % (time.strftime("%Y-%m-%dT%H:%M:%S%z"), self.out_err[1]))
            except (OSError, ValueError):
                pass

    def out_recovered(self):
        """Every file flushed: the episode is over. How much of can_raw.log it cost is exact here."""
        lost = self.raw.lost()
        per_line = self.raw.handed / self.written if self.written else 45.0
        self.note("output writes failed for %.1f s (first: %s; %d failures in all so far); can_raw.log lost "
                  "%d bytes, about %d frame lines" % (time.monotonic() - self.out_err[0], self.out_err[1],
                                                     self.out_errors, lost - self.raw_lost,
                                                     round((lost - self.raw_lost) / per_line)))
        self.sync_row("output writes failed %.1f s" % (time.monotonic() - self.out_err[0]))
        self.raw_lost = lost
        self.out_err = None

    def sync_row(self, event, offset="", mkr_s="", mono=None):
        mono = time.monotonic() if mono is None else mono
        self.sync.write("%.6f,%.6f,%.6f,%d,%s,%s,%s\n" % (mono + self.wall_minus_mono, mono, self.wall_minus_mono,
                                                         self.timeline_no, event, offset, mkr_s))

    # ── timelines ───────────────────────────────────────────────────────────────

    def new_timeline(self, why):
        # Frames still in the hold queue keep a reference to their own Timeline, so they are written
        # later with the offset they were measured against — nothing needs flushing early here.
        self.timeline_no += 1
        self.tl = Timeline()
        self.rx_since_fs = 0
        self.prev_streamed = None
        self.sync_row("new timeline (%s)" % why)
        self.note("new timeline %d: %s" % (self.timeline_no, why))

    def place(self, raw, frame, host_mono, first):
        us = self.tl.unwrap(raw, frame, host_mono)
        if us is None:
            if self.tl.provisional:
                # What it does not fit after came only from the first read since the open: the packet the
                # MKR's bank kept from before it (module docstring). Not a new boot: start this timeline's
                # unwrapping again from this line. The stale frames keep their own Timeline object, which
                # never gets a sample (the first read is none), so they are written with their arrival time.
                self.note("first packet after the port opened was stale (left in the MKR's USB bank before "
                          "it: %s); its %d frame line(s) carry their arrival time" % (self.tl.why, self.tl.frames))
                self.sync_row("stale first packet (%s)" % self.tl.why)
                self.tl = Timeline()
            else:
                self.new_timeline(self.tl.why)
            us = self.tl.unwrap(raw, frame, host_mono)
        if not first:
            self.tl.provisional = False
        return us / 1e6

    # ── input ───────────────────────────────────────────────────────────────────

    def bad(self, line):
        self.malformed += 1
        self.note("malformed: " + line[:120].decode("ascii", "replace"))

    def handle(self, line, host_mono, first=False):
        """One complete line (no newline). Returns its unwrapped MKR seconds for F/FS lines, else None.

        @first: the line came in the first read since the port opened (see the module docstring)."""
        if line[:2] == b"F ":
            fr = parse_frame(line)
            if fr is None:
                self.bad(line)
                return None
            mkr_s = self.place(fr[0], True, host_mono, first)
            self.pending.append((host_mono, self.tl, mkr_s, fr))
            self.frames += 1
            self.rx_since_fs += 1            # after place(): a frame that began a timeline counts in it
            return mkr_s
        if line[:3] == b"FS ":
            st = parse_stats(line)
            if st is None:
                self.bad(line)
                return None
            mkr_s = self.place(st[0], False, host_mono, first)
            self.stat_lines += 1
            self.stats.write("%.6f,%.6f,%d,%d,%.6f,%s,%d\n" % (host_mono + self.wall_minus_mono, host_mono,
                                                                self.timeline_no, st[0], mkr_s,
                                                                ",".join(str(c) for c in st[1]), self.rx_since_fs))
            streamed = st[1][1]
            if self.prev_streamed is not None:
                d = (streamed - self.prev_streamed) % WRAP
                self.usb_checked += d
                self.usb_lost += d - self.rx_since_fs
            # Not from the first read since the open: that FS line may be the bank's, written before the
            # previous reader closed, and the frames streamed between it and that close never came here.
            self.prev_streamed = None if first else streamed
            self.rx_since_fs = 0
            return mkr_s
        text = line.decode("utf-8", "replace")
        # The banner can only reach us on a fresh timeline (a reset re-enumerates the port, and the
        # reopen started one), so a banner on a timeline that already carried stream lines means the
        # reset went unreported — or the firmware stopped re-enumerating; either way, start afresh.
        if "BOOT" in text and "reset cause" in text.lower() and self.tl.newest is not None:
            self.new_timeline("MKR boot banner")
        self.console_lines += 1
        self.console(text)
        return None

    def data(self, host_mono, chunk):
        """One read's bytes. Complete lines are handled; the read is one delivery sample."""
        self.buf += chunk
        if b"\n" not in chunk:
            if len(self.buf) > MAX_PARTIAL:
                self.malformed += 1
                self.note("overlong line dropped (%d bytes): %s" % (len(self.buf), self.buf[:120].decode("ascii", "replace")))
                self.buf = b""
            return
        *lines, self.buf = self.buf.split(b"\n")
        first, self.after_open = self.after_open, False
        tl, newest = self.tl, None
        for raw in lines:
            if raw.endswith(b"\r"):
                raw = raw[:-1]
            t = self.handle(raw, host_mono, first)
            if t is None:
                continue
            if self.tl is not tl:            # a new timeline began inside this read: only its lines count
                tl, newest = self.tl, None
            if newest is None or t > newest:
                newest = t
        # The read's LARGEST MKR time is its tightest candidate: every line in it had arrived by host_mono.
        # Except in the first read since the open, which may be the bank's leftover (module docstring).
        if newest is not None and not first and tl is self.tl and tl.observe(host_mono, newest):
            self.sync_row("offset", "%.6f" % tl.offset, "%.6f" % newest, mono=host_mono)

    def port_lost(self, why):
        self.note("port lost (%s)" % why)
        if self.buf:
            self.note("partial line at port loss: " + self.buf[:120].decode("ascii", "replace"))
            self.buf = b""

    def port_open(self, dev, lost_for):
        self.note("port open %s" % dev + (" (was lost %.1f s)" % lost_for if lost_for is not None else ""))
        if lost_for is not None:
            self.new_timeline("port reopened")
        self.buf = b""
        self.after_open = True

    # ── output timing ───────────────────────────────────────────────────────────

    def check_clock(self):
        """Records a wall-clock step.

        Checked every main-loop pass (<= 0.1 s apart), so a slew — 0.5 ms/s for timesyncd and ntpd, 83 ms/s
        at chrony's fastest — moves the difference far less than STEP_S between two checks, while a step
        lands whole. 50 ms rather than seconds because timesyncd steps whenever its offset passes 0.4 s:
        on a jittery link it does so every few polls (seen on this Jetson over WiFi, 2026-10-05: steps of
        about 0.5 s every 30-60 s), and every one of those moves can_raw.log's epoch times."""
        e_m = wall_minus_mono()
        if abs(e_m - self.wall_minus_mono) > STEP_S:
            step = e_m - self.wall_minus_mono
            self.wall_minus_mono = e_m
            self.sync_row("clock step %+.6f s" % step)
            self.note("wall clock stepped %+.3f s; frames written from now on carry the new time" % step)
        else:
            self.wall_minus_mono = e_m

    def flush_pending(self, force=False):
        now = time.monotonic()
        e_m = self.wall_minus_mono
        write = self.raw.write
        pending = self.pending
        n = 0
        while pending and (force or now - pending[0][0] >= HOLD_S):
            host_mono, tl, mkr_s, (_us, ident, ext, rtr, dlc, data) = pending.popleft()
            # No offset only if the timeline never completed a read; the frame's own delivery is then
            # the best (latest-possible) estimate.
            off = tl.offset if tl.offset is not None else host_mono - mkr_s
            write(candump_line(mkr_s + off + e_m, ident, ext, rtr, dlc, data, self.iface))
            n += 1
        self.written += n

    def periodic(self):
        now = time.monotonic()
        self.check_clock()
        self.flush_pending()
        if now - self.last_flush >= FLUSH_S:
            ok = True
            for f in (self.raw, self.stats, self.con, self.sync):
                ok = f.flush() and ok
            self.last_flush = now
            if ok and self.out_err is not None:
                self.out_recovered()
        if now - self.last_fsync >= FSYNC_S:
            for f in (self.raw, self.stats, self.con, self.sync):
                f.fsync()
            self.last_fsync = now

    def close(self):
        self.check_clock()
        self.flush_pending(force=True)
        if self.raw.flush() and self.out_err is not None:
            self.out_recovered()
        self.note("logger end: %d frames, %d FS lines, %d console lines, %d malformed, %d timeline(s); "
                  "between FS lines %d frames streamed by the MKR, %d not received here"
                  % (self.frames, self.stat_lines, self.console_lines, self.malformed, self.timeline_no + 1,
                     self.usb_checked, self.usb_lost)
                  + ("; %d output write failures%s" % (self.out_errors, ", STILL FAILING" if self.out_err else "")
                     if self.out_errors else ""))
        for f in (self.raw, self.stats, self.sync, self.con):
            f.flush()
            f.fsync()
            f.close()


class PortReader(threading.Thread):
    """Owns the tty: finds, opens, reads, notices loss. Hands (kind, host_mono, ...) to the main thread.

    Its own thread so that nothing the main thread does — a write, an fsync — can pause the reads;
    see READING in the module docstring. host_mono is taken the moment os.read() returns."""

    def __init__(self, port, events, stop):
        super().__init__(name="mkr-port", daemon=True)
        self.port, self.events, self.stop = port, events, stop

    def run(self):
        fd, lost_at, last_err = None, None, None
        put = self.events.put
        while not self.stop.is_set():
            if fd is None:
                dev = self.port or find_port()
                if dev is None or not os.path.exists(dev):
                    self.stop.wait(0.02)
                    continue
                try:
                    fd = open_port(dev)
                except (OSError, termios.error) as e:
                    msg = "cannot open %s: %s" % (dev, e)
                    if msg != last_err:          # once per distinct failure, not 5 times a second
                        put(("note", time.monotonic(), msg))
                        last_err = msg
                    self.stop.wait(0.2)
                    continue
                last_err = None
                put(("open", time.monotonic(), dev, None if lost_at is None else time.monotonic() - lost_at))
                lost_at = None
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
            except (OSError, ValueError):
                r = [fd]                         # let the read report what is wrong with the fd
            if not r:
                continue
            why = "EOF"
            try:
                chunk = os.read(fd, READ_SIZE)
            except BlockingIOError:
                continue
            except OSError as e:
                chunk, why = b"", str(e)
            mono = time.monotonic()
            if chunk:
                put(("data", mono, chunk))
                continue
            put(("lost", mono, why))
            try:
                os.close(fd)
            except OSError:
                pass
            fd, lost_at = None, mono
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass


def dispatch(lg, ev):
    kind = ev[0]
    if kind == "data":
        lg.data(ev[1], ev[2])
    elif kind == "open":
        lg.port_open(ev[2], ev[3])
    elif kind == "lost":
        lg.port_lost(ev[2])
    else:
        lg.note(ev[2])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("seconds", type=float)
    ap.add_argument("outdir")
    ap.add_argument("--port", help="device to open instead of the MKR found by USB identity (tests: a pty)")
    ap.add_argument("--iface", default="can0", help="interface name written into can_raw.log")
    a = ap.parse_args(argv)

    # The reader thread wants the GIL back promptly after its read returns: its timestamp is taken
    # then, and every millisecond it waits is a millisecond added to that delivery sample.
    sys.setswitchinterval(0.001)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    lg = Logger(a.outdir, a.iface)
    try:
        lg.note("logger start, %.0f s" % a.seconds)
        events = queue.SimpleQueue()
        reader = PortReader(a.port, events, stop)
        reader.start()
        m0 = time.monotonic()
        next_mark = 0.0
        try:
            while not stop.is_set() and time.monotonic() - m0 < a.seconds:
                now = time.monotonic()
                if now >= next_mark:
                    next_mark = now + MARK_S
                    lg.con.write("# mark %s mono=%.3f\n" % (time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                                                            time.clock_gettime(time.CLOCK_MONOTONIC)))
                # Whatever has queued, but bounded, so a backlog after a disk stall cannot hold off
                # the flushes and the hold queue for long.
                try:
                    ev = events.get(timeout=0.1)
                    for _ in range(2000):
                        dispatch(lg, ev)
                        ev = events.get_nowait()
                    dispatch(lg, ev)
                except queue.Empty:
                    pass
                lg.periodic()
        finally:
            stop.set()
            reader.join(timeout=2.0)
            while True:                          # what was read before the stop still gets written
                try:
                    dispatch(lg, events.get_nowait())
                except queue.Empty:
                    break
            lg.close()
    except BaseException:
        # Under docker --rm the container's own log dies with it, so the reason goes beside the data.
        try:
            with open(os.path.join(a.outdir, "mkr_stream_log.err"), "a") as f:
                f.write("%s\n%s\n" % (time.strftime("%Y-%m-%dT%H:%M:%S%z"), traceback.format_exc()))
        except OSError:
            pass
        raise


if __name__ == "__main__":
    main()
