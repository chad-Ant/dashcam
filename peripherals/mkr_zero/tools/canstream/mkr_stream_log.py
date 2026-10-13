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
                          row of the same timeline. Lines leave in order and
                          an FS line is formatted after every earlier packet was
                          written, so rx_frames must equal the change in
                          `streamed`; any shortfall was lost between the MKR's USB
                          write and this logger (totalled in the end note).
                          Then rx_dropped: how many of those rx_frames were
                          received but DROPPED here, never written to can_raw.log
                          (HUNG DISK below; 0 otherwise), and fs_dropped: FS lines
                          dropped since the previous row, whose rows are missing
                          (this row's deltas then span them). Columns are only
                          ever added at the end; a can_stats.csv started by an
                          older logger keeps its old header and row width.
  OUTDIR/can_sync.csv     every event that changes how MKR time becomes host
                          time: logger start, new timeline, offset change, wall
                          clock step (and 'output writes failed N s', see
                          DISK below, and 'queue full N s ...', see HUNG DISK).
                          Each row carries host epoch, host
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
The reader also cuts each read at its last newline and hands over only complete
lines (the rest waits for the next read), so whatever happens to a hand-over
happens to whole lines.

── DISK ────────────────────────────────────────────────────────────────────────
A failing write, flush or fsync (the backup disk full, or gone) is counted, not
fatal: the logger keeps reading the port and writes again as soon as the disk
takes it, losing whole lines only, and then notes in mkr_console.txt for how
long writes failed and exactly how many bytes of can_raw.log (about how many
frames) never reached the disk. The first failure also goes to stderr (docker
logs), since the files may be what cannot be written. See class Out.

── HUNG DISK ───────────────────────────────────────────────────────────────────
A disk that hangs WITHOUT erroring (a USB disk mid-reset, a dying card: write or
fsync just does not return) stops the main thread, while the reader keeps
reading. What it reads waits in the hand-over queue (class Handover), which is
bounded: QUEUE_BYTES = 64 MiB, charged as each read's bytes plus EVENT_COST for
the Python objects around them (129 B measured with tracemalloc, CPython 3.10; a
queue filled to the bound with one-line reads measured 62.9 MB of RSS, under the
charge). The worst case is one 34-byte frame line per read (the MKR writes
packets of at most 63 bytes, so one 8-byte frame line each): 194 B charged per
frame, 0.24 MB/s at the Brio's ~1230 frames/s, so the bound holds 281 s (4.7
min) of a hung disk before anything is lost; reads carrying more lines last
longer. That outlasts a USB disk reset or a stalled card; a disk that never
comes back costs this much memory and no more (unbounded, it grew ~0.85 GB/h),
and one the kernel gives up on errors out, where DISK above takes over.

When a read does not fit, the reader drops it WHOLE and counts its lines by the
file each would have gone to: can_raw.log, can_stats.csv, mkr_console.txt;
lines and bytes as received, CR/LF included. Once dropping, it queues again only
after the writer has freed 1/8 of the bound, so a disk that hangs and recovers
costs ONE contiguous gap. It never blocks and never stops reading: the MKR never
sees a stalled host, and its ring stays as empty as ever. The count rides in the
queue as a record at the exact place of the gap, so:
  - can_stats.csv: dropped frame lines still count in rx_frames (they were
    received: the USB check against `streamed` stays exact) and also in
    rx_dropped; fs_dropped counts the FS rows missing just before a row.
  - an episode runs from the first drop until the queue is back under half the
    bound. It goes to stderr at its start (written by a thread of its own:
    stderr may be on the hung disk too), and at its end to mkr_console.txt
    ('queue full for N s ... dropped whole lines, as received: can_raw.log N
    frame lines (N bytes), ...', exact per file), can_sync.csv ('queue full N s
    ...') and stderr. The end note totals every episode and gives the queue's
    peak, so a run without drops still shows how close it came.
A dropped run hides whatever it held: an unreported MKR reset inside it is seen
(if at all) by the first line after the gap that does not fit the timeline, and
a dropped FS line is not checked. SIGTERM is handled only when the hung write
returns (docker stop then kills after its timeout); what was already written is
complete lines.

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
QUEUE_BYTES = 64 << 20   # bound on the reader -> writer hand-over (HUNG DISK): >= 4.7 min of this bus
EVENT_COST = 160         # charged per queued event on top of its bytes: 129 B measured, plus allocator slack

STATS_HEADER = ("host_epoch,host_mono,timeline,mkr_us,mkr_s,drained,streamed,ovf,ringdrop,nohost,rx_frames,"
                "rx_dropped,fs_dropped\n")

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


def count_lines(blob, acc):
    """Adds the complete lines of @blob (which ends in a newline) to @acc by the file Logger.handle would put
    each in: acc = [can_raw.log lines, bytes, can_stats.csv lines, bytes, mkr_console.txt lines, bytes].
    Bytes as received, newline (and CR) included, so the three byte counts add up to len(blob). A malformed
    F or FS line would have been a note in mkr_console.txt, so it counts there."""
    for line in blob.split(b"\n")[:-1]:
        n = len(line) + 1
        if line[-1:] == b"\r":
            line = line[:-1]
        if line[:2] == b"F " and parse_frame(line) is not None:
            k = 0
        elif line[:3] == b"FS " and parse_stats(line) is not None:
            k = 2
        else:
            k = 4
        acc[k] += 1
        acc[k + 1] += n


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
        self.stats_wide = True       # rows carry rx_dropped,fs_dropped (not if appending to an older header)
        if self.stats.start == 0:
            self.stats.write(STATS_HEADER)
        else:
            try:
                with open(os.path.join(outdir, "can_stats.csv"), "rb") as f:
                    self.stats_wide = f.readline(4096).decode("ascii", "replace") == STATS_HEADER
            except OSError:
                self.stats_wide = False
        # The hand-over queue's drops (HUNG DISK): [can_raw.log lines, bytes, can_stats.csv lines, bytes,
        # mkr_console.txt lines, bytes] in all, and [first monotonic] + the same six for the episode under way.
        self.handover = None         # main() sets it: the Handover, for its bound and peak in the notes
        self.drops = [0] * 6
        self.drop_ep = None
        self.drop_episodes = 0
        self.rx_dropped = 0          # frame lines dropped since the last FS line (counted in rx_since_fs too)
        self.fs_dropped = 0          # FS lines dropped since the last FS line written
        if self.sync.start == 0:
            self.sync.write("host_epoch,host_mono,wall_minus_mono,timeline,event,offset_s,mkr_s\n")
        if not self.stats_wide:
            self.note("can_stats.csv has an older header: rows keep its columns (no rx_dropped, fs_dropped)")
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
            self.stderr("%s - frames are dropped until it can write again" % self.out_err[1])

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

    def stderr(self, text):
        try:
            sys.stderr.write("mkr_stream_log: %s %s\n" % (time.strftime("%Y-%m-%dT%H:%M:%S%z"), text))
        except (OSError, ValueError):
            pass

    def dropped(self, ev):
        """A drop record from the reader: whole lines it could not queue (HUNG DISK), at their place in the
        stream. Dropped frame lines were received, so they count in rx_frames as well as in rx_dropped."""
        _, t0, _t1, *c = ev
        if self.drop_ep is None:
            self.drop_ep = [t0, 0, 0, 0, 0, 0, 0]
            self.drop_episodes += 1
        for i, v in enumerate(c):
            self.drop_ep[1 + i] += v
            self.drops[i] += v
        self.rx_since_fs += c[0]
        self.rx_dropped += c[0]
        self.fs_dropped += c[2]
        # A record after an open means the first read since it that completed a line was dropped: the
        # next one queued is fresh, not the MKR's bank leftover (module docstring).
        self.after_open = False

    def drained(self, mono, tail=""):
        """The episode is over (the queue is back under half its bound): report what it cost, exactly."""
        if self.drop_ep is None:
            return
        t0, rl, rb, sl, sb, cl, cb = self.drop_ep
        limit = self.handover.limit if self.handover is not None else QUEUE_BYTES
        text = ("queue full for %.1f s%s: the output did not keep up (a hung or slow disk); dropped whole lines, "
                "as received: can_raw.log %d frame lines (%d bytes), can_stats.csv %d FS lines (%d bytes), "
                "mkr_console.txt %d lines (%d bytes); queue bound %d bytes"
                % (mono - t0, tail, rl, rb, sl, sb, cl, cb, limit))
        self.note(text)
        self.sync_row("queue full %.1f s%s dropped %d frame lines (%d B) %d FS lines (%d B) %d console lines (%d B)"
                      % (mono - t0, tail, rl, rb, sl, sb, cl, cb), mono=mono)
        self.stderr(text)
        self.drop_ep = None

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
        self.rx_since_fs = self.rx_dropped = self.fs_dropped = 0
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
            row = "%.6f,%.6f,%d,%d,%.6f,%s,%d" % (host_mono + self.wall_minus_mono, host_mono, self.timeline_no,
                                                  st[0], mkr_s, ",".join(str(c) for c in st[1]), self.rx_since_fs)
            if self.stats_wide:
                row += ",%d,%d" % (self.rx_dropped, self.fs_dropped)
            self.stats.write(row + "\n")
            self.rx_dropped = self.fs_dropped = 0
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

    def overlong(self, n, head):
        self.malformed += 1
        self.note("overlong line dropped (%d bytes): %s" % (n, head[:120].decode("ascii", "replace")))

    def data(self, host_mono, chunk):
        """One read's bytes. Complete lines are handled; the read is one delivery sample.

        The reader hands over complete lines only (PortReader.received), so self.buf stays empty there;
        it holds a partial line only for a caller that feeds raw chunks."""
        self.buf += chunk
        if b"\n" not in chunk:
            if len(self.buf) > MAX_PARTIAL:
                self.overlong(len(self.buf), self.buf)
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

    def port_lost(self, why, partial=b""):
        """@partial: the line in progress the reader held (or self.buf, for a caller feeding raw chunks)."""
        self.note("port lost (%s)" % why)
        partial = partial or self.buf
        if partial:
            self.note("partial line at port loss: " + partial[:120].decode("ascii", "replace"))
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
        peak = self.handover.peak if self.handover is not None else 0
        limit = self.handover.limit if self.handover is not None else QUEUE_BYTES
        if self.drop_ep is not None:
            self.drained(time.monotonic(), " (still under way at exit)")
        self.note("logger end: %d frames, %d FS lines, %d console lines, %d malformed, %d timeline(s); "
                  "between FS lines %d frames streamed by the MKR, %d not received here"
                  % (self.frames, self.stat_lines, self.console_lines, self.malformed, self.timeline_no + 1,
                     self.usb_checked, self.usb_lost)
                  + ("; %d output write failures%s" % (self.out_errors, ", STILL FAILING" if self.out_err else "")
                     if self.out_errors else "")
                  + ("; queue full %d time(s): dropped %d frame lines (%d bytes), %d FS lines (%d bytes), "
                     "%d console lines (%d bytes)" % ((self.drop_episodes,) + tuple(self.drops))
                     if self.drop_episodes else "")
                  + "; queue peak %d of %d bytes" % (peak, limit))
        for f in (self.raw, self.stats, self.sync, self.con):
            f.flush()
            f.fsync()
            f.close()


class Handover:
    """The reader thread's queue to the main thread, bounded in bytes (HUNG DISK in the module docstring).

    Each event is charged EVENT_COST, plus its bytes for a "data" event. offer() queues all of its events or
    none, and never waits. force() ignores the bound, for events that must not be lost and that only the
    port's own changes produce: open, lost (with the drop record that has to precede it), a 'cannot open'
    note (once per distinct error), and the one "drained" per episode — a handful per reconnect, never the
    stream itself. get()/get_nowait() as queue.SimpleQueue's. @peak: the highest level so far."""

    def __init__(self, limit):
        self.limit = limit
        self.resume = limit - limit // 8     # once dropping, queue again only at or below this level
        self.items = collections.deque()
        self.cond = threading.Condition(threading.Lock())
        self.level = 0                       # charged bytes queued
        self.peak = 0

    @staticmethod
    def cost(ev):
        return EVENT_COST + len(ev[2]) if ev[0] == "data" else EVENT_COST

    def offer(self, *evs):
        c = sum(self.cost(e) for e in evs)
        with self.cond:
            if self.level + c > self.limit:
                return False
            self._push(evs, c)
        return True

    def force(self, *evs):
        with self.cond:
            self._push(evs, sum(self.cost(e) for e in evs))

    def _push(self, evs, c):
        self.items.extend(evs)
        self.level += c
        if self.level > self.peak:
            self.peak = self.level
        self.cond.notify()

    def get(self, timeout=None):
        with self.cond:
            if not self.items:
                self.cond.wait(timeout)
                if not self.items:
                    raise queue.Empty
            ev = self.items.popleft()
            self.level -= self.cost(ev)
        return ev

    def get_nowait(self):
        with self.cond:
            if not self.items:
                raise queue.Empty
            ev = self.items.popleft()
            self.level -= self.cost(ev)
        return ev


class StderrWriter(threading.Thread):
    """Writes the reader's stderr lines from a thread of its own: stderr can be a pipe nobody drains, or a
    file on the very disk that hung, and the reader must never wait on either."""

    def __init__(self):
        super().__init__(name="mkr-stderr", daemon=True)
        self.lines = queue.SimpleQueue()

    def say(self, text):
        self.lines.put("mkr_stream_log: %s %s\n" % (time.strftime("%Y-%m-%dT%H:%M:%S%z"), text))

    def run(self):
        while True:
            text = self.lines.get()
            try:
                sys.stderr.write(text)
                sys.stderr.flush()
            except (OSError, ValueError):
                pass


class PortReader(threading.Thread):
    """Owns the tty: finds, opens, reads, notices loss. Hands (kind, host_mono, ...) to the main thread.

    Its own thread so that nothing the main thread does — a write, an fsync — can pause the reads;
    see READING in the module docstring. host_mono is taken the moment os.read() returns. Hands over
    complete lines only, through a bounded queue: what does not fit is dropped whole and counted (HUNG
    DISK), and nothing here waits on the main thread or on any file."""

    def __init__(self, port, events, stop, say=None):
        super().__init__(name="mkr-port", daemon=True)
        self.port, self.events, self.stop = port, events, stop
        self.say = say or (lambda text: None)
        self.partial = b""       # the line in progress: what followed the last newline read
        self.dropping = None     # the drop run under way: count_lines' six counts, or None
        self.drop_span = None    # ... and [first, last] host_mono of the reads it dropped
        self.episode = False     # dropped since the queue was last under half its bound

    def drop_record(self):
        return ("drop", self.drop_span[0], self.drop_span[1]) + tuple(self.dropping)

    def hand(self, ev):
        """Queues @ev ("data" or "overlong"), or drops it whole and counts it into the drop run."""
        q = self.events
        if self.dropping is None:
            if q.offer(ev):
                return
            self.dropping = [0] * 6
            self.drop_span = [ev[1], ev[1]]
            if not self.episode:
                self.episode = True
                self.say("queue full (%d of %d bytes): the output is not keeping up (a hung or slow disk?); "
                         "whole lines are dropped and counted until it catches up" % (q.level, q.limit))
        elif q.level <= q.resume and q.offer(self.drop_record(), ev):
            self.dropping = None
            return
        if ev[0] == "data":
            count_lines(ev[2], self.dropping)
        else:                                    # an overlong fragment: its console note is what is lost
            self.dropping[4] += 1
            self.dropping[5] += ev[2]
        self.drop_span[1] = ev[1]

    def flush_drops(self, mono):
        """Queues the drop run's record if it fits (@mono None: whatever the level, ahead of a port event),
        and ends the episode once the queue is back under half its bound."""
        q = self.events
        if self.dropping is not None:
            if mono is None:
                q.force(self.drop_record())
            elif not (q.level <= q.resume and q.offer(self.drop_record())):
                return
            self.dropping = None
        if self.episode and mono is not None and q.level <= q.limit // 2:
            self.episode = False
            q.force(("drained", mono))

    def received(self, mono, chunk):
        buf = self.partial + chunk if self.partial else chunk
        cut = buf.rfind(b"\n") + 1
        if cut == 0:
            if len(buf) > MAX_PARTIAL:
                self.hand(("overlong", mono, len(buf), buf[:120]))
                buf = b""
            self.partial = buf
        else:
            self.partial = buf[cut:]
            self.hand(("data", mono, buf[:cut] if cut < len(buf) else buf))
        if self.episode:
            self.flush_drops(mono)

    def run(self):
        try:
            self.read_port()
        finally:
            if self.dropping is not None:        # what was dropped before the stop still gets counted
                self.flush_drops(None)

    def read_port(self):
        fd, lost_at, last_err = None, None, None
        force = self.events.force
        while not self.stop.is_set():
            if fd is None:
                if self.episode:
                    self.flush_drops(time.monotonic())
                dev = self.port or find_port()
                if dev is None or not os.path.exists(dev):
                    self.stop.wait(0.02)
                    continue
                try:
                    fd = open_port(dev)
                except (OSError, termios.error) as e:
                    msg = "cannot open %s: %s" % (dev, e)
                    if msg != last_err:          # once per distinct failure, not 5 times a second
                        force(("note", time.monotonic(), msg))
                        last_err = msg
                    self.stop.wait(0.2)
                    continue
                last_err = None
                self.partial = b""
                force(("open", time.monotonic(), dev, None if lost_at is None else time.monotonic() - lost_at))
                lost_at = None
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
            except (OSError, ValueError):
                r = [fd]                         # let the read report what is wrong with the fd
            if not r:
                if self.episode:
                    self.flush_drops(time.monotonic())
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
                self.received(mono, chunk)
                continue
            if self.dropping is not None:
                self.flush_drops(None)           # the gap goes before the loss, where it happened
            force(("lost", mono, why, self.partial))
            self.partial = b""
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
    elif kind == "drop":
        lg.dropped(ev)
    elif kind == "drained":
        lg.drained(ev[1])
    elif kind == "open":
        lg.port_open(ev[2], ev[3])
    elif kind == "lost":
        lg.port_lost(ev[2], ev[3])
    elif kind == "overlong":
        lg.overlong(ev[2], ev[3])
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
        events = Handover(QUEUE_BYTES)
        lg.handover = events
        stderr_writer = StderrWriter()
        stderr_writer.start()
        reader = PortReader(a.port, events, stop, stderr_writer.say)
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
