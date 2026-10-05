"""Shared pieces of the canstream tests: a synthetic MKR, a pty to play it into,
the logger under test, and parsers for what the logger writes.

The synthetic MKR follows the firmware's stream rules (CANRawStream.h), not
just its line format, because the logger's timing logic depends on them:
  - F lines leave in DRAIN order (the ring is FIFO), each stamped with the
    micros() at which its frame was read — so a frame line can arrive any time
    after that stamp, but never before an earlier frame's line;
  - an FS line is stamped as it is written and goes FIRST in its packet, so it
    can arrive ahead of frames drained before it;
  - writes are whole lines, at most 63 bytes, except where a test splits a line
    on purpose to model a host read landing mid-packet.
Every frame carries its TRUE host time (sim seconds since `base`), so the
logger's timestamps can be checked against truth, not against themselves.

The logger under test is mkr_stream_log.py beside this directory, or whatever
CANSTREAM_LOGGER names (used once to run this suite against the previous
version of the logger, to show which of its defects the tests catch).
"""
import csv
import json
import os
import random
import re
import resource
import select
import signal
import subprocess
import sys
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
LOGGER = os.environ.get("CANSTREAM_LOGGER", os.path.join(TOOLS, "mkr_stream_log.py"))
PATCHED = os.path.join(HERE, "patched_logger.py")
CAN_DECODE = os.path.join(TOOLS, "can_decode")
EMIT = os.path.join(TOOLS, "can_contract_emit")
WRAP = 1 << 32
PACKET = 63

# ─── line formats (an independent Python rendering of the contract) ───────────


class Frame:
    __slots__ = ("s", "us", "id", "ext", "rtr", "dlc", "data")

    def __init__(self, s, us, ident, ext, rtr, dlc, data):
        self.s, self.us, self.id, self.ext, self.rtr, self.dlc, self.data = s, us, ident, ext, rtr, dlc, data

    def line(self):
        idt = ("%08X" % self.id) if self.ext else ("%03X" % self.id)
        if self.rtr:
            return "F %08X %s R %d\n" % (self.us, idt, self.dlc)
        return "F %08X %s %d%s\n" % (self.us, idt, self.dlc, (" " + self.data) if self.dlc else "")

    def key(self):
        return (self.id, self.ext, self.rtr, self.dlc, self.data)


def fs_line(us, counters):
    return "FS %08X %s\n" % (us, " ".join(str(c) for c in counters))


MAP_IDS = (0x158, 0x17C, 0x191, 0x1AB, 0x1D0, 0x294, 0x100, 0x2A0, 0x3C0, 0x055)


def random_frame(rng, s, us):
    """A frame of the kind this bus carries, plus the shapes the parser must not get wrong."""
    r = rng.random()
    if r < 0.70:
        ident, ext, rtr, dlc = rng.choice(MAP_IDS), False, False, 8
    elif r < 0.82:
        ident, ext, rtr, dlc = rng.randrange(0x800), False, False, rng.randrange(9)
    elif r < 0.90:
        ident, ext, rtr, dlc = rng.randrange(1 << 29), True, False, rng.randrange(9)
    elif r < 0.95:
        ident, ext, rtr, dlc = rng.randrange(0x800), False, True, rng.randrange(9)
    else:
        ident, ext, rtr, dlc = rng.randrange(1 << 29), True, True, rng.randrange(9)
    data = "" if rtr else "".join("%02X" % rng.randrange(256) for _ in range(dlc))
    return Frame(s, us, ident, ext, rtr, dlc, data)


# ─── a schedule: what the synthetic MKR writes, when, and the truth behind it ──


class Schedule:
    """Writes in order: (sim_time, bytes, action). `action` is None, or 'hangup' (the port is lost and
    a new one appears: an MKR reset as the host sees it)."""

    def __init__(self):
        self.writes = []
        self.frames = []       # truth, in stream order
        self.fs = []           # truth: (s, us, counters, timeline)
        self.console = []      # console lines, in order
        self.malformed = 0
        self.timeline_of = []  # per truth frame: synthetic timeline index

    def add(self, t, data, action=None):
        self.writes.append((t, data if isinstance(data, bytes) else data.encode(), action))


class Mkr:
    """One synthetic boot: micros() = (start_us + (s - boot_s) * 1e6 * (1 + ppm/1e6)) mod 2^32 — a crystal
    running ppm fast (or slow, negative) against the host's clock."""

    def __init__(self, start_us, boot_s=0.0, ppm=0.0):
        self.start_us, self.boot_s, self.rate = start_us, boot_s, 1.0 + ppm * 1e-6

    def us(self, s):
        return (self.start_us + int(round((s - self.boot_s) * 1e6 * self.rate))) % WRAP


def build(seed, duration, rate=600.0, start_us=0, events=(), burst=None, stall=None, wrap_hold=None,
          console_rate=2.0, split_frac=0.03, malformed_at=(), fs_period=1.0, ppm=0.0):
    """Builds a schedule.

    events: (s, kind, start_us) MKR resets: kind 'ahead' / 'back' (no banner), 'banner' (banner then the
            new boot), 'hangup' (port lost, banner on the new port).
    burst:  (s0, s1): frames drained in [s0, s1) are delivered together at s1 (late delivery).
    stall:  (s0, s1): the host reads nothing in [s0, s1); the ring keeps the first 1024 frames, drops the
            rest, and at s1 an FS line leads the stale frames (6 s older than it).
    wrap_hold: frames drained within this many seconds before a micros() wrap wait 0.12 s, so the FS just
            after the wrap overtakes them."""
    rng = random.Random(seed)
    sch = Schedule()
    resets = sorted(events)
    boots = [(0.0, Mkr(start_us, 0.0, ppm), None)] + [(s, Mkr(su, s, ppm), kind) for s, kind, su in resets]

    def boot_at(s):
        k = 0
        for i, (bs, _, _) in enumerate(boots):
            if s >= bs:
                k = i
        return k

    # Frames: (drain s, delivery s); FS: (s, s). Each reset leaves 1.5 s of boot silence.
    items = []
    s = 0.05
    last_delivery = 0.0
    counters = [0, 0, 0, 0, 0]
    ring = []
    def cutoff(k):
        """Delivery deadline for boot k: its frames still in the ring at the next reset are lost, and
        nothing is written in the 0.3 s before a hangup (closing a pty master discards unread input)."""
        if k + 1 >= len(boots):
            return float("inf")
        return boots[k + 1][0] - (0.3 if boots[k + 1][2] == "hangup" else 0.0)

    while s < duration:
        k = boot_at(s)
        if k > 0 and s < boots[k][0] + 1.5:
            s = boots[k][0] + 1.5
            continue
        mkr = boots[k][1]
        f = random_frame(rng, s, mkr.us(s))
        lat = rng.expovariate(1 / 0.003)
        if rng.random() < 0.05:
            lat += rng.uniform(0.03, 0.25)          # a loop() pass that held the ring
        d = s + lat
        if wrap_hold is not None:
            u = f.us
            if WRAP - u <= wrap_hold * 1e6:
                d = max(d, s + 0.12)
        if burst and burst[0] <= s < burst[1]:
            d = burst[1]
        if stall and stall[0] <= s < stall[1]:
            if len(ring) < 1024:
                ring.append(f)
                d = stall[1]
            else:
                counters[3] += 1                    # ringdrop: never delivered, not in the truth
                s += rng.expovariate(rate)
                continue
        d = max(d, last_delivery)                   # FIFO
        if d >= cutoff(k):                          # lost with the MKR's RAM at its reset
            s += rng.expovariate(rate)
            continue
        last_delivery = d
        items.append((d, 1, f, k))
        s += rng.expovariate(rate)
    # FS lines once a second of each boot's uptime; none while stalled (the bank is armed) — the one
    # owed is written at the end of the stall, with its fresh stamp.
    t = fs_period
    while t < duration:
        k = boot_at(t)
        if not (k > 0 and t < boots[k][0] + 1.5) and not (stall and stall[0] <= t < stall[1]) and t < cutoff(k):
            items.append((t, 0, t, k))
        t += fs_period
    if stall:
        items.append((stall[1], 0, stall[1], boot_at(stall[1])))
    for bs, mkr, kind in boots[1:]:
        items.append((bs, -1, kind, None))           # the reset itself sorts first at its instant
    items.sort(key=lambda it: (it[0], it[1]))

    pending_packet = []                              # lines due at the same instant go out packed

    def flush_packet(at):
        buf = b""
        for line in pending_packet:
            b = line.encode()
            if len(buf) + len(b) > PACKET:
                sch.add(at, buf)
                buf = b""
            buf += b
        if buf:
            sch.add(at, buf)
        pending_packet.clear()

    last_t = None
    for d, kind, obj, k in items:
        if last_t is not None and d != last_t:
            flush_packet(last_t)
        last_t = d
        if kind == -1:
            flush_packet(d)
            if obj == "hangup":
                sch.add(d, b"", "hangup")
            if obj in ("banner", "hangup"):
                for text in ("MKR Zero telemetry master", "BOOT: reset cause 0x40 - external reset"):
                    sch.add(d + 0.5, text + "\r\n")
                    sch.console.append(text)
            continue
        if kind == 0:
            mkr = boots[k][1]
            counters = [counters[0] + 600, counters[1] + 590, counters[2] + (k + 1), counters[3], counters[4] + 1]
            line = fs_line(mkr.us(obj), counters)
            sch.fs.append((obj, mkr.us(obj), list(counters), k))
            pending_packet.append(line)             # first at its instant: FS sorts before frames
            continue
        f = obj
        sch.frames.append(f)
        sch.timeline_of.append(k)
        line = f.line()
        if not pending_packet and rng.random() < split_frac:
            cut = rng.randrange(1, len(line) - 1)   # a read boundary inside a line
            sch.add(d, line[:cut])
            sch.add(d + 0.004, line[cut:])
            continue
        pending_packet.append(line)
    if last_t is not None:
        flush_packet(last_t)

    # Console lines between packets (whole lines, as the firmware prints them), and malformed ones.
    extra = []
    t = 0.3
    texts = ["MKR status can=discover rx=%d ovf=0 drop=0", "Fault: none (starts with F, not 'F ')",
             "FSM idle (starts with FS, not 'FS ')", "", "CAN: mode -> DISCOVER", "GPS: up sats=%d"]
    hangups = [bs for bs, _, kind in boots[1:] if kind == "hangup"]
    while t < duration:
        if any(h - 0.4 <= t <= h + 0.1 for h in hangups):
            t += 0.5
            continue
        text = rng.choice(texts)
        if "%d" in text:
            text = text % rng.randrange(1000)
        extra.append((t, text + ("\r\n" if rng.random() < 0.5 else "\n"), text))
        t += rng.expovariate(console_rate)
    bad = ["F 1234 17C 8 0011223344556677", "F 0000ABCD 17C 9 001122334455667788", "F 0000ABCD 17C 2 001",
           "F 0000ABCD 17C 0 ", "F 0000abcd 17C 1 00", "F 0000ABCD 1FFFFFFFF 1 00", "F 0000ABCD 800 1 00",
           "F 0000ABCD 20000000 1 00", "F 0000ABCD 17C R 9", "FS 0000ABCD 1 2 3",
           "FS 0000ABCD 1 2 3 4 99999999999", "FS 0000ABCD 1 2 3 4 4294967296", "F  0000ABCD 17C 1 00",
           "F 0000ABCD 17C 1 00 ", "FS 0000ABCD 1 2 3 4 5 6"]
    for i, t in enumerate(malformed_at):
        extra.append((t, bad[i % len(bad)] + "\n", None))
        sch.malformed += 1
    # Merge the extras in as separate writes between packets, keeping both lists' order.
    merged = []
    ei = 0
    extra.sort(key=lambda e: e[0])
    for w in sch.writes:
        while ei < len(extra) and extra[ei][0] <= w[0] and not _mid_line(merged):
            merged.append((extra[ei][0], extra[ei][1].encode(), None))
            if extra[ei][2] is not None:
                sch.console.append(extra[ei][2])
            ei += 1
        merged.append(w)
    for e in extra[ei:]:
        merged.append((max(e[0], merged[-1][0] if merged else 0.0), e[1].encode(), None))
    # Console/banner order: rebuild from the merged writes so the expectation follows the stream exactly.
    sch.writes = merged
    sch.console = _console_lines(merged)
    return sch


def _mid_line(writes):
    """True if the last write left a line unfinished (a console line must not land inside it)."""
    for t, data, action in reversed(writes):
        if data:
            return not data.endswith(b"\n")
    return False


def _console_lines(writes):
    """Every complete line the logger should file as console, in stream order."""
    buf = b""
    out = []
    for _, data, action in writes:
        if action == "hangup":
            buf = b""
            continue
        buf += data
        *lines, buf = buf.split(b"\n")
        for raw in lines:
            line = raw.rstrip(b"\r").decode()
            if line.startswith("F ") or line.startswith("FS "):
                continue
            out.append(line)
    return out


# ─── the pty and the logger ───────────────────────────────────────────────────


class Pty:
    """A pty whose slave is reached through a symlink, so a 'reset' can replace the device."""

    def __init__(self, link):
        self.link = link
        self.master = None
        self.blocked = 0.0          # seconds writes spent waiting: the reader had stopped reading
        self.worst_block = 0.0
        self.fresh()

    def fresh(self):
        m, s = os.openpty()
        tty.setraw(s)               # nothing echoes back before the logger sets its own modes
        name = os.ttyname(s)
        os.close(s)
        os.set_blocking(m, False)
        tmp = self.link + ".tmp"
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(name, tmp)
        os.replace(tmp, self.link)
        old, self.master = self.master, m
        return old

    def hangup(self):
        """The MKR reset: its USB device goes away (the logger's read fails) and a new one appears."""
        old = self.fresh()
        os.close(old)

    def write(self, data, timeout=30.0):
        """Writes all of @data, waiting while the reader is behind — but not for ever: a reader that has
        died or stopped reading for @timeout s fails the test instead of hanging it."""
        view = memoryview(data)
        waited = 0.0
        while view:
            try:
                n = os.write(self.master, view)
                view = view[n:]
            except BlockingIOError:
                t = time.monotonic()
                select.select([], [self.master], [], max(0.0, timeout - waited))
                dt = time.monotonic() - t
                self.blocked += dt
                self.worst_block = max(self.worst_block, dt)
                waited += dt
                if waited >= timeout:
                    raise AssertionError("the reader took nothing for %.0f s: dead or stopped" % waited)

    def close(self):
        if self.master is not None:
            os.close(self.master)
            self.master = None


class Logger:
    """The logger under test, always on a FIXED wall clock (patched_logger.py: time.time() = monotonic + K),
    so its epoch timestamps convert back to monotonic exactly, whatever the host's NTP client does."""

    def __init__(self, outdir, port, seconds=600, patch=None, fsize=None):
        """@fsize: RLIMIT_FSIZE soft limit for the logger's files (bytes) — writes past it fail with EFBIG,
        a full disk without needing one; raise it again with resource.prlimit(self.p.pid, ...)."""
        self.outdir = outdir
        self.K = 1_790_000_000.0 - time.monotonic()
        patch = dict(patch or {}, fixed_wall=self.K)
        cmd = [sys.executable, PATCHED, json.dumps(patch), LOGGER, str(seconds), outdir, "--port", port]
        self.err = open(os.path.join(outdir + ".stderr"), "w")
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        limit = None
        if fsize is not None:
            limit = lambda: resource.setrlimit(resource.RLIMIT_FSIZE, (fsize, resource.RLIM_INFINITY))
        self.p = subprocess.Popen(cmd, stdout=self.err, stderr=subprocess.STDOUT, env=env, preexec_fn=limit)

    def console_text(self):
        try:
            with open(os.path.join(self.outdir, "mkr_console.txt")) as f:
                return f.read()
        except OSError:
            return ""

    def wait_open(self, count=1, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.console_text().count("port open") >= count:
                return True
            if self.p.poll() is not None:
                return False
            time.sleep(0.01)
        return False

    def cpu_seconds(self):
        with open("/proc/%d/stat" % self.p.pid) as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")

    def stop(self, timeout=20.0):
        if self.p.poll() is None:
            self.p.send_signal(signal.SIGTERM)
        try:
            rc = self.p.wait(timeout)
        except subprocess.TimeoutExpired:
            self.p.kill()
            rc = self.p.wait()
        self.err.close()
        return rc


def play(sch, pty, logger, speed=1.0):
    """Writes the schedule in real time. Returns the host monotonic time of sim t = 0."""
    base = time.monotonic() + 0.05
    opens = 1
    for t, data, action in sch.writes:
        due = base + t / speed
        now = time.monotonic()
        if due > now:
            time.sleep(due - now)
        if action == "hangup":
            pty.hangup()
            opens += 1
            if not logger.wait_open(opens):
                raise AssertionError("logger did not reopen the port after the hangup")
            continue
        pty.write(data)
    return base


# ─── reading what the logger wrote ────────────────────────────────────────────

CANDUMP_RE = re.compile(r"\((\d+)\.(\d{6})\) (\S+) ([0-9A-F]{3}|[0-9A-F]{8})#(R[0-8]?|(?:[0-9A-F]{2}){0,8})$")


def read_candump(path):
    """[(t_us, iface, id, ext, rtr, dlc, data_hex)]; raises on any line that is not exact candump -l."""
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            m = CANDUMP_RE.match(line.rstrip("\n"))
            if m is None:
                raise AssertionError("can_raw.log line %d is not candump: %r" % (n, line))
            sec, frac, iface, idt, body = m.groups()
            ext = len(idt) == 8
            if body.startswith("R"):
                rtr, dlc, data = True, int(body[1:] or "0"), ""
            else:
                rtr, dlc, data = False, len(body) // 2, body
            out.append((int(sec) * 1000000 + int(frac), iface, int(idt, 16), ext, rtr, dlc, data))
    return out


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def console_body(text):
    """mkr_console.txt minus the logger's own '#' lines, with the time prefix removed."""
    out = []
    for line in text.splitlines():
        if line.startswith("#"):
            continue
        out.append(line.split(" ", 1)[1] if " " in line else "")
    return out
