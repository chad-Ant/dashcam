"""Bench MKR USB protocol, integrity checking and bounded clock mapping.

Production CAN already owns this port. SerialBridge is ONLY for GRF250Bridge
bench firmware. Production integration must multiplex in the existing reader.
"""
import binascii
from dataclasses import dataclass
import math
import re
import time
from .core import RangeSample


def crc_line(body):
    return body + b" %04X\n" % binascii.crc_hqx(body, 0)


def decode_line(line):
    if len(line) > 63 or not line.endswith(b"\n"):
        raise ValueError("oversize or incomplete bridge line")
    body, check = line[:-1].rsplit(b" ", 1)
    if not re.fullmatch(rb"[0-9A-F]{4}", check) or int(check, 16) != binascii.crc_hqx(body, 0):
        raise ValueError("bridge CRC")
    fields = body.split(b" ")
    count = {b"L1": 7, b"T1": 3, b"S1": 7}.get(fields[0])
    if count is None or len(fields) != count or any(not re.fullmatch(rb"[0-9A-F]{8}", v) for v in fields[1:]):
        raise ValueError("bridge schema")
    return fields[0], tuple(int(v,16) for v in fields[1:])


def signed(u):
    return u if u < 0x80000000 else u - 0x100000000


@dataclass(frozen=True)
class LatencyBounds:
    """Measured acquisition-to-MKR-receipt bounds, including sampling and UART."""
    minimum_s: float
    maximum_s: float

    def __post_init__(self):
        if not (math.isfinite(self.minimum_s) and math.isfinite(self.maximum_s)
                and 0 <= self.minimum_s <= self.maximum_s <= .5):
            raise ValueError("invalid latency envelope")


class ClockMapper:
    def __init__(self):
        self.anchor = None

    def observe(self, tick_us, sent_s, received_s):
        if (not 0 <= tick_us <= 0xffffffff or not math.isfinite(sent_s)
                or not math.isfinite(received_s) or not 0 <= received_s-sent_s <= .05):
            return False
        self.anchor = (tick_us, (sent_s+received_s)/2, (received_s-sent_s)/2)
        return True

    def map(self, tick_us, received_s):
        if self.anchor is None:
            return None
        tick, midpoint, error = self.anchor
        delta = ((tick_us-tick+0x80000000) & 0xffffffff)-0x80000000
        stamp = midpoint + delta*1e-6
        if not 0 <= received_s-midpoint <= 5 or abs(delta) > 5_000_000 or stamp-error > received_s:
            return None
        # Clock drift allowance 1000 ppm, deliberately pessimistic until measured.
        return stamp, error + abs(delta)*1e-9 + .001


class BridgeDecoder:
    def __init__(self, latency=None):
        self.clock = ClockMapper()
        self.latency = latency
        self.previous = None
        self.errors = 0
        self.generation = 0

    def sample(self, values, received_s):
        sequence, tick, first, first_db, last, last_db = values
        if self.previous is not None:
            delta = (sequence-self.previous) & 0xffffffff
            if delta == 0:
                return None
            if delta >= 0x80000000:
                # Reboot/replay: invalidate mapping. Caller must reset target/filter
                # on reconnect; no attempt to disguise a restart as micros wrap.
                self.clock.anchor = None
                self.generation += 1
        self.previous = sequence
        mapped = self.clock.map(tick, received_s)
        verified = mapped is not None and self.latency is not None
        stamp, uncertainty = mapped if mapped else (received_s, 1.)
        if verified:
            stamp -= (self.latency.minimum_s+self.latency.maximum_s)/2
            uncertainty += (self.latency.maximum_s-self.latency.minimum_s)/2
        return RangeSample(sequence, stamp, uncertainty, signed(first)/100., signed(last)/100.,
                           signed(first_db), signed(last_db), verified)


class SerialBridge:
    """Linux nonblocking bounded bench transport; poll from main thread only.

    Clock sync must not depend on the caller's poll rate: T1 is stamped when
    read, so poll() blocks up to sync_wait_s (once per second) for the reply.
    """
    def __init__(self, path, *, bench_firmware=False, latency=None, sync_wait_s=.02):
        if not bench_firmware:
            raise ValueError("requires dedicated GRF250Bridge bench firmware; never steal production USB")
        if not (isinstance(sync_wait_s, float) and 0 < sync_wait_s <= .05):
            raise ValueError("sync_wait_s must be within (0, 0.05] s, the ClockMapper RTT limit")
        import os
        import fcntl
        import select
        import termios
        self.os = os
        self.select = select.select
        self.sync_wait_s = sync_wait_s
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL) # prevents later opens, not an existing reader
            attributes = termios.tcgetattr(self.fd)
            attributes[0] = attributes[1] = attributes[3] = 0
            attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attributes[4] = attributes[5] = termios.B115200
            attributes[6][termios.VMIN] = 0; attributes[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except Exception:
            os.close(self.fd)
            raise
        self.decoder = BridgeDecoder(latency)
        self.buffer = bytearray()
        self.discarding = False
        self.pending = None
        self.next_sync = 0.
        self.nonce = 0
        self.status = None

    def close(self):
        if self.fd is not None:
            self.os.close(self.fd); self.fd = None

    def poll(self):
        samples = []
        # Drain the backlog first so the sync wait below sees fresh bytes only.
        for _ in range(8): # <= 4096 bytes
            if not self._read(samples):
                break
        now = time.monotonic()
        if now >= self.next_sync:
            self.next_sync = now+1.
            self.nonce = (self.nonce+1) & 0xffffffff
            message = b"Q1 %08X\n" % self.nonce
            sent = time.monotonic()
            try:
                n = self.os.write(self.fd, message)
            except BlockingIOError:
                n = 0
            # A late T1 is still matched by a later poll(), with its honest RTT.
            self.pending = (self.nonce, sent) if n == len(message) else None
            for _ in range(8): # bounded: deadline AND read count
                remaining = sent+self.sync_wait_s-time.monotonic()
                if self.pending is None or remaining <= 0 or not self.select([self.fd], [], [], remaining)[0]:
                    break
                if not self._read(samples):
                    break
        return samples

    def _read(self, samples):
        try:
            data = self.os.read(self.fd, 512)
        except BlockingIOError:
            return False
        if not data:
            return False
        received = time.monotonic()
        for byte in data:
            if byte == 10:
                if not self.discarding:
                    try:
                        self._line(bytes(self.buffer)+b"\n", received, samples)
                    except ValueError:
                        self.decoder.errors += 1
                self.buffer.clear(); self.discarding = False
            elif len(self.buffer) < 62 and not self.discarding:
                self.buffer.append(byte)
            else:
                self.discarding = True
        return True

    def _line(self, line, received, samples):
        kind, values = decode_line(line)
        if kind == b"T1" and self.pending and values[0] == self.pending[0]:
            self.decoder.clock.observe(values[1], self.pending[1], received)
            self.pending = None
        elif kind == b"L1":
            sample = self.decoder.sample(values, received)
            if sample is not None:
                samples.append(sample)
        elif kind == b"S1":
            self.status = dict(zip(("micros", "ready", "firmware", "faults", "usb_drops", "parse_errors"), values))
            self.status["received_s"] = received
