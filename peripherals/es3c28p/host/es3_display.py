#!/usr/bin/env python3
"""Jetson/Linux client for the ES3C28P native USB display channel (stdlib only).

One outstanding request per client; serialize calls from application threads.
Commands are never automatically replayed after an uncertain write or timeout.
Also mirrors the E3 29 video record codec (protocol v1) for bench diagnostics,
and `play` streams ready-made JPEG files over it (bench playback, see
`play --help`); production video goes through the C++ link (lib/libes3display).
"""
import argparse
import binascii
import collections
import errno
import fcntl
import json
import math
import os
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty
import zlib

MAGIC = b"\xe3\x28"
VERSION = 1
MAX_PAYLOAD = 128
TEXT_BYTES = 24
HELLO, PING, STATUS, PAGE, ALERT, STOP_AUDIO = range(1, 7)
CAPABILITIES, START_VIDEO, STOP_VIDEO = 7, 8, 9
REPLY, ERROR = 0x80, 0xff
ERR_BAD_COMMAND, ERR_BAD_PAYLOAD, ERR_NOT_READY, ERR_NO_SESSION = range(1, 5)
ERROR_NAMES = {ERR_BAD_COMMAND: "unknown command", ERR_BAD_PAYLOAD: "bad payload",
               ERR_NOT_READY: "not ready / alert.wav unavailable", ERR_NO_SESSION: "session expired; reopen"}
VIDEO_BACKLOG = 64  # device->host video records kept for a reader; oldest dropped


def encode(sequence, kind, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 128 bytes")
    body = bytes((VERSION, sequence, kind, len(payload))) + payload
    return MAGIC + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_byte = None

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.last_byte is not None and now - self.last_byte > 0.250:
            self.buffer.clear()
        self.last_byte = now
        frames = []
        for byte in data:
            self.buffer.append(byte)
            while len(self.buffer) >= 2:
                if self.buffer[:2] != MAGIC:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 6:
                    break
                length = self.buffer[5]
                if self.buffer[2] != VERSION or length > MAX_PAYLOAD:
                    del self.buffer[0]
                    continue
                total = length + 8
                if len(self.buffer) < total:
                    break
                expected = struct.unpack_from("<H", self.buffer, total - 2)[0]
                if binascii.crc_hqx(self.buffer[2:total - 2], 0xffff) != expected:
                    del self.buffer[0]
                    continue
                frames.append((self.buffer[3], self.buffer[4], bytes(self.buffer[6:total - 2])))
                del self.buffer[:total]
        return frames


def page_payload(index, title, lines):
    if not 0 <= index < 5 or len(lines) != 4:
        raise ValueError("page index must be 0..4 with exactly four lines")
    fields = []
    for field in [title, *lines]:
        value = field.encode("ascii")
        if len(value) >= TEXT_BYTES or any(byte < 32 or byte > 126 for byte in value):
            raise ValueError("page fields must be printable ASCII, at most 23 characters")
        fields.append(value.ljust(TEXT_BYTES, b"\0"))
    return bytes((index,)) + b"".join(fields)


# --- E3 29 video records, protocol v1 (frozen 2026-10-13) --------------------
# Mirrors lib/es3UsbLink/Es3VideoProtocol.h (the C++ header is the reference);
# tests/video_golden_vectors.txt freezes the bytes for both implementations.
#   E3 29 | version u8 | type u8 | payload_length u16 | session_token u32 |
#   frame_id u32 | payload | crc16 u16      (little-endian, CRC as for E3 28)
VIDEO_MAGIC = b"\xe3\x29"
VIDEO_VERSION = 1
VIDEO_HEADER = 14
VIDEO_OVERHEAD = 16
CHUNK_OFFSET_BYTES = 4
MAX_CHUNK_DATA = 4096          # hard ceiling of CHUNK data bytes
DEFAULT_CHUNK_DATA = 1024      # initial negotiated chunk size
MAX_RECORD_PAYLOAD = CHUNK_OFFSET_BYTES + MAX_CHUNK_DATA
MAX_JPEG_BYTES = 65536
MAX_CREDITS = 2
MAX_TARGET_FPS = 30
MAX_TIMEOUT_MS = 10000
CANVAS_WIDTH, CANVAS_HEIGHT = 320, 240
REC_START, REC_CHUNK, REC_END, REC_ABORT = 0x01, 0x02, 0x03, 0x04
REC_RESULT, REC_PRESENTED, REC_CREDIT, REC_VISIBILITY = 0x81, 0x82, 0x83, 0x84
REASONS = ("Ok", "BadLength", "BadOffset", "BadCrc", "StaleSession", "NoCredit", "TooLarge",
           "BadDimensions", "AssemblyTimeout", "DecodeFailed", "Superseded", "Hidden", "Busy",
           "BadFrameId", "Malformed", "Cancelled", "Internal")
REASON = {name: code for code, name in enumerate(REASONS)}
DEMAND_ACTIVE, DEMAND_HIDDEN, DEMAND_CALIBRATING = 0, 1, 2
CODEC_BASELINE_JPEG = 0
CODEC_MASK_BASELINE_JPEG = 1 << CODEC_BASELINE_JPEG
_FIXED_PAYLOAD = {REC_START: 16, REC_END: 4, REC_ABORT: 1, REC_RESULT: 1,
                  REC_PRESENTED: 8, REC_CREDIT: 4, REC_VISIBILITY: 1}
CAPABILITIES_FORMAT = "<BBHHHHIHBB"       # 18 bytes
CAPABILITIES_FIELDS = ("video_protocol_version", "codec_mask", "canvas_width", "canvas_height",
                       "max_width", "max_height", "max_jpeg_bytes", "max_chunk_bytes",
                       "max_credits", "flags")
START_VIDEO_FORMAT = "<BBHHHHIHBHHH"      # 23 bytes
START_VIDEO_FIELDS = ("video_protocol_version", "codec", "viewport_x", "viewport_y",
                      "viewport_w", "viewport_h", "max_jpeg_bytes", "chunk_bytes", "target_fps",
                      "assembly_timeout_ms", "stale_after_ms", "placeholder_after_ms")

VideoRecord = collections.namedtuple("VideoRecord", "type token frame_id payload")


def crc32(data, value=0):
    """CRC-32/ISO-HDLC with zlib-style chaining (whole-JPEG checksum in END)."""
    return zlib.crc32(data, value) & 0xffffffff


if crc32(b"123456789") != 0xCBF43926:
    raise ImportError("zlib.crc32 is not CRC-32/ISO-HDLC")


class VideoError(ValueError):
    """A video record or payload failed validation; .reason is the Reason code."""

    def __init__(self, reason):
        super().__init__(REASONS[reason] if 0 <= reason < len(REASONS) else "Unknown")
        self.reason = reason


def names_frame(kind):
    return kind not in (REC_CREDIT, REC_VISIBILITY)


def check_framing(kind, length, token, frame_id, max_chunk=MAX_CHUNK_DATA):
    """Header rules every record obeys; mirror of es3video::checkFraming()."""
    if kind == REC_CHUNK:
        if length <= CHUNK_OFFSET_BYTES:
            return REASON["BadLength"]
        if length > CHUNK_OFFSET_BYTES + min(max_chunk, MAX_CHUNK_DATA):
            return REASON["TooLarge"]
    elif kind not in _FIXED_PAYLOAD:
        return REASON["Malformed"]
    elif length != _FIXED_PAYLOAD[kind]:
        return REASON["BadLength"]
    if token == 0:
        return REASON["StaleSession"]
    if names_frame(kind) != (frame_id != 0):
        return REASON["BadFrameId"]
    return REASON["Ok"]


def _record_fields(kind, payload):
    """(reason, fields) of a framing-valid record; mirror of the C++ typed decoders."""
    ok = REASON["Ok"]
    if kind == REC_START:
        width, height, jpeg_bytes, timestamp = struct.unpack("<HHIQ", payload)
        fields = dict(width=width, height=height, jpeg_bytes=jpeg_bytes, source_timestamp_us=timestamp)
        if not width or not height:
            return REASON["BadDimensions"], fields
        if not jpeg_bytes:
            return REASON["BadLength"], fields
        return (REASON["TooLarge"] if jpeg_bytes > MAX_JPEG_BYTES else ok), fields
    if kind == REC_CHUNK:
        offset = struct.unpack_from("<I", payload)[0]
        fields = dict(offset=offset, data=bytes(payload[CHUNK_OFFSET_BYTES:]))
        return (REASON["BadOffset"] if offset > MAX_JPEG_BYTES - len(fields["data"]) else ok), fields
    if kind == REC_END:
        return ok, dict(crc32=struct.unpack("<I", payload)[0])
    if kind in (REC_ABORT, REC_RESULT):
        valid = payload[0] < len(REASONS) and not (kind == REC_ABORT and payload[0] == ok)
        return (ok if valid else REASON["Malformed"]), dict(reason=payload[0])
    if kind == REC_PRESENTED:
        decode_us, blit_us = struct.unpack("<II", payload)
        return ok, dict(decode_us=decode_us, blit_us=blit_us)
    if kind == REC_CREDIT:
        return ok, dict(cumulative_grant=struct.unpack("<I", payload)[0])
    return (ok if payload[0] <= DEMAND_CALIBRATING else REASON["Malformed"]), dict(demand=payload[0])


def check_record(kind, token, frame_id, payload):
    """Full validation (framing + fields); mirror of es3video::checkRecord()."""
    if len(payload) > MAX_RECORD_PAYLOAD:
        return REASON["TooLarge"] if kind == REC_CHUNK else REASON["BadLength"]
    reason = check_framing(kind, len(payload), token, frame_id)
    return reason if reason else _record_fields(kind, payload)[0]


def encode_record(kind, token, frame_id, payload=b""):
    reason = check_record(kind, token, frame_id, payload)
    if reason:
        raise VideoError(reason)
    body = struct.pack("<BBHII", VIDEO_VERSION, kind, len(payload), token, frame_id) + payload
    return VIDEO_MAGIC + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


def encode_start(token, frame_id, width, height, jpeg_bytes, source_timestamp_us):
    return encode_record(REC_START, token, frame_id,
                         struct.pack("<HHIQ", width, height, jpeg_bytes, source_timestamp_us))


def encode_chunk(token, frame_id, offset, data):
    return encode_record(REC_CHUNK, token, frame_id, struct.pack("<I", offset) + bytes(data))


def encode_end(token, frame_id, jpeg_crc32):
    return encode_record(REC_END, token, frame_id, struct.pack("<I", jpeg_crc32))


def encode_abort(token, frame_id, reason):
    return encode_record(REC_ABORT, token, frame_id, bytes((reason,)))


def encode_result(token, frame_id, reason):
    return encode_record(REC_RESULT, token, frame_id, bytes((reason,)))


def encode_presented(token, frame_id, decode_us, blit_us):
    return encode_record(REC_PRESENTED, token, frame_id, struct.pack("<II", decode_us, blit_us))


def encode_credit(token, cumulative_grant):
    return encode_record(REC_CREDIT, token, 0, struct.pack("<I", cumulative_grant))


def encode_visibility(token, demand):
    return encode_record(REC_VISIBILITY, token, 0, bytes((demand,)))


def decode_record(wire, max_chunk=MAX_CHUNK_DATA):
    """One complete record -> VideoRecord; framing rules as MuxParser (VideoError)."""
    wire = bytes(wire)
    if len(wire) < VIDEO_OVERHEAD:
        raise VideoError(REASON["BadLength"])
    if wire[:2] != VIDEO_MAGIC or wire[2] != VIDEO_VERSION:
        raise VideoError(REASON["Malformed"])
    _, kind, length, token, frame_id = struct.unpack_from("<BBHII", wire, 2)
    reason = check_framing(kind, length, token, frame_id, max_chunk)
    if reason:
        raise VideoError(reason)
    if len(wire) != VIDEO_OVERHEAD + length:
        raise VideoError(REASON["BadLength"])
    if binascii.crc_hqx(wire[2:-2], 0xffff) != struct.unpack_from("<H", wire, len(wire) - 2)[0]:
        raise VideoError(REASON["BadCrc"])
    return VideoRecord(kind, token, frame_id, wire[VIDEO_HEADER:-2])


def record_fields(record):
    """Typed fields of a VideoRecord as a dict; VideoError if any check fails."""
    reason = check_record(record.type, record.token, record.frame_id, record.payload)
    if reason:
        raise VideoError(reason)
    return _record_fields(record.type, record.payload)[1]


class MuxParser:
    """Python mirror of es3video::MuxParser: E3 28 control frames and E3 29
    video records on one stream. feed() returns ("control", (seq, kind, body))
    and ("video", VideoRecord) items in arrival order. A rejected candidate
    loses one byte and the rest is rescanned; accepted payloads never are."""

    def __init__(self, max_chunk=DEFAULT_CHUNK_DATA, timeout=0.250):
        self.max_chunk = min(max(max_chunk, 1), MAX_CHUNK_DATA)
        self.timeout = timeout
        self.buffer = bytearray()
        self.last_byte = None
        self.crc_errors = self.header_errors = self.oversize = self.timeouts = 0

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.buffer and now - self.last_byte > self.timeout:
            self._recover_timeout()
        if data:
            self.last_byte = now
            self.buffer += data
        items, buf = [], self.buffer
        while buf:
            if buf[0] != MAGIC[0]:
                sync = buf.find(MAGIC[0], 1)
                del buf[:sync if sync > 0 else len(buf)]
                continue
            step, total = self._examine(buf)
            if step == "need":
                break
            if step == "control":
                items.append(("control", (buf[3], buf[4], bytes(buf[6:total - 2]))))
            elif step == "video":
                kind, length, token, frame_id = struct.unpack_from("<BHII", buf, 3)
                items.append(("video", VideoRecord(kind, token, frame_id, bytes(buf[VIDEO_HEADER:total - 2]))))
            else:
                if step == "crc":
                    self.crc_errors += 1
                elif step in ("header", "oversize"):
                    self.header_errors += 1
                    self.oversize += step == "oversize"
                sync = buf.find(MAGIC[0], 1)
                del buf[:sync if sync > 0 else len(buf)]
                continue
            del buf[:total]
        return items

    def _recover_timeout(self):
        kept = bytearray()
        scan, timed_out = 0, False
        while scan < len(self.buffer):
            step, total = self._examine(self.buffer[scan:])
            if step in ("control", "video"):
                kept += self.buffer[scan:scan + total]
                scan += total  # skip accepted payloads, including embedded magic
            else:
                timed_out |= step == "need"
                self.crc_errors += step == "crc"
                self.header_errors += step in ("header", "oversize")
                self.oversize += step == "oversize"
                scan += 1
        self.timeouts += timed_out
        self.buffer = kept

    def _examine(self, buf):
        n = len(buf)
        if n and buf[0] != MAGIC[0]:
            return "garbage", 0
        if n < 2:
            return "need", 0
        if buf[1] == MAGIC[1]:
            if n < 6:
                return "need", 0
            if buf[2] != VERSION or buf[5] > MAX_PAYLOAD:
                return "header", 0
            total = buf[5] + 8
            if n < total:
                return "need", 0
            good = binascii.crc_hqx(buf[2:total - 2], 0xffff) == struct.unpack_from("<H", buf, total - 2)[0]
            return ("control" if good else "crc"), total
        if buf[1] != VIDEO_MAGIC[1]:
            return "garbage", 0
        if n < 3:
            return "need", 0
        if buf[2] != VIDEO_VERSION:
            return "header", 0
        if n < 4:
            return "need", 0
        kind = buf[3]
        if kind not in _FIXED_PAYLOAD and kind != REC_CHUNK:
            return "header", 0
        if n < 6:
            return "need", 0
        length = struct.unpack_from("<H", buf, 4)[0]
        early = check_framing(kind, length, 1, int(names_frame(kind)), self.max_chunk)
        if early:
            return ("oversize" if early == REASON["TooLarge"] else "header"), 0
        if n < VIDEO_HEADER:
            return "need", 0
        token, frame_id = struct.unpack_from("<II", buf, 6)
        if check_framing(kind, length, token, frame_id, self.max_chunk):
            return "header", 0
        total = VIDEO_OVERHEAD + length
        if n < total:
            return "need", 0
        good = binascii.crc_hqx(buf[2:total - 2], 0xffff) == struct.unpack_from("<H", buf, total - 2)[0]
        return ("video" if good else "crc"), total


def valid_capabilities(c):
    return (c["video_protocol_version"] == VIDEO_VERSION and c["codec_mask"] & CODEC_MASK_BASELINE_JPEG
            and c["canvas_width"] > 0 and c["canvas_height"] > 0
            and 0 < c["max_width"] <= c["canvas_width"] and 0 < c["max_height"] <= c["canvas_height"]
            and 0 < c["max_jpeg_bytes"] <= MAX_JPEG_BYTES and 0 < c["max_chunk_bytes"] <= MAX_CHUNK_DATA
            and 0 < c["max_credits"] <= MAX_CREDITS)


def capabilities_payload(**fields):
    """CAPABILITIES reply body (bench/fake-device use); ValueError if invalid."""
    caps = dict(video_protocol_version=VIDEO_VERSION, codec_mask=CODEC_MASK_BASELINE_JPEG, flags=0)
    caps.update(fields)
    if set(caps) != set(CAPABILITIES_FIELDS) or not valid_capabilities(caps):
        raise ValueError("invalid capabilities")
    return struct.pack(CAPABILITIES_FORMAT, *(caps[name] for name in CAPABILITIES_FIELDS))


def parse_capabilities(body):
    if len(body) != struct.calcsize(CAPABILITIES_FORMAT):
        raise ValueError("capabilities reply must be 18 bytes")
    caps = dict(zip(CAPABILITIES_FIELDS, struct.unpack(CAPABILITIES_FORMAT, body)))
    if not valid_capabilities(caps):
        raise ValueError(f"unsupported capabilities {caps}")
    return caps


def valid_start_video(r, caps):
    if not valid_capabilities(caps) or r["video_protocol_version"] != VIDEO_VERSION:
        return False
    if not 0 <= r["codec"] < 8 or not caps["codec_mask"] & (1 << r["codec"]):
        return False
    if not (0 < r["viewport_w"] <= caps["max_width"] and 0 < r["viewport_h"] <= caps["max_height"]
            and r["viewport_x"] + r["viewport_w"] <= caps["canvas_width"]
            and r["viewport_y"] + r["viewport_h"] <= caps["canvas_height"]):
        return False
    return (0 < r["max_jpeg_bytes"] <= caps["max_jpeg_bytes"]
            and 0 < r["chunk_bytes"] <= caps["max_chunk_bytes"] and 0 < r["target_fps"] <= MAX_TARGET_FPS
            and 0 < r["assembly_timeout_ms"] <= MAX_TIMEOUT_MS and 0 < r["stale_after_ms"] <= MAX_TIMEOUT_MS
            and r["stale_after_ms"] <= r["placeholder_after_ms"] <= MAX_TIMEOUT_MS)


def start_video_payload(caps, **fields):
    """START_VIDEO request body checked against the device's capabilities."""
    request = dict(video_protocol_version=VIDEO_VERSION, codec=CODEC_BASELINE_JPEG)
    request.update(fields)
    if set(request) != set(START_VIDEO_FIELDS) or not valid_start_video(request, caps):
        raise ValueError("START_VIDEO request outside the device's capabilities")
    return struct.pack(START_VIDEO_FORMAT, *(request[name] for name in START_VIDEO_FIELDS))


def parse_start_video(body, caps):
    if len(body) != struct.calcsize(START_VIDEO_FORMAT):
        raise ValueError("START_VIDEO request must be 23 bytes")
    request = dict(zip(START_VIDEO_FIELDS, struct.unpack(START_VIDEO_FORMAT, body)))
    if not valid_start_video(request, caps):
        raise ValueError("START_VIDEO request outside the device's capabilities")
    return request


def start_video_reply_payload(session_token, initial_grant):
    if not session_token or not 1 <= initial_grant <= MAX_CREDITS:
        raise ValueError("START_VIDEO reply needs a nonzero token and a grant of 1..2")
    return struct.pack("<II", session_token, initial_grant)


def parse_start_video_reply(body):
    if len(body) != 8:
        raise ValueError("START_VIDEO reply must be 8 bytes")
    token, grant = struct.unpack("<II", body)
    if not token or not 1 <= grant <= MAX_CREDITS:
        raise ValueError("START_VIDEO reply needs a nonzero token and a grant of 1..2")
    return dict(session_token=token, initial_grant=grant)


def stop_video_payload(session_token):
    if not session_token:
        raise ValueError("session token must be nonzero")
    return struct.pack("<I", session_token)


def parse_stop_video(body):
    if len(body) != 4 or not struct.unpack("<I", body)[0]:
        raise ValueError("STOP_VIDEO request needs a nonzero 4-byte token")
    return struct.unpack("<I", body)[0]


class RemoteError(RuntimeError):
    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code  # ERR_* from an Error reply, None for other failures


class Es3Display:
    def __init__(self, device="/dev/dashcam-display", timeout=3.0):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.device = device
        self.timeout = timeout
        self.fd = None
        self.sequence = 0
        self.parser = MuxParser()
        self.video_records = collections.deque(maxlen=VIDEO_BACKLOG)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def open(self):
        self.close()
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            tty.setraw(self.fd, termios.TCSANOW)
            attrs = termios.tcgetattr(self.fd)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[2] &= ~(termios.HUPCL | termios.CRTSCTS)
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            self.parser = MuxParser()
            self.video_records.clear()
            if self.request(HELLO) != b"ES3C28P":
                raise RemoteError("USB peer is not an ES3C28P display")
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except OSError:
                pass  # a physically disconnected USB tty may already be gone
            os.close(self.fd)
            self.fd = None

    def _wait(self, deadline, write=False):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("ES3C28P USB request timed out")
        readable, writable, _ = select.select([] if write else [self.fd],
                                              [self.fd] if write else [], [], remaining)
        if not (writable if write else readable):
            raise TimeoutError("ES3C28P USB request timed out")

    def request(self, kind, payload=b""):
        if self.fd is None:
            raise RuntimeError("display is not open")
        self.sequence = self.sequence % 255 + 1
        frame = encode(self.sequence, kind, payload)
        deadline = time.monotonic() + self.timeout
        offset = 0
        try:
            while offset < len(frame):
                self._wait(deadline, write=True)
                try:
                    count = os.write(self.fd, frame[offset:])
                except BlockingIOError:
                    continue
                if count == 0:
                    raise OSError(errno.EIO, "USB write returned no progress")
                offset += count
            while True:
                self._wait(deadline)
                try:
                    data = os.read(self.fd, 256)
                except BlockingIOError:
                    continue
                if not data:
                    raise OSError(errno.EIO, "USB device disconnected")
                reply = None
                for item_kind, item in self.parser.feed(data):
                    if item_kind == "video":
                        self.video_records.append(item)  # RESULT/CREDIT/... for a reader
                    elif reply is None and item[0] == self.sequence and item[1] in (ERROR, kind | REPLY):
                        reply = item
                if reply is not None:
                    _, response, body = reply
                    if response == ERROR:
                        code = body[0] if len(body) == 1 else None
                        raise RemoteError(ERROR_NAMES.get(code, "malformed error reply"), code)
                    return body
        except (OSError, TimeoutError):
            # A partial exchange has uncertain results. Reopen/HELLO explicitly;
            # never retry a notification and accidentally play it twice.
            self.close()
            raise

    def ping(self):
        self.request(PING)

    def status(self):
        body = self.request(STATUS)
        if len(body) != 9:
            raise RemoteError("malformed status reply")
        uptime, page, brightness, sd, calibrated, playing = struct.unpack("<IBBBBB", body)
        return dict(uptime_ms=uptime, page=page, brightness=brightness,
                    sd=bool(sd), calibrated=bool(calibrated), audio_playing=bool(playing))

    def set_page(self, index, title, lines):
        self.request(PAGE, page_payload(index, title, lines))

    def alert(self, volume=80):
        if not 0 <= volume <= 100:
            raise ValueError("volume must be 0..100")
        self.request(ALERT, bytes((volume,)))

    def stop_audio(self):
        self.request(STOP_AUDIO)

    def capabilities(self):
        """Device video limits as a dict, or None when the firmware has no video
        (unknown command) or no video memory (not ready). Never retried."""
        try:
            body = self.request(CAPABILITIES)
        except RemoteError as exc:
            if exc.code in (ERR_BAD_COMMAND, ERR_NOT_READY):
                return None
            raise
        try:
            return parse_capabilities(body)
        except ValueError as exc:
            raise RemoteError(f"unusable capabilities reply: {exc}") from None

    def start_video(self, caps, **request):
        """START_VIDEO with fields of START_VIDEO_FIELDS -> session_token / initial_grant."""
        body = self.request(START_VIDEO, start_video_payload(caps, **request))
        try:
            return parse_start_video_reply(body)
        except ValueError as exc:
            raise RemoteError(f"malformed START_VIDEO reply: {exc}") from None

    def stop_video(self, session_token):
        if self.request(STOP_VIDEO, stop_video_payload(session_token)):
            raise RemoteError("malformed STOP_VIDEO reply")


# --- Bench playback (`play`) --------------------------------------------------
# Streams ready-made baseline JPEG files over video protocol v1 for protocol
# diagnostics and soak runs. Production video never passes through Python: the
# C++ link (lib/libes3display) owns the port, and the two must not run at once.
JPEG_SUFFIXES = (".jpg", ".jpeg")
HEARTBEAT_S = 1.0             # PING period (DisplayPreview HeartbeatMs); the firmware expires a session after 5 s
ASSEMBLY_TIMEOUT_MS, STALE_AFTER_MS, PLACEHOLDER_AFTER_MS = 500, 500, 2000   # DisplayPreview defaults
VISIBILITY_GRACE_S = 0.5      # wait this long for the post-negotiation VISIBILITY, then assume active
OUTCOME_TIMEOUT_S = 2.0       # a START with no PRESENTED / failed RESULT by then is dropped as NoOutcome
CREDIT_STALL_S = 3.0          # blocked on credit this long while active: renegotiate rather than stall
FLUSH_TIMEOUT_S = 1.0         # finish a partly written record before STOP_VIDEO
REPORT_S = 1.0
MAX_FRAMES_IN_FLIGHT = 4
LATENCY_SAMPLES = 1 << 16     # bounded history behind the summary percentiles
DEMAND_NAMES = ("active", "hidden", "calibrating")
_PROGRESSIVE_SOF = (0xC2, 0xC6, 0xCA, 0xCE)

PLAY_DESCRIPTION = """\
Bench playback over video protocol v1 (diagnostics and soak runs). Production
video is sent by the C++ link (lib/libes3display); never run this while
another client has the display port open.

Frames are sent as they are: each file must be a baseline (not progressive)
8-bit JPEG of at most 65536 bytes that fits the device canvas (320x240), and
every file must have the first file's size. That size, centred in the canvas,
is the negotiated viewport (320x180 -> rows 30..209, letterboxed). A directory
plays its *.jpg/*.jpeg files in name order. Make frames on any machine that
has the tools (nothing extra is needed on the Jetson), for example:
  ffmpeg -i clip.mp4 -vf fps=10,scale=320:180 -pix_fmt yuvj420p -q:v 6 out/%05d.jpg
  convert in.jpg -resize '320x180!' -interlace none -quality 65 out.jpg
  Pillow: Image.open(src).convert("RGB").resize((320, 180)).save(dst, quality=65)
(lib/libvideopreview makes the same format in production: q65, 4:2:0.)

Every second prints sent/presented fps, wire KiB/s, latency from START write
to PRESENTED p50/p95 (host clock; includes the return path, so an upper
bound), the credit state and drop reasons. Sending pauses while the display
is hidden or calibrating. Ctrl-C, SIGTERM or --duration send STOP_VIDEO and
print a summary (exit 0); a second Ctrl-C abandons the clean stop."""


def jpeg_dimensions(data):
    """(width, height) of a baseline JPEG (SOF0, 8-bit, 1 or 3 components),
    else ValueError. Mirrors the firmware's pre-decode check
    (lib/es3Video/JpegDecoder.h): segments are bounds-checked up to SOS, the
    entropy-coded data is never scanned, and the file must end with EOI."""
    n = len(data)
    if n < 4 or data[0] != 0xFF or data[1] != 0xD8:
        raise ValueError("not a JPEG (no SOI marker)")
    if data[n - 2] != 0xFF or data[n - 1] != 0xD9:
        raise ValueError("truncated JPEG (does not end with EOI)")
    pos, size = 2, None
    while pos + 1 < n:
        if data[pos] != 0xFF:
            raise ValueError(f"corrupt JPEG (no marker at byte {pos})")
        pos += 1
        while pos < n and data[pos] == 0xFF:
            pos += 1
        marker = data[pos]
        pos += 1
        if marker in (0x00, 0x01, 0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            raise ValueError(f"corrupt JPEG (marker FF{marker:02X} before SOS)")
        if pos + 2 > n:
            raise ValueError("truncated JPEG (header cut short)")
        length = data[pos] << 8 | data[pos + 1]
        if length < 2 or length > n - pos:
            raise ValueError(f"truncated JPEG (segment FF{marker:02X} runs past the end)")
        if marker == 0xDA:
            if size is None or length < 6:
                raise ValueError("corrupt JPEG (SOS before SOF, or short SOS)")
            return size
        if 0xC0 <= marker <= 0xCF and marker != 0xC4:   # SOFn; FFC8/FFCC are refused too, as on the board
            if marker in _PROGRESSIVE_SOF:
                raise ValueError(f"progressive JPEG (SOF{marker & 0x0F}) is not supported; re-encode as baseline")
            if marker != 0xC0:
                raise ValueError(f"unsupported JPEG frame type FF{marker:02X}; only baseline SOF0")
            if size is not None or length < 11:
                raise ValueError("corrupt JPEG (repeated or short SOF)")
            precision, height, width, components = struct.unpack_from(">BHHB", data, pos + 2)
            if precision != 8:
                raise ValueError(f"{precision}-bit JPEG is not supported (8-bit only)")
            if components not in (1, 3) or length != 8 + 3 * components:
                raise ValueError(f"unsupported JPEG ({components} components; 1 or 3 only)")
            if not width or not height:
                raise ValueError("JPEG has a zero dimension")
            size = (width, height)
        pos += length
    raise ValueError("truncated JPEG (no SOS segment)")


def prepare_frames(named):
    """[(name, bytes)] -> ([(name, bytes, crc32)], (width, height)); every item
    must be a baseline JPEG of at most MAX_JPEG_BYTES with the first one's size."""
    frames, size = [], None
    for name, data in named:
        if len(data) > MAX_JPEG_BYTES:
            raise ValueError(f"{name}: larger than the protocol limit of {MAX_JPEG_BYTES} bytes")
        try:
            dims = jpeg_dimensions(data)
        except ValueError as exc:
            raise ValueError(f"{name}: {exc}") from None
        if size is None:
            size = dims
        elif dims != size:
            raise ValueError(f"{name}: {dims[0]}x{dims[1]}, but the first frame is {size[0]}x{size[1]}")
        frames.append((name, bytes(data), crc32(data)))
    if not frames:
        raise ValueError("no .jpg/.jpeg frames to play")
    return frames, size


def load_jpegs(sources):
    """Files and directories (their *.jpg/*.jpeg in name order) -> prepare_frames()."""
    paths = []
    for source in sources:
        if os.path.isdir(source):
            paths += [os.path.join(source, name) for name in sorted(os.listdir(source))
                      if name.lower().endswith(JPEG_SUFFIXES)]
        else:
            paths.append(source)
    named = []
    for path in paths:
        with open(path, "rb") as file:
            named.append((path, file.read(MAX_JPEG_BYTES + 1)))  # one byte over is enough to refuse it
    return prepare_frames(named)


def centred_viewport(caps, width, height):
    """START_VIDEO viewport fields for a width x height image centred in the canvas."""
    return dict(viewport_x=(caps["canvas_width"] - width) // 2, viewport_y=(caps["canvas_height"] - height) // 2,
                viewport_w=width, viewport_h=height)


class _Counters:
    """Playback statistics: one instance per report window plus a run total."""

    def __init__(self, samples=None):
        self.sent = self.assembled = self.presented = 0
        self.tx_bytes = self.jpeg_bytes = self.decode_us = self.blit_us = 0
        self.drops = collections.Counter()
        self.latencies = collections.deque(maxlen=samples) if samples else []


class _Transmission:
    """The frame whose records are being written; abort = reason for an ABORT."""
    __slots__ = ("frame_id", "data", "crc", "offset", "abort")

    def __init__(self, frame_id, data, crc):
        self.frame_id, self.data, self.crc, self.offset, self.abort = frame_id, data, crc, 0, None


def _percentile(ordered, fraction):
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)] if ordered else None


def _ms(seconds):
    return "-" if seconds is None else f"{seconds * 1000:.0f} ms"


def _drop_list(drops):
    return ",".join(f"{name}={count}" for name, count in sorted(drops.items())) or "-"


class Player:
    """Bench playback of JPEG frames over video protocol v1, in the caller's thread.

    Negotiates CAPABILITIES + START_VIDEO, then writes START/CHUNK/END records
    paced to the target fps while the device's cumulative grant and the host's
    frames-in-flight cap allow. A frame is in flight from its START until
    PRESENTED, a failed RESULT or OUTCOME_TIMEOUT_S. Writes are non-blocking;
    a control frame (PING, STOP_VIDEO) is only written between whole records,
    and a new record only while TIOCOUTQ holds less than one chunk record.
    Records with another session's token are ignored. Dropped or lost frames
    are never retransmitted."""

    def __init__(self, frames, size, fps=10, loop=False, chunk=DEFAULT_CHUNK_DATA, frames_in_flight=2,
                 duration=None, stop=None, out=None, log=None, heartbeat=HEARTBEAT_S):
        if not frames:
            raise ValueError("no frames to play")
        if not (1 <= fps <= MAX_TARGET_FPS and 1 <= chunk <= MAX_CHUNK_DATA
                and 1 <= frames_in_flight <= MAX_FRAMES_IN_FLIGHT and heartbeat > 0):
            raise ValueError("fps, chunk, frames in flight or heartbeat out of range")
        if duration is not None and not duration > 0:
            raise ValueError("duration must be positive")
        self.frames, self.size, self.fps, self.period = frames, size, fps, 1.0 / fps
        self.loop, self.chunk, self.frames_in_flight = loop, chunk, frames_in_flight
        self.duration, self.heartbeat = duration, heartbeat
        self.stop = stop if stop is not None else threading.Event()
        self.out, self.log = out or sys.stdout, log or sys.stderr
        self.total, self.window = _Counters(LATENCY_SAMPLES), _Counters()
        # Records ignored (wrong token / no matching frame / invalid), grant
        # regressions, recoveries, frames still in flight at a clean exit.
        self.stale = self.unmatched = self.malformed = self.regressions = 0
        self.reconnects = self.resyncs = self.unresolved = 0
        self.index = 0                      # next frame to send; survives reconnects
        self.display = self.fd = self.caps = self.request = None
        self.token = self.grant = self.started = 0
        self.next_frame_id, self.demand = 1, None
        self.inflight = {}                  # frame_id -> [START time, assembled]
        self.tx = None
        self.wbuf, self.woff = b"", 0       # the one video record being written
        self.deadline = self.credit_wait = None
        self.t0 = self.window_start = self.next_report = 0.0
        self.next_due = self.next_ping = self.visibility_deadline = 0.0
        self.outq_limit, self._gated = 0, False

    # -- lifecycle -------------------------------------------------------------
    def play(self, device, timeout=3.0, reconnect_delay=1.0):
        """Open DEVICE, play until done / stopped / out of time, reopening after
        USB loss once a session has worked; print a summary. Returns 0. Raises
        OSError, ValueError or RemoteError before the first session or when the
        device refuses video."""
        self.t0 = self.window_start = time.monotonic()
        self.next_report = self.t0 + REPORT_S
        self.deadline = None if self.duration is None else self.t0 + self.duration
        largest = max(len(data) for _, data, _ in self.frames)
        self._note(f"{len(self.frames)} frame(s) {self.size[0]}x{self.size[1]}, largest {largest} B, "
                   f"{self.fps} fps{', looping' if self.loop else ''}")
        connected = False
        try:
            while not self._done():
                try:
                    with Es3Display(device, timeout) as display:
                        connected = True
                        self.run(display)
                    break
                except (OSError, RemoteError) as exc:
                    if not connected or (isinstance(exc, RemoteError) and exc.code != ERR_NO_SESSION):
                        raise
                    if self._done():
                        break
                    self.reconnects += 1
                    self._note(f"USB offline ({exc}); reconnecting in {reconnect_delay:g} s")
                    self.stop.wait(reconnect_delay)
            return 0
        finally:
            self._summary(time.monotonic() - self.t0)

    def run(self, display):
        """One video session on an open display; returns after STOP_VIDEO."""
        self.display, self.fd = display, display.fd
        caps = display.capabilities()
        if caps is None:
            raise RemoteError("video unsupported by this firmware (control-only)")
        self._check_fits(caps)
        self.caps = caps
        self.request = dict(centred_viewport(caps, *self.size), max_jpeg_bytes=caps["max_jpeg_bytes"],
                            chunk_bytes=self.chunk, target_fps=self.fps, assembly_timeout_ms=ASSEMBLY_TIMEOUT_MS,
                            stale_after_ms=STALE_AFTER_MS, placeholder_after_ms=PLACEHOLDER_AFTER_MS)
        self.outq_limit = VIDEO_OVERHEAD + CHUNK_OFFSET_BYTES + self.chunk
        self._negotiate()
        clean = False
        try:
            while not self._finished():
                self._step()
            clean = True
        except KeyboardInterrupt:
            clean = True
            raise
        finally:
            self._end(clean)

    def _check_fits(self, caps):
        (width, height), largest = self.size, max(len(data) for _, data, _ in self.frames)
        if (width > min(caps["max_width"], caps["canvas_width"])
                or height > min(caps["max_height"], caps["canvas_height"])):
            raise ValueError(f"frames are {width}x{height}; the device takes at most {caps['max_width']}x"
                             f"{caps['max_height']} on a {caps['canvas_width']}x{caps['canvas_height']} canvas")
        if largest > caps["max_jpeg_bytes"]:
            raise ValueError(f"largest frame is {largest} B; the device takes at most {caps['max_jpeg_bytes']} B")
        if self.chunk > caps["max_chunk_bytes"]:
            raise ValueError(f"--chunk {self.chunk} exceeds the device maximum {caps['max_chunk_bytes']}")

    def _negotiate(self):
        session = self.display.start_video(self.caps, **self.request)
        now = time.monotonic()
        self.token, self.grant = session["session_token"], session["initial_grant"]
        self.started, self.next_frame_id, self.demand = 0, 1, None
        self.inflight.clear()
        self.tx, self.wbuf, self.woff, self.credit_wait = None, b"", 0, None
        self.visibility_deadline, self.next_due, self.next_ping = now + VISIBILITY_GRACE_S, now, now + self.heartbeat
        r, c = self.request, self.caps
        self._note(f"session {self.token:#010x}, grant {self.grant}; viewport ({r['viewport_x']},{r['viewport_y']}) "
                   f"{r['viewport_w']}x{r['viewport_h']} on {c['canvas_width']}x{c['canvas_height']}, "
                   f"chunk {self.chunk}, {self.frames_in_flight} in flight "
                   f"(device: {c['max_credits']} credit(s), chunk <= {c['max_chunk_bytes']})")
        self._drain()  # records that arrived during the control exchanges

    def _end(self, clean):
        try:
            if clean and self.token and self.display.fd is not None:
                if self._flush():
                    self.display.stop_video(self.token)
                    self._drain()   # outcomes that arrived before the STOP_VIDEO reply
                else:
                    self._note("could not finish the partly written record; STOP_VIDEO not sent")
        except (OSError, RemoteError) as exc:
            self._note(f"STOP_VIDEO failed: {exc}")
        finally:
            if clean:
                self.unresolved += len(self.inflight)
                self.inflight.clear()
            self._lose_inflight("SessionLost")   # after USB loss the session's frames are gone
            self.token, self.wbuf, self.woff = 0, b"", 0

    def _flush(self):
        """Complete the partly written record (bounded); True when none is left."""
        deadline = time.monotonic() + FLUSH_TIMEOUT_S
        while not self._write():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            select.select([self.fd], [self.fd], [], min(remaining, 0.05))
            self._read()   # keep the board's TX moving; it resets a session stalled for 1 s
        return True

    def _expired(self):
        return self.stop.is_set() or (self.deadline is not None and time.monotonic() >= self.deadline)

    def _done(self):
        return self._expired() or (not self.loop and self.index >= len(self.frames))

    def _finished(self):
        return self._expired() or (not self.loop and self.index >= len(self.frames) and self.tx is None
                                   and not self.wbuf and not self.inflight)

    # -- one loop turn ---------------------------------------------------------
    def _step(self):
        self._read()
        now = time.monotonic()
        self._expire(now)
        gated = busy = False
        for _ in range(16):   # bounded so replies keep being read between bursts
            if not self.wbuf:
                if now >= self.next_ping:
                    self._ping()
                    now = time.monotonic()
                record = self._next_record(now)
                if record is None:
                    gated = self._gated
                    break
                self.wbuf, self.woff = record, 0
            if not self._write():
                break
        else:
            busy = True
        self._report(now)
        self._wait(now, gated, busy)

    def _wait(self, now, gated, busy):
        timeout = min(0.05, self.next_report - now)
        if not self.wbuf:
            timeout = min(timeout, self.next_ping - now)
        if self.next_due > now:
            timeout = min(timeout, self.next_due - now)
        if self.demand is None:
            timeout = min(timeout, self.visibility_deadline - now)
        if gated:
            timeout = min(timeout, 0.001)   # TIOCOUTQ has no wakeup; poll it briefly
        if busy:
            timeout = 0.0
        select.select([self.fd], [self.fd] if self.wbuf else [], [], max(timeout, 0.0))

    def _read(self):
        for _ in range(16):
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                return
            if not data:
                raise OSError(errno.EIO, "USB device disconnected")
            for kind, item in self.display.parser.feed(data):
                if kind == "video":
                    self._on_record(item)
                else:
                    self.unmatched += 1   # a control frame outside any request (late reply)

    def _drain(self):
        while self.display.video_records:
            self._on_record(self.display.video_records.popleft())

    def _ping(self):
        self.display.request(PING)   # device->host records meanwhile land in display.video_records
        now = time.monotonic()
        self.next_ping += self.heartbeat
        if self.next_ping <= now:
            self.next_ping = now + self.heartbeat
        self._drain()

    def _outq(self):
        try:
            return struct.unpack("i", fcntl.ioctl(self.fd, termios.TIOCOUTQ, b"\0\0\0\0"))[0]
        except OSError:
            return 0

    def _may_start(self, now):
        return (self.demand == DEMAND_ACTIVE and now >= self.next_due and self.started < self.grant
                and len(self.inflight) < self.frames_in_flight and self.next_frame_id <= 0xFFFFFFFF
                and (self.loop or self.index < len(self.frames)))

    def _next_record(self, now):
        """The next video record to write, or None (nothing due, no credit, or
        the kernel already holds a chunk record: self._gated)."""
        self._gated = False
        if self.tx is None and not self._may_start(now):
            return None
        if self._outq() >= self.outq_limit:
            self._gated = True
            return None
        tx = self.tx
        if tx is None:
            return self._start_frame(now)
        if tx.abort is not None:
            self.tx = None   # the frame stays in flight until its RESULT
            return encode_abort(self.token, tx.frame_id, tx.abort)
        if tx.offset < len(tx.data):
            end = min(tx.offset + self.chunk, len(tx.data))
            record = encode_chunk(self.token, tx.frame_id, tx.offset, tx.data[tx.offset:end])
            tx.offset = end
            return record
        self.tx = None
        return encode_end(self.token, tx.frame_id, tx.crc)

    def _start_frame(self, now):
        _, data, crc = self.frames[self.index]
        self.index = (self.index + 1) % len(self.frames) if self.loop else self.index + 1
        frame_id, self.next_frame_id = self.next_frame_id, self.next_frame_id + 1
        self.started += 1
        self.tx = _Transmission(frame_id, data, crc)
        self.inflight[frame_id] = [time.monotonic(), False]   # latency runs from the START write
        self.next_due += self.period
        if self.next_due < now:
            self.next_due = now + self.period   # late (credit/in-flight bound): no catch-up burst
        self._count("sent")
        self._count("jpeg_bytes", len(data))
        return encode_start(self.token, frame_id, self.size[0], self.size[1], len(data), int(now * 1e6))

    def _write(self):
        """Write the pending record; True once it is completely written."""
        while self.wbuf:
            try:
                count = os.write(self.fd, memoryview(self.wbuf)[self.woff:])
            except BlockingIOError:
                return False
            if count == 0:
                raise OSError(errno.EIO, "USB write returned no progress")
            self.woff += count
            self._count("tx_bytes", count)
            if self.woff == len(self.wbuf):
                self.wbuf, self.woff = b"", 0
        return True

    def _expire(self, now):
        if self.demand is None and now >= self.visibility_deadline:
            self._note("no VISIBILITY after START_VIDEO; assuming the display is active")
            self.demand = DEMAND_ACTIVE
        for frame_id in [f for f, flight in self.inflight.items() if now - flight[0] > OUTCOME_TIMEOUT_S]:
            self._finish(frame_id, "NoOutcome")
        if self.wbuf or self.tx is not None:
            return
        if self.next_frame_id > 0xFFFFFFFF:
            self._resync("frame ids exhausted")
            return
        blocked = (self.demand == DEMAND_ACTIVE and self.started >= self.grant
                   and (self.loop or self.index < len(self.frames)))
        if not blocked:
            self.credit_wait = None
        elif self.credit_wait is None:
            self.credit_wait = now
        elif now - self.credit_wait > CREDIT_STALL_S:
            self._resync(f"no credit for {CREDIT_STALL_S:g} s (grant {self.grant}, started {self.started})")

    def _resync(self, why):
        """Forced session resynchronisation: a lost grant must not stall forever."""
        self.resyncs += 1
        self._note(f"{why}; renegotiating")
        self._lose_inflight("Resync")
        token, self.token = self.token, 0
        self.display.stop_video(token)
        self._negotiate()

    # -- device -> host records ------------------------------------------------
    def _on_record(self, record):
        if check_record(*record):
            self.malformed += 1
            return
        if record.token != self.token:
            self.stale += 1   # old session (or none): never credits, pauses or completes anything
            return
        fields = _record_fields(record.type, record.payload)[1]
        if record.type == REC_CREDIT:
            grant = fields["cumulative_grant"]
            self.regressions += grant < self.grant
            self.grant = max(self.grant, grant)   # cumulative: repeats are harmless
        elif record.type == REC_VISIBILITY:
            if fields["demand"] != self.demand:
                self.demand = fields["demand"]
                active = self.demand == DEMAND_ACTIVE
                self._note(f"display {DEMAND_NAMES[self.demand]}{'' if active else '; sending paused'}")
                if not active and self.tx is not None:
                    self.tx.abort = REASON["Hidden"]   # end the partial frame at the next record boundary
        elif record.type == REC_RESULT:
            flight, reason = self.inflight.get(record.frame_id), fields["reason"]
            if flight is None:
                self.unmatched += 1
            elif reason == REASON["Ok"]:
                if not flight[1]:
                    flight[1] = True
                    self._count("assembled")
            else:
                if reason in (REASON["NoCredit"], REASON["BadFrameId"]):
                    self.started -= 1   # refused before it was counted: it consumed no grant
                self._finish(record.frame_id, REASONS[reason])
        elif record.type == REC_PRESENTED:
            flight = self.inflight.pop(record.frame_id, None)
            if flight is None:
                self.unmatched += 1
                return
            latency = time.monotonic() - flight[0]
            for counters in (self.total, self.window):
                counters.presented += 1
                counters.latencies.append(latency)
                counters.decode_us += fields["decode_us"]
                counters.blit_us += fields["blit_us"]
        else:
            self.malformed += 1   # a host->device record type coming back

    def _finish(self, frame_id, reason):
        self.inflight.pop(frame_id, None)
        for counters in (self.total, self.window):
            counters.drops[reason] += 1
        if self.tx is not None and self.tx.frame_id == frame_id:
            self.tx = None   # the device already dropped it: send no more of its records

    def _lose_inflight(self, reason):
        for _ in range(len(self.inflight)):
            for counters in (self.total, self.window):
                counters.drops[reason] += 1
        self.inflight.clear()
        self.tx = None

    # -- reporting -------------------------------------------------------------
    def _count(self, field, value=1):
        for counters in (self.total, self.window):
            setattr(counters, field, getattr(counters, field) + value)

    def _note(self, message):
        print(f"play: {message}", file=self.log, flush=True)

    def _report(self, now):
        if now < self.next_report:
            return
        span, w = max(now - self.window_start, 1e-3), self.window
        latencies = sorted(w.latencies)
        state = "WAITING" if self.demand is None else DEMAND_NAMES[self.demand].upper()
        print(f"[{now - self.t0:7.1f}s] {state:<11} sent {w.sent / span:4.1f} fps  "
              f"presented {w.presented / span:4.1f} fps  tx {w.tx_bytes / 1024 / span:6.1f} KiB/s  "
              f"latency p50 {_ms(_percentile(latencies, 0.5))} "
              f"p95 {_ms(_percentile(latencies, 0.95))}  grant {self.grant} started {self.started} "
              f"in flight {len(self.inflight)}  drops {_drop_list(w.drops)}", file=self.out, flush=True)
        self.window, self.window_start = _Counters(), now
        self.next_report += REPORT_S
        if self.next_report <= now:
            self.next_report = now + REPORT_S

    def _summary(self, elapsed):
        t = self.total
        latencies = sorted(t.latencies)

        def mean_ms(total_us):
            return f"{total_us / t.presented / 1000:.1f} ms" if t.presented else "-"

        print(f"play: summary: {t.sent} sent, {t.assembled} assembled, {t.presented} presented in {elapsed:.1f} s "
              f"({t.presented / elapsed if elapsed > 0 else 0.0:.1f} fps); {t.jpeg_bytes / 1024:.1f} KiB JPEG, "
              f"{t.tx_bytes / 1024:.1f} KiB on the wire\n"
              f"play: latency START->PRESENTED p50 {_ms(_percentile(latencies, 0.5))} "
              f"p95 {_ms(_percentile(latencies, 0.95))} max {_ms(latencies[-1] if latencies else None)}; "
              f"device decode {mean_ms(t.decode_us)}, blit {mean_ms(t.blit_us)} (mean)\n"
              f"play: drops {_drop_list(t.drops)}; in flight at exit {self.unresolved}; ignored records: "
              f"stale {self.stale}, unmatched {self.unmatched}, malformed {self.malformed}; grant regressions "
              f"{self.regressions}; reconnects {self.reconnects}, resyncs {self.resyncs}", file=self.out, flush=True)


def _bounded(kind, low, high):
    """argparse type: kind(text) within [low, high] (NaN and inf rejected)."""
    def parse(text):
        try:
            value = kind(text)
        except ValueError:
            raise argparse.ArgumentTypeError(f"invalid {kind.__name__} value: {text!r}") from None
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f"{text} is outside {low}..{high}")
        return value
    return parse


def _play_command(args):
    frames, size = load_jpegs(args.sources)
    stop = threading.Event()

    def on_signal(*_):
        if stop.is_set():
            raise KeyboardInterrupt   # second Ctrl-C: give up on the clean STOP_VIDEO
        stop.set()

    previous = {sig: signal.signal(sig, on_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        player = Player(frames, size, fps=args.fps, loop=args.loop, chunk=args.chunk,
                        frames_in_flight=args.frames_in_flight, duration=args.duration, stop=stop)
        return player.play(args.device)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/dashcam-display", help="explicit display tty or persistent by-id path")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("status", "watch", "ping", "stop-audio", "capabilities"):
        commands.add_parser(name)
    alert = commands.add_parser("alert")
    alert.add_argument("--volume", type=int, default=80)
    page = commands.add_parser("page")
    page.add_argument("index", type=int, choices=range(5))
    page.add_argument("--title", required=True)
    page.add_argument("--line", action="append", required=True, help="repeat exactly four times")
    play = commands.add_parser("play", help="bench playback of baseline JPEG files (protocol diagnostics)",
                               description=PLAY_DESCRIPTION, formatter_class=argparse.RawDescriptionHelpFormatter)
    play.add_argument("sources", nargs="+", metavar="DIR_OR_FILE", help="JPEG files and/or directories of them")
    play.add_argument("--fps", type=_bounded(int, 1, MAX_TARGET_FPS), default=10,
                      help="target frame rate, 1-30 (default 10; the dashcam config allows 1-15)")
    play.add_argument("--loop", action="store_true", help="repeat the frames until Ctrl-C or --duration")
    play.add_argument("--chunk", type=_bounded(int, 1, MAX_CHUNK_DATA), default=DEFAULT_CHUNK_DATA,
                      help="JPEG bytes per CHUNK record (default 1024; at most the device's limit)")
    play.add_argument("--frames-in-flight", type=_bounded(int, 1, MAX_FRAMES_IN_FLIGHT), default=2, metavar="N",
                      help="host cap on frames between START and PRESENTED/drop, 1-4 (default 2); "
                           "the device's credits apply as well")
    play.add_argument("--duration", type=_bounded(float, 0.01, 1e6), metavar="S", help="stop after S seconds")
    args = parser.parse_args()
    try:
        if args.command == "play":
            return _play_command(args)
        if args.command == "watch":
            # Monitoring alone is safe to reconnect/repeat; other commands run once.
            while True:
                try:
                    with Es3Display(args.device) as display:
                        while True:
                            print(json.dumps(display.status()), flush=True)
                            time.sleep(1)  # also keeps the firmware's session alive
                except (OSError, TimeoutError) as exc:
                    print(f"USB offline: {exc}; retrying in 1 s", file=sys.stderr)
                    time.sleep(1)
        with Es3Display(args.device) as display:
            if args.command == "status":
                print(json.dumps(display.status()))
            elif args.command == "ping":
                display.ping()
                print("ES3C28P USB OK")
            elif args.command == "page":
                display.set_page(args.index, args.title, args.line)
            elif args.command == "alert":
                display.alert(args.volume)
            elif args.command == "stop-audio":
                display.stop_audio()
            elif args.command == "capabilities":
                print(json.dumps(display.capabilities()))  # null = video unsupported
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RemoteError) as exc:
        print(f"ES3C28P: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
