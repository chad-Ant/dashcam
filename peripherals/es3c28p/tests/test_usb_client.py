"""No board needed: exercise actual POSIX serial I/O through Linux pseudo-ttys."""
import contextlib
import heapq
import importlib.util
import io
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

MODULE = Path(__file__).resolve().parents[1] / "host" / "es3_display.py"
VECTORS = Path(__file__).resolve().parent / "video_golden_vectors.txt"
MAP_VECTORS = Path(__file__).resolve().parent / "map_golden_vectors.txt"
spec = importlib.util.spec_from_file_location("es3_display", MODULE)
es3 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(es3)

# Field values behind tests/video_golden_vectors.txt; video_protocol_test.cpp
# builds the same vectors independently with the C++ header.
TOKEN = 0x89ABCDEF
CAPS = dict(canvas_width=320, canvas_height=240, max_width=320, max_height=240,
            max_jpeg_bytes=65536, max_chunk_bytes=1024, max_credits=2)
START_VIDEO_REQUEST = dict(viewport_x=0, viewport_y=30, viewport_w=320, viewport_h=180,
                           max_jpeg_bytes=65536, chunk_bytes=1024, target_fps=10,
                           assembly_timeout_ms=500, stale_after_ms=500, placeholder_after_ms=2000)


def raw_record(kind, length, token, frame_id, payload, version=1):
    """Any header, correct CRC: builds records the encoder refuses to make."""
    body = struct.pack("<BBHII", version, kind, length, token, frame_id) + payload
    return es3.VIDEO_MAGIC + body + struct.pack("<H", es3.binascii.crc_hqx(body, 0xffff))


def chunk_data():
    # JPEG-looking bytes with a complete control frame and a video header inside.
    return b"\xff\xd8" + es3.encode(5, es3.PING) + b"\xe3\x29\x01\x02" + b"\xff\xd9"


def expected_vectors():
    """name -> (wire bytes, extra field) for every line of the golden file."""
    start = es3.encode_start(TOKEN, 1, 320, 180, 8195, 0x8877665544332211)
    request = es3.encode(1, es3.CAPABILITIES)
    v = {
        "rec.start": (start, None),
        "rec.chunk_first": (es3.encode_chunk(TOKEN, 1, 0, chunk_data()), None),
        "rec.chunk_next": (es3.encode_chunk(TOKEN, 1, 1024, bytes(range(0xe0, 0xf0))), None),
        "rec.end": (es3.encode_end(TOKEN, 1, 0xCBF43926), None),
        "rec.abort": (es3.encode_abort(TOKEN, 2, es3.REASON["Superseded"]), None),
        "rec.result_ok": (es3.encode_result(TOKEN, 1, es3.REASON["Ok"]), None),
        "rec.result_decode_failed": (es3.encode_result(TOKEN, 3, es3.REASON["DecodeFailed"]), None),
        "rec.presented": (es3.encode_presented(TOKEN, 1, 12345, 23456), None),
        "rec.credit": (es3.encode_credit(TOKEN, 3), None),
        "rec.visibility": (es3.encode_visibility(TOKEN, es3.DEMAND_CALIBRATING), None),
        "ctl.capabilities_request": (request, None),
        "ctl.capabilities_reply": (es3.encode(1, es3.CAPABILITIES | es3.REPLY, es3.capabilities_payload(**CAPS)), None),
        "ctl.capabilities_unsupported": (es3.encode(1, es3.ERROR, bytes((es3.ERR_BAD_COMMAND,))), None),
        "ctl.start_video_request": (es3.encode(2, es3.START_VIDEO, es3.start_video_payload(
            es3.parse_capabilities(es3.capabilities_payload(**CAPS)), **START_VIDEO_REQUEST)), None),
        "ctl.start_video_reply": (es3.encode(2, es3.START_VIDEO | es3.REPLY, es3.start_video_reply_payload(TOKEN, 1)), None),
        "ctl.stop_video_request": (es3.encode(3, es3.STOP_VIDEO, es3.stop_video_payload(TOKEN)), None),
        "ctl.stop_video_reply": (es3.encode(3, es3.STOP_VIDEO | es3.REPLY), None),
        "crc32.check": (b"123456789", "cbf43926"),
        "crc32.bytes_0_255": (bytes(range(256)), "29058c73"),
        "bad.crc": (start[:-1] + bytes((start[-1] ^ 0xff,)), "drop"),
        "bad.length": (raw_record(es3.REC_START, 15, TOKEN, 1, bytes(15)), "drop"),
        "bad.version": (raw_record(es3.REC_CREDIT, 4, TOKEN, 0, bytes((3, 0, 0, 0)), version=2), "drop"),
        "bad.zero_token": (raw_record(es3.REC_CREDIT, 4, 0, 0, bytes((3, 0, 0, 0))), "drop"),
        "bad.frame_id_on_credit": (raw_record(es3.REC_CREDIT, 4, TOKEN, 7, bytes((3, 0, 0, 0))), "drop"),
        "bad.zero_frame_id": (raw_record(es3.REC_END, 4, TOKEN, 0, bytes(4)), "drop"),
        "bad.unknown_type": (raw_record(0x05, 1, TOKEN, 1, b"\0"), "drop"),
        "bad.empty_chunk": (raw_record(es3.REC_CHUNK, 4, TOKEN, 1, bytes(4)), "drop"),
        # Header only: rejected from its length field, before any payload arrives.
        "bad.oversize_chunk": (raw_record(es3.REC_CHUNK, 4 + 4097, TOKEN, 1, b"")[:14] + b"\x55" * 8, "drop"),
        "bad.control_crc": (request[:-1] + bytes((request[-1] ^ 0xff,)), "drop"),
        "bad.start_too_large": (raw_record(es3.REC_START, 16, TOKEN, 1,
                                           struct.pack("<HHIQ", 320, 180, 65537, 0)), "TooLarge"),
        "bad.start_zero_width": (raw_record(es3.REC_START, 16, TOKEN, 1,
                                            struct.pack("<HHIQ", 0, 180, 8195, 0)), "BadDimensions"),
        "bad.chunk_past_ceiling": (raw_record(es3.REC_CHUNK, 20, TOKEN, 1,
                                              struct.pack("<I", 65530) + bytes(16)), "BadOffset"),
        "bad.result_reason": (raw_record(es3.REC_RESULT, 1, TOKEN, 1, b"\x7f"), "Malformed"),
        "bad.visibility_demand": (raw_record(es3.REC_VISIBILITY, 1, TOKEN, 0, b"\x03"), "Malformed"),
    }
    return v


def load_vectors(path=VECTORS):
    vectors = {}
    for number, line in enumerate(path.read_text().splitlines(), 1):
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        if len(fields) not in (2, 3) or fields[0] in vectors:
            raise AssertionError(f"{path.name}:{number}: malformed or duplicate vector")
        vectors[fields[0]] = (bytes.fromhex(fields[1]), fields[2] if len(fields) == 3 else None)
    return vectors


# Field values behind tests/map_golden_vectors.txt; map_protocol_test.cpp builds
# the same vectors independently with Es3MapProtocol.h. The video capabilities
# are the ES3C28P firmware's own (one credit).
MAP_TOKEN = 0x2468ACE0
BOARD_CAPS = dict(video_protocol_version=1, codec_mask=1, canvas_width=320, canvas_height=240, max_width=320,
                  max_height=240, max_jpeg_bytes=65536, max_chunk_bytes=1024, max_credits=1, flags=0)
MAP_CAPS = dict(page_count=6, map_page=5, viewport_x=0, viewport_y=0, viewport_w=320, viewport_h=224,
                max_jpeg_bytes=65536)
START_MAP_REQUEST = dict(max_jpeg_bytes=65536, chunk_bytes=1024, target_fps=4, assembly_timeout_ms=500,
                         stale_after_ms=2000, placeholder_after_ms=5000, generation=7)


def expected_map_vectors():
    m = es3.parse_map_capabilities(es3.map_capabilities_payload(BOARD_CAPS, **MAP_CAPS), BOARD_CAPS)
    good = struct.unpack(es3.START_MAP_FORMAT, es3.start_map_payload(m, BOARD_CAPS, **START_MAP_REQUEST))
    fields = dict(zip(es3.START_MAP_FIELDS, good))

    def start_map_raw(**changes):
        f = dict(fields)
        f.update(changes)
        return struct.pack(es3.START_MAP_FORMAT, *(f[name] for name in es3.START_MAP_FIELDS))

    return {
        "ctl.map_capabilities_request": (es3.encode(0x10, es3.CMD_MAP_CAPABILITIES), None),
        "ctl.map_capabilities_reply": (es3.encode(0x10, es3.CMD_MAP_CAPABILITIES | es3.REPLY,
                                                  es3.map_capabilities_payload(BOARD_CAPS, **MAP_CAPS)), None),
        "ctl.map_capabilities_unsupported": (es3.encode(0x10, es3.ERROR, bytes((es3.ERR_BAD_COMMAND,))), None),
        "ctl.start_map_request": (es3.encode(0x11, es3.CMD_START_MAP,
                                             es3.start_map_payload(m, BOARD_CAPS, **START_MAP_REQUEST)), None),
        "ctl.start_map_reply": (es3.encode(0x11, es3.CMD_START_MAP | es3.REPLY,
                                           es3.start_map_reply_payload(MAP_TOKEN, 1)), None),
        "ctl.start_map_busy": (es3.encode(0x11, es3.ERROR, bytes((es3.ERR_NOT_READY,))), None),
        "ctl.stop_map_request": (es3.encode(0x12, es3.CMD_STOP_MAP, es3.stop_map_payload(MAP_TOKEN)), None),
        "ctl.stop_map_reply": (es3.encode(0x12, es3.CMD_STOP_MAP | es3.REPLY), None),
        "rec.map_start": (es3.encode_start(MAP_TOKEN, 1, 320, 224, 30000, 0x0102030405060708), None),
        "pay.start_map_wrong_page": (start_map_raw(page=1), "reject"),
        "pay.start_map_wrong_viewport": (start_map_raw(viewport_y=8), "reject"),
        "pay.start_map_jpeg_too_large": (start_map_raw(max_jpeg_bytes=65537), "reject"),
        "pay.start_map_chunk_too_large": (start_map_raw(chunk_bytes=1025), "reject"),
        "pay.start_map_placeholder_before_stale": (start_map_raw(placeholder_after_ms=1999), "reject"),
        "pay.start_map_version": (start_map_raw(map_protocol_version=2), "reject"),
        "pay.map_caps_page_out_of_range": (struct.pack(es3.MAP_CAPABILITIES_FORMAT, 1, 6, 6, 0, 0, 320, 224,
                                                       65536, 1), "reject"),
        "pay.map_caps_viewport_off_canvas": (struct.pack(es3.MAP_CAPABILITIES_FORMAT, 1, 6, 5, 0, 30, 320, 224,
                                                         65536, 1), "reject"),
    }


def feed_items(parser, data, split=None, now=0):
    if split is None:
        return parser.feed(data, now=now)
    return parser.feed(data[:split], now=now) + parser.feed(data[split:], now=now)


class ProtocolTests(unittest.TestCase):
    def test_cpp_vector(self):
        # run_tests.sh compiles this from the firmware's actual portable header.
        output = subprocess.check_output([os.environ["ES3_PROTOCOL_TEST"]], text=True).strip()
        self.assertEqual(output, es3.encode(42, es3.HELLO | es3.REPLY, b"ES3C28P").hex())

    def test_fragmentation_noise_crc_and_expiry(self):
        p = es3.Parser()
        frame = es3.encode(7, es3.STATUS, b"abc")
        bad = frame[:-1] + bytes((frame[-1] ^ 0xff,))
        results = []
        for byte in b"boot noise\x7e\xe3" + bad + frame:
            results += p.feed(bytes((byte,)), now=0)
        self.assertEqual(results, [(7, es3.STATUS, b"abc")])
        self.assertEqual(p.feed(frame[:4], now=1), [])
        self.assertEqual(p.feed(frame, now=2), [(7, es3.STATUS, b"abc")])
        p.feed(b"\xe3\x28\x01\x01\x01\xff" * 1000, now=2)
        self.assertLessEqual(len(p.buffer), es3.MAX_PAYLOAD + 8)

    def test_page_contract(self):
        payload = es3.page_payload(0, "LIVE", ["Record: ON", "Speed: 42 km/h", "GPS: fix", ""])
        self.assertEqual(len(payload), 121)
        self.assertEqual(payload[1:25], b"LIVE" + bytes(20))
        for title in ("a" * 24, "bad\nline", "bad\0line", "\N{SNOWMAN}"):
            with self.assertRaises(ValueError):
                es3.page_payload(0, title, [""] * 4)
        with self.assertRaises(ValueError):
            es3.page_payload(5, "TITLE", [""] * 4)


class VideoProtocolTests(unittest.TestCase):
    def test_crc32_check_value(self):
        self.assertEqual(es3.crc32(b"123456789"), 0xCBF43926)
        self.assertEqual(es3.crc32(b"56789", es3.crc32(b"1234")), 0xCBF43926)

    def test_golden_vectors(self):
        expected, frozen = expected_vectors(), load_vectors()
        self.assertEqual(sorted(frozen), sorted(expected), "vector names drifted")
        for name, (wire, extra) in expected.items():
            with self.subTest(name):
                self.assertEqual(frozen[name][0].hex(), wire.hex())
                self.assertEqual(frozen[name][1], extra)
                if name.startswith("crc32."):
                    self.assertEqual(f"{es3.crc32(wire):08x}", extra)

    def test_valid_vectors_decode(self):
        v = expected_vectors()
        start = es3.record_fields(es3.decode_record(v["rec.start"][0]))
        self.assertEqual(start, dict(width=320, height=180, jpeg_bytes=8195,
                                     source_timestamp_us=0x8877665544332211))
        chunk = es3.decode_record(v["rec.chunk_first"][0])
        self.assertEqual((chunk.type, chunk.token, chunk.frame_id), (es3.REC_CHUNK, TOKEN, 1))
        self.assertEqual(es3.record_fields(chunk), dict(offset=0, data=chunk_data()))
        self.assertEqual(es3.record_fields(es3.decode_record(v["rec.chunk_next"][0]))["offset"], 1024)
        self.assertEqual(es3.record_fields(es3.decode_record(v["rec.end"][0])), dict(crc32=0xCBF43926))
        self.assertEqual(es3.record_fields(es3.decode_record(v["rec.abort"][0])),
                         dict(reason=es3.REASON["Superseded"]))
        self.assertEqual(es3.record_fields(es3.decode_record(v["rec.presented"][0])),
                         dict(decode_us=12345, blit_us=23456))
        credit = es3.decode_record(v["rec.credit"][0])
        self.assertEqual((credit.frame_id, es3.record_fields(credit)), (0, dict(cumulative_grant=3)))
        self.assertEqual(es3.record_fields(es3.decode_record(v["rec.visibility"][0])),
                         dict(demand=es3.DEMAND_CALIBRATING))
        # Explicit layout check of one record, independent of the encoder.
        self.assertEqual(v["rec.credit"][0][:14].hex(), "e3290183" "0400" "efcdab89" "00000000")

    def test_malformed_vectors(self):
        valid = expected_vectors()["rec.credit"][0]
        for name, (wire, extra) in expected_vectors().items():
            if not name.startswith("bad."):
                continue
            with self.subTest(name):
                items = es3.MuxParser(es3.MAX_CHUNK_DATA).feed(wire + valid, now=0)
                if extra == "drop":
                    self.assertEqual(items, [("video", es3.decode_record(valid))])
                    if wire[:2] == es3.VIDEO_MAGIC:
                        with self.assertRaises(es3.VideoError):
                            es3.decode_record(wire)
                else:
                    record = es3.decode_record(wire)
                    self.assertEqual(items, [("video", record), ("video", es3.decode_record(valid))])
                    self.assertEqual(es3.check_record(*record), es3.REASON[extra])
                    with self.assertRaises(es3.VideoError) as caught:
                        es3.record_fields(record)
                    self.assertEqual(caught.exception.reason, es3.REASON[extra])

    def test_encoder_refuses_invalid_records(self):
        bad = [lambda: es3.encode_credit(0, 1), lambda: es3.encode_end(TOKEN, 0, 1),
               lambda: es3.encode_record(es3.REC_CREDIT, TOKEN, 1, bytes(4)),
               lambda: es3.encode_chunk(TOKEN, 1, 0, b""), lambda: es3.encode_chunk(TOKEN, 1, 0, bytes(4097)),
               lambda: es3.encode_chunk(TOKEN, 1, 65535, b"ab"),
               lambda: es3.encode_start(TOKEN, 1, 320, 0, 10, 0),
               lambda: es3.encode_start(TOKEN, 1, 320, 180, 65537, 0),
               lambda: es3.encode_visibility(TOKEN, 3), lambda: es3.encode_result(TOKEN, 1, 17),
               lambda: es3.encode_abort(TOKEN, 1, es3.REASON["Ok"]),
               lambda: es3.encode_record(0x05, TOKEN, 1, b"x")]
        for build in bad:
            with self.assertRaises(es3.VideoError):
                build()
        self.assertEqual(len(es3.encode_chunk(TOKEN, 1, 0, bytes(4096))), 16 + 4 + 4096)

    def test_mux_parser_split_coalesced_and_embedded(self):
        v = expected_vectors()
        stream = b"".join(wire for name, (wire, _) in sorted(v.items()) if name[:4] in ("rec.", "ctl."))
        reference = es3.MuxParser().feed(stream, now=0)
        self.assertEqual(len(reference), sum(name[:4] in ("rec.", "ctl.") for name in v))
        # The PING frame and the E3 29 header inside CHUNK data are never items.
        self.assertNotIn(("control", (5, es3.PING, b"")), reference)
        for split in range(len(stream) + 1):
            self.assertEqual(feed_items(es3.MuxParser(), stream, split), reference, split)
        p = es3.MuxParser()
        self.assertEqual([item for byte in stream for item in p.feed(bytes((byte,)), now=0)], reference)

    def test_mux_parser_garbage_truncation_and_timeout(self):
        v = expected_vectors()
        credit, start = v["rec.credit"][0], v["rec.start"][0]
        ping = es3.encode(9, es3.PING)
        noise = b"\x00\xe3\xe3\x28\xe3\x29\x01\xe3\x29\x02\xff\x00" + start[:20]
        items = es3.MuxParser().feed(noise + credit + b"junk\xe3" + ping + start, now=0)
        self.assertEqual([kind for kind, _ in items], ["video", "control", "video"])
        # A false header whose length covers later records: the rescan recovers them.
        false_chunk = raw_record(es3.REC_CHUNK, 4 + 1000, TOKEN, 1, b"")[:14]
        items = es3.MuxParser().feed(false_chunk + credit + ping + bytes(1100) + credit, now=0)
        self.assertEqual([kind for kind, _ in items], ["video", "control", "video"])
        # Truncated record, then a gap longer than the timeout.
        p = es3.MuxParser()
        self.assertEqual(p.feed(start[:10], now=0.0), [])
        self.assertEqual(p.feed(credit, now=0.3), [("video", es3.decode_record(credit))])
        self.assertEqual(p.timeouts, 1)
        # Oversize relative to the configured chunk size (1024 by default).
        big = es3.encode_chunk(TOKEN, 1, 0, bytes(1025))
        self.assertEqual(es3.MuxParser().feed(big + credit, now=0), [("video", es3.decode_record(credit))])
        self.assertEqual(len(es3.MuxParser(4096).feed(big, now=0)), 1)

    def test_timeout_recovers_buffered_records(self):
        v = expected_vectors()
        credit, start = v["rec.credit"][0], v["rec.start"][0]
        ping = es3.encode(9, es3.PING)
        stalled = raw_record(es3.REC_CHUNK, 1004, TOKEN, 1, b"")[:14]
        chunk = es3.encode_chunk(TOKEN, 9, 0, ping + credit)
        buffered = stalled + credit + stalled + chunk + ping + start[:10]
        expected = es3.MuxParser().feed(credit + chunk + ping, now=0)
        for incoming in (b"", start):
            p = es3.MuxParser()
            self.assertEqual(p.feed(buffered, now=0), [])
            self.assertEqual(p.feed(incoming, now=0.3), expected +
                             ([("video", es3.decode_record(start))] if incoming else []))
            self.assertEqual(p.timeouts, 1)
            self.assertEqual(p.feed(b"", now=1), [])
            self.assertEqual(p.buffer, b"")
        bad = es3.decode_record(raw_record(es3.REC_ABORT, 1, TOKEN, 1, b"\x00"))
        self.assertEqual(es3.check_record(bad.type, bad.token, bad.frame_id, bad.payload),
                         es3.REASON["Malformed"])

    def test_control_payload_validation(self):
        caps = es3.parse_capabilities(es3.capabilities_payload(**CAPS))
        good = es3.start_video_payload(caps, **START_VIDEO_REQUEST)
        self.assertEqual(len(good), 23)
        self.assertEqual(es3.parse_start_video(good, caps)["viewport_y"], 30)
        for change in (dict(viewport_w=0), dict(viewport_h=0), dict(viewport_y=61), dict(viewport_x=1),
                       dict(chunk_bytes=1025), dict(chunk_bytes=0), dict(max_jpeg_bytes=0),
                       dict(max_jpeg_bytes=65537), dict(target_fps=0), dict(target_fps=31),
                       dict(placeholder_after_ms=499), dict(assembly_timeout_ms=0), dict(codec=1),
                       dict(video_protocol_version=2)):
            with self.subTest(change), self.assertRaises(ValueError):
                es3.start_video_payload(caps, **{**START_VIDEO_REQUEST, **change})
        for size in range(30):
            if size != 23:
                with self.assertRaises(ValueError):
                    es3.parse_start_video(good[:size].ljust(size, b"\0"), caps)
        for change in (dict(max_credits=3), dict(max_credits=0), dict(max_width=321), dict(max_chunk_bytes=4097),
                       dict(max_jpeg_bytes=65537), dict(video_protocol_version=2), dict(codec_mask=2)):
            with self.subTest(change), self.assertRaises(ValueError):
                es3.capabilities_payload(**{**CAPS, **change})
        with self.assertRaises(ValueError):
            es3.parse_capabilities(es3.capabilities_payload(**CAPS)[:-1])
        for token, grant in ((0, 1), (TOKEN, 0), (TOKEN, 3)):
            with self.assertRaises(ValueError):
                es3.parse_start_video_reply(struct.pack("<II", token, grant))
        with self.assertRaises(ValueError):
            es3.parse_stop_video(bytes(4))
        self.assertEqual(es3.parse_stop_video(es3.stop_video_payload(TOKEN)), TOKEN)


class MapProtocolTests(unittest.TestCase):
    def test_golden_vectors(self):
        self.assertEqual({k: (v[0].hex(), v[1]) for k, v in load_vectors(MAP_VECTORS).items()},
                         {k: (v[0].hex(), v[1]) for k, v in expected_map_vectors().items()})

    def test_rejected_payloads(self):
        m = es3.parse_map_capabilities(es3.map_capabilities_payload(BOARD_CAPS, **MAP_CAPS), BOARD_CAPS)
        for name, (body, extra) in expected_map_vectors().items():
            if not name.startswith("pay."):
                continue
            self.assertEqual(extra, "reject")
            parse = es3.parse_map_capabilities if name.startswith("pay.map_caps") else (
                lambda b, c: es3.parse_start_map(b, m, c))
            with self.assertRaises(ValueError, msg=name):
                parse(body, BOARD_CAPS)

    def test_map_frames_are_ordinary_video_records(self):
        rec = es3.decode_record(expected_map_vectors()["rec.map_start"][0])
        self.assertEqual(es3.record_fields(rec)["width"], 320)
        self.assertEqual(es3.record_fields(rec)["height"], 224)


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)
        self.requests = []
        self.stop = threading.Event()
        self.ignore_status = False
        self.bad_identity = False
        self.video = False
        self.map = False
        self.worker = threading.Thread(target=self.board, daemon=True)
        self.worker.start()

    def tearDown(self):
        self.stop.set()
        self.worker.join(1)
        if self.master is not None:
            os.close(self.master)
        os.close(self.slave)

    def board(self):
        parser = es3.Parser()
        while not self.stop.is_set():
            readable, _, _ = select.select([self.master], [], [], 0.02)
            if not readable:
                continue
            for seq, kind, body in parser.feed(os.read(self.master, 256)):
                self.requests.append((kind, body))
                if kind == es3.STATUS and self.ignore_status:
                    continue
                if kind == es3.HELLO:
                    response = b"OTHER" if self.bad_identity else b"ES3C28P"
                elif kind == es3.STATUS:
                    response = struct.pack("<IBBBBB", 1234, 2, 180, 1, 1, 0)
                elif kind in (es3.CAPABILITIES, es3.START_VIDEO, es3.STOP_VIDEO) and not self.video:
                    # Older firmware: unknown command, session stays usable.
                    os.write(self.master, es3.encode(seq, es3.ERROR, bytes((es3.ERR_BAD_COMMAND,))))
                    continue
                elif kind == es3.CAPABILITIES:
                    response = es3.capabilities_payload(**CAPS)
                elif kind in (es3.CMD_MAP_CAPABILITIES, es3.CMD_START_MAP, es3.CMD_STOP_MAP) and not self.map:
                    os.write(self.master, es3.encode(seq, es3.ERROR, bytes((es3.ERR_BAD_COMMAND,))))
                    continue
                elif kind == es3.CMD_MAP_CAPABILITIES:
                    response = es3.map_capabilities_payload(BOARD_CAPS, page_count=6, map_page=5,
                        viewport_x=0, viewport_y=0, viewport_w=320, viewport_h=224,
                        max_jpeg_bytes=65536)
                elif kind == es3.CMD_START_MAP:
                    m = es3.parse_map_capabilities(es3.map_capabilities_payload(BOARD_CAPS, **MAP_CAPS), BOARD_CAPS)
                    es3.parse_start_map(body, m, BOARD_CAPS)
                    response = es3.start_map_reply_payload(TOKEN, 1)
                elif kind == es3.CMD_STOP_MAP:
                    es3.parse_stop_map(body)
                    response = b""
                elif kind == es3.START_VIDEO:
                    es3.parse_start_video(body, es3.parse_capabilities(es3.capabilities_payload(**CAPS)))
                    # A device->host record may precede the reply on the same stream.
                    os.write(self.master, es3.encode_visibility(TOKEN, es3.DEMAND_ACTIVE))
                    response = es3.start_video_reply_payload(TOKEN, 1)
                else:
                    response = b""
                # Send stale sequence then fragmented matching reply.
                wire = es3.encode((seq + 1) % 256, kind | es3.REPLY, response)
                wire += es3.encode(seq, kind | es3.REPLY, response)
                for offset in range(0, len(wire), 3):
                    os.write(self.master, wire[offset:offset + 3])
                    time.sleep(0.001)

    def test_real_tty_roundtrip_and_reopen(self):
        with es3.Es3Display(self.device, timeout=0.5) as display:
            self.assertEqual(display.status(), dict(uptime_ms=1234, page=2, brightness=180,
                                                    sd=True, calibrated=True, audio_playing=False))
            display.set_page(0, "LIVE", ["one", "two", "three", "four"])
            display.alert(25)
            display.stop_audio()
            display.ping()
        with es3.Es3Display(self.device, timeout=0.5) as display:
            display.ping()
        self.assertEqual([kind for kind, _ in self.requests].count(es3.HELLO), 2)
        self.assertIn((es3.ALERT, bytes((25,))), self.requests)

    def test_map_page_requires_capability(self):
        self.video = True
        with es3.Es3Display(self.device, timeout=0.5) as display:
            with self.assertRaises(ValueError):
                display.set_page(5, "MAP", ["", "", "", ""])
            self.assertNotIn(es3.PAGE, [kind for kind, _ in self.requests])
            display.ping()
            self.map = True
            display.set_page(5, "MAP", ["", "", "", ""])
            m = display.map_capabilities()
            reply = display.start_map(m, BOARD_CAPS, **START_MAP_REQUEST)
            self.assertEqual(reply["session_token"], TOKEN)
            display.stop_map(TOKEN)
            with self.assertRaises(ValueError):
                display.set_page(6, "BAD", ["", "", "", ""])
        pages = [body for kind, body in self.requests if kind == es3.PAGE]
        self.assertEqual(len(pages), 1)
        self.assertEqual(pages[0][0], 5)

    def test_timeout_closes_without_replaying_command(self):
        with es3.Es3Display(self.device, timeout=0.1) as display:
            self.ignore_status = True
            with self.assertRaises(TimeoutError):
                display.status()
            self.assertIsNone(display.fd)
        self.assertEqual([kind for kind, _ in self.requests].count(es3.STATUS), 1)

    def test_disconnect_closes_port(self):
        with es3.Es3Display(self.device, timeout=0.2) as display:
            self.stop.set()
            self.worker.join(1)
            os.close(self.master)  # PTY hangup models the USB tty disappearing
            self.master = None
            with self.assertRaises(OSError):
                display.status()
            self.assertIsNone(display.fd)

    def test_video_unsupported_keeps_controls(self):
        with es3.Es3Display(self.device, timeout=0.5) as display:
            self.assertIsNone(display.capabilities())
            display.ping()
        self.assertEqual([kind for kind, _ in self.requests].count(es3.CAPABILITIES), 1)

    def test_video_negotiation(self):
        self.video = True
        with es3.Es3Display(self.device, timeout=0.5) as display:
            caps = display.capabilities()
            self.assertEqual(caps["max_chunk_bytes"], 1024)
            session = display.start_video(caps, **START_VIDEO_REQUEST)
            self.assertEqual(session, dict(session_token=TOKEN, initial_grant=1))
            self.assertEqual(list(display.video_records), [es3.decode_record(es3.encode_visibility(TOKEN, 0))])
            display.stop_video(session["session_token"])
            with self.assertRaises(ValueError):
                display.start_video(caps, **{**START_VIDEO_REQUEST, "viewport_w": 0})
        self.assertIn((es3.STOP_VIDEO, struct.pack("<I", TOKEN)), self.requests)

    def test_wrong_identity_rejected(self):
        self.bad_identity = True
        display = es3.Es3Display(self.device, timeout=0.5)
        with self.assertRaises(es3.RemoteError):
            display.open()
        self.assertIsNone(display.fd)


# --- Bench playback (`play`) --------------------------------------------------

def make_jpeg(width=320, height=180, sof=0xC0, components=3, precision=8, entropy=None, eoi=True):
    """Structurally valid JPEG headers around arbitrary "entropy-coded" bytes.
    Nothing in these tests decodes pixels. The default payload holds FF 00
    stuffing plus E3 28 / E3 29 framing bytes that must stay plain CHUNK data."""
    def segment(marker, body):
        return bytes((0xFF, marker)) + struct.pack(">H", len(body) + 2) + body

    sampling = [0x22] + [0x11] * (components - 1)
    sof_body = struct.pack(">BHHB", precision, height, width, components) + b"".join(
        bytes((i + 1, sampling[i], min(i, 1))) for i in range(components))
    sos_body = bytes((components,)) + b"".join(bytes((i + 1, 0)) for i in range(components)) + b"\x00\x3f\x00"
    if entropy is None:
        entropy = b"\x12\xff\x00\xe3\x29\x01\x02\xe3\x28\x01" * 20
    return (b"\xff\xd8" + segment(0xE0, b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00")
            + segment(0xDB, b"\x00" + bytes(range(1, 65))) + segment(sof, sof_body)
            + segment(0xC4, b"\x00" + bytes([1] + [0] * 15) + b"\x00") + segment(0xDA, sos_body)
            + entropy + (b"\xff\xd9" if eoi else b""))


def big_jpeg():
    return make_jpeg(entropy=bytes(range(256)) * 230)   # ~58 KiB: many records per frame


class JpegFileTests(unittest.TestCase):
    def test_baseline_sof_dimensions(self):
        self.assertEqual(es3.jpeg_dimensions(make_jpeg()), (320, 180))
        self.assertEqual(es3.jpeg_dimensions(make_jpeg(160, 120, components=1)), (160, 120))
        self.assertEqual(es3.jpeg_dimensions(big_jpeg()), (320, 180))   # entropy data is never scanned

    def test_rejected_files(self):
        good = make_jpeg()
        cases = {
            "progressive": make_jpeg(sof=0xC2),
            "SOF0": make_jpeg(sof=0xC1),                          # extended sequential
            "8-bit": make_jpeg(precision=12),
            "components": make_jpeg(components=2),
            "zero dimension": make_jpeg(width=0),
            "EOI": good[:-10],                                    # cut inside the entropy data
            "runs past the end": good[:40] + b"\xff\xd9",         # cut inside the DQT segment
            "SOI": b"GIF89a" + good,
            "SOS before SOF": good[:2] + good[good.index(b"\xff\xda"):],
        }
        for message, data in cases.items():
            with self.subTest(message), self.assertRaisesRegex(ValueError, message):
                es3.jpeg_dimensions(data)

    def test_load_jpegs_directory_order_and_checks(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "b.jpg").write_bytes(make_jpeg(entropy=b"\x02" * 50))
            Path(folder, "a.JPG").write_bytes(make_jpeg(entropy=b"\x01" * 50))
            Path(folder, "notes.txt").write_text("not a frame")
            frames, size = es3.load_jpegs([folder])
            self.assertEqual(size, (320, 180))
            self.assertEqual([Path(name).name for name, _, _ in frames], ["a.JPG", "b.jpg"])
            self.assertEqual(frames[0][2], es3.crc32(frames[0][1]))
            Path(folder, "c.jpg").write_bytes(make_jpeg(160, 120))
            with self.assertRaisesRegex(ValueError, "c.jpg: 160x120, but the first frame is 320x180"):
                es3.load_jpegs([folder])
            Path(folder, "c.jpg").write_bytes(make_jpeg(entropy=bytes(es3.MAX_JPEG_BYTES)))
            with self.assertRaisesRegex(ValueError, "c.jpg: larger than"):
                es3.load_jpegs([folder])
            Path(folder, "c.jpg").write_bytes(make_jpeg(sof=0xC2))
            with self.assertRaisesRegex(ValueError, "c.jpg: progressive"):
                es3.load_jpegs([folder])
            os.mkdir(Path(folder, "empty"))
            with self.assertRaisesRegex(ValueError, "no .jpg"):
                es3.load_jpegs([str(Path(folder, "empty"))])

    def test_centred_viewport(self):
        caps = es3.parse_capabilities(es3.capabilities_payload(**CAPS))
        for size, expected in (((320, 180), (0, 30)), ((320, 240), (0, 0)), ((160, 120), (80, 60)),
                               ((319, 179), (0, 30))):
            self.assertEqual(es3.centred_viewport(caps, *size),
                             dict(viewport_x=expected[0], viewport_y=expected[1],
                                  viewport_w=size[0], viewport_h=size[1]))

    def test_play_arguments_are_bounded(self):
        for bad in (["--fps", "0"], ["--fps", "31"], ["--chunk", "4097"], ["--frames-in-flight", "5"],
                    ["--duration", "nan"], ["--duration", "0"]):
            argv = ["es3_display.py", "--device", "/nonexistent", "play", "frames", *bad]
            with self.subTest(bad), mock.patch.object(sys, "argv", argv), \
                    contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                es3.main()


class FakeDevice:
    """In-process ES3C28P stand-in on a pty master. It answers the E3 28
    controls and assembles v1 video as the firmware does: VISIBILITY after
    the START_VIDEO reply; per frame RESULT Ok at END, one cumulative CREDIT,
    then PRESENTED (RESULT Hidden instead while not active). Every host
    protocol violation it can see goes to .violations. hooks[record type]
    (device, record) runs in the device thread before a record is handled;
    hooks["start_video"](device) after a session starts. Device->host bytes
    go through a timed queue so tests can delay credits and presentations."""

    def __init__(self, master, caps=None, video=True, credit_delay=0.0, present_delay=0.0,
                 duplicate_credits=False, ignore_pings=0, read_size=65536, read_pause=0.0):
        self.master = master
        os.set_blocking(master, False)
        self.caps = dict(CAPS, **(caps or {}))
        self.caps_fields = es3.parse_capabilities(es3.capabilities_payload(**self.caps))
        self.video, self.duplicate_credits, self.ignore_pings = video, duplicate_credits, ignore_pings
        self.credit_delay, self.present_delay = credit_delay, present_delay
        self.read_size, self.read_pause = read_size, read_pause
        self.demand = es3.DEMAND_ACTIVE
        self.parser = es3.MuxParser(es3.MAX_CHUNK_DATA)
        self.controls, self.records, self.violations = [], [], []
        self.starts, self.frames, self.tokens, self.stop_tokens = [], [], [], []
        self.session, self.hooks = None, {}
        self.token = self.grant = self.started = self.last_frame = 0
        self.frame = None                      # [frame id, declared bytes, received bytearray]
        self.outstanding, self.max_outstanding, self.presented = set(), 0, 0
        self.queue, self.sequence = [], 0
        self.halt = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def close(self):
        self.halt.set()
        self.thread.join(2)

    def send(self, wire, delay=0.0):
        """Queue bytes, or a callable returning bytes/None at send time."""
        self.sequence += 1
        heapq.heappush(self.queue, (time.monotonic() + delay, self.sequence, wire))

    def run(self):
        while not self.halt.is_set():
            while self.queue and self.queue[0][0] <= time.monotonic():
                wire = heapq.heappop(self.queue)[2]
                wire = wire() if callable(wire) else wire
                if wire:
                    self.write(wire)
            if not select.select([self.master], [], [], 0.002)[0]:
                continue
            try:
                data = os.read(self.master, self.read_size)
            except BlockingIOError:
                continue
            except OSError:
                return
            for kind, item in self.parser.feed(data):
                if kind == "control":
                    self.control(*item)
                else:
                    self.on_video(item)
            if self.read_pause:
                time.sleep(self.read_pause)   # a slow reader: the host's writes back up

    def write(self, wire):
        view, deadline = memoryview(wire), time.monotonic() + 1.0
        while view and time.monotonic() < deadline:
            try:
                view = view[os.write(self.master, view):]
            except BlockingIOError:
                select.select([], [self.master], [], 0.01)

    def end_session(self):
        self.token = self.grant = self.started = self.last_frame = 0
        self.frame = None
        self.outstanding.clear()

    def control(self, seq, kind, body):
        self.controls.append((kind, body))

        def reply(payload=b""):
            self.send(es3.encode(seq, kind | es3.REPLY, payload))

        if kind == es3.HELLO:
            self.end_session()
            reply(b"ES3C28P")
        elif kind == es3.PING:
            if self.ignore_pings:
                self.ignore_pings -= 1
            else:
                reply()
        elif kind in (es3.CAPABILITIES, es3.START_VIDEO, es3.STOP_VIDEO) and not self.video:
            self.send(es3.encode(seq, es3.ERROR, bytes((es3.ERR_BAD_COMMAND,))))
        elif kind == es3.CAPABILITIES:
            reply(es3.capabilities_payload(**self.caps))
        elif kind == es3.START_VIDEO:
            try:
                self.session = es3.parse_start_video(body, self.caps_fields)
            except ValueError as exc:
                self.violations.append(f"START_VIDEO: {exc}")
                self.send(es3.encode(seq, es3.ERROR, bytes((es3.ERR_BAD_PAYLOAD,))))
                return
            self.end_session()
            self.tokens.append(0x5E550000 + len(self.tokens) + 1)
            self.token, self.grant = self.tokens[-1], self.caps["max_credits"]
            reply(es3.start_video_reply_payload(self.token, self.grant))
            self.send(es3.encode_visibility(self.token, self.demand))
            if "start_video" in self.hooks:
                self.hooks["start_video"](self)
        elif kind == es3.STOP_VIDEO:
            token = es3.parse_stop_video(body)
            self.stop_tokens.append(token)
            if token == self.token:
                self.end_session()
            reply()
        else:
            reply()

    def on_video(self, r):
        self.records.append(r)
        if r.token != self.token or es3.check_record(*r):
            self.violations.append(f"record {r.type:#x}/{r.frame_id} token {r.token:#x} (session {self.token:#x})")
            return
        if r.type in self.hooks:
            self.hooks[r.type](self, r)
        f = es3.record_fields(r)
        if r.type == es3.REC_START:
            self.starts.append((r.frame_id, time.monotonic()))
            if self.frame is not None:
                self.violations.append(f"START {r.frame_id} while frame {self.frame[0]} is incomplete")
            if r.frame_id <= self.last_frame:
                self.violations.append(f"START {r.frame_id} after frame {self.last_frame}")
            if (f["width"], f["height"]) != (self.session["viewport_w"], self.session["viewport_h"]):
                self.violations.append(f"START {r.frame_id} is {f['width']}x{f['height']}")
            self.last_frame, self.started = r.frame_id, self.started + 1
            if self.started > self.grant:
                self.violations.append(f"START {r.frame_id} beyond the grant {self.grant}")
                self.started -= 1
                self.send(es3.encode_result(self.token, r.frame_id, es3.REASON["NoCredit"]))
                return
            self.outstanding.add(r.frame_id)
            self.max_outstanding = max(self.max_outstanding, len(self.outstanding))
            if self.demand != es3.DEMAND_ACTIVE:
                self.drop(r.frame_id, "Hidden")
                return
            self.frame = [r.frame_id, f["jpeg_bytes"], bytearray()]
        elif self.frame is None or r.frame_id != self.frame[0]:
            return   # as on the board: records of a dropped or unknown frame are ignored
        elif r.type == es3.REC_CHUNK:
            if f["offset"] != len(self.frame[2]) or len(f["data"]) > self.session["chunk_bytes"]:
                self.violations.append(f"CHUNK {r.frame_id} at {f['offset']} with {len(f['data'])} B")
            self.frame[2] += f["data"]
        elif r.type == es3.REC_END:
            frame_id, size, data = self.frame
            self.frame = None
            if len(data) != size or es3.crc32(data) != f["crc32"]:
                self.violations.append(f"frame {frame_id} assembled wrong")
                self.drop(frame_id, "BadCrc")
                return
            self.frames.append(bytes(data))
            self.send(es3.encode_result(self.token, frame_id, es3.REASON["Ok"]))
            self.credit()
            if self.demand == es3.DEMAND_ACTIVE:
                self.present(frame_id)
            else:
                self.drop(frame_id, "Hidden", credit=False)
        elif r.type == es3.REC_ABORT:
            frame_id, self.frame = self.frame[0], None
            self.drop(frame_id, es3.REASONS[f["reason"]])

    def drop(self, frame_id, reason, credit=True):
        token = self.token

        def result():
            self.outstanding.discard(frame_id)
            return es3.encode_result(token, frame_id, es3.REASON[reason])
        self.send(result)
        if credit:
            self.credit()

    def credit(self):
        token = self.token

        def grant():
            if token != self.token:
                return None
            self.grant += 1
            wire = es3.encode_credit(token, self.grant)
            return wire + wire if self.duplicate_credits else wire   # repeats must be harmless
        self.send(grant, self.credit_delay)

    def present(self, frame_id):
        token = self.token

        def presented():
            if token != self.token:
                return None
            self.outstanding.discard(frame_id)
            self.presented += 1
            return es3.encode_presented(token, frame_id, 1500, 2500)
        self.send(presented, self.present_delay)


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)
        self.out, self.log = io.StringIO(), io.StringIO()
        self.fake = self.player = None

    def tearDown(self):
        if self.fake is not None:
            self.fake.close()
        os.close(self.master)
        os.close(self.slave)

    def start_fake(self, **options):
        if self.fake is not None:
            self.fake.close()
        self.fake = FakeDevice(self.master, **options)
        return self.fake

    def play(self, frames=None, timeout=1.0, **options):
        frames, size = frames or es3.prepare_frames([("frame", make_jpeg())])
        self.player = es3.Player(frames, size, out=self.out, log=self.log, **options)
        result = self.player.play(self.device, timeout=timeout, reconnect_delay=0.05)
        self.fake.close()   # join the device thread before reading its state
        return result

    def kinds(self):
        return [kind for kind, _ in self.fake.controls]

    def test_negotiation_heartbeats_stats_and_stop_on_exit(self):
        fake = self.start_fake()
        data = [make_jpeg(entropy=bytes([i + 1]) * 300) for i in range(3)]
        frames = es3.prepare_frames([(f"f{i}", d) for i, d in enumerate(data)])
        self.assertEqual(self.play(frames, fps=30, loop=True, duration=1.3, heartbeat=0.2), 0)
        self.assertEqual(self.kinds()[:3], [es3.HELLO, es3.CAPABILITIES, es3.START_VIDEO])
        self.assertEqual(self.kinds()[-1], es3.STOP_VIDEO)
        self.assertEqual(fake.stop_tokens, fake.tokens)
        self.assertEqual(len(fake.tokens), 1)
        self.assertEqual(fake.session, dict(START_VIDEO_REQUEST, video_protocol_version=1, codec=0, target_fps=30))
        self.assertEqual(fake.violations, [])
        self.assertEqual(fake.frames[:6], data * 2)   # in order, looped, byte-exact
        self.assertGreaterEqual(len(fake.frames), 10)
        self.assertGreaterEqual(self.kinds().count(es3.PING), 4)
        # PINGs and STOP only between whole records: the board's parser never lost sync.
        self.assertEqual((fake.parser.crc_errors, fake.parser.header_errors, fake.parser.timeouts), (0, 0, 0))
        self.assertGreater(self.player.total.presented, 0)
        self.assertEqual(self.player.stale + self.player.malformed + self.player.unmatched, 0)
        self.assertRegex(self.out.getvalue(), r"ACTIVE +sent +[\d.]+ fps +presented +[\d.]+ fps +tx +[\d.]+ KiB/s"
                                              r" +latency p50 \d+ ms p95 \d+ ms +grant \d+ started \d+")
        self.assertIn("play: summary:", self.out.getvalue())
        self.assertIn("display active", self.log.getvalue())

    def test_heartbeats_never_split_a_record(self):
        # A slow reader keeps the host's writes backed up, so a PING is often
        # due while a record is half written; it must wait for the boundary.
        fake = self.start_fake(read_size=512, read_pause=0.002)
        frames = es3.prepare_frames([("big", big_jpeg())])
        self.play(frames, fps=30, loop=True, duration=1.0, heartbeat=0.03, timeout=2.0)
        self.assertGreaterEqual(self.kinds().count(es3.PING), 5)
        self.assertEqual((fake.parser.crc_errors, fake.parser.header_errors, fake.parser.timeouts), (0, 0, 0))
        self.assertEqual(fake.violations, [])
        self.assertGreaterEqual(len(fake.frames), 1)
        self.assertTrue(all(frame == frames[0][0][1] for frame in fake.frames))

    def test_cumulative_credit_window_is_never_exceeded(self):
        fake = self.start_fake(caps=dict(max_credits=1), credit_delay=0.06, duplicate_credits=True)
        self.play(fps=30, loop=True, duration=0.8)
        self.assertEqual(fake.violations, [])   # no START beyond the grant, despite doubled CREDITs
        self.assertGreaterEqual(len(fake.starts), 4)   # credits keep coming back...
        self.assertLessEqual(len(fake.starts), 15)     # ...and bound 30 fps to one frame per credit
        self.assertNotIn("NoCredit", self.player.total.drops)

    def test_frames_in_flight_cap(self):
        # Two credits returned at END, PRESENTED 120 ms later: only the host cap binds.
        for limit in (1, 2):
            with self.subTest(limit=limit):
                fake = self.start_fake(present_delay=0.12)
                self.play(fps=30, loop=True, duration=0.7, frames_in_flight=limit)
                self.assertEqual(fake.violations, [])
                self.assertEqual(fake.max_outstanding, limit)

    def test_hidden_pauses_sending_until_active(self):
        fake = self.start_fake(caps=dict(max_credits=1), credit_delay=0.01)
        marks = {}

        def on_end(device, record):
            if record.frame_id == 3 and not marks:
                device.demand, token = es3.DEMAND_HIDDEN, device.token

                def hide():
                    marks["hidden"] = time.monotonic()
                    return es3.encode_visibility(token, es3.DEMAND_HIDDEN)

                def show():
                    device.demand, marks["active"] = es3.DEMAND_ACTIVE, time.monotonic()
                    return es3.encode_visibility(token, es3.DEMAND_ACTIVE)
                device.send(hide)          # before frame 3's CREDIT, as on the board
                device.send(show, 0.35)
        fake.hooks[es3.REC_END] = on_end
        self.play(fps=30, loop=True, duration=1.0)
        self.assertEqual(fake.violations, [])
        self.assertEqual([f for f, t in fake.starts if marks["hidden"] <= t < marks["active"]], [])
        self.assertTrue(any(t >= marks["active"] for _, t in fake.starts))
        self.assertIn("display hidden; sending paused", self.log.getvalue())
        self.assertEqual(self.player.total.drops["Hidden"], 1)   # frame 3 was never shown

    def test_hidden_mid_frame_aborts_at_a_record_boundary(self):
        fake = self.start_fake(caps=dict(max_credits=1), read_size=512, read_pause=0.002)

        def on_chunk(device, record):
            if record.frame_id == 2 and es3.record_fields(record)["offset"] == 0:
                device.demand = es3.DEMAND_HIDDEN
                device.send(es3.encode_visibility(device.token, es3.DEMAND_HIDDEN))
        fake.hooks[es3.REC_CHUNK] = on_chunk
        frames = es3.prepare_frames([("big", big_jpeg())])
        self.play(frames, chunk=256, fps=30, loop=True, duration=1.2, timeout=2.0)
        self.assertEqual(fake.violations, [])
        aborts = [(r.frame_id, r.payload) for r in fake.records if r.type == es3.REC_ABORT]
        self.assertEqual(aborts, [(2, bytes((es3.REASON["Hidden"],)))])
        self.assertNotIn((es3.REC_END, 2), [(r.type, r.frame_id) for r in fake.records])
        self.assertEqual([f for f, _ in fake.starts], [1, 2])
        self.assertEqual((fake.parser.crc_errors, fake.parser.header_errors), (0, 0))

    def test_stale_token_records_are_ignored(self):
        fake = self.start_fake(caps=dict(max_credits=1), credit_delay=0.03)

        def stale(device):
            old = device.token ^ 0x00FF0000
            for wire in (es3.encode_credit(old, 50), es3.encode_visibility(old, es3.DEMAND_HIDDEN),
                         es3.encode_presented(old, 1, 1, 1), es3.encode_result(old, 1, es3.REASON["DecodeFailed"])):
                device.send(wire)
        fake.hooks["start_video"] = stale
        self.play(fps=30, loop=True, duration=0.6)
        self.assertEqual(self.player.stale, 4)
        self.assertEqual(fake.violations, [])           # the stale CREDIT 50 granted nothing
        self.assertGreaterEqual(len(fake.starts), 4)    # the stale VISIBILITY paused nothing
        self.assertNotIn("DecodeFailed", self.player.total.drops)
        self.assertLessEqual(self.player.total.presented, fake.presented)

    def test_stop_mid_frame_finishes_the_record_then_sends_stop(self):
        stop = threading.Event()
        fake = self.start_fake(caps=dict(max_chunk_bytes=4096), read_size=1024, read_pause=0.003)

        def on_chunk(device, record):
            if record.frame_id == 2:
                stop.set()   # Ctrl-C while frame 2 is being written
        fake.hooks[es3.REC_CHUNK] = on_chunk
        frames = es3.prepare_frames([("big", big_jpeg())])
        self.assertEqual(self.play(frames, chunk=4096, fps=30, loop=True, stop=stop, timeout=3.0), 0)
        self.assertEqual(fake.violations, [])
        self.assertEqual(fake.stop_tokens, fake.tokens)
        self.assertEqual(self.kinds()[-1], es3.STOP_VIDEO)
        self.assertEqual((fake.parser.crc_errors, fake.parser.header_errors, fake.parser.timeouts), (0, 0, 0))
        self.assertGreaterEqual(self.player.unresolved, 1)
        self.assertNotIn(es3.REC_END, [r.type for r in fake.records if r.frame_id == 2])

    def test_heartbeat_loss_reconnects_and_renegotiates(self):
        fake = self.start_fake(ignore_pings=1)
        self.assertEqual(self.play(fps=20, loop=True, duration=1.6, heartbeat=0.2, timeout=0.3), 0)
        self.assertEqual(self.kinds().count(es3.HELLO), 2)
        self.assertEqual(self.kinds().count(es3.START_VIDEO), 2)
        self.assertEqual(fake.stop_tokens, fake.tokens[1:])   # only the live session is stopped
        self.assertEqual(self.player.reconnects, 1)
        self.assertIn("reconnecting", self.log.getvalue())
        self.assertEqual(fake.violations, [])

    def test_video_unsupported_and_oversized_frames_are_refused(self):
        self.start_fake(video=False)
        with self.assertRaisesRegex(es3.RemoteError, "unsupported"):
            self.play(duration=0.5)
        self.assertEqual(self.kinds(), [es3.HELLO, es3.CAPABILITIES])
        self.start_fake(caps=dict(max_width=160, max_height=120))
        with self.assertRaisesRegex(ValueError, "frames are 320x180"):
            self.play(duration=0.5)
        self.assertNotIn(es3.START_VIDEO, self.kinds())
        self.start_fake()
        with self.assertRaisesRegex(ValueError, "--chunk 2048 exceeds the device maximum 1024"):
            self.play(duration=0.5, chunk=2048)
        self.assertNotIn(es3.START_VIDEO, self.kinds())

    def test_main_play_sigint_sends_stop_and_exits_zero(self):
        fake = self.start_fake()
        data = [make_jpeg(entropy=bytes([i + 1]) * 100) for i in range(3)]

        def on_end(device, record):
            if record.frame_id == 5:
                os.kill(os.getpid(), signal.SIGINT)   # a real Ctrl-C, delivered to the main thread
        fake.hooks[es3.REC_END] = on_end
        previous = signal.getsignal(signal.SIGINT)
        with tempfile.TemporaryDirectory() as folder:
            for i, d in enumerate(data):
                Path(folder, f"{i:03d}.jpg").write_bytes(d)
            argv = ["es3_display.py", "--device", self.device, "play", folder, "--fps", "30", "--loop",
                    "--duration", "10"]
            out, err = io.StringIO(), io.StringIO()
            with mock.patch.object(sys, "argv", argv), contextlib.redirect_stdout(out), \
                    contextlib.redirect_stderr(err):
                code = es3.main()
        fake.close()
        self.assertEqual(code, 0, err.getvalue())
        self.assertIs(signal.getsignal(signal.SIGINT), previous)
        self.assertEqual(fake.stop_tokens, fake.tokens)
        self.assertEqual(fake.frames[:3], data)
        self.assertEqual(fake.violations, [])
        self.assertIn("play: summary:", out.getvalue())


if __name__ == "__main__":
    unittest.main()
