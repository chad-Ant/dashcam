#!/usr/bin/env python3
"""Convert a 24-bit uncompressed BMP to ES3C28P display-wire RGB565.

The output is headerless: exactly two bytes per pixel, top-to-bottom, with
the most significant RGB565 byte first. It needs only Python's standard
library and deliberately validates the ES3C28P splash dimensions by default.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path


def fail(message: str) -> None:
    raise SystemExit(f"error: {message}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="24-bit uncompressed BMP input")
    parser.add_argument("destination", type=Path, help=".rgb565 output")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=224)
    args = parser.parse_args()

    data = args.source.read_bytes()
    if len(data) < 54 or data[:2] != b"BM":
        fail("input is not a BMP")
    pixel_offset = struct.unpack_from("<I", data, 10)[0]
    dib_size = struct.unpack_from("<I", data, 14)[0]
    width, signed_height = struct.unpack_from("<ii", data, 18)
    planes, bits_per_pixel = struct.unpack_from("<HH", data, 26)
    compression = struct.unpack_from("<I", data, 30)[0]
    if dib_size < 40 or planes != 1 or bits_per_pixel != 24 or compression != 0:
        fail("input must be an uncompressed 24-bit BMP")
    height = abs(signed_height)
    if width <= 0 or height <= 0 or width != args.width or height != args.height:
        fail(f"expected {args.width}x{args.height}, got {width}x{height}")
    if pixel_offset < 14 + dib_size or pixel_offset > len(data):
        fail("BMP pixel data overlaps the header or starts outside the file")
    row_bytes = (width * 3 + 3) & ~3
    if pixel_offset + row_bytes * height > len(data):
        fail("BMP pixel data is truncated")

    output = bytearray(width * height * 2)
    out = 0
    # BMP is normally bottom-up; RGB565 assets are always top-to-bottom.
    for destination_y in range(height):
        source_y = height - 1 - destination_y if signed_height > 0 else destination_y
        row_start = pixel_offset + source_y * row_bytes
        for x in range(width):
            blue, green, red = data[row_start + x * 3:row_start + x * 3 + 3]
            rgb565 = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
            output[out] = rgb565 >> 8
            output[out + 1] = rgb565 & 0xFF
            out += 2
    args.destination.write_bytes(output)
    print(f"wrote {args.destination} ({len(output)} bytes, {width}x{height} RGB565 big-endian)")


if __name__ == "__main__":
    main()
