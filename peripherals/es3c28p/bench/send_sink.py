#!/usr/bin/env python3
"""Explicit-path raw throughput sender; ONLY for usb_sink bench firmware."""
import argparse
import fcntl
import os
import select
import termios
import time
import tty

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('device', help='exact display tty/by-id path (never auto-probed)')
parser.add_argument('--seconds', type=float, default=10)
args = parser.parse_args()
if not 0 < args.seconds <= 3600:
    parser.error('--seconds must be in (0, 3600]')
fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    fcntl.ioctl(fd, termios.TIOCEXCL)
    tty.setraw(fd)
    block = bytes(range(256))*4
    offset = total = 0
    started = time.monotonic()
    deadline = started + args.seconds
    while time.monotonic() < deadline:
        _, writable, _ = select.select([], [fd], [], min(0.05, max(0, deadline-time.monotonic())))
        if writable:
            try:
                count = os.write(fd, block[offset:])
            except BlockingIOError:
                continue
            total += count
            offset = (offset+count) % len(block)
    print(f'host queued {total} bytes in {time.monotonic()-started:.3f}s; use UART sink rate as received throughput')
finally:
    os.close(fd)
