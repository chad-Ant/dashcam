#!/usr/bin/env python3
"""dashcam_ctl — authenticating client for the dashcam remote control + telemetry channel.

The device's ControlServer (libnetwork) protects the control channel with a
nonce + HMAC-SHA256 pre-shared-key handshake, so a bare `nc` cannot connect when a
token is configured (nc cannot compute the HMAC).  This client does the handshake,
then bridges the terminal to the channel:

  - socket -> stdout : prints replies (OK/ERR) and streaming telemetry JSON lines
  - stdin  -> socket : sends whatever you type as a command line

Commands (type HELP once connected for the authoritative list):
  RTP lane|driver <host|here> [port]   re-point a stream
  RTP lane|driver on|off               pause / resume a stream
  STATUS                               show destinations
  HELP                                 server help text

Usage:
  dashcam_ctl.py <host> <port> [--token-file PATH]

When authentication is enabled, the client prompts for the shared secret with
terminal echo disabled.  For non-interactive use, --token-file reads it from a
regular file that is owned by the current user (or root) and inaccessible to
group/other users.
"""

import argparse
import getpass
import hashlib
import hmac
import os
import select
import socket
import stat
import sys
import warnings


_MAX_TOKEN_BYTES = 65536


def _read_token_file(path):
    """Read a PSK without following symlinks or accepting permissive file modes."""
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(path, flags)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise ValueError("token file must be a regular file")
        if info.st_uid not in (os.geteuid(), 0):
            raise ValueError("token file must be owned by the current user or root")
        if info.st_mode & (stat.S_IRWXG | stat.S_IRWXO):
            raise ValueError("token file must not be accessible by group or other users")
        with os.fdopen(fd, "r", encoding="utf-8") as stream:
            fd = -1
            token = stream.read(_MAX_TOKEN_BYTES + 1)
    finally:
        if fd >= 0:
            os.close(fd)

    if len(token.encode("utf-8")) > _MAX_TOKEN_BYTES:
        raise ValueError("token file is too large")
    token = token.rstrip("\r\n")
    if not token:
        raise ValueError("token file is empty")
    if "\n" in token or "\r" in token:
        raise ValueError("token file must contain exactly one line")
    return token


def _prompt_token():
    """Prompt without permitting getpass's visible-input fallback."""
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", getpass.GetPassWarning)
            token = getpass.getpass("Control token: ")
    except (EOFError, getpass.GetPassWarning):
        print("cannot securely prompt for token; use --token-file",
              file=sys.stderr)
        return None
    except KeyboardInterrupt:
        print("\nauthentication cancelled", file=sys.stderr)
        return None
    if not token:
        print("control token must not be empty", file=sys.stderr)
        return None
    return token


def _read_line(sock, buf):
    """Return (line_str_or_None, remaining_buf); blocks until a newline arrives."""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            return None, buf
        buf += chunk
    line, _, rest = buf.partition(b"\n")
    return line.decode("utf-8", "replace").rstrip("\r"), rest


def _handshake(sock, token=None):
    """Answer an AUTH-CHALLENGE if the server issues one. Returns (ok, remaining_buf)."""
    line, buf = _read_line(sock, b"")
    if line is None:
        print("connection closed before greeting", file=sys.stderr)
        return False, buf

    if line.startswith("AUTH-CHALLENGE"):
        parts = line.split(None, 1)
        nonce = parts[1].strip() if len(parts) > 1 else ""
        if token is None:
            token = _prompt_token()
        if not token:
            return False, buf
        mac = hmac.new(token.encode("utf-8"), nonce.encode("utf-8"),
                       hashlib.sha256).hexdigest()
        sock.sendall(("AUTH " + mac + "\n").encode("utf-8"))
        # Expect AUTH-OK (+ greeting) or AUTH-FAIL.
        line, buf = _read_line(sock, buf)
        if line is None:
            print("connection closed during authentication", file=sys.stderr)
            return False, buf
        print(line)
        if line.startswith("AUTH-FAIL"):
            print("authentication failed (wrong token?)", file=sys.stderr)
            return False, buf
    else:
        # No auth configured on the device — the first line is the plain greeting.
        print(line)

    return True, buf


def main(argv):
    parser = argparse.ArgumentParser(
        prog="dashcam_ctl.py",
        description="Authenticated dashcam control and telemetry client")
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    parser.add_argument(
        "--token-file", metavar="PATH",
        help="read the control token from a private regular file")
    args = parser.parse_args(argv[1:])

    if not 1 <= args.port <= 65535:
        parser.error("port must be in the range 1..65535")

    token = None
    if args.token_file:
        try:
            token = _read_token_file(args.token_file)
        except (OSError, UnicodeError, ValueError) as exc:
            print("cannot read token file: %s" % exc, file=sys.stderr)
            return 2

    try:
        sock = socket.create_connection((args.host, args.port), timeout=10)
    except OSError as exc:
        print("connect failed: %s" % exc, file=sys.stderr)
        return 1
    sock.settimeout(None)

    ok, buf = _handshake(sock, token)
    if not ok:
        sock.close()
        return 1

    print("[connected — type commands, Ctrl-D / Ctrl-C to quit]", file=sys.stderr)
    try:
        while True:
            readable, _, _ = select.select([sock, sys.stdin], [], [])
            for src in readable:
                if src is sock:
                    chunk = sock.recv(4096)
                    if not chunk:
                        print("[connection closed by device]", file=sys.stderr)
                        return 0
                    buf += chunk
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        sys.stdout.write(line.decode("utf-8", "replace").rstrip("\r") + "\n")
                        sys.stdout.flush()
                else:
                    cmd = sys.stdin.readline()
                    if not cmd:            # EOF (Ctrl-D)
                        return 0
                    sock.sendall(cmd.encode("utf-8"))
    except KeyboardInterrupt:
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
