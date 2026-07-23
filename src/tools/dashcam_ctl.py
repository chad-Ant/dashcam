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
  dashcam_ctl.py <host> <port> [token]

  token  the shared secret matching <Network><ControlAuthToken> on the device.
         Omit it only when the device runs with auth disabled (empty token).
"""

import hashlib
import hmac
import select
import socket
import sys


def _read_line(sock, buf):
    """Return (line_str_or_None, remaining_buf); blocks until a newline arrives."""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            return None, buf
        buf += chunk
    line, _, rest = buf.partition(b"\n")
    return line.decode("utf-8", "replace").rstrip("\r"), rest


def _handshake(sock, token):
    """Answer an AUTH-CHALLENGE if the server issues one. Returns (ok, remaining_buf)."""
    line, buf = _read_line(sock, b"")
    if line is None:
        print("connection closed before greeting", file=sys.stderr)
        return False, buf

    if line.startswith("AUTH-CHALLENGE"):
        parts = line.split(None, 1)
        nonce = parts[1].strip() if len(parts) > 1 else ""
        if not token:
            print("server requires authentication but no token was given",
                  file=sys.stderr)
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
    if len(argv) < 3:
        print("usage: dashcam_ctl.py <host> <port> [token]", file=sys.stderr)
        return 2
    host = argv[1]
    try:
        port = int(argv[2])
    except ValueError:
        print("port must be an integer", file=sys.stderr)
        return 2
    token = argv[3] if len(argv) > 3 else ""

    try:
        sock = socket.create_connection((host, port), timeout=10)
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
