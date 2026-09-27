#!/usr/bin/env python3
"""Record CANRawLog's stream from the MKR Zero's USB port to a file.

    can_log.py OUT_FILE SECONDS

Frame lines are written exactly as received. Each once-a-second stats line
('S us frames ovf rtr ext') is written with ' @<host epoch>' appended, which is
what ties the board's micros() timeline to wall-clock time — and so to the
dashcam footage, whose segment names and sidecar carry wall-clock time.

Needs the port exclusively and tty permission: on the Jetson run it inside the
dev container (docker run --privileged -v /dev:/dev ...).
"""
import errno, fcntl, glob, os, sys, termios, time

def main():
    out_path, secs = sys.argv[1], float(sys.argv[2])
    ports = glob.glob("/dev/serial/by-id/*Arduino*MKRZero*")
    if len(ports) != 1:
        hint = ("is it plugged in, and /dev mounted into the container?" if not ports
                else "unplug the others")
        sys.exit(f"can_log: expected exactly one MKR Zero under /dev/serial/by-id, "
                 f"found {len(ports)} - {hint}")
    dev = ports[0]
    hints = {errno.EACCES: "no tty permission - run inside the dev container",
             errno.ENOENT: "it went away - unplugged?",
             errno.EBUSY: "another program holds the port",
             errno.EWOULDBLOCK: "another program holds the port"}
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # A tty is not exclusive by itself: a second reader (a forgotten cat, a
        # serial monitor, another can_log) silently takes a share of the lines.
        # flock is what pyserial's exclusive=True takes; TIOCEXCL also stops a
        # non-root opener that does not look for the lock.
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        a = termios.tcgetattr(fd)
        a[0] = 0; a[1] = 0; a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        # Pinned, not inherited: a tool that left VMIN=0 behind (pyserial does)
        # makes an empty non-blocking read return b'' instead of EAGAIN, which
        # the EOF test below would take for a disconnect on a live port.
        a[6][termios.VMIN] = 1; a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
    except (OSError, termios.error) as e:
        code = e.errno if isinstance(e, OSError) else e.args[0]
        sys.exit(f"can_log: cannot set up {dev}: {e}" + (f" ({hints[code]})" if code in hints else ""))

    buf = b""; t0 = time.time(); m0 = time.monotonic(); last_note = m0; lines = 0; last_stats = ""
    lost = None
    with open(out_path, "w") as out:
        out.write(f"# host start {t0:.3f} device {os.path.realpath(dev)}\n")
        # Monotonic for the deadline: a clock step (NTP, the GPS time fix) must
        # not end the recording early. Wall-clock only for the @epoch stamps.
        while time.monotonic() - m0 < secs:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                time.sleep(0.005); continue
            except OSError as e:          # EIO once the device is gone
                lost = str(e); break
            if not chunk:                 # a hung-up tty reads EOF forever
                lost = "end of file (USB disconnected?)"; break
            now = time.time()
            buf += chunk
            *complete, buf = buf.split(b"\n")
            for raw in complete:
                line = raw.decode("ascii", "replace").rstrip("\r")
                if line.startswith("S "):
                    last_stats = line
                    line += f" @{now:.3f}"
                out.write(line + "\n"); lines += 1
            if time.monotonic() - last_note >= 10:
                last_note = time.monotonic()
                out.flush()   # so a recording in progress can be analysed
                print(f"{last_note - m0:6.0f} s  {lines} lines  last stats: {last_stats}", flush=True)
        if buf:               # a line cut off by the deadline or the disconnect
            out.write("# partial: " + buf.decode("ascii", "replace").rstrip("\r") + "\n")
        if lost:
            out.write(f"# port lost after {time.monotonic() - m0:.1f} s: {lost}\n")
    os.close(fd)
    elapsed = time.monotonic() - m0
    if lost:
        sys.exit(f"can_log: port lost after {elapsed:.0f} s ({lost}); "
                 f"{lines} lines kept in {out_path}")
    print(f"done: {lines} lines in {elapsed:.0f} s -> {out_path}", flush=True)

if __name__ == "__main__":
    main()
