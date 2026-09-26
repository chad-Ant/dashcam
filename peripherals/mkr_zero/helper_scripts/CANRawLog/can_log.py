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
import glob, os, sys, termios, time

def main():
    out_path, secs = sys.argv[1], float(sys.argv[2])
    dev = glob.glob("/dev/serial/by-id/*Arduino*MKRZero*")[0]
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)

    buf = b""; t0 = time.time(); last_note = t0; lines = 0; last_stats = ""
    with open(out_path, "w") as out:
        out.write(f"# host start {t0:.3f} device {os.path.realpath(dev)}\n")
        while time.time() - t0 < secs:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                time.sleep(0.005); continue
            now = time.time()
            buf += chunk
            *complete, buf = buf.split(b"\n")
            for raw in complete:
                line = raw.decode("ascii", "replace").rstrip("\r")
                if line.startswith("S "):
                    last_stats = line
                    line += f" @{now:.3f}"
                out.write(line + "\n"); lines += 1
            if now - last_note >= 10:
                last_note = now
                out.flush()   # so a recording in progress can be analysed
                print(f"{now - t0:6.0f} s  {lines} lines  last stats: {last_stats}", flush=True)
    os.close(fd)
    print(f"done: {lines} lines in {time.time() - t0:.0f} s -> {out_path}", flush=True)

if __name__ == "__main__":
    main()
