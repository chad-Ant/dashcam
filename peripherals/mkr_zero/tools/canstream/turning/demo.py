#!/usr/bin/env python3
"""Passenger-operated turning demo; only reads existing candump files.

run --input CAN_LOG --out NEW_DIR [--follow] [--serve 8765]
fit --samples samples.csv --references refs.json --out NEW_DIR
synthetic --out NEW_DIR
"""
import argparse
import collections
import csv
import dataclasses
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import signal
import threading
import time

from calibrate import fit
from model import Decoder, Profile, parse_line
from dr_binding import DeadReckoning
from fusion import FusionConfig, FIELDS as FUSION_FIELDS
from fusion_input import FusionSession, TelemetryInput

HERE = Path(__file__).resolve().parent
FIELDS = ["time", "segment", "gear_raw", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr", "status",
          "speed_kmh", "corrected_diff", "yaw_raw_dps", "yaw_dps", "curvature_per_m", "angle_deg",
          "radius_m", "direction", "dr_x_m", "dr_y_m", "dr_heading_deg", "dr_distance_m",
          "dr_status", "dr_continuous", "dr_limited"] + FUSION_FIELDS


def write_json(path, data):
    with path.open("x") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def load_profile(path):
    if not path:
        return Profile()
    with Path(path).open() as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("profile must be a JSON object")
    return Profile(**data.get("profile", data))


class Display:
    def __init__(self, live, synthetic):
        self.lock = threading.Lock()
        self.latest = {"status": "waiting for wheel data"}
        self.updated = None
        self.history = collections.deque(maxlen=300)
        self.next_history = 0
        self.state = "running"
        self.errors = self.frames = self.samples = 0
        self.live, self.synthetic = live, synthetic

    def publish(self, row):
        with self.lock:
            self.latest, self.updated = dict(row), time.monotonic()
            self.samples += 1
            if row["time"] >= self.next_history or row["time"] < self.next_history - 1:
                self.history.append(dict(row))
                self.next_history = row["time"] + .1

    def snapshot(self):
        with self.lock:
            row = dict(self.latest)
            stale = self.updated is None or time.monotonic() - self.updated > 1
            lag = time.time() - row["time"] if self.live and "time" in row else None
            if stale or (lag is not None and not -0.25 <= lag <= 5):
                row = {"status": "stale / delayed source — estimates hidden"}
            return {"latest": row, "history": list(self.history), "state": self.state,
                    "live": self.live, "synthetic": self.synthetic, "lag_s": lag,
                    "frames": self.frames, "samples": self.samples, "errors": self.errors}


def start_server(port, display):
    page = (HERE / "index.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(1)

        def do_GET(self):
            if self.path == "/":
                content, mime = page, "text/html; charset=utf-8"
            elif self.path == "/state":
                content, mime = json.dumps(display.snapshot(), allow_nan=False).encode(), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    # Local only, read-only endpoints; use SSH forwarding for the passenger.
    server = HTTPServer(("127.0.0.1", port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def lines(path, follow, stop):
    """Bounded partial records; refuse replacement/truncation, never replay it silently."""
    with path.open("rb") as stream:
        identity = path.stat().st_ino
        partial = b""
        if follow:
            # Start at EOF, but discard a pre-existing incomplete tail first.
            stream.seek(0, 2)
            end = stream.tell()
            discard = False
            if end:
                stream.seek(end - 1)
                discard = stream.read(1) != b"\n"
        else:
            discard = False
        while not stop.is_set():
            block = stream.readline(4097)
            if not block:
                if not follow:
                    if partial or discard:
                        raise ValueError("incomplete last input line")
                    return
                stat = path.stat()
                if stat.st_ino != identity or stat.st_size < stream.tell():
                    raise ValueError("input replaced/truncated; start a new demo session")
                stop.wait(.05)
                continue
            partial += block
            if len(partial) > 4096:
                raise ValueError("input line exceeds 4096 bytes")
            if not partial.endswith(b"\n"):
                continue
            if not discard:
                yield partial.decode("ascii")
            partial, discard = b"", False


def run(args):
    path = Path(args.input).resolve(strict=True)
    profile = load_profile(args.profile)
    dr = None
    telemetry = fusion = None
    fusion_config = None
    if args.telemetry:
        with Path(args.fusion_config).open() as stream:
            fusion_config = FusionConfig(**json.load(stream))
    if not path.is_file():
        raise ValueError("input must be a regular candump log")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    synthetic = (path.parent / "synthetic_truth.json").exists()
    write_json(out / "session.json", {"input": str(path), "follow": args.follow, "synthetic": synthetic,
               "created_epoch": time.time(), "profile": dataclasses.asdict(profile),
               "profile_status": "unvalidated defaults" if not args.profile else "user-selected candidate",
               "interface": args.interface, "dr_library": args.dr_library,
               "telemetry": str(Path(args.telemetry).resolve()) if args.telemetry else None,
               "fusion_config": dataclasses.asdict(fusion_config) if fusion_config else None,
               "start_time": args.start_time, "end_time": args.end_time,
               "geometry": "rear axle centre; equivalent bicycle front angle",
               "note": "no ECU angle/yaw sensor; logger delays live data; inspect can_sync/stats before fitting"})
    display, decoder = Display(args.follow, synthetic), Decoder(profile)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    timer = threading.Timer(args.seconds, stop.set)
    timer.daemon = True
    timer.start()
    server = start_server(args.serve, display) if args.serve else None
    print("Read-only", "LIVE (delayed)" if args.follow else "REPLAY", path, flush=True)
    if server:
        print("Passenger display: http://127.0.0.1:%d" % args.serve, flush=True)
    previous, deadline = None, time.monotonic()
    last_flush = last_print = 0.0
    success = False
    try:
        if fusion_config:
            telemetry = TelemetryInput(Path(args.telemetry).resolve(strict=True), fusion_config, args.follow)
            fusion = FusionSession(profile, fusion_config, telemetry)
        if args.dr_library:
            dr = DeadReckoning(Path(args.dr_library).resolve(strict=True), profile)
        with (out / "samples.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, FIELDS)
            writer.writeheader()
            for line in lines(path, args.follow, stop):
                try:
                    frame = parse_line(line, args.interface)
                    if frame is None:
                        continue
                    stamp, ident, data = frame
                    if args.start_time is not None and stamp < args.start_time:
                        decoder.feed(stamp, ident, data)  # warm gear/decode, not the local pose
                        continue
                    if args.end_time is not None and stamp > args.end_time:
                        break
                    if previous is None:
                        deadline = time.monotonic()
                    if not args.follow and args.speed > 0 and previous is not None:
                        # Large discontinuities create a new segment, not a long wait.
                        dt = stamp - previous
                        deadline += dt / args.speed if 0 < dt <= 1 else 0
                        if stop.wait(max(0, deadline - time.monotonic())):
                            break
                    previous = stamp
                    row = decoder.feed(stamp, ident, data)
                except ValueError:
                    display.errors += 1
                    decoder.segment += 1
                    decoder.reset()
                    display.publish({"status": "malformed input — state reset", "time": previous or 0})
                    continue
                display.frames += 1
                if row:
                    if fusion:
                        row.update(fusion.update(row))
                    if dr:
                        row.update(dr.update(row))
                    writer.writerow(row)
                    display.publish(row)
                now = time.monotonic()
                if now - last_flush >= 1:
                    stream.flush()  # fail visibly; no unbounded writer queue
                    last_flush = now
                if now - last_print >= 2:
                    print(json.dumps(display.snapshot()["latest"], allow_nan=False), flush=True)
                    last_print = now
            stream.flush()
        display.state = "stopped" if stop.is_set() else "replay complete"
        success = True
        if server and not args.follow:
            print("Replay complete; display stays open until timeout or Ctrl-C", flush=True)
            stop.wait()
    finally:
        timer.cancel()
        if dr:
            dr.close()
        if telemetry:
            telemetry.close()
        if server:
            server.shutdown()
            server.server_close()
        write_json(out / "result.json", {"complete": success, "frames": display.frames,
                   "samples": display.samples, "malformed": display.errors, "state": display.state})
    return 3 if display.errors else 0


def fit_command(args):
    initial = load_profile(args.profile)
    with Path(args.samples).open() as stream:
        # Calibration sessions only: refuse unlimited history in memory.
        rows = []
        for row in csv.DictReader(stream):
            rows.append(row)
            if len(rows) > 500000:
                raise ValueError("more than 500,000 samples; select a shorter calibration session")
    with Path(args.references).open() as stream:
        document = json.load(stream)
    if document.get("clock_and_loss_checked") is not True:
        raise ValueError("check source can_sync.csv/can_stats.csv and set clock_and_loss_checked=true")
    candidate, report = fit(rows, document["runs"], initial)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    provenance = {}
    for label, name in (("samples", args.samples), ("references", args.references)):
        digest = hashlib.sha256()
        with Path(name).open("rb") as stream:
            for chunk in iter(lambda: stream.read(65536), b""):
                digest.update(chunk)
        provenance[label] = {"path": str(Path(name).resolve()), "sha256": digest.hexdigest()}
    write_json(out / "candidate.json", {"profile": dataclasses.asdict(candidate), "provenance": provenance,
                                        "status": report["status"]})
    write_json(out / "report.json", report)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


def pack_fields(fields):
    data = bytearray(8)
    for start, length, value in fields:
        bit = start
        for shift in range(length - 1, -1, -1):
            data[bit // 8] |= ((value >> shift) & 1) << (bit % 8)
            bit = bit + 15 if bit % 8 == 0 else bit - 1
    return data.hex().upper()


def synthetic_command(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    truth = Profile(mismatch_ratio=.012, wheel_kmh_per_count=.0101, yaw_dps_per_count=.1106)
    refs, tick = [], 0
    epoch, hz = 1800000000, 50
    with (out / "can_raw.log").open("x") as stream:
        for split, speed, radius in (("fit", 7, 10), ("validation", 9, 15)):
            for kind in ("straight", "left", "right"):
                duration = 15 if kind == "straight" else 2 * math.pi * radius / (speed / 3.6)
                n = round(duration * hz)
                duration = n / hz
                # Fit round-count integrals to an exact lap despite time quantisation.
                yaw = 0 if kind == "straight" else (1 if kind == "left" else -1) * 360 / duration
                actual_speed = speed if kind == "straight" else 2 * math.pi * radius / duration * 3.6
                start = epoch + tick / hz
                mean = actual_speed / truth.wheel_kmh_per_count
                diff = yaw / truth.yaw_dps_per_count + truth.mismatch_ratio * mean
                wheels = [round(mean), round(mean), round(mean - diff / 2), round(mean + diff / 2)]
                for _ in range(n + 1):
                    t = epoch + tick / hz
                    stream.write("(%.6f) can0 191#%s\n" % (t, pack_fields([(44, 5, 4)])))
                    stream.write("(%.6f) can0 1D0#%s\n" % (t, pack_fields(list(zip((7, 8, 25, 42), (15,) * 4, wheels)))))
                    tick += 1
                ref = {"name": split + "_" + kind, "split": split, "kind": kind,
                       "segment": 0, "start": start, "end": epoch + (tick - 1) / hz}
                if kind == "straight":
                    ref["distance_m"] = speed / 3.6 * duration
                else:
                    ref.update(radius_m=radius, laps=1)
                refs.append(ref)
    write_json(out / "synthetic_truth.json", {"SYNTHETIC_NOT_HARDWARE": True, "profile": dataclasses.asdict(truth)})
    write_json(out / "references.json", {"clock_and_loss_checked": True, "runs": refs})
    print("SYNTHETIC data only:", out)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run_p = commands.add_parser("run")
    run_p.add_argument("--input", required=True)
    run_p.add_argument("--out", required=True)
    run_p.add_argument("--profile")
    run_p.add_argument("--dr-library", help="optional absolute path to built libdeadreckoning.so")
    run_p.add_argument("--telemetry", help="v0.4 telemetry CSV from same session; enables EKF")
    run_p.add_argument("--fusion-config", help="required EKF mounting/geometry/noise JSON")
    run_p.add_argument("--start-time", type=float, help="replay window start (source epoch seconds)")
    run_p.add_argument("--end-time", type=float, help="replay window end (source epoch seconds)")
    run_p.add_argument("--follow", action="store_true", help="follow NEW lines only, from EOF")
    run_p.add_argument("--interface", default="can0")
    run_p.add_argument("--serve", type=int, default=0, help="localhost HTTP port, 0 disables")
    run_p.add_argument("--speed", type=float, default=1, help="replay speed; 0 = as fast as possible")
    run_p.add_argument("--seconds", type=float, default=7200, help="bounded session lifetime")
    fit_p = commands.add_parser("fit")
    fit_p.add_argument("--samples", required=True)
    fit_p.add_argument("--references", required=True)
    fit_p.add_argument("--profile")
    fit_p.add_argument("--out", required=True)
    synth_p = commands.add_parser("synthetic")
    synth_p.add_argument("--out", required=True)
    fusion_synth_p = commands.add_parser("synthetic-fusion")
    fusion_synth_p.add_argument("--out", required=True)
    args = parser.parse_args()
    try:
        if args.command == "run":
            if bool(args.telemetry) != bool(args.fusion_config):
                raise ValueError('--telemetry and --fusion-config must be supplied together')
            if not math.isfinite(args.speed) or not 0 <= args.speed <= 1000:
                raise ValueError("replay speed must be 0..1000")
            if not math.isfinite(args.seconds) or not 0 < args.seconds <= 86400:
                raise ValueError("seconds must be >0 and <=86400")
            if args.serve and not 1024 <= args.serve <= 65535:
                raise ValueError("serve port must be 1024..65535")
            if args.start_time is not None or args.end_time is not None:
                if args.follow or args.start_time is None or args.end_time is None or not (
                        math.isfinite(args.start_time) and math.isfinite(args.end_time) and
                        0 <= args.start_time < args.end_time):
                    raise ValueError("replay windows need finite start < end and cannot use --follow")
            return run(args)
        if args.command == "fit":
            return fit_command(args)
        if args.command == 'synthetic-fusion':
            from fusion_synthetic import generate
            return generate(args)
        return synthetic_command(args)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, "turning demo: %s\n" % exc)


if __name__ == "__main__":
    raise SystemExit(main())
