#!/usr/bin/env python3
"""lane_supervisor.py — the 1 Hz supervisor of a lane test run (plan P5b, section 12.4).

    python3 tools/lane_eval/lane_supervisor.py --run RUN_DIR --cams csi,usb [--can] [--expect-can]
                                               [--minutes N] [--max-cam-restarts 3]

Started (and stopped, SIGTERM) by lane_run.sh; runs on the HOST as the
jetson user (docker group), stdlib only.  Every second it:

  * restarts a lane_drive container that exited with a non-zero code into
    the same run directory (`lane_run.sh _start_cam RUN CAM INSTANCE MIN`:
    --instance K appends to the CSVs, review parts from 100*K, lanes on at
    once, the run's mount reference), at most --max-cam-restarts (10) times
    per camera, >= 20 s apart; a camera that exits 0 or 4 has finished;
  * restarts the MKR logger into can/partN when its container died, or when
    can_raw.log stopped growing for 10 s after it had been growing
    (`lane_run.sh _start_mkr RUN PART`);
  * runs the degrade ladder (each step once, an event; RUN/ctl/degrade):
    step up one level per 10 s while MemAvailable < 1.0 GB, tj > 92 C or the
    trailing-5-min rate > 22 GB/h; straight to 6 (stop the run, exit 0) when
    free disk < 5 GB or MemAvailable < 600 MB;
  * GPU saturation: a camera with lanes on below 0.8 x its target rate for
    30 s while tj < 85 C -> UGREEN lanes to 10 Hz (RUN/ctl/usb_lane_hz);
  * aggregates RUN/<cam>/status.json, can/live_state.json and the last
    sys/sysmon.csv row into RUN/status.json (atomic): overall GREEN / AMBER /
    RED, alerts, and four 23-character display lines for lane_status;
  * prints "LANES ON - drive" once every producer is healthy (both cameras at
    rate, review footage and tegrastats growing, CAN frames if expected).

Events go to RUN/events_host.csv (source supervisor).
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
LANE_RUN = os.path.join(HERE, "lane_run.sh")
stop = False


def on_signal(*_):
    global stop
    stop = True


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def last_csv_row(path):
    """dict of the last complete row of a CSV (header + tail read)."""
    try:
        with open(path, "rb") as f:
            header = f.readline().decode().strip().split(",")
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 8192))
            lines = f.read().decode(errors="replace").splitlines()
        for line in reversed(lines):
            cells = line.split(",")
            if len(cells) == len(header) and cells[0].isdigit():
                return dict(zip(header, cells))
    except OSError:
        pass
    return {}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def write_atomic(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


class Supervisor:
    def __init__(self, a):
        self.a = a
        self.run = a.run
        self.cams = [c for c in a.cams.split(",") if c]
        self.t0 = time.monotonic()
        self.end = self.t0 + a.minutes * 60 if a.minutes else None
        self.ev = open(os.path.join(self.run, "events_host.csv"), "a")
        os.makedirs(os.path.join(self.run, "ctl"), exist_ok=True)
        self.degrade = 0
        self.degrade_cond_since = None
        self.last_step = 0.0
        self.usb_hz = None
        self.slow_since = {}
        self.cam_restarts = {c: 0 for c in self.cams}
        self.cam_done = set()
        self.cam_last_restart = {}
        self.cam_finished = set()        # exited 0 / 4: its --minutes ran out (not a fault)
        self.mkr_part = 0
        self.mkr_dead_ticks = 0
        self.eff_hist = {}
        self.mkr_last_restart = 0.0
        self.can_size = None
        self.can_grew = False
        self.can_growth_t = time.monotonic()
        self.can_stale_after_restart = False
        self.review_size = {c: (0, time.monotonic()) for c in self.cams}
        self.teg_size = (0, time.monotonic())
        self.ready = False
        self.alerts_seen = set()
        self.last_alert = ""
        self.stopping = False

    def event(self, typ, detail=""):
        self.ev.write("%d,%d,supervisor,%s,%s\n" % (time.monotonic_ns(), time.time_ns(), typ, str(detail).replace(",", ";")))
        self.ev.flush()
        print("[supervisor %s] %s %s" % (time.strftime("%H:%M:%S"), typ, detail), flush=True)

    def containers(self):
        names = ["lane_" + c for c in self.cams] + (["lane_mkr"] if self.a.can else [])
        out = {}
        try:
            r = subprocess.run(["docker", "inspect", "-f", "{{.Name}} {{.State.Running}} {{.State.ExitCode}}"] + names,
                               capture_output=True, text=True, timeout=10)
            for line in r.stdout.splitlines():
                n, running, code = line.split()
                out[n.lstrip("/")] = (running == "true", int(code))
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        return out

    def active(self):
        """The run is still the current one (lane_run.sh stop removes .current first)."""
        try:
            with open(os.path.join(os.path.dirname(os.path.abspath(self.run)), ".current")) as f:
                return os.path.abspath(f.read().strip()) == os.path.abspath(self.run)
        except OSError:
            return False

    def lane_run(self, *args):
        if args and args[0] in ("_start_cam", "_start_mkr") and not self.active():
            self.event("restart_skipped", "%s: the run is stopping" % " ".join(str(a) for a in args[:3]))
            return
        subprocess.Popen([LANE_RUN] + [str(x) for x in args], stdout=open(os.path.join(self.run, "sys", "supervisor_cmd.log"), "a"),
                         stderr=subprocess.STDOUT, start_new_session=True)

    def tick(self):
        now = time.monotonic()
        ctr = self.containers()
        if not ctr:
            # docker inspect failed or timed out: unknown, not dead (2026-10-26:
            # one empty answer restarted a healthy logger).
            ctr = None
        alerts = []      # (level, text)

        # ── camera containers ────────────────────────────────────────────────
        cam_st = {}
        for c in self.cams:
            st = read_json(os.path.join(self.run, c, "status.json")) or {}
            cam_st[c] = st
            running, code = ctr.get("lane_" + c, (False, None)) if ctr is not None else (True, None)
            if not running and c not in self.cam_done and code is not None and now - self.t0 > 20:
                if code == 0 or code == 4 or (self.end and now >= self.end - 5):
                    self.cam_done.add(c)
                    self.cam_finished.add(c)
                    self.event("cam_finished", "%s exit %s" % (c, code))
                elif self.cam_restarts[c] < self.a.max_cam_restarts and now - self.cam_last_restart.get(c, 0) < 20:
                    pass                               # space restarts >= 20 s apart
                elif self.cam_restarts[c] < self.a.max_cam_restarts:
                    self.cam_last_restart[c] = now
                    self.cam_restarts[c] += 1
                    left = "" if not self.end else "%.2f" % max(0.5, (self.end - now) / 60)
                    self.event("cam_container_restart", "%s exit %s -> instance %d" % (c, code, self.cam_restarts[c]))
                    self.lane_run("_start_cam", self.run, c, self.cam_restarts[c], left)
                else:
                    self.cam_done.add(c)
                    self.event("cam_lost", "%s exit %s after %d container restarts" % (c, code, self.cam_restarts[c]))
                    alerts.append(("RED", "%s LOST" % c.upper()))

        # ── MKR logger ───────────────────────────────────────────────────────
        live = read_json(os.path.join(self.run, "can", "live_state.json")) or {}
        if self.a.can:
            running, code = ctr.get("lane_mkr", (False, None)) if ctr is not None else (True, None)
            self.mkr_dead_ticks = self.mkr_dead_ticks + 1 if not running else 0
            log = os.path.join(self.run, "can", "can_raw.log") if self.mkr_part == 0 else \
                os.path.join(self.run, "can", "part%d" % self.mkr_part, "can_raw.log")
            try:
                size = os.path.getsize(log)
            except OSError:
                size = 0
            if self.can_size is None or size > self.can_size:
                if self.can_size is not None and size > self.can_size:
                    self.can_grew = True
                    self.can_stale_after_restart = False
                self.can_growth_t = now
            self.can_size = size
            stale = self.can_grew and now - self.can_growth_t > 10
            dead = self.mkr_dead_ticks >= 3 and now - self.t0 > 20
            if (dead or stale) and now - self.mkr_last_restart > 30:
                self.mkr_part += 1
                self.mkr_last_restart = now
                self.event("logger_restart", "%s -> part%d" % ("dead (exit %s)" % code if dead else "can_raw.log stale 10 s",
                                                              self.mkr_part))
                if stale:
                    self.event("can_stale", "%.0f s" % (now - self.can_growth_t))
                    self.can_stale_after_restart = True
                self.can_size, self.can_grew, self.can_growth_t = None, False, now
                self.lane_run("_start_mkr", self.run, self.mkr_part)
            if self.can_stale_after_restart and now - self.mkr_last_restart > 10 and not self.can_grew:
                alerts.append(("RED", "CAN log not growing"))

        # ── system ───────────────────────────────────────────────────────────
        sm = last_csv_row(os.path.join(self.run, "sys", "sysmon.csv"))
        mem_gb = (num(sm.get("mem_avail_kb")) or 0) / 1048576 if sm else None
        tj = num(sm.get("t_tj_thermal_c")) if sm else None
        gbh = num(sm.get("run_gbph_5min")) if sm else None
        free = num(sm.get("disk_free_gb")) if sm else None

        # ── degrade ladder ───────────────────────────────────────────────────
        want = self.degrade
        hard = (free is not None and free < 5) or (mem_gb is not None and 0 < mem_gb < 0.6)
        soft = (mem_gb is not None and 0 < mem_gb < 1.0) or (tj is not None and tj > 92) or (gbh is not None and gbh > 22)
        if hard:
            want = 6
        elif soft and now - self.last_step >= 10:
            want = min(5, self.degrade + 1)
        if want > self.degrade:
            why = "free %.1f GB, MemAvailable %.2f GB, tj %s C, %s GB/h" % (free or -1, mem_gb or -1, tj, gbh)
            for lvl in range(self.degrade + 1, want + 1):
                self.event("degrade_%d" % lvl, why)
            self.degrade = want
            self.last_step = now
            write_atomic(os.path.join(self.run, "ctl", "degrade"), "%d\n" % self.degrade)
            if self.degrade >= 6 and not self.stopping:
                self.stopping = True
                self.event("stop_request", "degrade 6: " + why)
                self.lane_run("stop")
        if self.degrade >= 3:
            alerts.append(("RED", "degrade %d" % self.degrade))
        elif self.degrade >= 1:
            alerts.append(("AMBER", "degrade %d" % self.degrade))

        # ── GPU saturation → UGREEN lanes to 10 Hz ───────────────────────────
        for c in self.cams:
            st = cam_st[c]
            eff, tgt = num(st.get("eff_hz")), num(st.get("target_hz")) or 20
            if st.get("lanes_on") == 1 and st.get("state") == "RUNNING" and eff is not None and eff < 0.8 * tgt \
                    and (tj is None or tj < 85):
                self.slow_since.setdefault(c, now)
            else:
                self.slow_since.pop(c, None)
            if c in self.slow_since and now - self.slow_since[c] >= 30 and "usb" in self.cams and self.usb_hz is None:
                self.usb_hz = 10
                write_atomic(os.path.join(self.run, "ctl", "usb_lane_hz"), "10\n")
                self.event("degrade_gpu", "%s at %.1f Hz for 30 s -> UGREEN lanes 10 Hz" % (c, eff))

        # ── per-camera health ────────────────────────────────────────────────
        lines = []
        for c in self.cams:
            st = cam_st[c]
            age = (time.monotonic_ns() - (st.get("mono_ns") or 0)) / 1e9
            state = st.get("state", "?")
            if c in self.cam_finished:
                state = "DONE"
            elif c in self.cam_done:
                state = "LOST"
            elif c not in self.cam_done and (not st or age > 5) and now - self.t0 > 30:
                state = "SILENT"
            eff, tgt = num(st.get("eff_hz")), num(st.get("target_hz")) or 20
            mount = st.get("mount", "?")
            if state in ("LOST", "SILENT"):
                alerts.append(("RED", "%s %s" % (c.upper(), state)))
            elif state == "RESTARTING":
                alerts.append(("AMBER", "%s restarting" % c.upper()))
            h = self.eff_hist.setdefault(c, [])
            h.append(eff if st.get("lanes_on") == 1 and eff is not None else None)
            del h[:-10]
            recent = [v for v in h if v is not None]
            if len(recent) == 10 and sum(recent) / 10 < 0.9 * tgt and now - self.t0 > 150:
                alerts.append(("AMBER", "%s %.1f Hz (10 s)" % (c.upper(), sum(recent) / 10)))
            if mount.startswith("FAIL"):
                alerts.append(("RED", "%s mount FAIL" % c.upper()))
            ms = {"ok": "ok", "no_ref": "ref", "no_measure": "m?", "lost": "LOST", "off": "-", "FAIL": "MNT!", "FAIL_back": "MNT!"}.get(mount, "?")
            lines.append("%s %s %s L%s E%s %s" % (c.upper(), state[:3], "%.0fHz" % eff if eff is not None else "--",
                                                  st.get("num_lanes", "-"), st.get("ego", "-"), ms))
        # CAN line
        if not self.a.can:
            can_line = "CAN off"
        elif live.get("frames"):
            age = live.get("age_ms")
            sp = live.get("speed_kmh")
            can_line = "CAN %s %s %s age%.1fs" % ("%.0f" % sp if sp is not None else "--", live.get("gear") or "-",
                                                  "<" if live.get("turn_l") else ">" if live.get("turn_r") else "-",
                                                  (age or 0) / 1000)
            if age is not None and age > 10000:
                alerts.append(("AMBER", "CAN stale %.0fs" % (age / 1000)))
        else:
            can_line = "CAN idle (no frames)"
            if self.a.expect_can and now - self.t0 > 60:
                alerts.append(("AMBER", "CAN no frames"))
        if tj is not None and tj > 92:
            alerts.append(("RED", "tj %.0f C" % tj))
        elif tj is not None and tj > 85:
            alerts.append(("AMBER", "tj %.0f C" % tj))
        if mem_gb is not None and 0 < mem_gb < 1.5:
            alerts.append(("AMBER", "mem %.1f GB" % mem_gb))

        # ── readiness ────────────────────────────────────────────────────────
        healthy = True
        for c in self.cams:
            st = cam_st[c]
            eff, tgt = num(st.get("eff_hz")), num(st.get("target_hz")) or 20
            rb = dir_bytes(os.path.join(self.run, c, "review")) if now % 10 < 1 or not self.ready else None
            if rb is not None:
                if rb > self.review_size[c][0]:
                    self.review_size[c] = (rb, now)
            if not (st.get("state") == "RUNNING" and st.get("lanes_on") == 1 and eff is not None and eff >= 0.9 * tgt):
                healthy = False
            if now - self.review_size[c][1] > 75:
                healthy = False
                alerts.append(("AMBER", "%s review stalled" % c.upper()))
        try:
            ts = os.path.getsize(os.path.join(self.run, "sys", "tegrastats.txt"))
        except OSError:
            ts = 0
        if ts > self.teg_size[0]:
            self.teg_size = (ts, now)
        if now - self.teg_size[1] > 10:
            healthy = False
        if self.a.can and self.a.expect_can and not live.get("frames"):
            healthy = False
        if healthy and not self.ready:
            self.ready = True
            self.event("ready", "all producers healthy")
            print("LANES ON - drive", flush=True)

        level = "RED" if any(l == "RED" for l, _ in alerts) else "AMBER" if alerts or not self.ready else "GREEN"
        for l, t in alerts:
            if (l, t) not in self.alerts_seen:
                self.alerts_seen.add((l, t))
                self.last_alert = "%s %s" % (l, t)
                self.event("alert", self.last_alert)
        self.alerts_seen = {x for x in self.alerts_seen if x in set(alerts)}
        el = int(now - self.t0)
        title = ("ALL GREEN " if level == "GREEN" else "") + "%s %d:%02d" % (os.path.basename(self.run), el // 3600, el // 60 % 60)
        sysline = "m%.1fG tj%s %sGB/h d%d" % (mem_gb or 0, "%.0f" % tj if tj is not None else "-",
                                              "%.0f" % gbh if gbh is not None else "-", self.degrade)
        alert_line = alerts[0][1] if alerts else sysline
        disp = (lines + ["", ""])[:2] + [can_line, alert_line]
        out = {
            "mono_ns": time.monotonic_ns(), "overall": level, "ready": self.ready, "elapsed_s": el,
            "alerts": ["%s %s" % x for x in alerts], "last_alert": self.last_alert,
            "cams": cam_st, "can": live if self.a.can else None, "mem_avail_gb": mem_gb, "tj": tj,
            "disk_gb_h": gbh, "disk_free_gb": free, "degrade": self.degrade, "usb_lane_hz": self.usb_hz,
            "cam_container_restarts": self.cam_restarts, "logger_part": self.mkr_part,
            "disp_title": title, "disp_l1": disp[0], "disp_l2": disp[1], "disp_l3": disp[2], "disp_l4": disp[3],
        }
        write_atomic(os.path.join(self.run, "status.json"), json.dumps(out) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--cams", default="csi,usb")
    ap.add_argument("--can", action="store_true")
    ap.add_argument("--expect-can", action="store_true", help="the car is on: no CAN frames is an alert")
    ap.add_argument("--minutes", type=float, default=0)
    ap.add_argument("--max-cam-restarts", type=int, default=10)
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    s = Supervisor(a)
    s.event("supervisor_start", "cams %s can %s expect_can %s" % (a.cams, a.can, a.expect_can))
    nxt = time.monotonic()
    while not stop:
        nxt += 1.0
        time.sleep(max(0.0, nxt - time.monotonic()))
        try:
            s.tick()
        except Exception:                          # never die mid-drive: log and carry on
            traceback.print_exc()
            sys.stdout.flush()
    s.event("supervisor_stop")
    return 0


if __name__ == "__main__":
    sys.exit(main())
