"""Read-only Brio rear-axle odometry. No serial, CAN transmit or auto-learning.

Positive yaw/curvature = left, forward travel only. Angle is the equivalent
bicycle-model ROAD wheel, not steering-wheel angle or either individual tyre.
"""
import dataclasses
import math
import re


@dataclasses.dataclass(frozen=True)
class Profile:
    wheelbase_m: float = 2.405  # existing Brio map; verify on the test vehicle
    wheel_kmh_per_count: float = 0.01
    yaw_dps_per_count: float = 0.1083  # map yawscale / 100, provisional
    mismatch_ratio: float = 0.0
    min_wheel_count: float = 300.0
    max_speed_kmh: float = 25.0  # demo envelope, not a road-use estimator
    max_yaw_dps: float = 60.0
    stale_s: float = 0.2
    filter_tau_s: float = 0.15
    straight_curvature_per_m: float = 0.002  # |R| >= 500 m: no finite radius

    def __post_init__(self):
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("profile values must be finite numbers: " + field.name)
        bounds = {
            "wheelbase_m": (1, 5), "wheel_kmh_per_count": (.001, .1),
            "yaw_dps_per_count": (.005, 1), "mismatch_ratio": (-.1, .1),
            "min_wheel_count": (300, 3000), "max_speed_kmh": (4, 30),
            "max_yaw_dps": (5, 90), "stale_s": (.02, .25),
            "filter_tau_s": (0, 2), "straight_curvature_per_m": (.0001, .02),
        }
        for key, (lo, hi) in bounds.items():
            if not lo <= getattr(self, key) <= hi:
                raise ValueError("profile out of range: " + key)


# Deliberately restricted to classic candump, one interface and the Brio map.
LINE = re.compile(r"\((\d+(?:\.\d+)?)\)\s+(\S+)\s+([0-9A-Fa-f]{3}|[0-9A-Fa-f]{8})#([0-9A-Fa-f]{0,16}|R[0-8]?)")


def parse_line(line, interface="can0"):
    m = LINE.fullmatch(line.strip())
    if not m:
        if not line.strip() or line.startswith("#"):
            return None
        raise ValueError("malformed classic candump line")
    stamp = float(m[1])
    if not math.isfinite(stamp):
        raise ValueError("invalid timestamp")
    payload = m[4]
    if m[2] != interface or len(m[3]) != 3 or payload.startswith("R"):
        return None
    ident = int(m[3], 16)
    if ident > 0x7ff or len(payload) % 2:
        raise ValueError("invalid standard CAN frame")
    return stamp, ident, bytes.fromhex(payload)


def motorola(data, start, length):
    value, bit = 0, start
    for _ in range(length):
        if bit // 8 >= len(data):
            raise ValueError("short CAN payload")
        value = (value << 1) | ((data[bit // 8] >> (bit % 8)) & 1)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return value


def estimate(rl, rr, profile):
    mean = (rl + rr) / 2.0
    speed = mean * profile.wheel_kmh_per_count
    diff = rr - rl - profile.mismatch_ratio * mean
    yaw = diff * profile.yaw_dps_per_count
    if min(rl, rr) < profile.min_wheel_count:
        return {"status": "below wheel-sensor cutoff"}
    if speed > profile.max_speed_kmh or abs(yaw) > profile.max_yaw_dps:
        return {"status": "outside demo envelope"}
    return {"status": "ok", "speed_kmh": speed, "yaw_raw_dps": yaw,
            "corrected_diff": diff}


def geometry(speed_kmh, yaw_dps, profile):
    curvature = math.radians(yaw_dps) / (speed_kmh / 3.6)
    straight = abs(curvature) < profile.straight_curvature_per_m
    return {"curvature_per_m": curvature,
            "angle_deg": math.degrees(math.atan(profile.wheelbase_m * curvature)),
            "radius_m": None if straight else 1 / abs(curvature),
            "direction": "near straight" if straight else ("left" if curvature > 0 else "right")}


class Decoder:
    def __init__(self, profile):
        self.profile = profile
        self.segment = 0
        self.last_frame = None
        self.reset()

    def reset(self):
        self.gear, self.gear_time = None, None
        self.filtered_yaw = self.filtered_speed = self.last_wheel = None

    def feed(self, stamp, ident, data):
        # Every frame contributes to gap detection, not only wheel frames.
        if self.last_frame is not None and (stamp < self.last_frame or stamp - self.last_frame > 1):
            self.segment += 1
            self.reset()
        self.last_frame = stamp
        if ident == 0x191:
            self.gear = motorola(data, 44, 5)
            self.gear_time = stamp
        if ident != 0x1D0:
            return None
        wheels = [motorola(data, bit, 15) for bit in (7, 8, 25, 42)]
        row = dict(time=stamp, segment=self.segment, gear_raw=self.gear,
                   wheel_fl=wheels[0], wheel_fr=wheels[1], wheel_rl=wheels[2], wheel_rr=wheels[3])
        row.update(estimate(wheels[2], wheels[3], self.profile))
        if self.gear_time is None or not 0 <= stamp - self.gear_time <= self.profile.stale_s:
            row["status"] = "gear missing/stale"
        elif self.gear not in (4, 7, 10):  # D, L, S; unsigned wheels cannot determine direction
            row["status"] = "forward gear required"
        dt = stamp - self.last_wheel if self.last_wheel is not None else None
        if row["status"] != "ok":
            self.filtered_yaw = self.filtered_speed = self.last_wheel = None
            return row
        tau = self.profile.filter_tau_s
        alpha = 1 if tau == 0 or dt is None or dt > self.profile.stale_s else -math.expm1(-dt / tau)
        if self.filtered_yaw is None:
            alpha = 1
        self.filtered_yaw = row["yaw_raw_dps"] if alpha == 1 else self.filtered_yaw + alpha * (row["yaw_raw_dps"] - self.filtered_yaw)
        self.filtered_speed = row["speed_kmh"] if alpha == 1 else self.filtered_speed + alpha * (row["speed_kmh"] - self.filtered_speed)
        self.last_wheel = stamp
        row["yaw_dps"] = self.filtered_yaw
        row.update(geometry(self.filtered_speed, self.filtered_yaw, self.profile))
        return row
