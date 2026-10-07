"""Small, dependency-free EKF for the Orin demo, not a vehicle controller.

State: [heading CCW from east (rad), forward speed (m/s), yaw (rad/s),
gyro bias (rad/s)]. No-slip individual wheel and antenna-velocity models.
All input times must already share a clock; arrival time is not acquisition time.
"""
import dataclasses
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def finite(v):
    return isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)


@dataclasses.dataclass(frozen=True)
class FusionConfig:
    # Required: never silently guess mounting orientation or physical geometry.
    gyro_up: tuple
    front_track_m: float
    rear_track_m: float
    antenna_x_m: float = 0.0  # forward of rear centre
    antenna_y_m: float = 0.0  # left of rear centre
    wheel_sigma_mps: float = 0.12
    gyro_sigma_dps: float = 1.0
    course_sigma_deg: float = 8.0  # CSV lacks receiver accuracy; conservative assumption
    speed_walk_mps_sqrt_s: float = 1.0
    yaw_walk_dps_sqrt_s: float = 12.0
    bias_walk_dps_sqrt_s: float = 0.03
    min_course_speed_mps: float = 1.5
    gyro_time_offset_s: float = 0.0  # acquisition = host arrival + offset
    gnss_time_offset_s: float = 0.0
    gyro_time_sigma_s: float = 0.10
    gnss_time_sigma_s: float = 0.50
    nis_gate: float = 16.0

    def __post_init__(self):
        if not isinstance(self.gyro_up, (tuple, list)) or len(self.gyro_up) != 3 or not all(finite(v) for v in self.gyro_up):
            raise ValueError("gyro_up must be a measured sensor-frame vehicle-up unit vector")
        if abs(sum(v * v for v in self.gyro_up) - 1) > .001:
            raise ValueError("gyro_up must have unit length")
        bounds = dict(front_track_m=(.8, 2.5), rear_track_m=(.8, 2.5),
                      antenna_x_m=(-5, 5), antenna_y_m=(-2, 2), wheel_sigma_mps=(.02, 2),
                      gyro_sigma_dps=(.1, 20), course_sigma_deg=(1, 45),
                      speed_walk_mps_sqrt_s=(.05, 5), yaw_walk_dps_sqrt_s=(.1, 60),
                      bias_walk_dps_sqrt_s=(.001, 1), min_course_speed_mps=(1, 5),
                      gyro_time_offset_s=(-2, 0), gnss_time_offset_s=(-2, 0),
                      gyro_time_sigma_s=(.01, 1), gnss_time_sigma_s=(.05, 2), nis_gate=(4, 100))
        for key, (lo, hi) in bounds.items():
            v = getattr(self, key)
            if not finite(v) or not lo <= v <= hi:
                raise ValueError("fusion config out of range: " + key)


def wheel_model(x, index, wheelbase, front_track, rear_track):
    """Predicted individual ground speed and analytic measurement Jacobian."""
    v, r = x[1:3]
    y = (front_track if index < 2 else rear_track) / 2 * (1 if index % 2 == 0 else -1)
    a = v - r * y
    if index >= 2:
        return a, [0, 1, -y, 0]
    b = r * wheelbase
    h = math.hypot(a, b)
    if h < 1e-8:
        return h, [0, 1, -y, 0]
    return h, [0, a / h, (-y * a + wheelbase * b) / h, 0]


def course_model(x, ax, ay):
    """Antenna COG differs from body heading while turning (lever arm)."""
    v, r = x[1:3]
    a, b = v - r * ay, r * ax
    d = a * a + b * b
    if d < 1e-8:
        raise ValueError("antenna velocity too small")
    return wrap(x[0] + math.atan2(b, a)), [1, -b / d, ax * v / d, 0]


class TurningEKF:
    def __init__(self, profile, config):
        self.p, self.c = profile, config
        self.rejected = self.accepted = self.resets = 0
        self.reset()

    def reset(self):
        self.x = [0., 0., 0., 0.]
        variances = [math.pi ** 2, 4., math.radians(30) ** 2, math.radians(5) ** 2]
        self.P = [[variances[i] if i == j else 0. for j in range(4)] for i in range(4)]
        self.t = self.wheel_time = self.gyro_time = self.course_time = None
        self.seen = {}
        self.aligned = False
        self.wheels_used = 0
        self.last_nis = None
        self.gyro_nis = self.course_nis = self.wheel_max_nis = None
        self.resets += 1

    def predict(self, t):
        if not finite(t) or t < 0 or (self.t is not None and t < self.t - 1e-6):
            self.rejected += 1
            return False
        if self.t is None:
            self.t = t
            return True
        dt = max(0., t - self.t)
        if dt > .5:
            self.reset()  # unknown travel, not a long extrapolation
            self.t = t
            return True
        self.x[0] = wrap(self.x[0] + self.x[2] * dt)
        # F=I with F[heading,yaw]=dt, exact linear constant-rate transition.
        old = self.P
        self.P = [[old[i][j] + (dt * old[2][j] if i == 0 else 0) +
                   (dt * old[i][2] if j == 0 else 0) +
                   (dt * dt * old[2][2] if i == j == 0 else 0)
                   for j in range(4)] for i in range(4)]
        q = math.radians(self.c.yaw_walk_dps_sqrt_s) ** 2
        self.P[0][0] += q * dt ** 3 / 3
        self.P[0][2] += q * dt ** 2 / 2
        self.P[2][0] += q * dt ** 2 / 2
        self.P[2][2] += q * dt
        self.P[1][1] += self.c.speed_walk_mps_sqrt_s ** 2 * dt
        self.P[3][3] += math.radians(self.c.bias_walk_dps_sqrt_s) ** 2 * dt
        self.t = t
        return True

    def correct(self, z, h, H, variance, angular=False):
        residual = wrap(z - h) if angular else z - h
        ph = [sum(self.P[i][j] * H[j] for j in range(4)) for i in range(4)]
        s = variance + sum(H[i] * ph[i] for i in range(4))
        if not finite(s) or s <= 0:
            raise ValueError("EKF invalid innovation covariance")
        self.last_nis = residual * residual / s
        if self.last_nis > self.c.nis_gate:
            self.rejected += 1
            return False
        k = [p / s for p in ph]
        candidate = [self.x[i] + k[i] * residual for i in range(4)]
        if not all(finite(v) for v in candidate) or not (-.2 <= candidate[1] <= self.p.max_speed_kmh / 3.6 + 1 and
                abs(candidate[2]) <= math.radians(self.p.max_yaw_dps) and abs(candidate[3]) <= math.radians(20)):
            self.rejected += 1
            return False
        self.x = candidate
        self.x[0] = wrap(self.x[0])
        # Joseph covariance update, then symmetrise roundoff. No matrix inverse.
        a = [[float(i == j) - k[i] * H[j] for j in range(4)] for i in range(4)]
        ap = [[sum(a[i][n] * self.P[n][j] for n in range(4)) for j in range(4)] for i in range(4)]
        p = [[sum(ap[i][n] * a[j][n] for n in range(4)) + k[i] * variance * k[j]
              for j in range(4)] for i in range(4)]
        self.P = [[(p[i][j] + p[j][i]) / 2 for j in range(4)] for i in range(4)]
        self.accepted += 1
        return True

    def sensor(self, event):
        kind, t = event['kind'], event['time']
        if kind not in ('gyro', 'course'):
            raise ValueError("unknown fusion sensor")
        # Consume each source observation only once, including rejected observations.
        if not finite(t) or t <= self.seen.get(kind, -1) or not self.predict(t):
            self.rejected += 1
            return False
        self.seen[kind] = t
        value = event['value']
        if not finite(value):
            self.rejected += 1
            return False
        if kind == 'gyro':
            if abs(value) > self.p.max_yaw_dps + 20:
                self.rejected += 1
                return False
            variance = math.radians(self.c.gyro_sigma_dps) ** 2 + (
                math.radians(self.c.yaw_walk_dps_sqrt_s) * self.c.gyro_time_sigma_s) ** 2
            ok = self.correct(math.radians(value), self.x[2] + self.x[3], [0, 0, 1, 1], variance)
            self.gyro_nis = self.last_nis
            if ok:
                self.gyro_time = t
            return ok
        speed = event.get('speed_mps')
        if not finite(speed) or not self.c.min_course_speed_mps <= speed <= self.p.max_speed_kmh / 3.6 + 2 or not 0 <= value <= 360:
            self.rejected += 1
            return False
        if self.wheel_time is None or t - self.wheel_time > .25 or self.x[1] < self.c.min_course_speed_mps:
            self.rejected += 1
            return False
        z = wrap(math.radians(90 - value))  # true-north CW -> east CCW
        h, H = course_model(self.x, self.c.antenna_x_m, self.c.antenna_y_m)
        variance = math.radians(self.c.course_sigma_deg) ** 2 + (self.x[2] * self.c.gnss_time_sigma_s) ** 2
        if not self.aligned:
            # First absolute reference may differ by pi: don't linear-gate its origin.
            self.x[0] = wrap(z - (h - self.x[0]))
            self.P[0] = [0.] * 4
            for row in self.P:
                row[0] = 0.
            self.P[0][0] = variance + sum(H[i] * self.P[i][j] * H[j] for i in (1, 2) for j in (1, 2))
            self.aligned = True
            self.course_time = t
            self.accepted += 1
            return True
        ok = self.correct(z, h, H, variance, angular=True)
        self.course_nis = self.last_nis
        if ok:
            self.course_time = t
        return ok

    def wheels(self, row):
        t = row['time']
        if not self.predict(t):
            return
        if t <= self.seen.get('wheels', -1):
            self.rejected += 1
            return
        dt = t - self.seen.get('wheels', t - .02)
        self.seen['wheels'] = t
        self.wheels_used = 0
        self.wheel_max_nis = None
        if row.get('gear_raw') not in (4, 7, 10) or row.get('status') in ('gear missing/stale', 'forward gear required'):
            self.reset()
            self.t = t
            return
        values = [row.get('wheel_' + side) for side in ('fl', 'fr', 'rl', 'rr')]
        # Rear tyre mismatch correction is a fixed externally calibrated profile.
        factors = [1., 1., 1 / (1 - self.p.mismatch_ratio / 2), 1 / (1 + self.p.mismatch_ratio / 2)]
        usable = [(i, v * self.p.wheel_kmh_per_count / 3.6 * factors[i]) for i, v in enumerate(values)
                  if finite(v) and self.p.min_wheel_count <= v <= 32767 and
                  v * self.p.wheel_kmh_per_count <= self.p.max_speed_kmh]
        if len(usable) < 2:
            return
        if self.wheel_time is None:
            rear = dict(usable)
            if 2 not in rear or 3 not in rear:
                return  # need a signed yaw seed, not front-wheel sign ambiguity
            self.x[1] = (rear[2] + rear[3]) / 2
            self.x[2] = (rear[3] - rear[2]) / self.c.rear_track_m
            if abs(self.x[2]) > math.radians(self.p.max_yaw_dps):
                self.reset()
                self.t = t
                return
        # A 50 Hz correlated wheel stream must not gain unlimited confidence.
        variance = self.c.wheel_sigma_mps ** 2 * max(1., .1 / max(.001, dt))
        for i, z in usable:
            h, H = wheel_model(self.x, i, self.p.wheelbase_m, self.c.front_track_m, self.c.rear_track_m)
            if self.correct(z, h, H, variance):
                self.wheels_used += 1
            self.wheel_max_nis = max(self.wheel_max_nis or 0, self.last_nis)
        if self.wheels_used >= 2:
            self.wheel_time = t

    def snapshot(self, t):
        out = dict(ekf_status='unavailable', ekf_wheels_used=self.wheels_used,
                   ekf_accepted=self.accepted, ekf_rejected=self.rejected, ekf_resets=self.resets,
                   ekf_nis=self.last_nis, ekf_sources='none')
        out.update(ekf_gyro_nis=self.gyro_nis, ekf_course_nis=self.course_nis,
                   ekf_wheel_max_nis=self.wheel_max_nis)
        if self.t is None or not 0 <= t - self.t <= .25:
            return out
        wheel = self.wheel_time is not None and 0 <= t - self.wheel_time <= .25
        gyro = self.gyro_time is not None and 0 <= t - self.gyro_time <= .35
        course = self.course_time is not None and 0 <= t - self.course_time <= 2.5
        out['ekf_sources'] = '+'.join(name for name, ok in [('wheels', wheel), ('gyro', gyro), ('GNSS', course)] if ok) or 'none'
        if not wheel or self.wheels_used < 2:
            return out
        v, r = self.x[1:3]
        out.update(ekf_status='fused' if gyro and course else 'degraded', ekf_speed_kmh=v * 3.6,
                   ekf_yaw_dps=math.degrees(r), ekf_yaw_sigma_dps=math.degrees(math.sqrt(max(0., self.P[2][2]))),
                   ekf_bias_dps=math.degrees(self.x[3]), ekf_heading_deg=(90 - math.degrees(self.x[0])) % 360 if course else None)
        if v < self.p.min_wheel_count * self.p.wheel_kmh_per_count / 3.6:
            out['ekf_status'] = 'low speed'
            return out
        L = self.p.wheelbase_m
        d = v * v + (L * r) ** 2
        jv, jr = -L * r / d, L * v / d
        variance = jv * jv * self.P[1][1] + 2 * jv * jr * self.P[1][2] + jr * jr * self.P[2][2]
        out.update(ekf_angle_deg=math.degrees(math.atan2(L * r, v)),
                   ekf_angle_sigma_deg=math.degrees(math.sqrt(max(0., variance))),
                   ekf_radius_m=None, ekf_radius_sigma_m=None)
        if abs(r / v) >= self.p.straight_curvature_per_m and abs(r) > 2 * math.sqrt(max(0., self.P[2][2])):
            # Local delta-method uncertainty, not a calibrated accuracy guarantee.
            jv, jr = 1 / r, -v / (r * r)
            variance = jv * jv * self.P[1][1] + 2 * jv * jr * self.P[1][2] + jr * jr * self.P[2][2]
            out.update(ekf_radius_m=abs(v / r), ekf_radius_sigma_m=math.sqrt(max(0., variance)))
        return out


FIELDS = ['ekf_status', 'ekf_sources', 'ekf_speed_kmh', 'ekf_yaw_dps', 'ekf_yaw_sigma_dps',
          'ekf_angle_deg', 'ekf_angle_sigma_deg', 'ekf_radius_m', 'ekf_radius_sigma_m',
          'ekf_heading_deg', 'ekf_bias_dps', 'ekf_wheels_used', 'ekf_accepted', 'ekf_rejected',
          'ekf_resets', 'ekf_nis', 'ekf_late_events', 'ekf_gyro_nis', 'ekf_course_nis', 'ekf_wheel_max_nis']
