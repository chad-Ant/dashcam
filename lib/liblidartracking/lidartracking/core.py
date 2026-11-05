"""Single-owner, bounded association core. All times are Orin monotonic seconds.

Input boxes MUST be from rectified images at the calibrated resolution. A box
is not a segmentation mask: association is a hypothesis, never proof of a hit.
No motor commands are executed here. Only measured servo feedback can gate range.
"""
from dataclasses import dataclass
from math import atan2, cos, sin, isfinite, sqrt
from typing import Optional, Protocol, Tuple


def finite(*values):
    return all(isfinite(v) for v in values)


def matvec(m, v):
    return tuple(sum(m[3*i+j] * v[j] for j in range(3)) for i in range(3))


def yaw(v, angle):
    c, s = cos(angle), sin(angle)
    return (c*v[0]+s*v[2], v[1], -s*v[0]+c*v[2])


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    # Camera axes: right, down, forward. R maps zero-pan mount to camera.
    rotation: Tuple[float, ...]
    pivot_m: Tuple[float, float, float]
    sensor_offset_m: Tuple[float, float, float]
    boresight_error_rad: float
    verified: bool = False
    beam_half_angle_rad: float = 0.004364 # 0.25 deg; verify delivered unit
    motion_bound_px_s: float = 2000. # Must bound apparent motion over time skew.
    # Bounds apparent box acceleration; used only when two bracketing frames are
    # interpolated. 1 deg pitch at 2 Hz is ~3200 px/s^2 at fx=1200. Verify on rig.
    motion_accel_bound_px_s2: float = 10000.

    def __post_init__(self):
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or not 1 <= self.width <= 8192 or not 1 <= self.height <= 8192
                or len(self.rotation) != 9 or len(self.pivot_m) != 3 or len(self.sensor_offset_m) != 3
                or not finite(self.fx, self.fy, self.cx, self.cy, *self.rotation,
                              *self.pivot_m, *self.sensor_offset_m,
                              self.boresight_error_rad, self.beam_half_angle_rad, self.motion_bound_px_s,
                              self.motion_accel_bound_px_s2)
                or self.fx <= 0 or self.fy <= 0 or not 0 <= self.cx < self.width
                or not 0 <= self.cy < self.height or not 0 <= self.boresight_error_rad < .1
                or not 0 < self.beam_half_angle_rad < .1 or not 0 < self.motion_bound_px_s <= 100000
                or not 0 < self.motion_accel_bound_px_s2 <= 1e7
                or type(self.verified) is not bool):
            raise ValueError("invalid camera / mount calibration")
        r = self.rotation
        for i in range(3):
            for j in range(3):
                if abs(sum(r[3*k+i]*r[3*k+j] for k in range(3)) - (i == j)) > 1e-5:
                    raise ValueError("rotation must be orthonormal")
        det = r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6])
        if abs(det-1) > 1e-5:
            raise ValueError("rotation must be right-handed")
        # JSON supplies lists; don't retain mutable calibration storage.
        for name in ("rotation", "pivot_m", "sensor_offset_m"):
            object.__setattr__(self, name, tuple(getattr(self, name)))

    def point(self, distance, pan, vertical=0.0):
        direction = yaw((0., sin(vertical), cos(vertical)), pan)
        local = yaw(self.sensor_offset_m, pan)
        transformed = matvec(self.rotation, tuple(local[i]+distance*direction[i] for i in range(3)))
        return tuple(self.pivot_m[i] + transformed[i] for i in range(3))

    def project(self, p):
        if p[2] <= .01:
            raise ValueError("beam behind camera")
        return (self.fx*p[0]/p[2]+self.cx, self.fy*p[1]/p[2]+self.cy)


@dataclass(frozen=True)
class Detection:
    track_id: int
    box: Tuple[float, float, float, float] # left, top, right, bottom
    confidence: float
    label: str = "car"


@dataclass(frozen=True)
class CameraFrame:
    time_s: float # exposure midpoint, NOT inference completion time
    uncertainty_s: float
    width: int
    height: int
    detections: Tuple[Detection, ...]
    rectified: bool = True


@dataclass(frozen=True)
class RangeSample:
    sequence: int
    time_s: float
    uncertainty_s: float
    first_m: float
    last_m: float
    first_db: float
    last_db: float
    # False until sensor acquisition-to-UART latency has a measured bound.
    timing_verified: bool = False


@dataclass(frozen=True)
class ServoFeedback:
    time_s: float
    pan_rad: float
    error_rad: float
    settled: bool
    # Time since which measured angle remained within error_rad; must cover
    # entire LiDAR acquisition interval, not just its serial reception time.
    settled_since_s: float
    valid_until_s: float # observed endpoint, NEVER a promise of future position


@dataclass(frozen=True)
class ServoCommand:
    track_id: int
    pan_rad: float
    expires_s: float


class ServoPort(Protocol):
    """Implement separately. Driver owns limits, watchdog, feedback and PWM."""
    def request_pan(self, command: ServoCommand) -> bool: ...
    def feedback(self) -> Optional[ServoFeedback]: ...
    def stop(self) -> None: ...


@dataclass(frozen=True)
class Result:
    valid: bool
    reason: str
    track_id: Optional[int] = None
    slant_m: Optional[float] = None # sensor origin to return
    forward_m: Optional[float] = None # camera-frame Z, NOT bumper clearance
    filtered_m: Optional[float] = None
    closing_mps: Optional[float] = None # positive = approaching; not ego velocity
    pan_request: Optional[ServoCommand] = None
    expires_s: Optional[float] = None # consumer MUST clear distance after this time


class VehicleRanger:
    """association selects how the beam must sit on the target box:

    "contained" (default): the whole beam, with pointing error, lies inside the
    central 60% of the box. Only the target can return; reach is short (a
    1.3 m tall car: ~38 m at fx=1200, 30 fps).

    "first_return": the beam may spill sideways or above the car, but never
    below the box's bottom edge, and its centre must be on the car. Range is
    the first return. Physics: with the box bottom at the vehicle's ground
    contact on locally flat road and the LiDAR above the road, every ray that
    is above that row at the car's range is above the road all the way there,
    so a nearer surface can enter the beam only below the box or as another
    vehicle, which the ambiguity check rejects. Spill lands on farther ground,
    background or sky, so it can only add a LATER return. Undetected nearer
    obstacles (poles, people, spray) defeat both modes alike.
    Bench-verify first-return bias on partly covered targets before relying on it.
    """
    def __init__(self, calibration, *, min_m=10., max_m=120., max_skew_s=.1,
                 pan_limits=(-.65, .65), min_db=0., range_noise_m=.5, association="contained"):
        if (not finite(min_m, max_m, max_skew_s, *pan_limits, min_db, range_noise_m)
                or not 0 < min_m < max_m <= 300 or not 0 < max_skew_s <= .25
                or not -1.4 < pan_limits[0] < pan_limits[1] < 1.4 or range_noise_m <= 0
                or association not in ("contained", "first_return")):
            raise ValueError("invalid ranger settings")
        self.first_return = association == "first_return"
        self.c = calibration
        self.min_m, self.max_m, self.skew = min_m, max_m, max_skew_s
        self.pan_limits, self.min_db, self.noise = pan_limits, min_db, range_noise_m
        self.selected = None
        self.last_sequence = None
        self.last_time = None
        self.filter = None
        self.last_frame_time = None

    def select(self, track_id):
        """Explicit target selection; never silently hand distance to another ID."""
        if track_id is not None and (not isinstance(track_id, int) or track_id < 0):
            raise ValueError("invalid track ID")
        if track_id != self.selected:
            self.filter = None
        self.selected = track_id

    def _reject(self, why, command=None):
        self.filter = None
        return Result(False, why, self.selected, pan_request=command)

    def _validate_frame(self, frame):
        if (frame.rectified is not True or (frame.width, frame.height) != (self.c.width, self.c.height)
                or not finite(frame.time_s, frame.uncertainty_s) or frame.uncertainty_s < 0
                or len(frame.detections) > 64):
            return False
        ids = set()
        for d in frame.detections:
            if (len(d.box) != 4 or not finite(*d.box, d.confidence)
                    or not isinstance(d.track_id, int) or d.track_id < 0 or d.track_id in ids
                    or not 0 <= d.confidence <= 1):
                return False
            x1, y1, x2, y2 = d.box
            if not (0 <= x1 < x2 <= frame.width and 0 <= y1 < y2 <= frame.height):
                return False
            ids.add(d.track_id)
        return True

    def aim(self, frame, now):
        """Bearing-only pointing hint; finite-range parallax checked at association."""
        if not self.c.verified or not self._validate_frame(frame) or not isfinite(now):
            return None
        if now < frame.time_s or now-frame.time_s+frame.uncertainty_s > self.skew:
            return None
        target = next((d for d in frame.detections if d.track_id == self.selected), None)
        if target is None or target.confidence < .6 or target.label not in ("car", "truck", "bus", "motorcycle"):
            return None
        x1, y1, x2, y2 = target.box
        ray = ((.5*(x1+x2)-self.c.cx)/self.c.fx, (.5*(y1+y2)-self.c.cy)/self.c.fy, 1.)
        r = self.c.rotation
        local = tuple(sum(r[3*j+i]*ray[j] for j in range(3)) for i in range(3))
        if local[2] <= 0:
            return None
        angle = atan2(local[0], local[2])
        if not self.pan_limits[0] <= angle <= self.pan_limits[1]:
            return None # do NOT saturate an unreachable bearing into a valid aim
        return ServoCommand(target.track_id, angle, frame.time_s + self.skew)

    def update(self, frame, sample, servo, now, next_frame=None):
        """next_frame: the frame after `frame`, with frame <= sample <= next_frame
        in time. Boxes are then interpolated to the LiDAR time, so the margin
        grows with acceleration x interval^2, not speed x skew. Without it, the
        single-frame margin motion_bound_px_s x skew applies (much shorter reach).
        """
        latest = frame if next_frame is None else next_frame
        command = self.aim(latest, now)
        if not self.c.verified:
            return self._reject("uncalibrated")
        if (not self._validate_frame(frame) or not finite(now)
                or (next_frame is not None and not self._validate_frame(next_frame))):
            return self._reject("invalid_camera_frame")
        if self.last_frame_time is not None and frame.time_s < self.last_frame_time:
            return self._reject("camera_out_of_order")
        if next_frame is not None and not 0 < next_frame.time_s-frame.time_s <= self.skew:
            return self._reject("invalid_frame_pair")
        self.last_frame_time = frame.time_s
        target = next((d for d in frame.detections if d.track_id == self.selected), None)
        later = next((d for d in latest.detections if d.track_id == self.selected), None)
        if target is None or later is None or command is None:
            return self._reject("target_missing_or_unreachable")
        if (sample.timing_verified is not True or not finite(sample.time_s, sample.uncertainty_s,
                sample.first_m, sample.last_m, sample.first_db, sample.last_db)
                or sample.uncertainty_s < 0 or not isinstance(sample.sequence, int)
                or not 0 <= sample.sequence <= 0xffffffff):
            return self._reject("invalid_range_or_timing", command)
        if self.last_sequence is not None:
            delta = (sample.sequence-self.last_sequence) & 0xffffffff
            if delta == 0 or delta >= 0x80000000 or sample.time_s <= self.last_time:
                return self._reject("range_out_of_order", command)
        self.last_sequence, self.last_time = sample.sequence, sample.time_s
        # Frame-time errors of one camera are treated as common mode (one clock).
        uncertainty = sample.uncertainty_s + max(frame.uncertainty_s, latest.uncertainty_s)
        if (now < sample.time_s or now < latest.time_s
                or now - sample.time_s + sample.uncertainty_s > self.skew
                or now - frame.time_s + frame.uncertainty_s > self.skew
                or abs(sample.time_s-frame.time_s) + uncertainty > self.skew):
            return self._reject("stale_or_unsynchronized", command)
        if next_frame is not None and not frame.time_s <= sample.time_s <= next_frame.time_s:
            return self._reject("range_not_bracketed", command)
        if (servo is None or servo.settled is not True or not finite(servo.time_s, servo.pan_rad,
                servo.error_rad, servo.settled_since_s, servo.valid_until_s)
                or not 0 <= servo.error_rad < .1 or not self.pan_limits[0] <= servo.pan_rad <= self.pan_limits[1]
                or servo.time_s > now or now-servo.time_s > self.skew
                or not servo.settled_since_s <= servo.valid_until_s <= servo.time_s
                or servo.settled_since_s > sample.time_s-sample.uncertainty_s
                or servo.valid_until_s < sample.time_s+sample.uncertainty_s):
            return self._reject("servo_not_verified", command)
        if self.first_return:
            # Spill may add a farther (last) return, never a nearer one. A last
            # return that is invalid or nearer than the first is not physical.
            if (not self.min_m <= sample.first_m <= self.max_m or sample.first_db < self.min_db
                    or sample.last_m < sample.first_m-2*self.noise):
                return self._reject("no_usable_return", command)
            distance = sample.first_m
        else:
            distances = (sample.first_m, sample.last_m)
            if (any(not self.min_m <= d <= self.max_m for d in distances)
                    or min(sample.first_db, sample.last_db) < self.min_db):
                return self._reject("no_usable_return", command)
            if abs(sample.first_m-sample.last_m) > 2*self.noise:
                return self._reject("multiple_returns", command)
            distance = min(distances) # never choose a farther background return silently
        pointing = self.c.boresight_error_rad + servo.error_rad
        try:
            a, b, c, d = self._footprint(distance, servo.pan_rad, self.c.beam_half_angle_rad+pointing)
            # Where the beam CENTRE can be: pointing error only, no divergence.
            ca, cb, cc, _ = self._footprint(distance, servo.pan_rad, pointing)
        except ValueError:
            return self._reject("beam_behind_camera", command)
        stamp = sample.time_s
        tracks = {o.track_id: [o.box, None] for o in frame.detections}
        if next_frame is not None:
            for o in next_frame.detections:
                tracks.setdefault(o.track_id, [None, None])[1] = o.box
        def moved(b0, b1):
            if b1 is None:
                return self._motion(b0, frame.time_s, None, None, stamp, uncertainty)
            if b0 is None:
                return self._motion(b1, next_frame.time_s, None, None, stamp, uncertainty)
            return self._motion(b0, frame.time_s, b1, next_frame.time_s, stamp, uncertainty)
        (x1, y1, x2, y2), (ex, ey) = moved(*tracks[target.track_id])
        # Shrink to central 60% to avoid edges/background within detector boxes.
        mx, my = .2*(x2-x1), .2*(y2-y1)
        if self.first_return:
            # Image y points down: y2 is the ground contact. 10% of the height
            # covers a loose box bottom; the centre stays off box edges/mirrors.
            inside = (d+ey <= y2-.5*my and y1+.5*my <= cb-ey
                      and x1+mx <= ca-ex <= cc+ex <= x2-mx)
        else:
            inside = x1+mx <= a-ex <= c+ex <= x2-mx and y1+my <= b-ey <= d+ey <= y2-my
        if not inside:
            return self._reject("beam_outside_target", command)
        for track_id, (b0, b1) in tracks.items():
            if track_id == target.track_id:
                continue
            (l, t, r, bottom), (ex, ey) = moved(b0, b1)
            if max(a-ex, l) < min(c+ex, r) and max(b-ey, t) < min(d+ey, bottom):
                return self._reject("ambiguous_target", command)
        forward = self.c.point(distance, servo.pan_rad)[2]
        expiry = min(sample.time_s-sample.uncertainty_s, frame.time_s-frame.uncertainty_s)+self.skew
        return self._filter(sample.time_s, distance, forward, command, expiry)

    def _footprint(self, distance, pan, error):
        """Image box bounding a ball around the return (beam cone, calibration,
        encoder and lever-arm error) by a 3D box, every corner projected at
        near/far range. Conservative vs sampling cone edges under arbitrary R.
        Raises ValueError if any corner is behind the camera."""
        points = []
        lever = sqrt(sum(v*v for v in self.c.sensor_offset_m))
        for d in (max(.01, distance-3*self.noise), distance+3*self.noise):
            center = self.c.point(d, pan)
            radius = 2*(d+lever)*sin(error/2)
            for x in (-radius, radius):
                for y in (-radius, radius):
                    for z in (-radius, radius):
                        points.append(self.c.project((center[0]+x, center[1]+y, center[2]+z)))
        return (min(p[0] for p in points), min(p[1] for p in points),
                max(p[0] for p in points), max(p[1] for p in points))

    def _motion(self, b0, t0, b1, t1, stamp, uncertainty):
        """Box at `stamp` and a pixel bound on its displacement error.

        One frame: speed bound V times skew. Two frames (t0 <= stamp <= t1, T =
        t1-t0, w = (stamp-t0)/T): linear interpolation is off by at most
        w(1-w)T*min(A*T/2, 2V) for acceleration bound A, and near stamp the speed
        is at most min(V, observed + A(T+u)). Detector jitter is not motion; the
        central-box shrink covers it, and interpolation never amplifies it.
        """
        speed_bound = self.c.motion_bound_px_s
        if b1 is None:
            m = 2 + speed_bound*(abs(stamp-t0)+uncertainty)
            return b0, (m, m)
        accel = self.c.motion_accel_bound_px_s2
        span = t1-t0
        w = (stamp-t0)/span
        error = w*(1-w)*span*min(accel*span/2, 2*speed_bound)
        margins = []
        for axis in (0, 1): # per axis: lateral motion must not widen the vertical gate
            observed = max(abs(b1[axis]-b0[axis]), abs(b1[axis+2]-b0[axis+2]))/span
            speed = min(speed_bound, observed + accel*(span+uncertainty))
            margins.append(2 + error + speed*uncertainty)
        return tuple(p+w*(q-p) for p, q in zip(b0, b1)), tuple(margins)

    def _filter(self, stamp, distance, forward, command, expiry):
        # Alpha-beta filter with innovation gate. Reset on any invalid association
        # or target change; never predict a distance through missing observations.
        closing = None
        if self.filter is None or stamp-self.filter[0] > .25:
            position, velocity = distance, 0.
        else:
            previous, position, velocity = self.filter
            dt = stamp-previous
            innovation = distance-(position+velocity*dt)
            if dt <= 0 or abs(innovation) > 3*self.noise+80*dt:
                return self._reject("range_jump", command)
            position += velocity*dt + .65*innovation
            velocity += .08*innovation/dt
            velocity = max(-80., min(80., velocity))
            closing = -velocity
        self.filter = (stamp, position, velocity)
        return Result(True, "associated", self.selected, distance, forward, position, closing, command, expiry)


class FrameHistory:
    """Bounded recent CameraFrames; finds the consecutive pair around a sample.

    LiDAR (20 Hz) and camera are unsynchronized, so the frame after a sample
    arrives later: keep frames here and call update() once it exists.
    """
    def __init__(self, size=8):
        if not isinstance(size, int) or not 2 <= size <= 64:
            raise ValueError("history size must be 2..64")
        self.size = size
        self.frames = []

    def add(self, frame):
        if not isfinite(frame.time_s) or (self.frames and frame.time_s <= self.frames[-1].time_s):
            raise ValueError("frames must be finite and strictly increasing in time")
        self.frames.append(frame)
        del self.frames[:-self.size]

    def bracket(self, time_s):
        """(before, after) with before.time_s <= time_s <= after.time_s, else None."""
        for before, after in zip(self.frames, self.frames[1:]):
            if before.time_s <= time_s <= after.time_s:
                return before, after
        return None

    def clear(self):
        self.frames = []


class BoxTracker:
    """Small IoU track adapter. No hallucinated boxes on missed detections.

    For occlusion/re-identification use an upstream production tracker instead.
    Ambiguous matches create NEW IDs; explicit selected IDs never auto-switch.
    """
    def __init__(self):
        self.previous = ()
        self.stamp = None
        self.next_id = 0

    @staticmethod
    def iou(a, b):
        area = max(0., min(a[2],b[2])-max(a[0],b[0])) * max(0., min(a[3],b[3])-max(a[1],b[1]))
        union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area
        return area/union if union > 0 else 0.

    def update(self, detections, stamp):
        if not isfinite(stamp) or len(detections) > 64:
            raise ValueError("invalid detections / timestamp")
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError("out-of-order image")
        if self.stamp is not None and stamp-self.stamp > .25:
            self.previous = ()
        for box, confidence, label in detections:
            if (len(box) != 4 or not finite(*box, confidence) or not 0 <= confidence <= 1
                    or not box[0] < box[2] or not box[1] < box[3]):
                raise ValueError("invalid detection")
        matches = [[p for p in self.previous if p.label == label and self.iou(p.box, box) >= .3]
                   for box, _, label in detections]
        output = []
        for i, (box, confidence, label) in enumerate(detections):
            candidates = matches[i]
            if len(candidates) == 1 and sum(candidates[0] in m for m in matches) == 1:
                identity = candidates[0].track_id
            else:
                identity = self.next_id; self.next_id += 1
            output.append(Detection(identity, tuple(box), confidence, label))
        self.previous, self.stamp = tuple(output), stamp
        return self.previous
