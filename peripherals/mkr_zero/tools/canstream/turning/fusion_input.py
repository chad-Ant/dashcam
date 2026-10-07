"""Bounded read-only join of v0.4 telemetry CSV and acquisition-stamped CAN.

Telemetry has host ARRIVAL time, not per-channel acquisition time. Offsets and
timing variances are explicit assumptions; no precision clock mapping is invented.
"""
import csv
import datetime
import heapq
import math


class TelemetryInput:
    REQUIRED = {'host_ms', 'master_ms', 'flags', 'imu_calib', 'imu_gx', 'imu_gy', 'imu_gz',
                'heading_deg', 'gps_speed_kmh', 'fix_valid', 'fix_type', 'sats', 'utc'}

    def __init__(self, path, config, follow=False):
        self.path, self.c, self.follow = path, config, follow
        if not path.is_file():
            raise ValueError('telemetry must be a regular CSV file')
        self.stream = path.open('rb')
        try:
            header = self.stream.readline(4097)
            if not header.endswith(b'\n') or len(header) > 4096:
                raise ValueError('telemetry header missing/incomplete/too large')
            self.header = next(csv.reader([header.decode('ascii').strip()]))
            if not self.REQUIRED <= set(self.header) or len(set(self.header)) != len(self.header):
                raise ValueError('not a supported v0.4 telemetry CSV header')
            self.identity = path.stat().st_ino
            self.partial = b''
            self.discard = False
            if follow:
                self.stream.seek(0, 2)
                end = self.stream.tell()
                if end:
                    self.stream.seek(end - 1)
                    self.discard = self.stream.read(1) != b'\n'
            self.pending, self.heap = None, []
            self.previous_host = self.previous_master = self.last_utc = None
            self.sequence = self.rows = 0
        except Exception:
            self.stream.close()
            raise

    def close(self):
        self.stream.close()

    def read_row(self):
        stat = self.path.stat()
        if stat.st_ino != self.identity or stat.st_size < self.stream.tell():
            raise ValueError('telemetry replaced/truncated; start a new session')
        # At most two lines: optionally finish the partial tail skipped at startup.
        for _ in range(2):
            block = self.stream.readline(4097)
            self.partial += block
            if len(self.partial) > 4096:
                raise ValueError('telemetry row exceeds 4096 bytes')
            if not self.partial.endswith(b'\n'):
                if self.partial and not self.follow:
                    raise ValueError('incomplete last telemetry row')
                return None
            line, self.partial = self.partial, b''
            if self.discard:
                self.discard = False
                continue
            cells = next(csv.reader([line.decode('ascii').strip()]))
            if len(cells) != len(self.header):
                raise ValueError('telemetry column count changed')
            row = dict(zip(self.header, cells))
            host, master = int(row['host_ms']) / 1000, int(row['master_ms'])
            if not math.isfinite(host) or host < 0 or not 0 <= master <= 0xffffffff:
                raise ValueError('invalid telemetry clock')
            if self.previous_host is not None:
                dh, dm = host - self.previous_host, (master - self.previous_master) / 1000
                if dh < 0 or dm < 0 or abs(dh - dm) > 1:
                    raise ValueError('telemetry clock step/reset/stall: split logs before fusion')
                if dm == 0:
                    continue  # duplicate master frame, no extra sensor evidence
            self.previous_host, self.previous_master = host, master
            self.rows += 1
            return host, row
        return None

    @staticmethod
    def number(row, key):
        try:
            v = float(row[key])
            return v if math.isfinite(v) else None
        except (ValueError, KeyError):
            return None

    def events(self, host, row):
        flags, calib = int(row['flags'], 0), int(row['imu_calib'], 0)
        if not 0 <= flags <= 65535 or not 0 <= calib <= 255:
            raise ValueError('telemetry flags out of range')
        axes = [self.number(row, k) for k in ('imu_gx', 'imu_gy', 'imu_gz')]
        # Conservative rejection of fallback/gap/low-power/saturation windows.
        gyro_ready = ((calib >> 4) & 3) >= 2
        if flags & 0x08 and not flags & (0x20 | 0x40 | 0x80 | 0x200) and gyro_ready and all(v is not None for v in axes):
            yield dict(kind='gyro', time=host + self.c.gyro_time_offset_s,
                       value=sum(a * b for a, b in zip(axes, self.c.gyro_up)))
        # 10 Hz telemetry repeats the 1 Hz fix: never count it ten times.
        utc = row['utc']
        if utc and flags & 0x04:
            try:
                fix_time = datetime.datetime.strptime(utc, '%Y-%m-%dT%H:%M:%SZ')
            except ValueError:
                return  # malformed / unrepresentable leap second: no course update
            if self.last_utc is not None and fix_time <= self.last_utc:
                return
            self.last_utc = fix_time
            heading, speed = self.number(row, 'heading_deg'), self.number(row, 'gps_speed_kmh')
            if (flags & 0x12) == 0x12 and row['fix_valid'] == '1' and int(row['fix_type']) in (2, 3) and 5 <= int(row['sats']) <= 255 and heading is not None and speed is not None:
                yield dict(kind='course', time=host + self.c.gnss_time_offset_s,
                           value=heading, speed_mps=speed / 3.6)

    def until(self, stamp):
        earliest_offset = min(self.c.gyro_time_offset_s, self.c.gnss_time_offset_s)
        # Bound catch-up and memory. Input logs should cover the same session.
        for _ in range(2000):
            if self.pending is None:
                self.pending = self.read_row()
            if self.pending is None:
                break
            host, row = self.pending
            if host + earliest_offset > stamp:
                break
            for event in self.events(host, row):
                self.sequence += 1
                heapq.heappush(self.heap, (event['time'], self.sequence, event))
            self.pending = None
            if len(self.heap) > 4096:
                raise ValueError('telemetry backlog too large; align/split the session logs')
        else:
            raise ValueError('telemetry catch-up exceeds 2000 rows; align/split input logs')
        while self.heap and self.heap[0][0] <= stamp:
            yield heapq.heappop(self.heap)[2]


class FusionSession:
    def __init__(self, profile, config, telemetry):
        from fusion import TurningEKF
        self.filter = TurningEKF(profile, config)
        self.telemetry = telemetry
        self.segment = None
        self.late = 0
        self.last_wheel = None

    def update(self, row):
        t = row['time']
        if self.segment != row['segment']:
            if self.segment is not None:
                # Cannot reliably match two clocks across a CAN clock reset.
                raise ValueError('CAN segment changed: split/re-align logs before fusion')
            self.segment = row['segment']
        events = self.telemetry.until(t)
        for event in events:
            if self.last_wheel is None and event['time'] < t - .5:
                self.late += 1
                continue
            if self.filter.t is not None and event['time'] < self.filter.t - 1e-6:
                self.late += 1
                continue  # no pretending a late sample was acquired now
            self.filter.sensor(event)
        self.filter.wheels(row)
        self.last_wheel = t
        return dict(self.filter.snapshot(t), ekf_late_events=self.late)
