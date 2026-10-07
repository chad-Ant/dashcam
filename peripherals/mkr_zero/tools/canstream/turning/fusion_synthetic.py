"""Deterministic, labelled synthetic multi-sensor course; no hardware claims."""
import csv
import dataclasses
import datetime
import math
from pathlib import Path
import random

from fusion import FusionConfig


def generate(args):
    # Imported at call time to avoid a demo module initialization cycle.
    from demo import pack_fields, write_json
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    config = FusionConfig(gyro_up=(0, 0, 1), front_track_m=1.475, rear_track_m=1.459,
                          antenna_x_m=2., course_sigma_deg=3, gyro_sigma_dps=.6,
                          gyro_time_sigma_s=.01, gnss_time_sigma_s=.05)
    write_json(out / 'fusion.json', dataclasses.asdict(config))
    write_json(out / 'synthetic_truth.json', dict(SYNTHETIC_NOT_HARDWARE=True, seed=71,
               gyro_bias_dps=1.8, wheelbase_m=2.405, note='GNSS outage 35..42 s; FL slip 23..24 s; gyro spike at 50 s'))
    rng, epoch, hz, heading = random.Random(71), 1800000000, 50, math.radians(89)
    fields = ['host_ms', 'master_ms', 'flags', 'imu_calib', 'imu_gx', 'imu_gy', 'imu_gz',
              'heading_deg', 'gps_speed_kmh', 'fix_valid', 'fix_type', 'sats', 'utc']
    with (out / 'can_raw.log').open('x') as can, (out / 'telemetry.csv').open('x') as tlm, (out / 'truth.csv').open('x') as truth:
        writer = csv.DictWriter(tlm, fields)
        writer.writeheader()
        truth_writer = csv.DictWriter(truth, ['time', 'speed_mps', 'yaw_dps', 'angle_deg'])
        truth_writer.writeheader()
        last_course = last_speed = last_utc = None
        for i in range(3001):
            s, t = i / hz, epoch + i / hz
            # Smooth left/right excitation with straight approaches, all within demo envelope.
            v = 2.5
            r = .22 * math.sin((s - 5) * math.pi / 20) if 5 <= s <= 45 else 0.
            if i:
                heading += r / hz
            values = []
            for j, (a, b) in enumerate([(2.405, .7375), (2.405, -.7375), (0., .7295), (0., -.7295)]):
                speed = math.hypot(v - r * b, r * a) + rng.gauss(0, .025)
                if j == 0 and 23 <= s < 24:
                    speed += 2
                values.append(round(speed * 360))
            can.write('(%.6f) can0 191#%s\n' % (t, pack_fields([(44, 5, 4)])))
            can.write('(%.6f) can0 1D0#%s\n' % (t, pack_fields(list(zip((7, 8, 25, 42), (15,) * 4, values)))))
            truth_writer.writerow(dict(time=t, speed_mps=v, yaw_dps=math.degrees(r),
                                       angle_deg=math.degrees(math.atan(2.405 * r / v))))
            if i % hz == 0:
                last_course = (90 - math.degrees(heading + math.atan2(r * 2., v)) + rng.gauss(0, 2)) % 360
                last_speed = math.hypot(v, r * 2.) * 3.6
                last_utc = datetime.datetime.fromtimestamp(t, datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
            if i % 5 == 0:
                gyro = math.degrees(r) + 1.8 + rng.gauss(0, .5)
                if s == 50:
                    gyro = 2000
                writer.writerow(dict(host_ms=round(t * 1000), master_ms=round(s * 1000),
                    flags='0x101c' if 35 <= s < 42 else '0x101e', imu_calib='0x30',
                    imu_gx=0, imu_gy=0, imu_gz=gyro, heading_deg=last_course,
                    gps_speed_kmh=last_speed, fix_valid=0 if 35 <= s < 42 else 1,
                    fix_type=3, sats=9, utc=last_utc))
    print('SYNTHETIC multi-sensor data only:', out)
    return 0
