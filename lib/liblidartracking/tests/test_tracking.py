import unittest
from dataclasses import replace
import math
import time
from lidartracking import *
from lidartracking.bridge import (crc_line, decode_line, ClockMapper, BridgeDecoder, LatencyBounds,
                                 SerialBridge)
from lidartracking.__main__ import synthetic


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.c = Calibration(1456,1088,1200.,1200.,728.,544.,
            (1.,0.,0.,0.,1.,0.,0.,0.,1.),(0.,0.,0.),(0.,0.,0.),.001,True)
        self.r = VehicleRanger(self.c)
        self.r.select(7)
        self.f = CameraFrame(1.,.001,1456,1088,(Detection(7,(650,480,806,608),.95),))
        self.s = RangeSample(1,1.,.002,100.,100.,20.,20.,True)
        self.p = ServoFeedback(1.01,0.,.001,True,.9,1.01)

    def result(self, **kw):
        return self.r.update(kw.get("f",self.f),kw.get("s",self.s),kw.get("p",self.p),kw.get("now",1.01))

    def test_good_range_units(self):
        r = self.result()
        self.assertTrue(r.valid)
        self.assertEqual(r.slant_m,100.)
        self.assertEqual(r.forward_m,100.)
        self.assertIsNone(r.closing_mps)

    def test_default_uncalibrated(self):
        self.r = VehicleRanger(replace(self.c,verified=False))
        self.assertEqual(self.result().reason,"uncalibrated")

    def test_servo_requires_encoder(self):
        for p in (None, replace(self.p,settled=False), replace(self.p,settled_since_s=1.),
                  replace(self.p,valid_until_s=.99), replace(self.p,pan_rad=1.),
                  replace(self.p,error_rad=float("nan")), replace(self.p,valid_until_s=2.)):
            self.r.last_sequence = None
            self.assertEqual(self.result(p=p).reason,"servo_not_verified")

    def test_time_and_sequences(self):
        self.assertTrue(self.result().valid)
        self.assertEqual(self.result().reason,"range_out_of_order")
        self.assertEqual(self.result(s=replace(self.s,sequence=2,time_s=.9)).reason,"range_out_of_order")

    def test_range_sequence_wrap(self):
        self.r.last_sequence=0xffffffff; self.r.last_time=.95
        self.assertTrue(self.result(s=replace(self.s,sequence=0)).valid)

    def test_stale_and_future(self):
        for s in (replace(self.s,time_s=.8),replace(self.s,time_s=1.02),replace(self.s,uncertainty_s=.2)):
            self.r.last_sequence=None
            self.assertEqual(self.result(s=s).reason,"stale_or_unsynchronized")

    def test_unknown_timing(self):
        self.assertEqual(self.result(s=replace(self.s,timing_verified=False)).reason,"invalid_range_or_timing")

    def test_return_gates(self):
        for s in (replace(self.s,first_m=-.1),replace(self.s,last_m=0),replace(self.s,first_m=200),
                  replace(self.s,first_db=-1)):
            self.r.last_sequence=None
            self.assertEqual(self.result(s=s).reason,"no_usable_return")

    def test_multireturn(self):
        self.assertEqual(self.result(s=replace(self.s,last_m=120)).reason,"multiple_returns")

    def test_nonfinite(self):
        self.assertEqual(self.result(s=replace(self.s,first_m=float("nan"))).reason,"invalid_range_or_timing")

    def test_no_horizontal_fix_for_vertical_error(self):
        f = replace(self.f,detections=(Detection(7,(650,650,806,778),.95),))
        self.assertEqual(self.result(f=f).reason,"beam_outside_target")

    def test_overlap(self):
        f = replace(self.f,detections=self.f.detections+(Detection(8,(670,500,780,600),.8),))
        self.assertEqual(self.result(f=f).reason,"ambiguous_target")

    def test_missing_target_does_not_switch(self):
        f = replace(self.f,detections=(Detection(8,self.f.detections[0].box,.95),))
        self.assertEqual(self.result(f=f).reason,"target_missing_or_unreachable")
        self.assertEqual(self.r.selected,7)

    def test_resolution_distortion_and_duplicate_id(self):
        for f in (replace(self.f,width=1280),replace(self.f,rectified=False),
                  replace(self.f,detections=self.f.detections*2)):
            self.assertEqual(self.result(f=f).reason,"invalid_camera_frame")

    def test_filter_resets_on_missing(self):
        self.assertTrue(self.result().valid)
        self.r.update(replace(self.f,time_s=1.05),replace(self.s,sequence=2,time_s=1.05,first_m=-.1),
                      replace(self.p,time_s=1.06,valid_until_s=1.06),1.06)
        self.assertIsNone(self.r.filter)

    def test_target_switch_resets_filter(self):
        self.result(); self.r.select(8)
        self.assertIsNone(self.r.filter)

    def test_aim_limit_not_saturated(self):
        self.r.pan_limits=(-.01,.01)
        f=replace(self.f,detections=(Detection(7,(1000,480,1150,608),.95),))
        self.assertIsNone(self.r.aim(f,1.01))

    def test_camera_out_of_order(self):
        self.result()
        self.assertEqual(self.result(f=replace(self.f,time_s=.99)).reason,"camera_out_of_order")

    def test_parallax(self):
        self.r=VehicleRanger(replace(self.c,pivot_m=(2.,0.,0.)))
        self.r.select(7)
        self.assertEqual(self.result(s=replace(self.s,first_m=10,last_m=10)).reason,"beam_outside_target")

    def test_calibration_rotation(self):
        with self.assertRaises(ValueError): replace(self.c,rotation=(1,0,0,0,1,0,0,0,-1))
        with self.assertRaises(ValueError): replace(self.c,fx=float("nan"))
        with self.assertRaises(ValueError): replace(self.c,verified="yes")

    def test_motion_skew_expands_footprint(self):
        self.assertEqual(self.result(s=replace(self.s,time_s=.97)).reason,"beam_outside_target")

    def test_synthetic(self):
        results=list(synthetic())
        self.assertEqual(len(results),20)
        self.assertEqual(sum(r.valid for r in results),19)
        self.assertEqual(results[10].reason,"no_usable_return")


class InterpolationTests(unittest.TestCase):
    """Real-size car (1.8 x 1.5 m) at 40 m, 30 fps camera, sample between frames."""
    def setUp(self):
        self.c = Calibration(1456,1088,1200.,1200.,728.,544.,
            (1.,0.,0.,0.,1.,0.,0.,0.,1.),(0.,0.,0.),(0.,0.,0.),.001,True)
        self.r = VehicleRanger(self.c); self.r.select(7)
        self.f0 = CameraFrame(1.,.001,1456,1088,(Detection(7,self.car(40.),.95),))
        self.f1 = replace(self.f0,time_s=1.+1/30)
        self.s = RangeSample(1,1.+1/60,.002,40.,40.,20.,20.,True)
        self.p = ServoFeedback(1.06,0.,.001,True,.9,1.06)

    @staticmethod
    def car(distance, dx=0.):
        w, h = 1200*1.8/distance, 1200*1.5/distance
        return (728+dx-w/2, 544-h/2, 728+dx+w/2, 544+h/2)

    def update(self, f0=None, f1="pair", s=None, ranger=None):
        f1 = self.f1 if f1 == "pair" else f1
        return (ranger or self.r).update(f0 or self.f0, s or self.s, self.p, 1.07, next_frame=f1)

    def test_single_frame_cannot_reach_real_car(self):
        self.assertEqual(self.update(f1=None).reason,"beam_outside_target")

    def test_bracketing_pair_reaches_real_car(self):
        r = self.update()
        self.assertTrue(r.valid, r.reason)
        self.assertEqual(r.slant_m,40.)

    def test_moving_car_interpolated_to_lidar_time(self):
        # Car crosses 24 px between frames; only its midpoint position is on the beam.
        f0 = replace(self.f0,detections=(Detection(7,self.car(40.,-12.),.95),))
        f1 = replace(self.f1,detections=(Detection(7,self.car(40.,12.),.95),))
        self.assertTrue(self.update(f0,f1).valid)
        self.r.last_sequence = None
        self.assertEqual(self.update(f0,None).reason,"beam_outside_target")

    def test_unverified_acceleration_bound_is_honoured(self):
        r = VehicleRanger(replace(self.c,motion_accel_bound_px_s2=1e7)); r.select(7)
        self.assertEqual(self.update(ranger=r).reason,"beam_outside_target")

    def test_pair_contracts(self):
        cases = ((dict(s=replace(self.s,time_s=1.04)),"range_not_bracketed"),
                 (dict(f1=replace(self.f1,time_s=1.)),"invalid_frame_pair"),
                 (dict(f1=replace(self.f1,time_s=1.2)),"invalid_frame_pair"),
                 (dict(f1=replace(self.f1,detections=())),"target_missing_or_unreachable"),
                 (dict(f1=replace(self.f1,width=1280)),"invalid_camera_frame"))
        for kwargs, reason in cases:
            self.r.last_sequence = None; self.r.last_frame_time = None
            self.assertEqual(self.update(**kwargs).reason,reason)

    def test_other_car_seen_only_in_next_frame(self):
        f1 = replace(self.f1,detections=self.f1.detections+(Detection(8,self.car(40.),.9),))
        self.assertEqual(self.update(f1=f1).reason,"ambiguous_target")

    def test_frame_history(self):
        h = FrameHistory(3)
        for i in range(5):
            h.add(replace(self.f0,time_s=1.+i/30))
        self.assertEqual(len(h.frames),3)
        before, after = h.bracket(1.+2.5/30)
        self.assertEqual((before.time_s,after.time_s),(1.+2/30,1.+3/30))
        self.assertIsNone(h.bracket(1.))   # trimmed
        self.assertIsNone(h.bracket(1.5))  # its next frame has not arrived yet
        with self.assertRaises(ValueError): h.add(replace(self.f0,time_s=1.))
        with self.assertRaises(ValueError): FrameHistory(1)


class FirstReturnTests(unittest.TestCase):
    """Low car (1.8 x 1.3 m) at 50 m, beyond "contained" reach (~38 m).

    Beam centre is image row 544. Box rows are chosen so the beam sits in the
    lower body: y1=531 (top) .. 562.2 (ground contact)."""
    def setUp(self):
        self.c = Calibration(1456,1088,1200.,1200.,728.,544.,
            (1.,0.,0.,0.,1.,0.,0.,0.,1.),(0.,0.,0.),(0.,0.,0.),.001,True)
        self.r = self.ranger("first_return")
        self.f0 = self.frame(1., self.car(531.))
        self.s = RangeSample(1,1.+1/60,.002,50.,50.,20.,20.,True)
        self.p = ServoFeedback(1.06,0.,.001,True,.9,1.06)

    def ranger(self, mode):
        r = VehicleRanger(self.c, association=mode); r.select(7)
        return r

    @staticmethod
    def car(top, distance=50.):
        w, h = 1200*1.8/distance, 1200*1.3/distance
        return (728-w/2, top, 728+w/2, top+h)

    @staticmethod
    def frame(t, *boxes):
        return CameraFrame(t,.001,1456,1088,tuple(Detection(7+i,b,.95) for i, b in enumerate(boxes)))

    def update(self, f0=None, s=None, ranger=None):
        f0 = f0 or self.f0
        return (ranger or self.r).update(f0, s or self.s, self.p, 1.07, next_frame=replace(f0,time_s=1.+1/30))

    def test_low_car_beyond_contained_reach(self):
        r = self.update()
        self.assertTrue(r.valid, r.reason)
        self.assertEqual(r.slant_m, 50.)
        self.assertEqual(self.update(ranger=self.ranger("contained")).reason, "beam_outside_target")

    def test_spill_below_ground_contact_rejected(self):
        # Box 7 px higher: the beam's lower edge reaches the road nearer than the car.
        self.assertEqual(self.update(self.frame(1., self.car(524.))).reason, "beam_outside_target")

    def test_beam_centre_above_car_rejected(self):
        # Box 6 px lower: the beam centre may be above the roof, the car barely in it.
        self.assertEqual(self.update(self.frame(1., self.car(537.))).reason, "beam_outside_target")

    def test_farther_background_return_uses_first(self):
        s = replace(self.s, last_m=90., last_db=5.)
        r = self.update(s=s)
        self.assertTrue(r.valid, r.reason)
        self.assertEqual(r.slant_m, 50.)
        self.assertEqual(self.update(s=s, ranger=self.ranger("contained")).reason, "multiple_returns")
        # A weak or out-of-range background return does not invalidate the car's.
        for s in (replace(self.s, sequence=2, time_s=1.02, last_m=90., last_db=-5.),
                  replace(self.s, sequence=3, time_s=1.03, last_m=250.)):
            self.assertTrue(self.update(s=s).valid)

    def test_unphysical_or_weak_returns_rejected(self):
        for s in (replace(self.s, last_m=45.),          # last nearer than first
                  replace(self.s, last_m=-.1),          # lost-signal sentinel
                  replace(self.s, first_db=-1.),
                  replace(self.s, first_m=150., last_m=150.)):
            self.r.last_sequence = None; self.r.last_frame_time = None
            self.assertEqual(self.update(s=s).reason, "no_usable_return")

    def test_detected_vehicle_in_spill_is_ambiguous(self):
        # A truck ahead, visible above the low car's roof, inside the upward spill.
        f0 = self.frame(1., self.car(531.), (700., 500., 760., 535.))
        self.assertEqual(self.update(f0).reason, "ambiguous_target")

    def test_association_validated(self):
        with self.assertRaises(ValueError): VehicleRanger(self.c, association="nearest")


class TrackerTests(unittest.TestCase):
    def test_ids_and_loss(self):
        t=BoxTracker(); d=[((10,10,100,100),.9,"car")]
        a=t.update(d,1); b=t.update(d,1.05)
        self.assertEqual(a[0].track_id,b[0].track_id)
        t.update([],1.1); c=t.update(d,1.15)
        self.assertNotEqual(a[0].track_id,c[0].track_id)

    def test_ambiguous_matches(self):
        t=BoxTracker(); d=[((10,10,100,100),.9,"car")]
        a=t.update(d,1); b=t.update(d*2,1.05)
        self.assertNotIn(a[0].track_id,[x.track_id for x in b])
        self.assertNotEqual(b[0].track_id,b[1].track_id)

    def test_bad_data(self):
        t=BoxTracker()
        with self.assertRaises(ValueError): t.update([((1,2,0,3),.9,"car")],1.)
        t.update([],1.)
        with self.assertRaises(ValueError): t.update([],1.)


class BridgeTests(unittest.TestCase):
    def test_crc_and_schema(self):
        body=b"L1 00000001 000F4240 00002710 00000019 00002710 00000019"
        kind,fields=decode_line(crc_line(body))
        self.assertEqual(kind,b"L1"); self.assertEqual(fields[2],10000)
        with self.assertRaises(ValueError): decode_line(crc_line(body)[:-2]+b"0\n")
        with self.assertRaises(ValueError): decode_line(crc_line(b"L1 00"))
        with self.assertRaises(ValueError): decode_line(b"x"*100)

    def test_clock_wrap(self):
        c=ClockMapper()
        self.assertTrue(c.observe(0xfffffff0,10.,10.002))
        stamp,error=c.map(0x10,10.005)
        self.assertAlmostEqual(stamp,10.001032)
        self.assertGreater(error,.001)
        self.assertIsNone(c.map(0x10,20.))
        self.assertFalse(c.observe(1,10,11))

    def test_unsynced_not_trusted(self):
        d=BridgeDecoder()
        s=d.sample((1,1000000,10000,20,10000,20),1.01)
        self.assertFalse(s.timing_verified)
        self.assertEqual(s.first_m,100.)

    def test_latency_and_reboot(self):
        d=BridgeDecoder(LatencyBounds(.01,.02))
        d.clock.observe(1000000,1.,1.002)
        s=d.sample((20,1000000,10000,20,0xfffffff6,20),1.003)
        self.assertTrue(s.timing_verified)
        self.assertEqual(s.last_m,-.1)
        self.assertAlmostEqual(s.time_s,.986)
        s=d.sample((1,2000000,10000,20,10000,20),2.)
        self.assertFalse(s.timing_verified)
        self.assertIsNone(d.clock.anchor)

    def test_invalid_bounds(self):
        for a,b in ((-.1,.1),(.1,0),(0,float("nan"))):
            with self.assertRaises(ValueError): LatencyBounds(a,b)


class FakeMkr:
    """Stands in for os/select: answers Q1 with T1 after reply_s of real time."""
    def __init__(self, reply_s):
        self.reply_s, self.ready_at, self.out = reply_s, None, b""

    def write(self, fd, data):
        self.out = crc_line(b"T1 " + data[3:11] + b" 000F4240")
        self.ready_at = time.monotonic()+self.reply_s
        return len(data)

    def read(self, fd, n):
        if self.ready_at is None or time.monotonic() < self.ready_at:
            raise BlockingIOError
        data, self.out, self.ready_at = self.out, b"", None
        return data

    def select(self, r, w, x, timeout):
        if self.ready_at is None:
            time.sleep(timeout); return [], [], []
        wait = self.ready_at-time.monotonic()
        time.sleep(max(0., min(wait, timeout)))
        return (r, [], []) if wait <= timeout else ([], [], [])


def fake_bridge(mkr, wait=.02):
    b = SerialBridge.__new__(SerialBridge) # skip the tty open; I/O is faked
    b.os, b.select, b.sync_wait_s, b.fd = mkr, mkr.select, wait, 3
    b.decoder, b.buffer, b.discarding = BridgeDecoder(), bytearray(), False
    b.pending, b.next_sync, b.nonce, b.status = None, 0., 0, None
    return b


class SyncWaitTests(unittest.TestCase):
    def test_one_poll_anchors_with_true_round_trip(self):
        b = fake_bridge(FakeMkr(.002))
        b.poll() # a 20 Hz caller would otherwise measure >= 50 ms and never sync
        self.assertIsNotNone(b.decoder.clock.anchor)
        self.assertLess(b.decoder.clock.anchor[2], .01)
        self.assertIsNone(b.pending)

    def test_wait_is_bounded_and_late_reply_still_matched(self):
        mkr = FakeMkr(.03)
        b = fake_bridge(mkr)
        start = time.monotonic(); b.poll()
        self.assertLess(time.monotonic()-start, .028)
        self.assertIsNone(b.decoder.clock.anchor)
        self.assertIsNotNone(b.pending)
        time.sleep(.01); b.poll() # 40 ms RTT: accepted, with honest error
        self.assertGreater(b.decoder.clock.anchor[2], .015)

    def test_sync_wait_validated(self):
        for wait in (0., .06, 1):
            with self.assertRaises(ValueError):
                SerialBridge("/must/not/be/opened", bench_firmware=True, sync_wait_s=wait)


if __name__ == "__main__": unittest.main()
