import unittest
from dataclasses import replace
import math
from lidartracking import *
from lidartracking.bridge import (crc_line, decode_line, ClockMapper, BridgeDecoder, LatencyBounds)
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


if __name__ == "__main__": unittest.main()
