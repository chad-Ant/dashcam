import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from stereoprototype.calibration import Calibration,fit_checkerboards
from stereoprototype.core import Rangefinder,Settings
from stereoprototype.geometry import error_budget,focal_from_hfov
from stereoprototype.timing import ExposurePair
from stereoprototype.__main__ import demo,evaluate_pair,write_json,main


def calibration(width=640,height=240,fx=500,baseline=.4):
    k=np.array([[fx,0,width/2],[0,fx,height/2],[0,0,1.]])
    return Calibration(width,height,'L','R',k,np.zeros(5),k.copy(),np.zeros(5),np.eye(3),[-baseline,0,0])


def scene(disparity=20,width=640,height=240):
    rng=np.random.default_rng(180)
    left=rng.integers(20,230,(height,width),dtype=np.uint8)
    left=cv2.GaussianBlur(left,(3,3),.6)
    right=cv2.warpAffine(left,np.float32([[1,0,-disparity],[0,1,0]]),(width,height),borderMode=cv2.BORDER_REFLECT)
    return left,right


def proof(**changes):
    params=dict(trigger_left=1,trigger_right=1,clock_domain='shared_hardware_clock',
                left_mid_us=1000.,right_mid_us=1020.,uncertainty_each_us=10.,exposure_us=100.,
                global_shutter=True,hardware_sync_verified=True)
    params.update(changes)
    return ExposurePair(**params)


class GeometryTests(unittest.TestCase):
    def test_focal(self):
        self.assertAlmostEqual(focal_from_hfov(640,60),554.2562584)

    def test_esp_100m_fails(self):
        b=error_budget(100,focal_from_hfov(1600,60),1,.5)
        self.assertGreater(b['conditional_error_m'],3)
        self.assertFalse(b['within_assumed_budget'])

    def test_35mm_ideal_budget(self):
        b=error_budget(100,35/.00274,1,.5)
        self.assertAlmostEqual(b['conditional_error_m'],.39297,delta=.001)
        self.assertFalse(b['accuracy_verified'])

    def test_singularity(self):
        self.assertIsNone(error_budget(100,100,1,1)['upper_m'])

    def test_invalid_inputs(self):
        for value in (float('nan'),float('inf'),-1,0):
            with self.assertRaises(ValueError):
                error_budget(value,100,1,.5)
        with self.assertRaises(ValueError):
            error_budget(1,100,1,.5,scale_error_fraction=1)

    def test_motion_penalty(self):
        clean=error_budget(100,10000,1,.5)
        bad=error_budget(100,10000,1,.5,skew_s=.01,lateral_speed_mps=10)
        self.assertGreater(bad['conditional_error_m'],10*clean['conditional_error_m'])

    def test_budget_cli_json(self):
        with patch('builtins.print') as printed:
            self.assertEqual(main(['budget','--focal-px','12774','--baseline-m','1',
                                  '--disparity-error-px','.5','--yaw-deg-s','5','--skew-ms','.2']),0)
        data=json.loads(printed.call_args[0][0])
        self.assertIsInstance(data[-1]['within_assumed_budget'],bool)


class TimingTests(unittest.TestCase):
    def test_bound_includes_uncertainty_and_exposure(self):
        self.assertAlmostEqual(proof().effective_interval_s(),140e-6)

    def test_reject_unsynced_or_rolling(self):
        for change in [dict(global_shutter=False),dict(hardware_sync_verified=False),
                       dict(global_shutter='yes'),dict(clock_domain='host_arrival')]:
            with self.assertRaises(ValueError):
                proof(**change).effective_interval_s()

    def test_sequence_mismatch(self):
        with self.assertRaises(ValueError):
            proof(trigger_right=2).effective_interval_s()

    def test_excess_skew(self):
        with self.assertRaises(ValueError):
            proof(right_mid_us=1200).effective_interval_s()

    def test_nan(self):
        with self.assertRaises(ValueError):
            proof(exposure_us=float('nan')).effective_interval_s()


class CoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(2)

    def finder(self):
        return Rangefinder(calibration(),Settings(3,100,.5,num_disparities=80))

    def test_depth_and_lr(self):
        f=self.finder()
        result=f.compute(*scene(),static_scene=True)
        m=f.measure_roi(result,[220,60,180,100])
        self.assertTrue(m['valid'])
        self.assertAlmostEqual(m['depth_m'],10,delta=.1)
        self.assertFalse(m['accuracy_verified'])

    def test_crop_preserves_range_and_coordinates(self):
        f=self.finder()
        roi=[250,60,180,100]
        cropped=f.compute(*scene(),static_scene=True,region=roi)
        full=f.compute(*scene(),static_scene=True)
        self.assertNotEqual(cropped['origin_xy'],(0,0))
        self.assertAlmostEqual(f.measure_roi(cropped,roi)['depth_m'],f.measure_roi(full,roi)['depth_m'],delta=.1)

    def test_moving_requires_proof(self):
        with self.assertRaisesRegex(ValueError,'exposure metadata'):
            self.finder().compute(*scene())

    def test_moving_with_bounds(self):
        f=self.finder()
        result=f.compute(*scene(),exposure_pair=proof(),lateral_speed_bound_mps=5,
                         yaw_rate_bound_rad_s=.05,forward_speed_bound_mps=20)
        m=f.measure_roi(result,[220,60,180,100])
        self.assertTrue(m['valid'])
        self.assertGreater(m['budget']['motion_disparity_px'],0)
        self.assertGreater(m['budget']['forward_motion_allowance_m'],0)

    def test_missing_motion_bound(self):
        with self.assertRaisesRegex(ValueError,'bound'):
            self.finder().compute(*scene(),exposure_pair=proof())

    def test_blank_scene_rejected(self):
        f=self.finder()
        image=np.full((240,640),100,np.uint8)
        result=f.compute(image,image,static_scene=True)
        self.assertFalse(f.measure_roi(result,[220,60,180,100])['valid'])

    def test_wrong_shape(self):
        with self.assertRaises(ValueError):
            self.finder().compute(*scene(width=320),static_scene=True)

    def test_bad_roi(self):
        f=self.finder()
        result=f.compute(*scene(),static_scene=True)
        with self.assertRaises(ValueError):
            f.measure_roi(result,[-5,0,40,40])

    def test_invalid_matching_settings(self):
        for kwargs in [dict(num_disparities=17),dict(block_size=4),dict(disparity_error_px=float('nan'))]:
            args=dict(min_range_m=3,max_range_m=100,disparity_error_px=.5)
            args.update(kwargs)
            with self.assertRaises(ValueError):
                Settings(**args)

    def test_near_range_search(self):
        with self.assertRaisesRegex(ValueError,'near distance'):
            Rangefinder(calibration(),Settings(1,100,.5,num_disparities=80))

    def test_large_dense_compute_refused(self):
        f=Rangefinder(calibration(1920,1080),Settings(3,100,.5,num_disparities=80))
        image=np.zeros((1080,1920),np.uint8)
        with self.assertRaisesRegex(ValueError,'target ROI'):
            f.compute(image,image,static_scene=True)


class CalibrationTests(unittest.TestCase):
    def test_roundtrip_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'cal.json'
            a=calibration()
            a.save(p)
            b=Calibration.load(p)
            np.testing.assert_equal(a.K1,b.K1)
            self.assertAlmostEqual(b.rectification()['baseline_m'],.4)
            with self.assertRaises(FileExistsError):
                a.save(p)

    def test_units_required(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.json'
            p.write_text('{}')
            with self.assertRaises(ValueError):
                Calibration.load(p)

    def test_left_right_order(self):
        c=calibration()
        c.T=-c.T
        with self.assertRaises(ValueError):
            c.rectification()

    def test_invalid_rotation(self):
        c=calibration()
        with self.assertRaises(ValueError):
            Calibration(c.width,c.height,'L','R',c.K1,c.D1,c.K2,c.D2,np.ones((3,3)),c.T)

    def test_invalid_board_inputs(self):
        with self.assertRaises(ValueError):
            fit_checkerboards([],9,6,.025,'L','R')

    def test_calibration_fit_known_projected_corners(self):
        truth=calibration(640,480,500,.4)
        obj=np.zeros((9*6,3),np.float32)
        obj[:,:2]=np.mgrid[:9,:6].T.reshape(-1,2)*.1
        detections=[]
        for i in range(20):
            r=np.array([-.2+.025*i,.12*np.sin(i),.05*np.cos(i)])
            t=np.array([-.4+.03*(i%3),-.3+.04*(i%4),2+.1*i])
            for offset in (np.zeros(3),truth.T.ravel()):
                corners,_=cv2.projectPoints(obj,r,t+offset,truth.K1,truth.D1)
                detections.append((True,corners))
        with patch('cv2.imread',return_value=np.zeros((480,640),np.uint8)), \
             patch('cv2.findChessboardCornersSB',side_effect=detections):
            cal,report=fit_checkerboards([('L','R')]*20,9,6,.1,'L','R')
        self.assertAlmostEqual(cal.rectification()['baseline_m'],.4,delta=.001)
        self.assertLess(report['holdout_vertical_p95_px'],.01)
        self.assertFalse(set(report['train_indices'])&set(report['holdout_indices']))


class EndToEndTests(unittest.TestCase):
    def test_100m_demo(self):
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)/'demo'
            report=demo(folder)
            m=report['measurement']
            self.assertTrue(m['valid'])
            self.assertAlmostEqual(m['depth_m'],100,delta=.6)
            self.assertFalse(report['accuracy_verified'])
            self.assertTrue((folder/'result/depth_preview.png').is_file())
            with self.assertRaises(FileExistsError):
                demo(folder)
            pair=json.loads((folder/'pair.json').read_text())
            pair['left_id']='wrong-camera'
            write_json(folder/'wrong.json',pair)
            with self.assertRaisesRegex(ValueError,'identities'):
                evaluate_pair(folder/'wrong.json',folder/'calibration.json',folder/'settings.json',
                              [1700,180,500,150],folder/'bad-result')


if __name__=='__main__':
    unittest.main(verbosity=2)
