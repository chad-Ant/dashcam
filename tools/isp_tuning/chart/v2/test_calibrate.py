#!/usr/bin/env python3
"""Synthetic regression suite. No camera access. Temp inputs/results are auto-cleaned."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import calibrate as c


def scene(kind):
    image,_=c.read_rgb(c.HERE/'output'/f'{c.SHEETS[kind]}_300dpi.png','standard')
    image=cv2.resize(image,(1100,778),interpolation=cv2.INTER_AREA)
    frame=np.full((1088,1456,3),220,np.float32)
    frame[155:933,178:1278]=image
    return frame


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.images={k:scene(k) for k in c.SHEETS}

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='chart-v2-test-')
        self.root=Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def write(self,name,rgb):
        path=self.root/name
        c.write_png(path,rgb)
        return path

    def flat(self,kind,paths,label='data',**kwargs):
        out=self.root/label
        out.mkdir()
        return c.analyse_flat(paths,c.load_geometry(kind),'standard',out,label,**kwargs)

    def test_all_three_sheets(self):
        for kind,image in self.images.items():
            h,points,report=c.locate(image,c.load_geometry(kind))
            self.assertEqual(points.shape,(16,2))
            self.assertLess(report['reprojection_max_px'],1)
            self.assertEqual(h.shape,(3,3))

    def test_wrong_sheet(self):
        with self.assertRaisesRegex(ValueError,'all four'):
            c.locate(self.images['noise'],c.load_geometry('colour'))

    def test_missing_marker(self):
        image=self.images['colour'].copy()
        image[155:320,178:340]=255
        with self.assertRaisesRegex(ValueError,'all four'):
            c.locate(image,c.load_geometry('colour'))

    def test_small_chart(self):
        small=cv2.resize(self.images['colour'],(728,544))
        h,_,_=c.locate(small,c.load_geometry('colour'))
        with self.assertRaisesRegex(ValueError,'too small'):
            c.roi_indices(small.shape,h,c.load_geometry('colour')['patches'][0]['roi_mm'],64)

    def test_roi_outside_frame(self):
        with self.assertRaisesRegex(ValueError,'outside'):
            c.roi_indices((100,100,3),np.eye(3),[-5,5,50,50])

    def test_flat_nominal_means(self):
        path=self.write('colour.png',self.images['colour'])
        report=self.flat('colour',[path])
        self.assertTrue(report['valid'])
        for p in c.load_geometry('colour')['patches']:
            np.testing.assert_allclose(report['patches'][p['id']]['mean_rgb'],p['nominal_srgb'],atol=1)

    def test_duplicate_frame(self):
        a=self.write('one.png',self.images['colour'])
        b=self.write('two.png',self.images['colour'])
        with self.assertRaisesRegex(ValueError,'Duplicate decoded'):
            self.flat('colour',[a,b])

    def test_motion_rejected(self):
        a=self.write('one.png',self.images['colour'])
        moved=cv2.warpAffine(self.images['colour'],np.float32([[1,0,3],[0,1,0]]),(1456,1088))
        b=self.write('two.png',moved)
        with self.assertRaisesRegex(ValueError,'motion/jitter'):
            self.flat('colour',[a,b])

    def test_drift_rejected(self):
        a=self.write('one.png',self.images['colour'])
        b=self.write('two.png',np.clip(self.images['colour']+5,0,255))
        report=self.flat('colour',[a,b])
        self.assertFalse(report['valid'])
        self.assertTrue(any('drift' in issue for issue in report['quality_issues']))

    def test_clipping_rejected(self):
        image=self.images['colour'].copy()
        geo=c.load_geometry('colour')
        h,_,_=c.locate(image,geo)
        image[c.roi_indices(image.shape,h,geo['patches'][0]['roi_mm'])]=255
        report=self.flat('colour',[self.write('clip.png',image)])
        self.assertFalse(report['valid'])

    def test_temporal_estimator(self):
        rng=np.random.default_rng(913)
        fixed=rng.normal(100,10,(5000,3))
        moments=c.Moments(fixed)
        for _ in range(64):
            moments.add(fixed+rng.normal(0,3,fixed.shape))
        report=moments.report()
        np.testing.assert_allclose(report['temporal_rms_std_rgb'],3,atol=.05)
        np.testing.assert_allclose(report['spatial_std_mean_image_rgb'],10,atol=.3)
        for value in report['pair_difference_rms_rgb'].values():
            np.testing.assert_allclose(value,3,atol=.05)

    def test_correlated_noise_lags(self):
        rng=np.random.default_rng(7)
        state=np.zeros((3000,3))
        moments=c.Moments(state)
        for _ in range(128):
            state=.9*state+rng.normal(0,1,state.shape)
            moments.add(100+state)
        lags=moments.report()['pair_difference_rms_rgb']
        self.assertGreater(np.mean(lags['8']),np.mean(lags['1'])*1.8)

    def test_known_matrix(self):
        rng=np.random.default_rng(88)
        source=rng.uniform(.05,.8,(18,3))
        expected=np.array([[1.10,-.06,-.04],[-.05,1.10,-.05],[-.03,-.10,1.13]])
        actual=c.fit_matrix(source,source@expected.T,ridge=1e-10)
        np.testing.assert_allclose(actual,expected,atol=1e-8)
        np.testing.assert_allclose(actual.sum(axis=1),1,atol=1e-12)

    def test_degenerate_fit(self):
        with self.assertRaisesRegex(ValueError,'rank-deficient'):
            c.fit_matrix(np.full((18,3),.4),np.full((18,3),.4))

    def test_colour_fit_holdout(self):
        ref=self.flat('colour',[self.write('ref.png',self.images['colour'])],'ref')
        cast=np.array([[.86,.08,.06],[.06,.89,.05],[.04,.09,.87]])
        source=c.encoded(c.linear(self.images['colour'])@cast.T)
        measured=self.flat('colour',[self.write('csi.png',source)],'csi')
        fit=c.colour_fit(measured,ref)
        self.assertEqual(fit['status'],'provisional_candidate')
        self.assertLess(fit['scores']['matrix_holdout_mean_de76'],1)
        self.assertFalse(set(fit['train_ids']) & set(fit['holdout_ids']))
        self.assertFalse(fit['installable_isp_profile'])

    def test_identical_reference_no_recommendation(self):
        ref=self.flat('colour',[self.write('ref.png',self.images['colour'])])
        fit=c.colour_fit(ref,ref)
        self.assertEqual(fit['status'],'not_recommended')
        self.assertLess(fit['scores']['matrix_holdout_mean_de76'],.001)

    def test_detail_blur_increases_width(self):
        image=self.images['detail']
        geo=c.load_geometry('detail')
        h,_,_=c.locate(image,geo)
        blur=cv2.GaussianBlur(image,(0,0),2)
        for edge in geo['edges']:
            sharp=c.edge_metric(image,h,edge)
            soft=c.edge_metric(blur,h,edge)
            self.assertGreater(soft['width_10_90_px'],sharp['width_10_90_px']+3)
            self.assertAlmostEqual(soft['width_10_90_px'],5.2,delta=.8)

    def test_input_natural_order_and_overlap(self):
        a=self.write('f2.png',self.images['colour'])
        b=self.write('f10.png',self.images['colour'])
        self.assertEqual(c.files([str(self.root/'*.png')]),[a,b])
        with self.assertRaisesRegex(ValueError,'duplicate paths'):
            c.files([str(a),str(a)])

    def test_noise_requires_burst(self):
        p=self.write('f.png',self.images['noise'])
        with self.assertRaisesRegex(ValueError,'32..256'):
            c.main(['noise','--images',str(p),'--out',str(self.root/'result')])
        self.assertFalse((self.root/'result').exists())

    def test_no_overwrite(self):
        p=self.write('f.png',self.images['colour'])
        with self.assertRaises(FileExistsError):
            c.main(['colour','--images',str(p),'--out',str(self.root)])
        self.assertTrue(p.exists())

    def test_forced_nvjpeg_rejects_png(self):
        p=self.write('f.png',self.images['colour'])
        with self.assertRaisesRegex(ValueError,'not a recognized'):
            c.read_rgb(p,'nvjpeg')

    def test_auto_decode_standard(self):
        p=self.write('f.png',self.images['colour'])
        auto,mode=c.read_rgb(p)
        standard,_=c.read_rgb(p,'standard')
        np.testing.assert_equal(auto,standard)
        self.assertEqual(mode,'standard-full-range')

    def test_full_noise_command(self):
        rng=np.random.default_rng(56)
        image=self.images['noise']
        geo=c.load_geometry('noise')
        h,_,_=c.locate(image,geo)
        indices=[c.roi_indices(image.shape,h,p['roi_mm']) for p in geo['patches'] if p['id'].startswith('N')]
        for i in range(32):
            frame=image.copy()
            for ix in indices:
                frame[ix] += rng.normal(0,2,frame[ix].shape)
            self.write(f'noise_{i:03}.png',frame)
        out=self.root/'result'
        status=c.main(['noise','--images',str(self.root/'noise_*.png'),'--out',str(out)])
        self.assertEqual(status,0)
        report=json.loads((out/'report.json').read_text())
        np.testing.assert_allclose(report['noise']['patches']['N096']['temporal_rms_std_rgb'],2,atol=.08)

    def test_failure_report(self):
        p=self.write('wrong.png',self.images['noise'])
        out=self.root/'result'
        with self.assertRaises(ValueError):
            c.main(['colour','--images',str(p),'--out',str(out)])
        report=json.loads((out/'report.json').read_text())
        self.assertFalse(report['valid'])
        self.assertIn('all four',report['error'])


if __name__=='__main__':
    unittest.main(verbosity=2)
