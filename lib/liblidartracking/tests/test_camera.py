"""Adapter tests use real rectification/DNN preprocessing, injected predictions.

No claim of accuracy/performance of a downloaded neural model is made here.
"""
import unittest
from unittest.mock import patch

try:
    import cv2
    import numpy as np
    AVAILABLE = True
except ImportError:
    AVAILABLE = False

from lidartracking import Calibration
from lidartracking.camera import CameraAdapter


@unittest.skipUnless(AVAILABLE,"OpenCV/NumPy required (run in l4t-ml-gpio)")
class CameraTests(unittest.TestCase):
    def setUp(self):
        self.c = Calibration(1456,1088,1200.,1200.,728.,544.,
                             (1.,0.,0.,0.,1.,0.,0.,0.,1.),(0.,0.,0.),(0.,0.,0.),.001,True)
        self.output = np.zeros((1,84,8400),dtype=np.float32)
        self.output[0,:4,0] = (320,320,80,60)
        self.output[0,6,0] = .95 # COCO class2 car
        self.blob = None
        class Net:
            def setInput(net,blob): self.blob=blob
            def forward(net): return self.output
        with patch.object(cv2.dnn,"readNetFromONNX",return_value=Net()):
            self.adapter = CameraAdapter(self.c,[[1200,0,728],[0,1200,544],[0,0,1]],
                                         [0,0,0,0,0],"test-only.onnx")

    def test_mapping_rectification_and_ids(self):
        image=np.zeros((1088,1456,3),dtype=np.uint8)
        frame,rectified=self.adapter.process(image,1.,.005)
        self.assertEqual(self.blob.shape,(1,3,640,640))
        self.assertEqual(rectified.shape,image.shape)
        self.assertEqual(len(frame.detections),1)
        self.assertEqual(frame.detections[0].label,"car")
        box=frame.detections[0].box
        self.assertAlmostEqual((box[0]+box[2])/2,728.,places=3)
        self.assertAlmostEqual((box[1]+box[3])/2,544.,places=3)
        second,_=self.adapter.process(image,1.05,.005)
        self.assertEqual(second.detections[0].track_id,frame.detections[0].track_id)

    def test_wrong_shape_and_model(self):
        with self.assertRaises(ValueError): self.adapter.process(np.zeros((10,10,3),np.uint8),1.,0.)
        self.output=np.zeros((1,85,8400),np.float32)
        with self.assertRaises(ValueError): self.adapter.process(np.zeros((1088,1456,3),np.uint8),1.,0.)

    def test_no_vehicle_no_track(self):
        self.output.fill(0)
        frame,_=self.adapter.process(np.zeros((1088,1456,3),np.uint8),1.,0.)
        self.assertEqual(frame.detections,())

    def test_nonfinite_model_output(self):
        self.output[0,0,0]=float("nan")
        with self.assertRaises(ValueError): self.adapter.process(np.zeros((1088,1456,3),np.uint8),1.,0.)
