"""Optional OpenCV adapter. No capture thread, downloads or recorder changes.

Caller supplies IMX296 BGR frames and exposure timestamps. Rectification must
precede detection; track boxes then share the range projector's coordinates.
Model contract: COCO YOLOv8 detection ONNX, fixed 640x640, no embedded NMS,
single output [1,84,8400] (80 classes). Other exports fail explicitly.
"""
import math
from .core import BoxTracker, CameraFrame


class CameraAdapter:
    def __init__(self, calibration, original_k, distortion, model_path):
        import cv2
        import numpy as np
        self.cv, self.np, self.c = cv2, np, calibration
        k = np.asarray(original_k, dtype=np.float64).reshape(3,3)
        d = np.asarray(distortion, dtype=np.float64)
        if (d.size not in (4,5,8,12,14) or not np.isfinite(k).all() or not np.isfinite(d).all()
                or k[0,0] <= 0 or k[1,1] <= 0 or not np.allclose(k[2],[0,0,1])
                or abs(k[0,1]) > 1e-9 or abs(k[1,0]) > 1e-9):
            raise ValueError("invalid intrinsic calibration")
        new_k = np.array([[calibration.fx,0,calibration.cx],[0,calibration.fy,calibration.cy],[0,0,1.]])
        self.map1, self.map2 = cv2.initUndistortRectifyMap(k, d, None, new_k,
            (calibration.width,calibration.height), cv2.CV_32FC1)
        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        # Explicit CPU fallback for portability; production TRT adapter can supply
        # detections directly. Benchmark before sharing Orin CPU with recording.
        self.tracker = BoxTracker()

    def process(self, bgr, exposure_s, uncertainty_s):
        cv, np = self.cv, self.np
        if bgr.shape != (self.c.height,self.c.width,3) or bgr.dtype != np.uint8:
            raise ValueError("image shape/dtype differs from calibration")
        if not math.isfinite(exposure_s) or not math.isfinite(uncertainty_s) or uncertainty_s < 0:
            raise ValueError("invalid exposure timing")
        rectified = cv.remap(bgr, self.map1, self.map2, cv.INTER_LINEAR)
        scale = min(640/self.c.width,640/self.c.height)
        w,h = round(self.c.width*scale),round(self.c.height*scale)
        left,top = (640-w)//2,(640-h)//2
        canvas = np.full((640,640,3),114,dtype=np.uint8)
        canvas[top:top+h,left:left+w] = cv.resize(rectified,(w,h))
        self.net.setInput(cv.dnn.blobFromImage(canvas,1/255.,(640,640),swapRB=True,crop=False))
        output = self.net.forward()
        if output.shape != (1,84,8400) or not np.isfinite(output).all():
            raise ValueError("expected finite COCO YOLOv8 [1,84,8400] output")
        rows = output[0].T
        categories = np.argmax(rows[:,4:],axis=1)
        scores = np.max(rows[:,4:],axis=1)
        # Bound NMS cost before constructing Python lists.
        indices = np.where((scores >= .6) & np.isin(categories,[2,3,5,7]))[0]
        indices = indices[np.argsort(scores[indices])[-256:]]
        boxes,confidences,labels = [],[],[]
        names = {2:"car",3:"motorcycle",5:"bus",7:"truck"}
        for index in indices:
            cx,cy,bw,bh = rows[index,:4]
            x1 = max(0.,float((cx-bw/2-left)*self.c.width/w))
            y1 = max(0.,float((cy-bh/2-top)*self.c.height/h))
            x2 = min(float(self.c.width),float((cx+bw/2-left)*self.c.width/w))
            y2 = min(float(self.c.height),float((cy+bh/2-top)*self.c.height/h))
            if x1 >= x2 or y1 >= y2:
                continue
            boxes.append([x1,y1,x2-x1,y2-y1]); confidences.append(float(scores[index]))
            labels.append(names[int(categories[index])])
        keep = cv.dnn.NMSBoxes(boxes,confidences,.6,.45)
        detections = []
        for index in np.asarray(keep,dtype=int).reshape(-1)[:64]:
            x,y,w,h = boxes[index]
            detections.append(((x,y,x+w,y+h),confidences[index],labels[index]))
        tracks = self.tracker.update(detections,exposure_s)
        return CameraFrame(exposure_s,uncertainty_s,self.c.width,self.c.height,tracks), rectified
