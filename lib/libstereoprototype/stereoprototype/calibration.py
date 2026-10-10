"""Metric pinhole stereo calibration and headless checkerboard fitting."""
from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np


def matrix(value, shape, name):
    a=np.asarray(value,np.float64)
    if a.shape!=shape or not np.isfinite(a).all():
        raise ValueError(f'Invalid {name}: expected finite {shape}')
    return a


@dataclass
class Calibration:
    width: int
    height: int
    left_id: str
    right_id: str
    K1: np.ndarray
    D1: np.ndarray
    K2: np.ndarray
    D2: np.ndarray
    R: np.ndarray
    T: np.ndarray

    def __post_init__(self):
        if type(self.width) is not int or type(self.height) is not int or not (64<=self.width<=4096 and 64<=self.height<=3000):
            raise ValueError('Calibration dimensions must be integers in 64..4096 x 64..3000')
        if not self.left_id or not self.right_id or self.left_id==self.right_id:
            raise ValueError('Distinct left/right camera IDs required')
        for name in ('K1','K2'):
            k=matrix(getattr(self,name),(3,3),name)
            if k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]) or abs(k[0,1])>1e-8:
                raise ValueError('Invalid pinhole intrinsic matrix')
            setattr(self,name,k)
        for name in ('D1','D2'):
            d=np.asarray(getattr(self,name),np.float64).ravel()
            if d.size not in (4,5,8) or not np.isfinite(d).all():
                raise ValueError('Pinhole distortion needs 4, 5 or 8 finite coefficients')
            setattr(self,name,d)
        self.R=matrix(self.R,(3,3),'R')
        self.T=np.asarray(self.T,np.float64).reshape(3,1)
        if not np.isfinite(self.T).all() or not .01<=np.linalg.norm(self.T)<=5:
            raise ValueError('T must be a metric baseline between 0.01 and 5 metres')
        if not np.allclose(self.R.T@self.R,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(self.R),1,atol=1e-5):
            raise ValueError('R is not a proper rotation')

    @classmethod
    def load(cls,path):
        data=json.loads(Path(path).read_text())
        if data.get('schema')!='stereo-prototype-calibration-v1' or data.get('units')!='metres':
            raise ValueError('Calibration schema/metric units missing')
        return cls(**{k:data[k] for k in cls.__dataclass_fields__})

    def save(self,path,diagnostics=None):
        data={name:(v.tolist() if isinstance(v,np.ndarray) else v) for name,v in vars(self).items()}
        data.update(schema='stereo-prototype-calibration-v1',units='metres',accuracy_verified=False,
                    diagnostics=diagnostics or {})
        with open(path,'x') as output:
            json.dump(data,output,indent=2,allow_nan=False)

    def rectification(self):
        size=(self.width,self.height)
        r1,r2,p1,p2,q,roi1,roi2=cv2.stereoRectify(self.K1,self.D1,self.K2,self.D2,size,
            self.R,self.T,flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
        baseline=-p2[0,3]/p2[0,0]
        if baseline<=0 or abs(p2[1,3])>1e-6 or not np.isclose(p1[0,2],p2[0,2],atol=1e-6):
            raise ValueError('Only horizontal left-to-right positive-disparity rigs supported; check IDs/T sign')
        maps=[cv2.initUndistortRectifyMap(k,d,r,p,size,cv2.CV_32FC1)
              for k,d,r,p in [(self.K1,self.D1,r1,p1),(self.K2,self.D2,r2,p2)]]
        return dict(maps=maps,R1=r1,R2=r2,P1=p1,P2=p2,Q=q,baseline_m=float(baseline),focal_px=float(p1[0,0]),
                    roi1=roi1,roi2=roi2)


def fit_checkerboards(pairs, inner_cols, inner_rows, square_m, left_id, right_id):
    """Fit on 3/4 views, hold out every fourth; never auto-refit on holdouts.

    Board must remain stationary per pair (software-triggered cameras).
    Ordinary symmetric checkerboards can have 180-degree ordering ambiguity;
    visually verify corner order/orientation before trusting any calibration.
    """
    if not 16<=len(pairs)<=100 or not (3<=inner_cols<=20 and 3<=inner_rows<=20) or not .001<=square_m<=1:
        raise ValueError('Need 16..100 pairs, 3..20 inner corners per axis and square size in metres')
    obj=np.zeros((inner_cols*inner_rows,3),np.float32)
    obj[:,:2]=np.mgrid[:inner_cols,:inner_rows].T.reshape(-1,2)*square_m
    points=[[],[]]
    size=None
    for pair in pairs:
        for side,path in enumerate(pair):
            gray=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise ValueError(f'Cannot read calibration image {path}')
            current=(gray.shape[1],gray.shape[0])
            if not (64<=current[0]<=4096 and 64<=current[1]<=3000):
                raise ValueError('Unsupported calibration image size')
            if size is None:
                size=current
            if current!=size:
                raise ValueError('Image dimensions changed across calibration pairs')
            ok,corners=cv2.findChessboardCornersSB(gray,(inner_cols,inner_rows))
            if not ok:
                raise ValueError(f'All board corners required: {path}')
            points[side].append(corners.astype(np.float32))
    train=[i for i in range(len(pairs)) if i%4!=3]
    hold=[i for i in range(len(pairs)) if i%4==3]
    objects=[obj for _ in train]
    lp=[points[0][i] for i in train]
    rp=[points[1][i] for i in train]
    rms1,k1,d1,_,_=cv2.calibrateCamera(objects,lp,size,None,None)
    rms2,k2,d2,_,_=cv2.calibrateCamera(objects,rp,size,None,None)
    rms,k1,d1,k2,d2,r,t,_,_=cv2.stereoCalibrate(objects,lp,rp,k1,d1,k2,d2,size,
        flags=cv2.CALIB_FIX_INTRINSIC,criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,1e-7))
    cal=Calibration(*size,left_id,right_id,k1,d1,k2,d2,r,t)
    rect=cal.rectification()
    errors=[]
    for i in hold:
        a=cv2.undistortPoints(points[0][i],k1,d1,R=rect['R1'],P=rect['P1']).reshape(-1,2)
        b=cv2.undistortPoints(points[1][i],k2,d2,R=rect['R2'],P=rect['P2']).reshape(-1,2)
        errors.extend(abs(a[:,1]-b[:,1]))
    diagnostics=dict(train_indices=train,holdout_indices=hold,left_rms_px=rms1,right_rms_px=rms2,
        stereo_rms_px=rms,holdout_vertical_p95_px=float(np.percentile(errors,95)),
        baseline_m=rect['baseline_m'],measured_range_validation_required=True)
    if max(rms1,rms2,rms)>1.0 or diagnostics['holdout_vertical_p95_px']>1.0:
        raise ValueError(f'Calibration quality failed: {diagnostics}')
    return cal,diagnostics
