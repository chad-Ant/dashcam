"""CPU OpenCV SGBM prototype, without production camera handles or service hooks."""
from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import error_budget, positive


@dataclass(frozen=True)
class Settings:
    min_range_m: float
    max_range_m: float
    disparity_error_px: float  # Assumed bound, NOT inferred from matcher confidence.
    scale_error_fraction: float = .01
    target_error_m: float = 1.
    num_disparities: int = 128
    block_size: int = 5
    lr_tolerance_px: float = 1.
    min_texture_std: float = 4.

    def __post_init__(self):
        for name in ('min_range_m','max_range_m','disparity_error_px','target_error_m','lr_tolerance_px'):
            positive(getattr(self,name),name)
        positive(self.scale_error_fraction,'scale_error_fraction',True)
        positive(self.min_texture_std,'min_texture_std',True)
        if self.max_range_m<=self.min_range_m or self.scale_error_fraction>=1:
            raise ValueError('Invalid range/scale error')
        if self.num_disparities not in range(16,513,16) or self.block_size not in (3,5,7,9,11):
            raise ValueError('Disparities must be a multiple of 16 in 16..512; block size odd 3..11')


class Rangefinder:
    """One instance per worker; not thread-safe. Moving scenes require timing proof."""
    def __init__(self,calibration,settings):
        self.cal=calibration
        self.settings=settings
        self.rect=calibration.rectification()
        if settings.num_disparities>=calibration.width//2:
            raise ValueError('Disparity search must be less than half the image width')
        max_d=self.rect['focal_px']*self.rect['baseline_m']/settings.min_range_m
        if max_d>settings.num_disparities-2:
            raise ValueError('Search cannot cover requested near distance; increase min range or num disparities')
        params=dict(numDisparities=settings.num_disparities,blockSize=settings.block_size,
            P1=8*settings.block_size**2,P2=32*settings.block_size**2,disp12MaxDiff=1,
            preFilterCap=31,uniquenessRatio=12,speckleWindowSize=50,speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.left_match=cv2.StereoSGBM_create(minDisparity=0,**params)
        self.right_match=cv2.StereoSGBM_create(minDisparity=-settings.num_disparities,**params)
        self.valid_maps=[]
        for mx,my in self.rect['maps']:
            self.valid_maps.append((mx>=1)&(mx<calibration.width-2)&(my>=1)&(my<calibration.height-2))

    def compute(self,left,right,*,static_scene=False,exposure_pair=None,
                lateral_speed_bound_mps=None,yaw_rate_bound_rad_s=None,
                forward_speed_bound_mps=None,region=None):
        if type(static_scene) is not bool:
            raise ValueError('static_scene must be an explicit boolean')
        interval=0.
        if not static_scene:
            if exposure_pair is None:
                raise ValueError('Moving scenes require verified exposure metadata; software triggers are insufficient')
            interval=exposure_pair.effective_interval_s()
            for name,value in [('lateral speed',lateral_speed_bound_mps),('yaw rate',yaw_rate_bound_rad_s),
                               ('forward speed',forward_speed_bound_mps)]:
                if value is None:
                    raise ValueError(f'Moving scenes require a conservative {name} bound')
                positive(value,name,True)
        images=[]
        for image,maps in zip((left,right),self.rect['maps']):
            if image is None or image.dtype!=np.uint8 or image.shape[:2]!=(self.cal.height,self.cal.width):
                raise ValueError('Frame must match calibrated dimensions and uint8 type; do not resize')
            if image.ndim==3 and image.shape[2]==3:
                image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            if image.ndim!=2:
                raise ValueError('Expected grayscale or 3-channel BGR')
            images.append(cv2.remap(image,*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT))
        # Preserve native pixel scale/focal length. Crop, never downsample, for a
        # bounded long-range target ROI. Full-frame 12 MP dense matching is refused.
        ox=oy=0
        cw,ch=self.cal.width,self.cal.height
        if region is not None:
            x,y,w,h=region
            if any(type(v) is not int for v in region) or min(x,y)<0 or min(w,h)<5 or x+w>cw or y+h>ch:
                raise ValueError('Invalid target region in full rectified coordinates')
            pad=self.settings.num_disparities+self.settings.block_size*2
            ox=max(0,x-pad)
            oy=max(0,y-16)
            x1=min(cw,x+w+pad)
            y1=min(ch,y+h+16)
            cw,ch=x1-ox,y1-oy
            images=[im[oy:y1,ox:x1].copy() for im in images]
        if cw*ch>1_000_000 or cw<=2*self.settings.num_disparities:
            raise ValueError('Use a native-resolution target ROI with <=1 MP and enough disparity margin')
        left,right=images
        dl=self.left_match.compute(left,right).astype(np.float32)/16
        dr=self.right_match.compute(right,left).astype(np.float32)/16
        yy,xx=np.indices(dl.shape)
        xr=np.rint(xx-dl).astype(int)
        in_bounds=(xr>=0)&(xr<cw)
        safe=np.clip(xr,0,cw-1)
        reverse=dr[yy,safe]
        local=left.astype(np.float32)
        mean=cv2.boxFilter(local,-1,(9,9))
        texture=np.sqrt(np.maximum(cv2.boxFilter(local*local,-1,(9,9))-mean*mean,0))
        s=self.settings
        mask=(dl>.5)&(dl<s.num_disparities-1)&in_bounds&(reverse<0)&(reverse>-s.num_disparities)
        mask &= abs(dl+reverse)<=s.lr_tolerance_px
        valid_left=self.valid_maps[0][oy:oy+ch,ox:ox+cw]
        valid_right=self.valid_maps[1][oy:oy+ch,ox:ox+cw]
        mask &= valid_left&valid_right[yy,safe]&(texture>=s.min_texture_std)
        z=np.full(dl.shape,np.nan,np.float32)
        np.divide(self.rect['focal_px']*self.rect['baseline_m'],dl,out=z,where=mask)
        mask &= (z>=s.min_range_m)&(z<=s.max_range_m)
        z[~mask]=np.nan
        dl[~mask]=np.nan
        return dict(depth_m=z,disparity_px=dl,valid=mask,left_rectified=left,right_rectified=right,
                    static_scene_assumed=static_scene,accuracy_verified=False,origin_xy=(ox,oy),
                    motion_interval_s=interval,lateral_speed_bound_mps=lateral_speed_bound_mps or 0.,
                    yaw_rate_bound_rad_s=yaw_rate_bound_rad_s or 0.,
                    forward_speed_bound_mps=forward_speed_bound_mps or 0.)

    def measure_roi(self,result,roi,min_valid_fraction=.5):
        x,y,w,h=roi
        if any(type(v) is not int for v in roi) or min(x,y)<0 or min(w,h)<5 or x+w>self.cal.width or y+h>self.cal.height:
            raise ValueError('ROI must be an integer rectified-image rectangle inside image')
        if not 0<min_valid_fraction<=1:
            raise ValueError('Invalid minimum valid fraction')
        ox,oy=result['origin_xy']
        rx,ry=x-ox,y-oy
        if min(rx,ry)<0 or rx+w>result['depth_m'].shape[1] or ry+h>result['depth_m'].shape[0]:
            raise ValueError('Measurement ROI is outside computed region')
        crop=result['depth_m'][ry:ry+h,rx:rx+w]
        values=crop[np.isfinite(crop)]
        fraction=float(values.size/crop.size)
        base=dict(valid=False,valid_fraction=fraction,accuracy_verified=False,roi=list(roi))
        if values.size<25 or fraction<min_valid_fraction:
            return dict(base,reason='insufficient_consistent_textured_matches',depth_m=None)
        q10,median,q90=np.percentile(values,[10,50,90])
        # Broad depth populations are not assumed to be one target.
        if q90-q10>max(.5,median*.15):
            return dict(base,reason='mixed_depths_or_unstable_matches',depth_m=None,
                        p10_m=float(q10),p90_m=float(q90))
        s=self.settings
        budget=error_budget(float(median),self.rect['focal_px'],self.rect['baseline_m'],
            s.disparity_error_px,scale_error_fraction=s.scale_error_fraction,target_error_m=s.target_error_m,
            skew_s=result['motion_interval_s'],lateral_speed_mps=result['lateral_speed_bound_mps'],
            yaw_rate_rad_s=result['yaw_rate_bound_rad_s'])
        forward=result['forward_speed_bound_mps']*result['motion_interval_s']
        budget['forward_motion_allowance_m']=forward
        if budget['conditional_error_m'] is not None:
            budget['conditional_error_m']+=forward
            budget['within_assumed_budget']=budget['conditional_error_m']<=s.target_error_m
        return dict(base,valid=True,reason='conditional_geometry_estimate_only',depth_m=float(median),
                    p10_m=float(q10),p90_m=float(q90),budget=budget,
                    distance_definition='rectified left-camera optical-axis Z, not Euclidean slant range')
