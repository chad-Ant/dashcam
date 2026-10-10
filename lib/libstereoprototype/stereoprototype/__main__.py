"""Run with PYTHONPATH=lib/libstereoprototype python3 -m stereoprototype."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from .calibration import Calibration,fit_checkerboards
from .core import Rangefinder,Settings
from .geometry import error_budget,focal_from_hfov
from .timing import ExposurePair


def write_json(path,data):
    with open(path,'x') as f:
        json.dump(data,f,indent=2,allow_nan=False)
        f.write('\n')


def write_image(path,image):
    if not cv2.imwrite(str(path),image):
        raise OSError(f'Cannot write {path}')


def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()


def save_result(out,result,measurement,elapsed,metadata):
    np.savez_compressed(out/'depth.npz',depth_m=result['depth_m'],disparity_px=result['disparity_px'],
                        origin_xy=result['origin_xy'])
    write_image(out/'left_rectified.png',result['left_rectified'])
    write_image(out/'right_rectified.png',result['right_rectified'])
    depth=result['depth_m']
    valid=np.isfinite(depth)
    visual=np.zeros(depth.shape,np.uint8)
    if valid.any():
        lo,hi=np.percentile(depth[valid],[5,95])
        visual[valid]=np.clip((depth[valid]-lo)/max(hi-lo,.1)*255,0,255).astype(np.uint8)
    shown=cv2.applyColorMap(visual,cv2.COLORMAP_TURBO)
    shown[~valid]=0
    cv2.putText(shown,'PROTOTYPE - NOT VERIFIED ACCURACY',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
    write_image(out/'depth_preview.png',shown)
    report=dict(measurement=measurement,compute_seconds=elapsed,origin_xy=result['origin_xy'],
                static_scene_assumed=result['static_scene_assumed'],metadata=metadata,
                safety_use=False,accuracy_verified=False)
    write_json(out/'report.json',report)
    return report


def evaluate_pair(pair_path,calibration_path,settings_path,roi,out):
    pair_path=Path(pair_path).resolve()
    pair=json.loads(pair_path.read_text())
    if pair.get('schema')!='stereo-prototype-pair-v1':
        raise ValueError('Pair schema missing')
    cal=Calibration.load(calibration_path)
    if (pair.get('left_id'),pair.get('right_id'))!=(cal.left_id,cal.right_id):
        raise ValueError('Image camera identities/order differ from calibration')
    files=[(pair_path.parent/pair[k]).resolve() for k in ('left','right')]
    if files[0]==files[1]:
        raise ValueError('Left/right images must be distinct files')
    static=pair.get('static_scene') is True
    timing=None if static else ExposurePair(**pair['exposure'])
    settings=Settings(**json.loads(Path(settings_path).read_text()))
    images=[cv2.imread(str(p),cv2.IMREAD_UNCHANGED) for p in files]
    finder=Rangefinder(cal,settings)
    start=time.monotonic()
    result=finder.compute(*images,static_scene=static,exposure_pair=timing,region=roi,
        lateral_speed_bound_mps=pair.get('lateral_speed_bound_mps'),
        yaw_rate_bound_rad_s=pair.get('yaw_rate_bound_rad_s'),
        forward_speed_bound_mps=pair.get('forward_speed_bound_mps'))
    elapsed=time.monotonic()-start
    measurement=finder.measure_roi(result,roi)
    out.mkdir(parents=True,exist_ok=False)
    report=save_result(out,result,measurement,elapsed,
        dict(pair_manifest_sha256=digest(pair_path),calibration_sha256=digest(calibration_path),
             settings=asdict(settings),inputs=[dict(file=str(p),sha256=digest(p)) for p in files]))
    return report


def demo(out):
    """Synthetic textured plane at 100m; not a rendered road or hardware test."""
    out.mkdir(parents=True,exist_ok=False)
    width,height=4096,512
    fx=35/.00274
    k=np.array([[fx,0,width/2],[0,fx,height/2],[0,0,1.]])
    cal=Calibration(width,height,'synthetic-left','synthetic-right',k,np.zeros(5),k.copy(),np.zeros(5),
                    np.eye(3),np.array([-1.,0,0]))
    cal.save(out/'calibration.json',{'synthetic':True,'not_for_real_cameras':True})
    settings=Settings(30.,120.,.5,scale_error_fraction=.002,num_disparities=448)
    write_json(out/'settings.json',asdict(settings))
    rng=np.random.default_rng(1123)
    left=rng.integers(20,235,(height,width),dtype=np.uint8)
    left=cv2.GaussianBlur(left,(3,3),.6)
    disparity=fx/100.
    right=cv2.warpAffine(left,np.float32([[1,0,-disparity],[0,1,0]]),(width,height),
                         flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
    write_image(out/'left.png',left)
    write_image(out/'right.png',right)
    write_json(out/'pair.json',dict(schema='stereo-prototype-pair-v1',left='left.png',right='right.png',
        left_id=cal.left_id,right_id=cal.right_id,static_scene=True,synthetic=True))
    report=evaluate_pair(out/'pair.json',out/'calibration.json',out/'settings.json',
                         [1700,180,500,150],out/'result')
    return dict(synthetic_truth_m=100.,**report)


def main(argv=None):
    cv2.setNumThreads(2)
    p=argparse.ArgumentParser(description=__doc__)
    commands=p.add_subparsers(dest='cmd',required=True)
    b=commands.add_parser('budget',help='Conditional geometry/motion sensitivity, not guaranteed accuracy')
    b.add_argument('--distance',type=float,nargs='+',default=[10,20,50,100])
    b.add_argument('--focal-px',type=float,required=True)
    b.add_argument('--baseline-m',type=float,required=True)
    b.add_argument('--disparity-error-px',type=float,required=True)
    b.add_argument('--scale-error-fraction',type=float,default=0.)
    b.add_argument('--skew-ms',type=float,default=0.)
    b.add_argument('--lateral-speed-mps',type=float,default=0.)
    b.add_argument('--yaw-deg-s',type=float,default=0.)
    r=commands.add_parser('range',help='Analyse calibrated saved stereo pair; no capture or vehicle control')
    r.add_argument('--pair',type=Path,required=True)
    r.add_argument('--calibration',type=Path,required=True)
    r.add_argument('--settings',type=Path,required=True)
    r.add_argument('--roi',type=int,nargs=4,required=True,metavar=('X','Y','W','H'))
    r.add_argument('--out',type=Path,required=True)
    d=commands.add_parser('demo',help='Generate/test a synthetic 100m plane at native 4096px width')
    d.add_argument('--out',type=Path,required=True)
    c=commands.add_parser('calibrate',help='Checkerboard calibration from 16..100 stationary pairs')
    c.add_argument('--pairs',type=Path,required=True,help='JSON list of [left_file,right_file], relative to manifest')
    c.add_argument('--inner-cols',type=int,required=True)
    c.add_argument('--inner-rows',type=int,required=True)
    c.add_argument('--square-m',type=float,required=True)
    c.add_argument('--left-id',required=True)
    c.add_argument('--right-id',required=True)
    c.add_argument('--out',type=Path,required=True)
    a=p.parse_args(argv)
    if a.cmd=='budget':
        result=[error_budget(z,a.focal_px,a.baseline_m,a.disparity_error_px,
            skew_s=a.skew_ms/1000,lateral_speed_mps=a.lateral_speed_mps,
            yaw_rate_rad_s=np.deg2rad(a.yaw_deg_s),scale_error_fraction=a.scale_error_fraction) for z in a.distance]
    elif a.cmd=='demo':
        result=demo(a.out)
    elif a.cmd=='range':
        result=evaluate_pair(a.pair,a.calibration,a.settings,a.roi,a.out)
    else:
        raw=json.loads(a.pairs.read_text())
        if not isinstance(raw,list) or any(not isinstance(pair,list) or len(pair)!=2 for pair in raw):
            raise ValueError('Pair list must contain [left_file,right_file] entries')
        pairs=[[(a.pairs.parent/p).resolve() for p in pair] for pair in raw]
        cal,result=fit_checkerboards(pairs,a.inner_cols,a.inner_rows,a.square_m,a.left_id,a.right_id)
        cal.save(a.out,result)
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0


if __name__=='__main__':
    try:
        sys.exit(main())
    except (ValueError,OSError,KeyError,TypeError,cv2.error) as exc:
        print(f'Stereo prototype refused: {exc}',file=sys.stderr)
        sys.exit(2)
