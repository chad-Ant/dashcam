#!/usr/bin/env python3
"""Offline v2 colour, temporal-noise and relative-detail analysis. Never installs an ISP.

Dependencies: NumPy, OpenCV (aruco), Pillow; existing ../../csi_decode.py.
See CALIBRATION.md for capture contracts, decoder selection and matrix limitations.
"""
import argparse
from collections import deque
import glob
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import cv2
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
SHEETS = {'colour': '01_colour', 'noise': '02_noise_tone', 'detail': '03_detail'}
LIMIT_FRAMES = 256


def require(condition, message):
    if not condition:
        raise ValueError(message)


def files(patterns, minimum=1, maximum=LIMIT_FRAMES):
    paths = []
    for pattern in patterns:
        matches = glob.glob(pattern)
        require(matches, f'No files match: {pattern}')
        paths.extend(Path(p).resolve() for p in matches)
    require(len(paths) == len(set(paths)), 'Input patterns include duplicate paths')
    require(minimum <= len(paths) <= maximum,
            f'Need {minimum}..{maximum} images; got {len(paths)}')
    require(all(p.is_file() for p in paths), 'Every input must be a regular file')
    # Natural order for frame_2 before frame_10. Prefer zero-padded capture names.
    return sorted(paths, key=lambda p: [int(t) if t.isdigit() else t
                                       for t in re.split(r'(\d+)', str(p))])


def read_rgb(path, decoder='auto'):
    with Image.open(path) as header:
        require(header.width * header.height <= 12_000_000, 'Image exceeds 12 MP limit')
        require(header.mode in ('RGB', 'L'), 'Only 8-bit RGB/grey JPEG/PNG inputs are supported')
    if str(HERE.parents[1]) not in sys.path:
        sys.path.insert(0, str(HERE.parents[1]))
    from csi_decode import is_nvjpeg, read_bgr
    nv = is_nvjpeg(str(path)) if decoder != 'standard' else False
    if decoder == 'nvjpeg':
        require(nv, f'{path}: not a recognized rig nvJPEG file; refusing forced range expansion')
    if nv:
        bgr = read_bgr(str(path))
        mode = 'rig-nvjpeg-limited-bt601'
    else:
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        mode = 'standard-full-range'
    require(bgr is not None, f'Cannot decode {path}')
    rgb = np.asarray(bgr[:, :, ::-1], dtype=np.float32)
    require(np.isfinite(rgb).all(), f'Nonfinite image: {path}')
    return rgb, mode


def u8(rgb):
    return np.clip(np.rint(rgb), 0, 255).astype(np.uint8)


def project(points, h):
    result = cv2.perspectiveTransform(np.asarray(points, np.float64).reshape(1, -1, 2), h)[0]
    require(np.isfinite(result).all(), 'Nonfinite chart projection')
    return result


def corners(box):
    x, y, w, h = box
    return [[x,y], [x+w,y], [x+w,y+h], [x,y+h]]


def load_geometry(kind):
    geo = json.loads((HERE / 'output' / f'{SHEETS[kind]}_geometry.json').read_text())
    first = {'colour':10, 'noise':14, 'detail':18}[kind]
    require(geo['schema'] == 'imx296-chart-v2' and geo['sheet'] == SHEETS[kind], 'Wrong geometry schema/sheet')
    require([m['id'] for m in geo['markers']] == list(range(first, first+4)), 'Wrong marker IDs in geometry')
    return geo


def locate(rgb, geo):
    grey = cv2.cvtColor(u8(rgb), cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    found, ids, _ = cv2.aruco.detectMarkers(grey, dictionary, parameters=params)
    wanted = [m['id'] for m in geo['markers']]
    got = [] if ids is None else list(map(int, ids.ravel()))
    require(all(got.count(i) == 1 for i in wanted),
            f'Need all four {geo["sheet"]} markers {wanted}; found {got}. Check sheet, focus and framing.')
    by_id = {i: c.reshape(4,2) for i,c in zip(got,found)}
    mm = np.concatenate([corners(m['rect_mm']) for m in geo['markers']])
    px = np.concatenate([by_id[i] for i in wanted])
    h, _ = cv2.findHomography(mm, px, method=0)
    require(h is not None and np.isfinite(h).all() and abs(np.linalg.det(h)) > 1e-10,
            'Degenerate marker homography')
    error = np.linalg.norm(project(mm,h)-px, axis=1)
    require(error.max() <= 2.5, f'Marker fit error {error.max():.2f}px >2.5; flatten chart/check distortion')
    outline = project(corners([0,0,*geo['paper_mm']]), h)
    require(cv2.isContourConvex(outline.astype(np.float32)), 'Nonconvex chart projection')
    return h, px, dict(reprojection_max_px=float(error.max()),
                       marker_ids=wanted, homography_mm_to_native_pixel=h.tolist())


def roi_indices(shape, h, box, min_side=8):
    poly = project(corners(box), h)
    lengths = np.linalg.norm(poly-np.roll(poly,1,axis=0), axis=1)
    require(lengths.min() >= min_side,
            f'Chart too small: inner ROI side {lengths.min():.1f}px <{min_side}px; move closer')
    height,width = shape[:2]
    require((poly[:,0] >= 1).all() and (poly[:,0] < width-1).all() and
            (poly[:,1] >= 1).all() and (poly[:,1] < height-1).all(), 'ROI outside captured image')
    lo = np.floor(poly.min(axis=0)).astype(int)
    hi = np.ceil(poly.max(axis=0)).astype(int)
    yy,xx = np.mgrid[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
    mapped = project(np.column_stack([xx.ravel(),yy.ravel()]), np.linalg.inv(h))
    x,y,w,hh = box
    valid = (mapped[:,0] >= x) & (mapped[:,0] <= x+w) & (mapped[:,1] >= y) & (mapped[:,1] <= y+hh)
    iy,ix = yy.ravel()[valid],xx.ravel()[valid]
    require(32 <= len(ix) <= 250_000, 'ROI pixel count outside 32..250000 bound')
    return iy,ix


def linear(rgb):
    v = np.asarray(rgb, np.float64)/255
    return np.where(v <= .04045, v/12.92, ((v+.055)/1.055)**2.4)


def encoded(lin):
    v = np.clip(lin,0,1)
    return 255*np.where(v <= .0031308, 12.92*v, 1.055*v**(1/2.4)-.055)


def lab(lin):
    xyz = np.asarray(lin) @ np.array([[.4124564,.3575761,.1804375],
        [.2126729,.7151522,.0721750],[.0193339,.1191920,.9503041]]).T
    xyz = xyz / [.95047,1,1.08883]
    f = np.where(xyz > (6/29)**3, np.cbrt(xyz), xyz/(3*(6/29)**2)+4/29)
    return np.column_stack([116*f[:,1]-16, 500*(f[:,0]-f[:,1]), 200*(f[:,1]-f[:,2])])


def delta_e(a,b):
    # CIE76 display-referred proxy, not a measured-print accuracy claim.
    return np.linalg.norm(lab(np.clip(a,0,1))-lab(np.clip(b,0,1)), axis=1)


def sha(path):
    digest = hashlib.sha256()
    with open(path,'rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_png(path,rgb):
    require(cv2.imwrite(str(path),u8(rgb)[:,:,::-1]), f'Cannot write {path}')


def overlay(rgb, geo, h, path):
    shown = u8(rgb).copy()
    for item in geo['patches']+geo['edges']+geo['textures']:
        box = item.get('roi_mm',item['rect_mm'])
        pts = np.rint(project(corners(box),h)).astype(np.int32)
        cv2.polylines(shown,[pts],True,(0,255,80),1,cv2.LINE_AA)
        cv2.putText(shown,item['id'],tuple(pts[0]),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,40,40),1,cv2.LINE_AA)
    write_png(path,shown)


class Moments:
    """Per-native-pixel streaming statistics; bounded nine-frame lag history."""
    def __init__(self, values):
        self.n = 0
        self.mean = np.zeros_like(values,dtype=np.float64)
        self.m2 = np.zeros_like(self.mean)
        self.history = deque(maxlen=8)
        self.diff = {lag:[] for lag in (1,4,8)}
        self.frame_means = []
        self.clipped = np.zeros(3)

    def add(self, values):
        v = np.asarray(values,np.float64)
        self.n += 1
        difference = v-self.mean
        self.mean += difference/self.n
        self.m2 += difference*(v-self.mean)
        self.frame_means.append(v.mean(axis=0))
        self.clipped += ((v <= 1) | (v >= 254)).mean(axis=0)
        for lag in self.diff:
            if len(self.history) >= lag:
                # Remove each difference image's spatial DC, not texture by warping.
                self.diff[lag].append(np.var(v-self.history[-lag],axis=0,ddof=1)/2)
        self.history.append(v.astype(np.float32))

    def report(self):
        drift = np.ptp(self.frame_means,axis=0)
        return dict(mean_rgb=self.mean.mean(axis=0).tolist(),
            spatial_std_mean_image_rgb=self.mean.std(axis=0,ddof=1).tolist(),
            temporal_rms_std_rgb=(np.sqrt((self.m2/(self.n-1)).mean(axis=0)).tolist() if self.n>1 else None),
            frame_mean_peak_to_peak_rgb=drift.tolist(),
            clipped_fraction_rgb=(self.clipped/self.n).tolist(),
            pair_difference_rms_rgb={str(k):np.sqrt(np.mean(v,axis=0)).tolist()
                                     for k,v in self.diff.items() if v})


def analyse_flat(paths,geo,decoder,out,label,min_main=64,max_motion=1.0,max_drift=2.0,noise=False):
    moments,indices,reference_px = {},{},None
    hashes,decoders,frames = set(),set(),[]
    first_shape = None
    for index,path in enumerate(paths):
        rgb,mode = read_rgb(path,decoder)
        decoders.add(mode)
        fingerprint = hashlib.sha256(rgb.tobytes()).hexdigest()
        require(fingerprint not in hashes, f'Duplicate decoded frame: {path}')
        hashes.add(fingerprint)
        h,px,registration = locate(rgb,geo)
        if reference_px is None:
            reference_px,first_shape = px,rgb.shape
            first_h = h
            overlay(rgb,geo,h,out/f'{label}_rois.png')
            for p in geo['patches']:
                minimum = min_main if p['id'].startswith(('C','N')) else 8
                indices[p['id']] = roi_indices(rgb.shape,h,p['roi_mm'],minimum)
        require(rgb.shape == first_shape, 'Image dimensions changed within burst')
        motion = float(np.linalg.norm(px-reference_px,axis=1).max())
        require(motion <= max_motion, f'{path}: marker motion/jitter {motion:.2f}px >{max_motion}; recapture stable burst')
        for p in geo['patches']:
            # Deliberately fixed native pixels: no frame interpolation or smoothing.
            values = rgb[indices[p['id']]]
            if index == 0:
                moments[p['id']] = Moments(values)
            moments[p['id']].add(values)
        frames.append(dict(file=str(path),sha256=sha(path),decoder=mode,
                           motion_px=motion,**registration))
    require(len(decoders)==1, 'Mixed decoder/range paths in one burst; split the inputs')
    patches = {name: dict(pixels=len(indices[name][0]),**m.report()) for name,m in moments.items()}
    issues=[]
    for name,p in patches.items():
        if name.startswith('S'):
            continue  # Deliberately dark shadow steps are diagnostic, never fitted.
        if max(p['frame_mean_peak_to_peak_rgb']) > max_drift:
            issues.append(f'{name}: lighting/exposure/white-balance drift exceeds {max_drift:g} RGB codes')
        if max(p['clipped_fraction_rgb']) > .01:
            issues.append(f'{name}: >1% clipped/near-clipped pixels in a channel')
    report = dict(frames=frames,patches=patches,quality_issues=issues,valid=not issues,
                  decoder=next(iter(decoders)),homography=first_h.tolist())
    if geo['sheet']=='01_colour':
        report['neutral_pairs_de76']={}
        for i in range(1,4):
            left=linear([patches[f'L{i}']['mean_rgb']])
            right=linear([patches[f'R{i}']['mean_rgb']])
            difference=float(delta_e(left,right)[0])
            report['neutral_pairs_de76'][f'L{i}-R{i}']=difference
            if difference > 5:
                issues.append(f'L{i}/R{i}: repeated-neutral difference >5 dE76; check lighting/print/shading')
        report['valid']=not issues
    if noise:
        report['interpretation']='Output-pipeline RGB code noise; not raw sensor noise. Lags are file intervals, not seconds.'
        report['warnings']=['Timestamps and dropped frames cannot be verified from still images.',
            'Temporal processing/correlation and illumination flicker can bias these metrics; compare lags 1,4,8.']
        report['neutral_response_rgb']={p['id']:patches[p['id']]['mean_rgb'] for p in geo['patches']}
    return report


def fit_matrix(source,target,ridge=.005):
    """Row-sum-one matrix, solving two free coefficients per output channel.

    RGB_out_column = M @ RGB_in_column, in sRGB-linearized *rendered* space.
    """
    require(np.isfinite(source).all() and np.isfinite(target).all(), 'Nonfinite fit data')
    a = source[:,:2]-source[:,2,None]
    require(np.linalg.matrix_rank(a)==2 and np.linalg.cond(a) < 100, 'Colour fit is rank-deficient/ill-conditioned')
    matrix=np.eye(3)
    for ch in range(3):
        prior=np.eye(3)[ch,:2]
        coeff=np.linalg.solve(a.T@a+ridge*np.eye(2),a.T@(target[:,ch]-source[:,2])+ridge*prior)
        matrix[ch]=[coeff[0],coeff[1],1-coeff.sum()]
    require(np.max(np.abs(matrix)) <= 4 and np.linalg.cond(matrix) < 20,
            'Unstable fitted matrix; check reference, exposure and geometry')
    return matrix


def colour_fit(csi,reference):
    names=[f'C{i:02}' for i in range(1,25)]
    src=linear([csi['patches'][k]['mean_rgb'] for k in names])
    dst=linear([reference['patches'][k]['mean_rgb'] for k in names])
    neutral=['L2','R2']
    sn=linear([csi['patches'][k]['mean_rgb'] for k in neutral]).mean(axis=0)
    dn=linear([reference['patches'][k]['mean_rgb'] for k in neutral]).mean(axis=0)
    require(min(sn.min(),dn.min()) > .01, 'Mid-grey reference too dark for stable normalization')
    # Match reference's grey chromaticity; do NOT assume printer greys truly neutral.
    exposure=float(dn[1]/sn[1])
    gains=(dn/sn)/exposure
    require(.25 <= exposure <= 4 and np.all((gains>=.5)&(gains<=2)), 'Excessive grey/exposure mismatch')
    normalized=src*exposure*gains
    train=np.array([i for i in range(18) if i%3 != 2])
    hold=np.array([i for i in range(18) if i%3 == 2])
    matrix=fit_matrix(normalized[train],dst[train])
    corrected=normalized@matrix.T
    baseline=delta_e(src,dst)
    grey_only=delta_e(normalized,dst)
    after=delta_e(corrected,dst)
    outside=((corrected < 0)|(corrected > 1)).any(axis=1)
    scores=dict(baseline_holdout_mean_de76=float(baseline[hold].mean()),
                grey_match_holdout_mean_de76=float(grey_only[hold].mean()),
                matrix_holdout_mean_de76=float(after[hold].mean()),
                training_mean_de76=float(after[train].mean()),
                baseline_all_mean_de76=float(baseline.mean()),
                corrected_all_mean_de76=float(after.mean()))
    # Provisional review only; don't refit on holdouts after judging them.
    improvement=scores['grey_match_holdout_mean_de76']-scores['matrix_holdout_mean_de76']
    useful=improvement > .2 and not outside.any() and csi['valid'] and reference['valid']
    return dict(status='provisional_candidate' if useful else 'not_recommended',
        reference='USB/rendered-image match, NOT calibrated print colour accuracy',
        colour_space='inverse-sRGB approximation of processed camera RGB; NOT sensor-linear',
        apply_formula='out_linear_column = M @ (grey_gains_rgb * exposure_scale * in_linear_column)',
        matrix_rows=matrix.tolist(),grey_gains_rgb=gains.tolist(),exposure_scale=exposure,
        installable_isp_profile=False,
        warning='Do not copy this post-render matrix into the Argus ISP CCM. Validate a separate ISP experiment in another light.',
        train_ids=[names[i] for i in train],holdout_ids=[names[i] for i in hold],scores=scores,
        out_of_gamut_patch_ids=[names[i] for i in np.where(outside)[0]],
        per_patch=[dict(id=k,source_rgb=encoded(src[i]).tolist(),reference_rgb=encoded(dst[i]).tolist(),
                        corrected_rgb=encoded(corrected[i]).tolist(),before_de76=float(baseline[i]),
                        after_de76=float(after[i])) for i,k in enumerate(names)])


def swatch_preview(fit,path):
    panel=np.full((24*30+60,660,3),245,np.uint8)
    for x,label in [(12,'Patch'),(105,'CSI'),(280,'Corrected'),(475,'Reference')]:
        cv2.putText(panel,label,(x,30),cv2.FONT_HERSHEY_SIMPLEX,.55,(30,30,30),1,cv2.LINE_AA)
    for i,p in enumerate(fit['per_patch']):
        y=50+i*30
        cv2.putText(panel,p['id'],(12,y+18),cv2.FONT_HERSHEY_SIMPLEX,.45,(30,30,30),1)
        for x,key in [(95,'source_rgb'),(280,'corrected_rgb'),(465,'reference_rgb')]:
            panel[y:y+24,x:x+165]=u8(np.array(p[key]))
    write_png(path,panel)


def edge_metric(rgb,h,edge):
    iy,ix=roi_indices(rgb.shape,h,edge['roi_mm'],32)
    x,y,w,hh=edge['rect_mm']
    t=math.tan(math.radians(edge['angle_from_axis_deg']))
    if edge['orientation']=='vertical':
        endpoints=[[x+w/2-hh*t/2,y],[x+w/2+hh*t/2,y+hh]]
    else:
        endpoints=[[x,y+hh/2-w*t/2],[x+w,y+hh/2+w*t/2]]
    p,q=project(endpoints,h)
    tangent=(q-p)/np.linalg.norm(q-p)
    normal=np.array([-tangent[1],tangent[0]])
    distance=(np.column_stack([ix,iy])-(p+q)/2)@normal
    luminance=rgb[iy,ix] @ np.array([.2126,.7152,.0722])
    bins=np.arange(-12,12.001,.25)
    bucket=np.digitize(distance,bins)-1
    esf=np.array([luminance[bucket==i].mean() if np.count_nonzero(bucket==i)>=3 else np.nan
                  for i in range(len(bins)-1)])
    centres=(bins[:-1]+bins[1:])/2
    require(np.isfinite(esf).all(), 'Edge undersampled or axis-aligned; adjust camera/chart angle slightly')
    a,b=float(np.median(esf[:16])),float(np.median(esf[-16:]))
    require(abs(b-a)>15,'Insufficient edge contrast or misplaced edge ROI')
    normalized=(esf-a)/(b-a)
    # Enforce monotonicity for width only; keep original profile for halo metrics.
    monotone=np.maximum.accumulate(np.clip(normalized,0,1))
    crossings=np.interp([.1,.5,.9],monotone,centres)
    require(-6 < crossings[1] < 6 and crossings[2]-crossings[0] < 12,
            'Edge centre/width outside measurement window')
    return dict(id=edge['id'],width_10_90_px=float(crossings[2]-crossings[0]),
        contrast_codes=abs(b-a),overshoot_fraction=float(max(0,normalized.max()-1)),
        undershoot_fraction=float(max(0,-normalized.min())),
        distance_px=centres.tolist(),normalized_esf=normalized.tolist(),
        note='Relative gamma-encoded edge width; includes print, focus and ISP. Not ISO MTF.')


def analyse_detail(paths,geo,decoder,out):
    require(len(paths)==1,'Detail mode uses one image at a time; compare reports for repeat captures')
    rgb,mode=read_rgb(paths[0],decoder)
    h,_,registration=locate(rgb,geo)
    overlay(rgb,geo,h,out/'detail_rois.png')
    edges=[]
    for edge in geo['edges']:
        try:
            edges.append(edge_metric(rgb,h,edge))
        except ValueError as error:
            edges.append(dict(id=edge['id'],valid=False,error=str(error)))
    textures=[]
    for tile in geo['textures']:
        x,y,w,hh=tile['rect_mm']
        iy,ix=roi_indices(rgb.shape,h,[x+w*.1,y+hh*.1,w*.8,hh*.8],16)
        v=rgb[iy,ix] @ np.array([.2126,.7152,.0722])
        textures.append(dict(id=tile['id'],period_mm=tile['period_mm'],
            rms_contrast=float(v.std()/max(v.mean(),1)),
            note='Includes aliasing, noise, printer and illumination; compare only matched geometry/exposure'))
    return dict(valid=all('error' not in e for e in edges),edges=edges,textures=textures,
                file=str(paths[0]),sha256=sha(paths[0]),decoder=mode,registration=registration)


def save_report(out,report):
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    lines=[f'# V2 {report["mode"]} report','',f'Quality status: {"PASS" if report["valid"] else "REJECT / REVIEW"}',
           '', 'See report.json for per-patch data and input hashes. ROI previews show sampled native pixels.',
           '', 'No system/camera changes were made. No ISP profile was installed.']
    if 'error' in report:
        lines.extend(['','Failure: '+report['error']])
    for label in ('csi','reference','noise'):
        if label in report:
            lines.extend(['',f'## {label}', '']+['- '+i for i in report[label]['quality_issues']])
    if 'fit' in report:
        lines.extend(['','## Colour fit','',report['fit']['status'],'',report['fit']['warning'],'',
                      json.dumps(report['fit']['scores'],indent=2)])
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main(argv=None):
    cv2.setNumThreads(1)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=SHEETS)
    parser.add_argument('--images',nargs='+',required=True,help='Quoted globs or explicit files in one stable burst')
    parser.add_argument('--reference',nargs='+',help='USB images of the same physical colour sheet in the same light')
    parser.add_argument('--decoder',choices=['auto','standard','nvjpeg'],default='auto')
    parser.add_argument('--reference-decoder',choices=['auto','standard','nvjpeg'],default='standard')
    parser.add_argument('--out',type=Path,required=True,help='NEW result directory; existing paths refused')
    parser.add_argument('--max-motion-px',type=float,default=1.)
    parser.add_argument('--max-drift-codes',type=float,default=2.)
    args=parser.parse_args(argv)
    require(math.isfinite(args.max_motion_px) and 0 < args.max_motion_px <= 3,'Motion limit must be >0 and <=3px')
    require(math.isfinite(args.max_drift_codes) and 0 < args.max_drift_codes <= 10,'Drift limit must be >0 and <=10 codes')
    require(args.mode=='colour' or not args.reference,'--reference is only supported for colour')
    paths=files(args.images,32 if args.mode=='noise' else 1,256 if args.mode=='noise' else 16)
    ref=files(args.reference,1,16) if args.reference else None
    geo=load_geometry(args.mode)
    args.out.mkdir(parents=True,exist_ok=False)
    report=dict(schema='imx296-calibration-v2',mode=args.mode,valid=False,
                geometry_sha256=sha(HERE/'output'/f'{SHEETS[args.mode]}_geometry.json'),
                thresholds=dict(max_motion_px=args.max_motion_px,max_drift_codes=args.max_drift_codes),
                limitations=['Rendered camera RGB is assumed sRGB for colour comparisons.',
                'Printed nominal RGB is not ground truth. USB is only a relative reference.',
                'Capture timestamps and control locks must be verified separately.'])
    try:
        if args.mode=='detail':
            report['detail']=analyse_detail(paths,geo,args.decoder,args.out)
            report['valid']=report['detail']['valid']
        else:
            key='noise' if args.mode=='noise' else 'csi'
            report[key]=analyse_flat(paths,geo,args.decoder,args.out,key,
                min_main=96 if args.mode=='noise' else 64,max_motion=args.max_motion_px,
                max_drift=args.max_drift_codes,noise=args.mode=='noise')
            report['valid']=report[key]['valid']
            if ref:
                report['reference']=analyse_flat(ref,geo,args.reference_decoder,args.out,'reference',
                    max_motion=args.max_motion_px,max_drift=args.max_drift_codes)
                report['valid'] &= report['reference']['valid']
                if report['valid']:
                    report['fit']=colour_fit(report['csi'],report['reference'])
                    swatch_preview(report['fit'],args.out/'colour_comparison.png')
        save_report(args.out,report)
    except (ValueError,OSError,cv2.error,KeyError,IndexError) as error:
        report['valid']=False
        report['error']=str(error)
        save_report(args.out,report)
        raise
    print(f'{"PASS" if report["valid"] else "REJECT / REVIEW"}: {args.out}/report.json')
    if 'fit' in report:
        print('Colour fit:',report['fit']['status'],'(post-render model only; NOT an ISP profile)')
    return 0 if report['valid'] else 2


if __name__=='__main__':
    try:
        sys.exit(main())
    except (ValueError,OSError,cv2.error,KeyError,IndexError) as error:
        print(f'Calibration refused: {error}',file=sys.stderr)
        sys.exit(2)
