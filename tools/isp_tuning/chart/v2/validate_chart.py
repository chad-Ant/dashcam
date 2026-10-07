#!/usr/bin/env python3
"""Offline artifact regression checks; requires OpenCV with aruco and NumPy.

Does not access devices, fit a CCM, or measure a real camera's noise.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def detect(image, geometry, transform, label):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners,ids,_=cv2.aruco.detectMarkers(image,dictionary)
    expected={m['id'] for m in geometry['markers']}
    found=set() if ids is None else set(int(i) for i in ids.ravel())
    assert found == expected, f'{label}: expected {expected}, got {found}'
    by_id={int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),corners)}
    for marker in geometry['markers']:
        x,y,w,h=marker['rect_mm']
        points=np.float32([[[x,y],[x+w,y],[x+w,y+h],[x,y+h]]])
        expected_xy=cv2.perspectiveTransform(points,transform)[0]
        error=np.linalg.norm(by_id[marker['id']]-expected_xy,axis=1).max()
        assert error < 3.0, f'{label}: marker {marker["id"]} corner error {error:.2f}px'
    print(f'PASS {label}: four correct IDs, ordered corners within 3 pixels')


def validate(root,pdf_prefix):
    manifest=json.loads((root/'manifest.json').read_text())
    all_ids=[]
    for number,stem in enumerate(manifest['pages'],1):
        geo=json.loads((root/f'{stem}_geometry.json').read_text())
        all_ids.extend(m['id'] for m in geo['markers'])
        im=cv2.imread(str(root/f'{stem}_300dpi.png'))
        assert im is not None, f'Missing PNG: {stem}'
        h,w=im.shape[:2]
        scale=300/25.4
        assert (w,h) == (3508,2480)
        mm_to_px=np.diag([scale,scale,1.0])
        detect(im,geo,mm_to_px,stem+' 300dpi')
        for p in geo['patches']:
            x,y,rw,rh=p['roi_mm']
            roi=im[int(np.ceil(y*scale)):int((y+rh)*scale),
                   int(np.ceil(x*scale)):int((x+rw)*scale),::-1].astype(np.int16)
            assert roi.size > 0
            err=np.abs(roi-np.array(p['nominal_srgb'])).max()
            assert err <= 1, f'{stem} {p["id"]}: nominal RGB error {err}'
        print(f'PASS {stem}: {len(geo["patches"])} flat patch ROIs match nominal RGB within 1 code')
        # Native IMX296-sized canvas, sheet spanning 70% of width.
        native_scale=1456*.7/297
        native_h=np.array([[native_scale,0,(1456-297*native_scale)/2],
                           [0,native_scale,(1088-210*native_scale)/2],[0,0,1.]])
        native=cv2.warpPerspective(im,native_h @ np.linalg.inv(mm_to_px),
                                  (1456,1088),borderValue=(225,225,225))
        detect(native,geo,native_h,stem+' native 70% fill')
        if stem == '01_colour':
            main=[p for p in geo['patches'] if p['id'].startswith('C')]
            assert min(min(p['roi_mm'][2:])*native_scale for p in main) >= 64
            print('PASS colour: all 24 central ROIs >=64 pixels per side at 70% fill')
        source=np.float32([[0,0],[297,0],[297,210],[0,210]])
        dest=np.float32([[250,170],[1260,195],[1210,935],[200,900]])
        angled_h=cv2.getPerspectiveTransform(source,dest)
        angled=cv2.warpPerspective(im,angled_h @ np.linalg.inv(mm_to_px),
                                   (1456,1088),borderValue=(225,225,225))
        detect(angled,geo,angled_h,stem+' native perspective')
        if pdf_prefix:
            rendered=cv2.imread(f'{pdf_prefix}-{number}.png')
            assert rendered is not None, f'Missing PDF render page {number}'
            pdf_h=np.diag([100/25.4,100/25.4,1.0])
            detect(rendered,geo,pdf_h,stem+' actual PDF render 100dpi')
    assert len(set(all_ids)) == 12 and not set(all_ids).intersection(range(4))
    print('PASS: 12 unique markers; no IDs shared with legacy chart')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parent/'output')
    parser.add_argument('--pdf-prefix',help='Optional pdftoppm -r 100 -png output prefix')
    args=parser.parse_args()
    validate(args.root,args.pdf_prefix)
