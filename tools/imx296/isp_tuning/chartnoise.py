"""chartnoise.py <shots dir> <name>...: CSI colour noise on the chart: the spread of L*, a*, b* inside
the mid-grey patches 20-22 (pixel level, no blur), averaged. Lower is cleaner.
Reads with csi_decode.read_bgr (limited-range CSI stills expanded; ~15 % higher than before 2026-10-01 23:59)."""
import sys
import cv2
import numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from chartcmp import homography, GEO, S, srgb_to_lin, lab
from csi_decode import read_bgr
for n in sys.argv[2:]:
    img = read_bgr(f"{sys.argv[1]}/{n}.csi.jpg"); H, _ = homography(img)
    sd = []
    for p in GEO["patches"][19:22]:
        g = np.linspace(-S / 4, S / 4, 15)
        pts = np.float32([[p["x_mm"] + S / 2 + u, p["y_mm"] + S / 2 + v] for u in g for v in g]).reshape(-1, 1, 2)
        px = cv2.perspectiveTransform(pts, H).reshape(-1, 2).round().astype(int)
        L = lab(srgb_to_lin(img[px[:, 1], px[:, 0]][:, ::-1].astype(float)))
        sd.append(L.std(0))
    sd = np.mean(sd, axis=0)
    print(f"{n:24s} noise in greys: L* {sd[0]:4.2f}  a* {sd[1]:4.2f}  b* {sd[2]:4.2f}")
