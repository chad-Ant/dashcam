"""chartcmp.py <shots dir> <name>... [--draw]: the printed chart, CSI vs UGREEN.

Finds the four ArUco markers (DICT_4X4_50, ids 0-3) in each image (upscaling the image
if needed), maps chart millimetres to pixels with a homography through the marker
centres, and averages the inner half of each of the 24 patches and five spots of bare
paper. Reports, per name:
  - the paper white and the six greys (patches 19-24) as L*a*b* for both cameras, and
    their mean a*, b* (the white-balance error: a neutral is a* = b* = 0);
  - the mean dE76 CSI vs UGREEN over the 18 colour patches and over the 6 greys;
  - the mean chroma ratio C*(CSI) / C*(UGREEN) over the colour patches.
--draw writes <name>.chart.png with the sampled areas outlined.
Images are read with csi_decode.read_bgr: the CSI stills (nvjpegenc) are limited-range YCbCr and are
expanded; before 2026-10-01 23:59 they were decoded as full range (lifted blacks, chroma x0.878).
Paper spot 5 (in the gap between patches 1 and 2) was dropped then too: sharpening halos reached it.
"""
import json
import sys
import cv2
import numpy as np
from csi_decode import read_bgr

HERE = __file__.rsplit("/", 1)[0]
GEO = json.load(open(f"{HERE}/chart/chart_geometry.json"))
MK = GEO["marker_mm"]
CENTRES_MM = np.float32([[x + MK / 2, y + MK / 2] for x, y in GEO["markers_tl_mm"]])
# bare paper: midway between the patch grid and the paper edge, left/right/top/bottom/centre-gap
P0 = GEO["patches"][0]; S = P0["size_mm"]
PAPER_MM = [(P0["x_mm"] - 9, 105), (297 - P0["x_mm"] + 9, 105), (148.5, P0["y_mm"] - 9),
            (148.5, 210 - P0["y_mm"] + 6)]

def srgb_to_lin(c):
    c = c / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)

def lab(rgb_lin):
    m = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
    x = rgb_lin @ m.T / np.array([0.95047, 1.0, 1.08883])
    f = np.where(x > 216 / 24389, np.cbrt(x), (24389 / 27 * x + 16) / 116)
    return np.stack([116 * f[..., 1] - 16, 500 * (f[..., 0] - f[..., 1]), 200 * (f[..., 1] - f[..., 2])], axis=-1)

def marker_corners_mm(i):
    x, y = GEO["markers_tl_mm"][i]
    return [[x, y], [x + MK, y], [x + MK, y + MK], [x, y + MK]]          # ArUco corner order: tl, tr, br, bl

FIND_SHARPEN = True          # find_markers' second, sharpened round (tests switch it off to compare)


def find_markers(img):
    """Detect the markers (several scales and thresholds); returns the 4x2 centres array when all four
    are seen (for the old callers) and stores every corner seen in find_markers.pairs for homography()."""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if img.dtype != np.uint8:                         # read_bgr gives float32; the detector wants uint8
        img = np.clip(img + 0.5, 0, 255).astype(np.uint8)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    best = {}
    # second round, only for markers the plain image misses: a zero-phase unsharp mask (Y + 1.5 (Y - G_sigma))
    # brings back the ~3 px cells that ee-mode=0 blurs (marker 0 in the 2026-10-02 trial); symmetric, so the
    # refined corners do not move (checked against frames where both rounds find the marker)
    for sharpen in (False, True):
        for up in (1, 1.5, 2, 3, 4):
            for win in (23, 53):
                params = cv2.aruco.DetectorParameters()
                params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                params.adaptiveThreshWinSizeMax = win
                g = cv2.resize(gray, None, fx=up, fy=up, interpolation=cv2.INTER_CUBIC) if up != 1 else gray
                if sharpen:
                    gf = g.astype(np.float32)
                    g = np.clip(gf + 1.5 * (gf - cv2.GaussianBlur(gf, (0, 0), 1.0 * up)) + 0.5, 0, 255).astype(np.uint8)
                corners, ids, _ = cv2.aruco.ArucoDetector(dic, params).detectMarkers(g)
                if ids is None:
                    continue
                for i, c in zip(ids.ravel().tolist(), corners):
                    if i in range(4) and i not in best:
                        best[i] = (c.reshape(4, 2) / up, up)
            if len(best) == 4:
                break
        if len(best) == 4 or not FIND_SHARPEN:
            break
    find_markers.pairs = best
    if len(best) == 4:
        return np.float32([best[i][0].mean(0) for i in range(4)]), max(u for _, u in best.values())
    return None, None

def homography(img):
    """chart mm -> pixels from every detected marker corner (needs at least two markers)."""
    find_markers(img)
    best = find_markers.pairs
    if len(best) < 2:
        return None, None
    src = np.float32([pt for i in best for pt in marker_corners_mm(i)])
    dst = np.float32([pt for i in best for pt in best[i][0]])
    H, _ = cv2.findHomography(src, dst, 0)
    return H, f"{len(best)}m"

def sample(img, H, cx, cy, half):
    """Mean sRGB of the square (cx +- half, cy +- half) mm, through the homography."""
    g = np.linspace(-half, half, 9)
    pts = np.float32([[cx + u, cy + v] for u in g for v in g]).reshape(-1, 1, 2)
    px = cv2.perspectiveTransform(pts, H).reshape(-1, 2)
    h, w = img.shape[:2]
    vals = [img[int(round(y)), int(round(x))] for x, y in px if 0 <= x < w and 0 <= y < h]
    # average over a small neighbourhood too, by blurring first
    return np.array(vals, dtype=np.float64)[:, ::-1].mean(0), px

def measure(path, draw=None):
    img = read_bgr(path)
    if img is None:
        return None
    H, up = homography(img)
    if H is None:
        return None
    blur = cv2.blur(img, (3, 3))
    pat, outl = [], []
    for p in GEO["patches"]:
        c, px = sample(blur, H, p["x_mm"] + S / 2, p["y_mm"] + S / 2, S / 4)
        pat.append(c); outl.append(px)
    paper = []
    for cx, cy in PAPER_MM:
        c, px = sample(blur, H, cx, cy, 2.0)
        paper.append(c); outl.append(px)
    if draw:
        out = np.clip(img + 0.5, 0, 255).astype(np.uint8)
        for px in outl:
            cv2.polylines(out, [cv2.convexHull(px.astype(np.int32))], True, (255, 0, 255), 1)
        cv2.imwrite(draw, out)
    lin_pat, lin_paper = srgb_to_lin(np.array(pat)), srgb_to_lin(np.mean(paper, axis=0))
    return lab(lin_pat), lab(lin_paper), up, lin_pat, lin_paper

if __name__ == "__main__":
    d = sys.argv[1]
    names = [a for a in sys.argv[2:] if not a.startswith("--")]
    for n in names:
        res = {}
        for cam in ("csi", "ug"):
            r = measure(f"{d}/{n}.{cam}.jpg", f"{d}/{n}.{cam}.chart.png" if "--draw" in sys.argv else None)
            if r is None:
                print(f"{n}: markers not found in the {cam} image"); break
            res[cam] = r
        if len(res) < 2:
            continue
        (pa, wa, ua, _, _), (pb, wb, ub, _, _) = res["csi"], res["ug"]
        greys, cols = slice(18, 24), slice(0, 18)
        dE_col = np.linalg.norm(pa[cols] - pb[cols], axis=1).mean()
        dE_grey = np.linalg.norm(pa[greys] - pb[greys], axis=1).mean()
        cr = (np.hypot(pa[cols, 1], pa[cols, 2]) / np.hypot(pb[cols, 1], pb[cols, 2])).mean()
        ga, gb = pa[greys], pb[greys]
        print(f"{n:24s} paper CSI L{wa[0]:3.0f} a{wa[1]:+5.1f} b{wa[2]:+5.1f} | UG L{wb[0]:3.0f} a{wb[1]:+5.1f} b{wb[2]:+5.1f}   "
              f"greys mean CSI a{ga[:,1].mean():+5.1f} b{ga[:,2].mean():+5.1f} | UG a{gb[:,1].mean():+5.1f} b{gb[:,2].mean():+5.1f}   "
              f"dE colour {dE_col:4.1f} grey {dE_grey:4.1f}  chroma {cr:4.2f}  (upscale {ua}/{ub})")
        if "--patches" in sys.argv:
            for i in range(24):
                print(f"   {i+1:2d} CSI L{pa[i,0]:4.0f} a{pa[i,1]:+5.0f} b{pa[i,2]:+5.0f}   UG L{pb[i,0]:4.0f} a{pb[i,1]:+5.0f} b{pb[i,2]:+5.0f}   dE {np.linalg.norm(pa[i]-pb[i]):4.1f}")
