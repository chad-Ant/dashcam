"""match.py <shots dir> <name>...: compare each <name>.csi.jpg with its <name>.ug.jpg.

SIFT features matched between the two views (different lenses, so outliers are
dropped with a RANSAC fundamental matrix), then a small patch around each matched
point is averaged in both images. Reported per name:
  n      matched patches used
  dE     median CIEDE76 colour difference (CSI vs UGREEN; lower is closer)
  dL     median L* difference (CSI - UGREEN; negative = CSI darker)
  Cr     median chroma ratio C*(CSI) / C*(UGREEN) (1 = same saturation)
  da db  median a*/b* difference (+da = CSI more magenta, +db = CSI more yellow)
  gains  per-channel linear gains that best map CSI to UGREEN (R G B, G normalised to 1)
  noise  Immerkaer noise sigma of the CSI luma
"""
import sys
import cv2
import numpy as np
from csi_decode import read_bgr

def srgb_to_lin(c):
    c = c / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)

def lin_to_lab(rgb):
    m = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
    xyz = rgb @ m.T / np.array([0.95047, 1.0, 1.08883])
    f = np.where(xyz > 216 / 24389, np.cbrt(xyz), (24389 / 27 * xyz + 16) / 116)
    return np.stack([116 * f[:, 1] - 16, 500 * (f[:, 0] - f[:, 1]), 200 * (f[:, 1] - f[:, 2])], axis=1)

def noise_sigma(y):
    y = y.astype(np.float64)
    c = (y[:-2, :-2] - 2 * y[:-2, 1:-1] + y[:-2, 2:] - 2 * y[1:-1, :-2] + 4 * y[1:-1, 1:-1]
         - 2 * y[1:-1, 2:] + y[2:, :-2] - 2 * y[2:, 1:-1] + y[2:, 2:])
    h, w = y.shape
    return np.sqrt(np.pi / 2) * np.abs(c).sum() / (6 * (w - 2) * (h - 2))

def patch_mean(img, x, y, r):
    h, w = img.shape[:2]
    x0, x1, y0, y1 = int(max(0, x - r)), int(min(w, x + r + 1)), int(max(0, y - r)), int(min(h, y + r + 1))
    return img[y0:y1, x0:x1].reshape(-1, 3).mean(axis=0)

def compare(d, name, sift):
    fa = read_bgr(f"{d}/{name}.csi.jpg"); fb = read_bgr(f"{d}/{name}.ug.jpg")   # CSI limited range expanded
    a, b = (np.clip(x + 0.5, 0, 255).astype(np.uint8) for x in (fa, fb))     # uint8 for SIFT only
    ga, gb = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
    # equalise for matching only, so exposure/colour differences do not hide features
    ka, da_ = sift.detectAndCompute(cv2.equalizeHist(ga), None)
    kb, db_ = sift.detectAndCompute(cv2.equalizeHist(gb), None)
    pairs = cv2.BFMatcher().knnMatch(da_, db_, k=2)
    good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < 0.75 * n.distance]
    pa = np.float32([ka[m.queryIdx].pt for m in good]); pb = np.float32([kb[m.trainIdx].pt for m in good])
    if len(good) < 12:
        print(f"{name:28s} too few matches ({len(good)})"); return
    _, mask = cv2.findFundamentalMat(pa, pb, cv2.FM_RANSAC, 4.0, 0.999)
    keep = mask.ravel().astype(bool)
    ra, rb = fa[..., ::-1].astype(np.float64), fb[..., ::-1].astype(np.float64)
    ca, cb = [], []
    for m, k in zip(good, keep):
        if not k:
            continue
        kpa, kpb = ka[m.queryIdx], kb[m.trainIdx]
        r_a = 4
        r_b = max(2, int(round(r_a * kpb.size / max(kpa.size, 1e-3))))
        ca.append(patch_mean(ra, *kpa.pt, r_a)); cb.append(patch_mean(rb, *kpb.pt, r_b))
    ca, cb = np.array(ca), np.array(cb)
    ok = (ca.max(1) < 245) & (cb.max(1) < 245) & (ca.mean(1) > 12) & (cb.mean(1) > 12)   # no clipped or black patches
    ca, cb = ca[ok], cb[ok]
    la, lb = srgb_to_lin(ca), srgb_to_lin(cb)
    La, Lb = lin_to_lab(la), lin_to_lab(lb)
    dE = np.linalg.norm(La - Lb, axis=1)
    Ca, Cb = np.hypot(La[:, 1], La[:, 2]), np.hypot(Lb[:, 1], Lb[:, 2])
    sat = Cb > 8
    gains = np.array([np.sum(la[:, i] * lb[:, i]) / np.sum(la[:, i] ** 2) for i in range(3)])
    neu = Cb < 10                                       # grey/white in the reference: white balance
    col = Cb > 20                                       # clearly coloured in the reference: saturation, hue
    hue = np.degrees(np.angle(np.exp(1j * (np.arctan2(La[col, 2], La[col, 1]) - np.arctan2(Lb[col, 2], Lb[col, 1])))))
    print(f"{name:28s} n {len(ca):3d}  dE {np.median(dE):5.1f}  dL {np.median(La[:,0]-Lb[:,0]):+5.1f}  "
          f"WB(neutral n{neu.sum():2d}) da {np.median(La[neu,1]-Lb[neu,1]) if neu.any() else 0:+5.1f} db {np.median(La[neu,2]-Lb[neu,2]) if neu.any() else 0:+5.1f}  "
          f"colour(n{col.sum():2d}) Cr {np.median(Ca[col]/Cb[col]) if col.any() else 0:4.2f} hue {np.median(hue) if col.any() else 0:+5.1f}deg  "
          f"gains {gains[0]/gains[1]:4.2f} 1 {gains[2]/gains[1]:4.2f} x{gains[1]:4.2f}  noise {noise_sigma(ga):4.2f}")

if __name__ == "__main__":
    s = cv2.SIFT_create(nfeatures=4000)
    for n in sys.argv[2:]:
        compare(sys.argv[1], n, s)
