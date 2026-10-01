"""fitccm.py <shots dir> <name> <ccm-source .isp>: fit a 3x3 linear-RGB correction CSI -> UGREEN.

Samples: SIFT-matched patches (as match.py) plus the hand-picked regions of regions.py
(each region counted 20 times, so the flat wall, the main neutral, carries weight).
Fit: lb ~= M @ la, iteratively reweighted least squares (Huber-like) in linear RGB.
Split: M = D @ Mn, where Mn preserves neutrals (rows sum to 1) and D = diag(M @ [1,1,1])
is a fixed tint/gain (normalised to G = 1). The ISP applies out = CCM @ WB @ raw, so:
  CCM_n = Mn @ CCM   (colour fix only)       CCM_t = (D/D_G) @ Mn @ CCM   (colour fix + tint)
Prints the predicted median dE on the samples for each, and the two matrices.
"""
import re
import sys
import numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import cv2
from match import srgb_to_lin, lin_to_lab, patch_mean
from csi_decode import read_bgr

R = {}
exec(re.search(r"^R = \{.*?^\}", open(__file__.rsplit("/", 1)[0] + "/regions.py").read(), re.M | re.S).group(0))

def samples(d, name):
    a = read_bgr(f"{d}/{name}.csi.jpg")[..., ::-1].astype(np.float64)
    b = read_bgr(f"{d}/{name}.ug.jpg")[..., ::-1].astype(np.float64)
    sift = cv2.SIFT_create(nfeatures=4000)
    ga = cv2.equalizeHist(cv2.cvtColor(np.clip(a + 0.5, 0, 255).astype(np.uint8), cv2.COLOR_RGB2GRAY))
    gb = cv2.equalizeHist(cv2.cvtColor(np.clip(b + 0.5, 0, 255).astype(np.uint8), cv2.COLOR_RGB2GRAY))
    ka, da = sift.detectAndCompute(ga, None); kb, db = sift.detectAndCompute(gb, None)
    good = [m for m, n in (p for p in cv2.BFMatcher().knnMatch(da, db, k=2) if len(p) == 2) if m.distance < 0.75 * n.distance]
    pa = np.float32([ka[m.queryIdx].pt for m in good]); pb = np.float32([kb[m.trainIdx].pt for m in good])
    _, mask = cv2.findFundamentalMat(pa, pb, cv2.FM_RANSAC, 4.0, 0.999)
    ca, cb = [], []
    for m, k in zip(good, mask.ravel()):
        if k:
            r_b = max(2, int(round(4 * kb[m.trainIdx].size / max(ka[m.queryIdx].size, 1e-3))))
            ca.append(patch_mean(a, *ka[m.queryIdx].pt, 4)); cb.append(patch_mean(b, *kb[m.trainIdx].pt, r_b))
    for (ra, rb) in R.values():
        ma = a[ra[1]:ra[3], ra[0]:ra[2]].reshape(-1, 3).mean(0); mb = b[rb[1]:rb[3], rb[0]:rb[2]].reshape(-1, 3).mean(0)
        ca += [ma] * 20; cb += [mb] * 20
    ca, cb = np.array(ca), np.array(cb)
    ok = (ca.max(1) < 245) & (cb.max(1) < 245) & (ca.mean(1) > 10) & (cb.mean(1) > 10)
    return srgb_to_lin(ca[ok]), srgb_to_lin(cb[ok])

def fit(la, lb):
    w = np.ones(len(la))
    for _ in range(10):
        W = np.sqrt(w)[:, None]
        M = np.linalg.lstsq(la * W, lb * W, rcond=None)[0].T
        r = np.linalg.norm(la @ M.T - lb, axis=1)
        s = np.median(r) * 1.5 + 1e-9
        w = np.where(r < s, 1.0, s / r)
    return M

def dE(x, y):
    return np.median(np.linalg.norm(lin_to_lab(np.clip(x, 0, 1)) - lin_to_lab(np.clip(y, 0, 1)), axis=1))

def read_ccm(path):
    rows = re.findall(r"^colorCorrection\.srgbMatrix\[(\d)\]\s*=\s*\{([^}]*)\}", open(path).read(), re.M)
    return np.array([[float(v) for v in r[1].split(",")] for r in sorted(rows)])

def fit_rows_sum1(la, lb):
    """Each output row r = [x, y, 1-x-y]: lb_i - a2 = x (a0 - a2) + y (a1 - a2); robust."""
    Mn = np.zeros((3, 3))
    X = np.stack([la[:, 0] - la[:, 2], la[:, 1] - la[:, 2]], axis=1)
    for i in range(3):
        y = lb[:, i] - la[:, 2]; w = np.ones(len(y))
        for _ in range(10):
            W = np.sqrt(w)
            c = np.linalg.lstsq(X * W[:, None], y * W, rcond=None)[0]
            r = np.abs(X @ c - y); sc = np.median(r) * 1.5 + 1e-9
            w = np.where(r < sc, 1.0, sc / r)
        Mn[i] = [c[0], c[1], 1 - c[0] - c[1]]
    return Mn

if __name__ == "__main__":
    d, name, isp = sys.argv[1:4]
    la, lb = samples(d, name)
    import cv2 as _c
    a = srgb_to_lin(read_bgr(f"{d}/{name}.csi.jpg")[..., ::-1].astype(np.float64))
    b = srgb_to_lin(read_bgr(f"{d}/{name}.ug.jpg")[..., ::-1].astype(np.float64))
    (ra, rb) = R["wall"]
    wa = a[ra[1]:ra[3], ra[0]:ra[2]].reshape(-1, 3).mean(0); wb = b[rb[1]:rb[3], rb[0]:rb[2]].reshape(-1, 3).mean(0)
    t = wb / wa; k = t[1]; D = t / k                    # tint from the neutral wall (G = 1), overall gain k
    la_d = la * D * k
    Mn = fit_rows_sum1(la_d, lb)
    ccm = read_ccm(isp)
    print(f"samples {len(la)}   dE as is {dE(la * k, lb):.1f}   tint only {dE(la_d, lb):.1f}   tint + colour {dE(la_d @ Mn.T, lb):.1f}   colour only {dE((la * k) @ Mn.T, lb):.1f}")
    print(f"wall tint D (G=1) = R {D[0]:.3f}  B {D[2]:.3f}   overall gain {k:.3f}")
    print("Mn =", np.array2string(Mn, precision=4, suppress_small=True).replace("\n", ""), " row sums", (Mn @ np.ones(3)).round(4))
    print("CCM_n =", repr((Mn @ ccm).round(6).tolist()))
    print("CCM_t =", repr((np.diag(D) @ Mn @ ccm).round(6).tolist()))
