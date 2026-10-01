"""convtest.py: which way does the ISP apply colorCorrection.srgbMatrix? Recover the pre-matrix colour
of every chart patch from the c1 capture under each convention, predict each candidate, compare with
what the camera produced (chromaticity r/g, b/g in linear RGB, so exposure differences drop out)."""
import sys
import numpy as np
sys.path.insert(0, "/w")
from chartcmp import measure
from fitccm import read_ccm
d = "/w/shots"
base = read_ccm("/w/c1_black50.isp")
_, _, _, p1, w1 = measure(f"{d}/chart_c1_ec0.csi.jpg")
x1 = np.vstack([p1, w1])
for cand in ("c3_tint", "c3_col50", "c3_col100"):
    M = read_ccm(f"/w/{cand}.isp")
    r = measure(f"{d}/{cand}_ec0.csi.jpg")
    y = np.vstack([r[3], r[4]])
    out = []
    for conv, A0, A in (("rows", base, M), ("columns", base.T, M.T)):
        pre = x1 @ np.linalg.inv(A0).T                  # pre-matrix colour, from c1
        pred = np.clip(pre @ A.T, 1e-4, None)
        chrom = lambda v: np.stack([v[:, 0] / v[:, 1], v[:, 2] / v[:, 1]], 1)
        err = np.abs(np.log(chrom(pred)) - np.log(chrom(np.clip(y, 1e-4, None)))).mean()
        out.append(f"{conv}: mean |log chromaticity error| {err:5.3f}  (paper predicted r/g {pred[-1,0]/pred[-1,1]:.2f} b/g {pred[-1,2]/pred[-1,1]:.2f})")
    print(f"{cand:10s} measured paper r/g {y[-1,0]/y[-1,1]:.2f} b/g {y[-1,2]/y[-1,1]:.2f}\n    " + "\n    ".join(out))
