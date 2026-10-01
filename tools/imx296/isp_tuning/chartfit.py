"""chartfit.py <shots dir> <name> <base .isp> <out prefix>: colour-matrix candidates from the chart.

1. Tint: from the CSI greys (patches 19-22) and paper white, D = mean (G/R, 1, G/B) in linear RGB,
   so that greys come out neutral (the target is a* = b* = 0, not the UGREEN, which renders greys
   blue).
2. Colour: the UGREEN patches with the UGREEN's own grey cast removed the same way are the target;
   fit a neutral-preserving Mn (rows sum to 1) from the neutralised CSI colour patches (1-18),
   robustly. Mn_a = I + a (Mn - I) for strength a.
3. New CCM = Mn_a @ diag(D) @ CCM(base). Writes <out prefix>_tint.isp, _col50.isp, _col100.isp.
"""
import re
import sys
import numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from chartcmp import measure, lab
from fitccm import fit_rows_sum1, read_ccm

d, name, base, out = sys.argv[1:5]
_, _, _, pa, wa = measure(f"{d}/{name}.csi.jpg")
_, _, _, pb, wb = measure(f"{d}/{name}.ug.jpg")
G = slice(18, 22); C = slice(0, 18)
na = np.vstack([pa[G], wa]); nb = np.vstack([pb[G], wb])
Dc = np.array([(na[:, 1] / na[:, 0]).mean(), 1.0, (na[:, 1] / na[:, 2]).mean()])
Du = np.array([(nb[:, 1] / nb[:, 0]).mean(), 1.0, (nb[:, 1] / nb[:, 2]).mean()])
la = pa * Dc
lb = pb * Du * (la[G, 1].mean() / (pb[G, 1] * Du[1]).mean())
Mn = fit_rows_sum1(la[C], lb[C])
ccm = read_ccm(base)
def stats(x, tag):
    L = lab(np.clip(x, 0, 1)); Lb = lab(np.clip(lb, 0, 1))
    g = L[G]; c = np.linalg.norm(L[C] - Lb[C], axis=1).mean()
    cr = (np.hypot(L[C, 1], L[C, 2]) / np.hypot(Lb[C, 1], Lb[C, 2])).mean()
    print(f"  {tag:10s} greys a{g[:,1].mean():+5.1f} b{g[:,2].mean():+5.1f}   colour dE vs neutralised UGREEN {c:4.1f}   chroma {cr:4.2f}")
print(f"CSI tint D = R {Dc[0]:.3f} G 1 B {Dc[2]:.3f}     (UGREEN's own: R {Du[0]:.3f} B {Du[2]:.3f})")
stats(pa, "as is")
stats(la, "tint")
for a in (0.5, 1.0):
    stats(la @ (np.eye(3) + a * (Mn - np.eye(3))).T, f"tint+col{int(a*100)}")
print("Mn =", np.array2string(Mn, precision=4, suppress_small=True).replace("\n", ""))
src = open(base).read()
def write(tag, M, note):
    new = M @ np.diag(Dc) @ ccm
    s = src
    for i in range(3):
        s, k = re.subn(rf"^(colorCorrection\.srgbMatrix\[{i}\]\s*=\s*)\{{[^}}]*\}}", lambda m: m.group(1) + "{" + ", ".join(f"{v:.8f}" for v in new[i]) + "}", s, flags=re.M)
        assert k == 1
    s = f"# {out.rsplit('/',1)[-1]}_{tag} (Claude, from the chart {name}): {note}\n" + s
    open(f"{out}_{tag}.isp", "w").write(s)
    print(f"wrote {out}_{tag}.isp  CCM rows", [list(np.round(r, 4)) for r in new], " row sums", np.round(new.sum(1), 4))
write("tint", np.eye(3), f"CCM tint D=({Dc[0]:.4f},1,{Dc[2]:.4f}) for neutral greys")
write("col50", np.eye(3) + 0.5 * (Mn - np.eye(3)), "tint + half the neutral-preserving colour fit to the UGREEN")
write("col100", Mn, "tint + the full neutral-preserving colour fit to the UGREEN")
