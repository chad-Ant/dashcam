"""c5gen.py: candidates in the ISP's real convention. The file holds the matrix COLUMN by column:
operator A = file^T (out = A @ in). Writes c5_rpi75T, c5_rpi100T (the Raspberry Pi IMX296 5600 K CCM at
75/100 %, transposed into the file) and c5_fit50/c5_fit100 (greys neutral + a neutral-preserving colour
fit to the UGREEN with its own cast removed), and prints the predicted chart result of each."""
import json, re, sys
import numpy as np
sys.path.insert(0, "/w")
from chartcmp import measure, lab
from fitccm import read_ccm, fit_rows_sum1
d = "/w/shots"; base_isp = "/w/c4_T.isp"
A_base = read_ccm(base_isp).T                                  # operator of c4_T (= the vendor matrix as intended)
_, _, _, pa, wa = measure(f"{d}/c4_T_ec0.csi.jpg")
_, _, _, pb, wb = measure(f"{d}/c4_T_ec0.ug.jpg")
G = slice(18, 22); C = slice(0, 18)
xa = np.vstack([pa, wa]); xb = np.vstack([pb, wb])
pre = xa @ np.linalg.inv(A_base).T                             # colour before the matrix
nb = np.vstack([pb[G], wb]); Du = np.array([(nb[:, 1] / nb[:, 0]).mean(), 1, (nb[:, 1] / nb[:, 2]).mean()])
tgt = xb * Du; tgt *= xa[G, 1].mean() / tgt[G, 1].mean()      # UGREEN, its blue cast removed, CSI brightness
def predict(A):
    y = np.clip(pre @ A.T, 0, None); y *= xa[G, 1].mean() / max(y[G, 1].mean(), 1e-6)
    L, Lt = lab(np.clip(y, 0, 1)), lab(np.clip(tgt, 0, 1))
    g = L[18:22]
    return (g[:, 1].mean(), g[:, 2].mean(), np.linalg.norm(L[C] - Lt[C], axis=1).mean(),
            (np.hypot(L[C, 1], L[C, 2]) / np.hypot(Lt[C, 1], Lt[C, 2])).mean())
j = json.load(open("/w/rpi/imx296.json"))
rpi = {c["ct"]: np.array(c["ccm"]).reshape(3, 3) for a in j["algorithms"] for k, v in a.items() if k == "rpi.ccm" for c in v["ccms"]}
R56 = rpi[5600]
cands = {"c4_T": A_base,
         "c5_rpi75T": np.eye(3) + 0.75 * (R56 - np.eye(3)),
         "c5_rpi100T": R56}
# chart fit: output tint so greys are neutral, then a neutral-preserving colour fit
out = pre @ A_base.T
ng = np.vstack([out[G], out[-1:]]); Dt = np.array([(ng[:, 1] / ng[:, 0]).mean(), 1, (ng[:, 1] / ng[:, 2]).mean()])
Mn = fit_rows_sum1((out * Dt)[C], tgt[C])
for a in (0.5, 1.0):
    cands[f"c5_fit{int(a*100)}"] = (np.eye(3) + a * (Mn - np.eye(3))) @ np.diag(Dt) @ A_base
src = open(base_isp).read().split("\n", 1)[1]                   # drop c4_T's own comment line
for name, A in cands.items():
    ga, gb, de, cr = predict(A)
    print(f"{name:11s} predicted greys a{ga:+5.1f} b{gb:+5.1f}   colour dE vs neutralised UGREEN {de:4.1f}   chroma {cr:4.2f}   "
          f"operator row max {np.abs(A).max():.2f}")
    if name == "c4_T":
        continue
    F = A.T
    s = src
    for i in range(3):
        s, k = re.subn(rf"^(colorCorrection\.srgbMatrix\[{i}\]\s*=\s*)\{{[^}}]*\}}", lambda m: m.group(1) + "{" + ", ".join(f"{v:.8f}" for v in F[i]) + "}", s, flags=re.M)
        assert k == 1
    open(f"/w/{name}.isp", "w").write(f"# {name} (Claude): operator {np.round(A, 4).tolist()} written column by column (file = operator^T)\n" + s)
