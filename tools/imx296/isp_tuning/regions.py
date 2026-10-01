"""regions.py <shots dir> <name>... [--show]: mean colour of hand-picked matching regions, CSI vs UGREEN."""
import sys
import numpy as np
from PIL import Image
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from csi_decode import read_bgr
# (x0, y0, x1, y1) in full-resolution pixels: CSI 1456x1088, UGREEN 1920x1080
R = {
    "wall":    ((480, 320, 800, 520),   (480, 40, 1300, 380)),
    "box":     ((520, 690, 640, 800),   (1100, 620, 1300, 720)),
    "redbag":  ((540, 565, 760, 605),   (1100, 400, 1300, 470)),
    "blue":    ((335, 420, 375, 560),   (300, 150, 370, 500)),
    "redflag": ((225, 460, 295, 560),   (30, 420, 220, 560)),
}
def lin(c):
    c = c / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
def lab(rgb):
    m = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
    x = rgb @ m.T / np.array([0.95047, 1.0, 1.08883])
    f = np.where(x > 216 / 24389, np.cbrt(x), (24389 / 27 * x + 16) / 116)
    return np.array([116 * f[1] - 16, 500 * (f[0] - f[1]), 200 * (f[1] - f[2])])
args = [a for a in sys.argv[2:] if not a.startswith("--")]
for name in args:
    a = read_bgr(f"{sys.argv[1]}/{name}.csi.jpg")[..., ::-1].astype(float)   # CSI limited range expanded
    b = read_bgr(f"{sys.argv[1]}/{name}.ug.jpg")[..., ::-1].astype(float)
    out = [f"{name:26s}"]
    tot = []
    for k, (ra, rb) in R.items():
        ma = lin(a[ra[1]:ra[3], ra[0]:ra[2]].reshape(-1, 3)).mean(0)
        mb = lin(b[rb[1]:rb[3], rb[0]:rb[2]].reshape(-1, 3)).mean(0)
        la, lb = lab(ma), lab(mb)
        tot.append(np.linalg.norm(la - lb))
        out.append(f"{k} L{la[0]:4.0f}/{lb[0]:3.0f} a{la[1]:+4.0f}/{lb[1]:+4.0f} b{la[2]:+4.0f}/{lb[2]:+4.0f}")
    print("  ".join(out) + f"  | dE mean {np.mean(tot):4.1f}")
    if "--show" in sys.argv:
        tiles = []
        for k, (ra, rb) in R.items():
            ta = Image.fromarray(a[ra[1]:ra[3], ra[0]:ra[2]].astype(np.uint8)).resize((200, 120))
            tb = Image.fromarray(b[rb[1]:rb[3], rb[0]:rb[2]].astype(np.uint8)).resize((200, 120))
            t = Image.new("RGB", (400, 120)); t.paste(ta, (0, 0)); t.paste(tb, (200, 0)); tiles.append(t)
        g = Image.new("RGB", (400, 120 * len(tiles)))
        for i, t in enumerate(tiles): g.paste(t, (0, 120 * i))
        g.save(f"{sys.argv[1]}/{name}_regions.png")
