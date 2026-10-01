"""chartsharp.py <shots dir> <name>... [--profiles] [--crops <out.png> <nameA> <nameB> ...]

Sharpening on the printed chart, CSI image (<name>.csi.jpg), measured directly. Run it in the
l4t-ml-gpio container (host cv2 is broken by numpy 2). Per image:

  pY, L*  mean luma (BT.601 of the decoded image) and L* of the bare-paper spots of chartcmp.py:
          flags AE differences between captures.
  Decode: csi_decode.read_bgr expands the CSI stills' limited range (since 2026-10-01 23:59). Normalised
  edge metrics are unchanged by that; pY, L* and the noise figures (x255/219, ~+16 %) are not. The clip
  limits below are the old raw Y codes 18 / 233 on the expanded scale.
  Edge spread functions (ESF), three sets of borders, each border normalised to its own dark and
  bright plateaus (0 = dark, 1 = bright) and weighted by step^2:
    MK    ArUco markers' outer borders, black ink -> paper: only the borders facing the inside of the
          chart (wide paper) and only where the black reaches >= 3 cells (~10 px) inward; >= 4 mm from
          the corners, 1.2 mm from black/white transitions along the border, runs >= 1.4 mm.
          Caveats: the printed marker black is not uniform (UGREEN sees Y 70-100 inside the markers,
          vs 47 on patch 24) and the ink reaches the Y=16 floor, so MK 'under' is not trustworthy.
    HI    13 high-contrast outer borders of the patch grid (dark/colour patch -> paper margin),
          central 60 % of each border. With the ISP's sharpening most of them clip at Y 16/235.
    LO    grey patches 20 and 21, bottom borders (-> paper): low contrast, mid-tone, unclipped at EV 0.
  Method: chartcmp.homography gives each border in pixels; the line is refined to the 50 % crossings of
  bilinear (cv2.remap) profiles sampled every 0.1 px along the normal; the profiles, aligned to the
  refined line, are averaged in 0.25 px bins (main table). Cross-check: every pixel of the band binned
  by its signed distance to the line (slanted-edge style, no interpolation blur, but sparse bins for
  the few, nearly axis-aligned MK and LO borders). Reported:
    rise   10-90 % distance in image pixels;
    over   peak on the bright side above the bright plateau, % of the step (the halo);
    under  dip on the dark side below the dark plateau, % of the step (the dark rim);
    clip   % of the pixels within 3.5 px of the edge at Y <= 18 or >= 233: nvarguscamerasrc's NV12 is
           limited range (Y 16..235), so a halo that reaches it is cut off and 'over'/'under' read low;
    D>B    the plateau luma levels (step = B - D);
    m.25   MTF at 0.25 cy/px (FFT of the derivative of the ESF, corrected for the bilinear
           interpolation's sinc^2); pk = max MTF in 0.02-0.5 cy/px (> 1 = boosted by sharpening).
  noise   flat grey patches 20-22, the inner 24 px square: plane-fit residual luma std (Ystd), and the
          radially averaged power spectrum (Hann window): HF/LF = mean power density in 0.25-0.5 cy/px
          over 0.05-0.15 cy/px (sharpening boosts the high band), sLF / sHF = the std in each band
          (luma codes). With 24 px boxes the LF band has few bins, so HF/LF scatters by ~+-30 %.
--profiles prints the averaged ESFs; --crops writes 3x nearest-neighbour crops of one chart region
(row 1: marker 0 corner + patch 1 corner; row 2: patch 24 corner + marker 2 corner; row 3: row 1
high-passed (Y - Gaussian(2 px)) x4, to show grain) of the listed names side by side.
"""
import sys
import cv2
import numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from chartcmp import homography, GEO, S, MK, PAPER_MM, srgb_to_lin, lab
from csi_decode import read_bgr, read_bgr_u8

BIN = 0.25
CELL = MK / 6.0
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# borders of each marker that face the inside of the chart
INNER_SIDES = {0: ("right", "bottom"), 1: ("left", "bottom"), 2: ("left", "top"), 3: ("right", "top")}
MK_WIN = dict(W=6.0, dark=(-5.5, -4.0), bright=(3.5, 6.0))     # px; black is >= 10 px deep there
PT_WIN = dict(W=10.0, dark=(-9.5, -5.0), bright=(5.0, 9.5))    # patches are ~36 px, margins >= 20 px


def to_px(H, pts):
    return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), H).reshape(-1, 2).astype(np.float64)


def marker_black(i):
    """6x6 cell grid of marker i, True = black."""
    return cv2.aruco.generateImageMarker(DICT, i, 6, borderBits=1) < 128


def marker_edges(min_depth=3):
    """(start mm, end mm, normal toward paper, tag) for the usable runs of the inner-facing borders:
    cells along the border where the black reaches >= min_depth cells (border included) inward."""
    out = []
    for i, sides in INNER_SIDES.items():
        mx, my = GEO["markers_tl_mm"][i]
        g = marker_black(i)
        for side in sides:
            # cell columns perpendicular to the border, read from the border inward, for cells k = 1..4
            lines = {"top": lambda k: g[:, k], "bottom": lambda k: g[::-1, k],
                     "left": lambda k: g[k, :], "right": lambda k: g[k, ::-1]}[side]
            depth = []
            for k in range(1, 5):
                ln = lines(k); n = 0
                while n < 6 and ln[n]:
                    n += 1
                depth.append(n)
            adj = [dp >= min_depth for dp in depth]
            if side in ("top", "bottom"):
                y = my if side == "top" else my + MK
                seg = lambda a, b, y=y: ((mx + a, y), (mx + b, y))
                nb = (0.0, -1.0) if side == "top" else (0.0, 1.0)
            else:
                x = mx if side == "left" else mx + MK
                seg = lambda a, b, x=x: ((x, my + a), (x, my + b))
                nb = (-1.0, 0.0) if side == "left" else (1.0, 0.0)
            k = 0
            while k < 4:                                   # runs of usable cells k0..k1 (cells 1..4)
                if not adj[k]:
                    k += 1; continue
                k0 = k
                while k + 1 < 4 and adj[k + 1]:
                    k += 1
                k1 = k; k += 1
                a = (k0 + 1) * CELL; b = (k1 + 2) * CELL
                a = max(a, 4.0) if k0 == 0 else a + 1.2
                b = min(b, MK - 4.0) if k1 == 3 else b - 1.2
                if b - a >= 1.4:                           # single cells between transitions (0.9 mm) are too short
                    p, q = seg(a, b)
                    out.append((p, q, nb, f"m{i}{side[0]}"))
    return out


# patch-grid borders: high contrast (dark/colour patch -> paper; most of them clip at the video range limits
# Y 16/235 once the ISP sharpens), and low contrast (light/mid grey -> paper; never clip at EV 0)
PT_HI = ["p1t", "p1l", "p3t", "p4t", "p5t", "p6t", "p6r", "p13l", "p18r", "p22b", "p23b", "p24b", "p24r"]
PT_LO = ["p20b", "p21b"]
# nvarguscamerasrc NV12 is limited range (Y 16..235); read_bgr expands it, so the old raw-code limits 18 / 233
# become these (a JFIF image is never clipped there in practice)
# (half a code inside, so float32 rounding cannot drop the raw 18 / 233 pixels themselves)
CLIP_LO, CLIP_HI = (18.5 - 16) * 255 / 219, (232.5 - 16) * 255 / 219


def patch_edges(tags=None):
    """Outer borders of the 6x4 patch grid, central 60 %; normal toward the paper margin."""
    out = []
    for idx, p in enumerate(GEO["patches"]):
        r, c = divmod(idx, 6)
        x0, y0 = p["x_mm"], p["y_mm"]
        a, b = 0.2 * S, 0.8 * S
        if r == 0: out.append(((x0 + a, y0), (x0 + b, y0), (0.0, -1.0), f"p{idx+1}t"))
        if r == 3: out.append(((x0 + a, y0 + S), (x0 + b, y0 + S), (0.0, 1.0), f"p{idx+1}b"))
        if c == 0: out.append(((x0, y0 + a), (x0, y0 + b), (-1.0, 0.0), f"p{idx+1}l"))
        if c == 5: out.append(((x0 + S, y0 + a), (x0 + S, y0 + b), (1.0, 0.0), f"p{idx+1}r"))
    return [e for e in out if tags is None or e[3] in tags]


def edge_samples(Y, H, p_mm, q_mm, nb_mm, win):
    """One border: refine its line from remap profiles, then return the band's pixels as
    (signed distance to the line, toward paper; luma) plus the remap samples, the plateaus and the fit rms."""
    W = win["W"]
    A, B = to_px(H, [p_mm, q_mm])
    Pn = to_px(H, [(p_mm[0] + nb_mm[0], p_mm[1] + nb_mm[1])])[0]
    L = np.linalg.norm(B - A); u = (B - A) / L
    n = Pn - A; n -= (n @ u) * u; n /= np.linalg.norm(n)
    ts = np.linspace(0.0, 1.0, max(5, int(L * 4)))
    ss = np.arange(-W - 2, W + 2 + 1e-9, 0.1)
    X = A[0] + ts[:, None] * (B - A)[0] + ss[None, :] * n[0]
    Yc = A[1] + ts[:, None] * (B - A)[1] + ss[None, :] * n[1]
    prof = cv2.remap(Y, X.astype(np.float32), Yc.astype(np.float32), cv2.INTER_LINEAR)
    # dark/bright levels relative to the homography's line, generous (+-2 px of slack), then the 50 % crossing
    dk = (ss > win["dark"][0] - 1) & (ss < win["dark"][1] - 1)
    br = (ss > win["bright"][0] + 1) & (ss < win["bright"][1] + 1)
    s50 = []
    for pr in prof:
        lo, hi = np.median(pr[dk]), np.median(pr[br])
        half = 0.5 * (lo + hi)
        sgn = np.sign(pr - half)
        cross = np.nonzero(np.diff(sgn) > 0)[0]           # dark -> bright going outward
        if hi - lo < 10 or len(cross) == 0:
            s50.append(np.nan); continue
        j = cross[np.argmin(np.abs(ss[cross]))]
        s50.append(ss[j] + (half - pr[j]) / (pr[j + 1] - pr[j]) * 0.1)
    s50 = np.array(s50); ok = np.isfinite(s50)
    if ok.sum() < 3:
        return None
    coef = np.polyfit(ts[ok], s50[ok], 1)
    for _ in range(2):                                     # drop outliers, refit
        res = s50 - np.polyval(coef, ts)
        ok2 = ok & (np.abs(res) < max(0.5, 3 * np.nanstd(res[ok])))
        coef = np.polyfit(ts[ok2], s50[ok2], 1)
    rms = np.std((s50 - np.polyval(coef, ts))[ok2])
    A2 = A + np.polyval(coef, 0.0) * n; B2 = A + (B - A) + np.polyval(coef, 1.0) * n
    u2 = (B2 - A2) / np.linalg.norm(B2 - A2); n2 = np.array([-u2[1], u2[0]])
    if n2 @ n < 0:
        n2 = -n2
    # remap samples, aligned to the refined line (cross-check of the binned ESF)
    rd = ss[None, :] - np.polyval(coef, ts)[:, None]
    rsel = np.abs(rd) <= W
    # every pixel of the band
    corners = np.array([A2 + W * n2, A2 - W * n2, B2 + W * n2, B2 - W * n2])
    x0, y0 = np.floor(corners.min(0)).astype(int); x1, y1 = np.ceil(corners.max(0)).astype(int)
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
    rel = np.stack([gx - A2[0], gy - A2[1]], -1)
    al = rel @ u2; d = rel @ n2
    sel = (al >= 0) & (al <= np.linalg.norm(B2 - A2)) & (np.abs(d) <= W)
    d, v = d[sel], Y[gy[sel], gx[sel]].astype(np.float64)
    if not (((d >= win["dark"][0]) & (d <= win["dark"][1])).any() and ((d >= win["bright"][0]) & (d <= win["bright"][1])).any()):
        return None
    D = np.median(v[(d >= win["dark"][0]) & (d <= win["dark"][1])])
    Bv = np.median(v[(d >= win["bright"][0]) & (d <= win["bright"][1])])
    return dict(d=d, v=v, rd=rd[rsel], rv=prof[rsel].astype(np.float64), D=D, B=Bv, rms=rms, n=sel.sum())


def esf(d, vn, W, wt, binw=BIN):
    """Weighted bin means of the normalised samples; empty bins are interpolated."""
    edges = np.arange(-W, W + 1e-9, binw)
    c = 0.5 * (edges[1:] + edges[:-1])
    k = np.digitize(d, edges) - 1
    good = (k >= 0) & (k < len(c))
    s = np.bincount(k[good], vn[good] * wt[good], len(c)); m = np.bincount(k[good], wt[good], len(c))
    cnt = np.bincount(k[good], None, len(c))
    e = np.full(len(c), np.nan); e[cnt > 0] = s[cnt > 0] / m[cnt > 0]
    e = np.interp(c, c[cnt > 0], e[cnt > 0])
    return c, e, cnt


def esf_metrics(c, e, win):
    j = np.nonzero((e[:-1] < 0.5) & (e[1:] >= 0.5))[0]
    if len(j) == 0:                                        # no clean 50 % crossing (sparse, noisy ESF)
        return np.nan, np.nan, np.nan, np.nan, np.nan
    j = j[np.argmin(np.abs(c[j]))]
    def cross(level, step):
        """first rising crossing of level, walking from the 50 % pair (j, j+1) left (step -1) or right (+1)"""
        i = j
        while 0 <= i < len(e) - 1:
            if e[i] < level <= e[i + 1]:
                return c[i] + (level - e[i]) / (e[i + 1] - e[i]) * (c[i + 1] - c[i])
            i += step
        return np.nan
    d10 = cross(0.1, -1); d90 = cross(0.9, +1)
    br = (c >= 0) & (c <= win["bright"][0]); dk = (c <= 0) & (c >= win["dark"][1])
    over = (e[br].max() - 1.0) * 100; under = (0.0 - e[dk].min()) * 100
    return d90 - d10, over, under, c[br][np.argmax(e[br])], c[dk][np.argmin(e[dk])]


def mtf(c, e, remap=False):
    """MTF from the ESF; for the remap ESF divide out the bilinear interpolation (averaged over sub-pixel
    phases it is a 1 px triangle filter along the normal: sinc^2)."""
    lsf = np.gradient(e, BIN) * np.hanning(len(e))
    nfft = 4096
    F = np.abs(np.fft.rfft(lsf, nfft)); F /= F[0]
    f = np.fft.rfftfreq(nfft, BIN)
    if remap:
        F = F / np.maximum(np.sinc(f) ** 2, 0.2)
    i = np.nonzero(F < 0.5)[0]
    m50 = f[i[0] - 1] + (0.5 - F[i[0] - 1]) / (F[i[0]] - F[i[0] - 1]) * (f[i[0]] - f[i[0] - 1]) if len(i) else np.nan
    band = (f >= 0.02) & (f <= 0.5)
    return m50, F[band].max(), np.interp(0.25, f, F)


def edge_set(Y, H, edges, win, binw=BIN):
    """Combine borders: each normalised to its own plateaus (0 dark, 1 bright) and weighted by step^2."""
    ds, vs, ws, rds, rvs, rws, Ds, Bs, rms, used, npx, nclip, nnear = [], [], [], [], [], [], [], [], [], [], 0, 0, 0
    for p, q, nb, tag in edges:
        r = edge_samples(Y, H, p, q, nb, win)
        if r is None or r["B"] - r["D"] < 5:
            continue
        st = r["B"] - r["D"]
        ds.append(r["d"]); vs.append((r["v"] - r["D"]) / st); ws.append(np.full(len(r["d"]), st * st))
        rds.append(r["rd"]); rvs.append((r["rv"] - r["D"]) / st); rws.append(np.full(len(r["rd"]), st * st))
        near = np.abs(r["d"]) <= 3.5
        nclip += ((r["v"][near] <= CLIP_LO) | (r["v"][near] >= CLIP_HI)).sum(); nnear += near.sum()
        Ds.append(r["D"]); Bs.append(r["B"]); rms.append(r["rms"]); used.append(tag); npx += r["n"]
    c, e, m = esf(np.concatenate(ds), np.concatenate(vs), win["W"], np.concatenate(ws), binw)
    cr, er, _ = esf(np.concatenate(rds), np.concatenate(rvs), win["W"], np.concatenate(rws))
    return dict(c=c, e=e, m=m, cr=cr, er=er, D=np.mean(Ds), B=np.mean(Bs), rms=np.mean(rms), clip=100.0 * nclip / nnear,
                used=used, npx=npx, met=esf_metrics(c, e, win), metr=esf_metrics(cr, er, win))


def noise(Y, H):
    lf, hf, sd, slf, shf, sizes = [], [], [], [], [], []
    for p in GEO["patches"][19:22]:
        cx, cy, h = p["x_mm"] + S / 2, p["y_mm"] + S / 2, 0.36 * S
        q = to_px(H, [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)])
        x0 = int(np.ceil(max(q[0, 0], q[3, 0]))); x1 = int(np.floor(min(q[1, 0], q[2, 0])))
        y0 = int(np.ceil(max(q[0, 1], q[1, 1]))); y1 = int(np.floor(min(q[2, 1], q[3, 1])))
        N = min(x1 - x0, y1 - y0) & ~1
        x0 += (x1 - x0 - N) // 2; y0 += (y1 - y0 - N) // 2
        z = Y[y0:y0 + N, x0:x0 + N].astype(np.float64)
        yy, xx = np.mgrid[0:N, 0:N]
        Am = np.stack([np.ones(N * N), xx.ravel(), yy.ravel()], 1)
        z = z - (Am @ np.linalg.lstsq(Am, z.ravel(), rcond=None)[0]).reshape(N, N)
        sd.append(z.std())
        w = np.outer(np.hanning(N), np.hanning(N))
        P = np.abs(np.fft.fft2(z * w)) ** 2 / (w ** 2).sum()          # PSD, var = mean(P) over all bins
        f = np.fft.fftfreq(N); r = np.hypot(*np.meshgrid(f, f))
        bl = (r >= 0.05) & (r < 0.15); bh = (r >= 0.25) & (r <= 0.5)
        lf.append(P[bl].mean()); hf.append(P[bh].mean())
        slf.append(np.sqrt(P[bl].sum() / N ** 2)); shf.append(np.sqrt(P[bh].sum() / N ** 2)); sizes.append(N)
    return dict(sd=np.mean(sd), ratio=np.mean(hf) / np.mean(lf), slf=np.mean(slf), shf=np.mean(shf), N=sizes)


def paper(img, Y, H):
    vals, rgb = [], []
    for cx, cy in PAPER_MM:
        g = np.linspace(-2.0, 2.0, 9)
        px = to_px(H, [(cx + a, cy + b) for a in g for b in g]).round().astype(int)
        vals.append(Y[px[:, 1], px[:, 0]]); rgb.append(img[px[:, 1], px[:, 0]][:, ::-1])
    L = lab(srgb_to_lin(np.concatenate(rgb).astype(np.float64).mean(0)))[0]
    return np.concatenate(vals).mean(), L


def analyse(path):
    img = read_bgr(path, clip=False)        # unclipped: Y below is the JPEG's own Y (expanded), halos past the range kept
    if img is None:
        return None
    H, how = homography(img)
    if H is None:
        return None
    Y = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)   # BT.601 luma (the JPEG's Y)
    pY, pL = paper(np.clip(img, 0.0, 255.0), Y, H)
    mk = edge_set(Y, H, marker_edges(), MK_WIN, binw=0.5)    # few, nearly axis-aligned runs: coarse bins
    hi = edge_set(Y, H, patch_edges(PT_HI), PT_WIN)
    lo = edge_set(Y, H, patch_edges(PT_LO), PT_WIN)
    for e in (mk, hi, lo):
        e["mtf"] = mtf(e["cr"], e["er"], remap=True)
    return dict(pY=pY, pL=pL, mk=mk, hi=hi, lo=lo, nz=noise(Y, H), how=how)


def crops(d, names, out):
    regions = [((8.0, 8.0), (44.0, 44.0)), ((250.0, 166.0), (288.0, 202.0))]
    rows = [[], [], []]
    for nm in names:
        img = read_bgr_u8(f"{d}/{nm}.csi.jpg"); H, _ = homography(img)
        for ri, (a, b) in enumerate(regions):
            q = to_px(H, [a, (b[0], a[1]), b, (a[0], b[1])])
            x0, y0 = np.floor(q.min(0)).astype(int); x1, y1 = np.ceil(q.max(0)).astype(int)
            cr = img[y0:y1, x0:x1]
            big = cv2.resize(cr, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
            rows[ri].append(big)
            if ri == 0:
                g = cv2.cvtColor(cr, cv2.COLOR_BGR2GRAY).astype(np.float32)
                hp = np.clip(128 + 4 * (g - cv2.GaussianBlur(g, (0, 0), 2.0)), 0, 255).astype(np.uint8)
                rows[2].append(cv2.cvtColor(cv2.resize(hp, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR))
    tiles = []
    for r in rows:
        h = max(t.shape[0] for t in r); w = max(t.shape[1] for t in r)
        tiles.append([cv2.copyMakeBorder(t, 0, h - t.shape[0], 0, w - t.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for t in r])
    gap = 8
    W = max(sum(t.shape[1] for t in r) + gap * (len(r) - 1) for r in tiles)
    canvas_rows = []
    head = np.full((26, W, 3), 255, np.uint8)
    x = 0
    for nm, t in zip(names, tiles[0]):
        cv2.putText(head, nm, (x + 1, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1, cv2.LINE_AA)
        x += t.shape[1] + gap
    canvas_rows.append(head)
    for r in tiles:
        row = np.full((r[0].shape[0], W, 3), 255, np.uint8); x = 0
        for t in r:
            row[:, x:x + t.shape[1]] = t; x += t.shape[1] + gap
        canvas_rows.append(row); canvas_rows.append(np.full((gap, W, 3), 255, np.uint8))
    cv2.imwrite(out, np.vstack(canvas_rows))
    print(f"crops -> {out}")


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--crops" in args:
        k = args.index("--crops")
        crops(args[0], args[k + 2:], args[k + 1])
        sys.exit(0)
    d = args[0]
    names = [a for a in args[1:] if not a.startswith("--")]
    print("ESFs from aligned remap profiles. MK = marker borders; HI = 13 high-contrast patch borders; LO = grey"
          " patches 20/21 bottom borders. rise = 10-90 % px; over/under = bright halo / dark rim, % of step;"
          " clip = % of pixels within 3.5 px of the edge at raw Y<=18 or >=233 (limited range); m.25 / pk = MTF at 0.25 cy/px / max MTF"
          " (interpolation-corrected)")
    print(f"{'name':22s} {'pY':>5s} {'L*':>4s} |MK {'rise':>4s} {'over':>5s} {'under':>5s} {'clip':>4s} |HI {'rise':>4s} {'over':>5s} "
          f"{'under':>5s} {'clip':>4s} {'m.25':>4s} |LO {'rise':>4s} {'over':>5s} {'under':>5s} {'clip':>4s} {'D>B':>7s} {'m.25':>4s} {'pk':>4s} |"
          f" {'Ystd':>4s} {'HF/LF':>5s} {'sLF':>4s} {'sHF':>4s}")
    res = {}
    for n in names:
        r = analyse(f"{d}/{n}.csi.jpg")
        if r is None:
            print(f"{n}: no image or markers not found"); continue
        res[n] = r
        mk, hi, lo, nz = r["mk"], r["hi"], r["lo"], r["nz"]
        print(f"{n:22s} {r['pY']:5.1f} {r['pL']:4.1f} |MK {mk['metr'][0]:4.2f} {mk['metr'][1]:+5.1f} {mk['metr'][2]:+5.1f} {mk['clip']:4.1f} "
              f"|HI {hi['metr'][0]:4.2f} {hi['metr'][1]:+5.1f} {hi['metr'][2]:+5.1f} {hi['clip']:4.1f} {hi['mtf'][2]:4.2f} "
              f"|LO {lo['metr'][0]:4.2f} {lo['metr'][1]:+5.1f} {lo['metr'][2]:+5.1f} {lo['clip']:4.1f} {lo['D']:3.0f}>{lo['B']:3.0f} "
              f"{lo['mtf'][2]:4.2f} {lo['mtf'][1]:4.2f} | {nz['sd']:4.2f} {nz['ratio']:5.3f} {nz['slf']:4.2f} {nz['shf']:4.2f}", flush=True)
    print("\ncross-check: ESFs from binned pixels (no interpolation blur; MK 0.5 px bins, HI/LO 0.25 px; LO and MK"
          " bins are sparse: few, nearly axis-aligned borders), rise/over/under; fit = rms (px) of the 50 % crossings"
          " about the refined border line; sample sizes")
    for n, r in res.items():
        mk, hi, lo = r["mk"], r["hi"], r["lo"]
        print(f"{n:22s} MKbin {mk['met'][0]:4.2f} {mk['met'][1]:+5.1f} {mk['met'][2]:+6.1f} | HIbin {hi['met'][0]:4.2f} "
              f"{hi['met'][1]:+5.1f} {hi['met'][2]:+5.1f} | LObin {lo['met'][0]:4.2f} {lo['met'][1]:+6.1f} {lo['met'][2]:+6.1f} | "
              f"fit {mk['rms']:.2f}/{hi['rms']:.2f}/{lo['rms']:.2f} | MK {len(mk['used'])} runs {mk['npx']} px D>B {mk['D']:.0f}>{mk['B']:.0f}, "
              f"HI {len(hi['used'])} {hi['npx']} px, LO {len(lo['used'])} {lo['npx']} px, noise {r['nz']['N']}, H {r['how']}")
    if "--profiles" in args:
        for n, r in res.items():
            for key, k, cc, ee in (("MK remap", "mk", "cr", "er"), ("HI remap", "hi", "cr", "er"), ("LO remap", "lo", "cr", "er")):
                c, e = r[k][cc], r[k][ee]
                sel = (c > -6.5) & (c < 6.5)
                print(f"{n} {key} ESF (d px: value)")
                print("  " + " ".join(f"{ci:+.2f}:{ei:+.2f}" for ci, ei in zip(c[sel], e[sel])))
