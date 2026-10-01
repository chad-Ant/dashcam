"""make_chart.py <out dir>: an A4-landscape test chart for comparing two cameras.

24 patches with the ColorChecker Classic sRGB values (as printed they are only
approximate, the comparison is between cameras on the same paper), unprinted white
around them, and four ArUco markers (DICT_4X4_50, ids 0-3 clockwise from top-left)
so the chart can be found automatically in both images.
"""
import sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
DPI = 300
mm = lambda v: int(round(v * DPI / 25.4))
W, H = mm(297), mm(210)
PATCHES = [
    (115, 82, 68), (194, 150, 130), (98, 122, 157), (87, 108, 67), (133, 128, 177), (103, 189, 170),
    (214, 126, 44), (80, 91, 166), (193, 90, 99), (94, 60, 108), (157, 188, 64), (224, 163, 46),
    (56, 61, 150), (70, 148, 73), (175, 54, 60), (231, 199, 31), (187, 86, 149), (8, 133, 161),
    (243, 243, 242), (200, 200, 200), (160, 160, 160), (122, 122, 121), (85, 85, 85), (52, 52, 52)]
P, G = mm(36), mm(5)                         # patch size, gap
gw, gh = 6 * P + 5 * G, 4 * P + 3 * G
x0, y0 = (W - gw) // 2, (H - gh) // 2
img = Image.new("RGB", (W, H), (255, 255, 255))
d = ImageDraw.Draw(img)
for i, c in enumerate(PATCHES):
    r, k = divmod(i, 6)
    x, y = x0 + k * (P + G), y0 + r * (P + G)
    d.rectangle([x, y, x + P - 1, y + P - 1], fill=c)
M, m = mm(20), mm(4)                         # marker size, margin from the paper edge
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
corners = [(m, m), (W - m - M, m), (W - m - M, H - m - M), (m, H - m - M)]
for i, (x, y) in enumerate(corners):
    mk = cv2.aruco.generateImageMarker(dic, i, M) if hasattr(cv2.aruco, "generateImageMarker") else cv2.aruco.drawMarker(dic, i, M)
    img.paste(Image.fromarray(mk).convert("RGB"), (x, y))
d.text((x0, y0 + gh + mm(3)), "IMX296 / UGREEN comparison chart - print A4 landscape, 100% scale, matte paper, no colour enhancement",
       fill=(120, 120, 120), font=ImageFont.load_default())
out = sys.argv[1]
img.save(f"{out}/chart_a4.png", dpi=(DPI, DPI))
img.save(f"{out}/chart_a4.pdf", resolution=DPI)
# geometry for the analysis: chart coordinates in mm of each marker's corners and each patch
import json
geo = {"paper_mm": [297, 210], "marker_mm": 20, "markers_tl_mm": [[x * 25.4 / DPI, y * 25.4 / DPI] for x, y in corners],
       "patches": [{"i": i + 1, "srgb": list(c), "x_mm": (x0 + (i % 6) * (P + G)) * 25.4 / DPI, "y_mm": (y0 + (i // 6) * (P + G)) * 25.4 / DPI,
                    "size_mm": P * 25.4 / DPI} for i, c in enumerate(PATCHES)]}
json.dump(geo, open(f"{out}/chart_geometry.json", "w"), indent=1)
print("ok", W, H)
