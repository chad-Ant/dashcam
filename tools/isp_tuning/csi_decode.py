"""csi_decode.py: read a chart image with the right YCbCr range.

nvjpegenc (the CSI stills: pair.sh, tune_session.py) writes nvarguscamerasrc's NV12 into the JPEG as is:
limited-range BT.601 YCbCr (Y 16..235, C 16..240), with no JFIF/APPn marker and SOF component ids 0,1,2.
Every JPEG decoder assumes JFIF full range, so a plain cv2.imread lifts the blacks (Y 16 -> RGB 16) and
shrinks the chroma by 224/255.  The UGREEN (and any JFIF file) is standard full-range BT.601: cv2 is right.

read_bgr(path)     float32 BGR 0..255, correctly decoded (clipped to 0..255 after the conversion; clip=False keeps
                   the out-of-range values, so the BT.601 luma of the result is exactly the JPEG's Y, expanded)
read_bgr_u8(path)  the same, rounded to uint8 (marker detection, drawing, SIFT)
is_nvjpeg(path)    True for an nvjpegenc still, decided from the header, not the file name
Checked 2026-10-01 against 250 CSI and 230 UGREEN files (reviews_20261001/verify_d123/range_survey.txt).
"""
import struct
import numpy as np


def _segments(data):
    """(marker, payload) of the JPEG header segments up to the first SOS."""
    i = 2
    while i + 4 <= len(data):
        if data[i] != 0xFF:
            raise ValueError("bad JPEG marker")
        m = data[i + 1]
        if m == 0xFF:                                   # fill byte
            i += 1
            continue
        n = struct.unpack(">H", data[i + 2:i + 4])[0]
        yield m, data[i + 4:i + 2 + n]
        if m == 0xDA:                                   # SOS: entropy-coded data follows
            return
        i += 2 + n


def is_nvjpeg(path):
    with open(path, "rb") as f:
        data = f.read(65536)
    if data[:2] != b"\xff\xd8":
        return False
    app, ids = False, None
    for m, p in _segments(data):
        if 0xE0 <= m <= 0xEF:
            app = True
        elif m in (0xC0, 0xC1, 0xC2) and len(p) >= 6:
            nc = p[5]
            ids = tuple(p[6 + 3 * k] for k in range(nc))
    return not app and ids == (0, 1, 2)


def read_bgr(path, clip=True):
    import cv2
    try:
        nv = is_nvjpeg(path)
    except (OSError, ValueError):                       # missing / corrupt header: None, as cv2.imread
        return None
    if not nv:
        img = cv2.imread(path)
        return None if img is None else img.astype(np.float32)
    from PIL import Image
    try:
        im = Image.open(path)
        im.draft("YCbCr", im.size)                      # the decoder's own YCbCr: no colour conversion
        mode = im.mode
        a = np.asarray(im, np.float32) if mode == "YCbCr" else None
    except OSError:                                     # truncated / corrupt data (UnidentifiedImageError is an OSError)
        return None
    if a is None:
        raise ValueError(f"{path}: PIL gave {mode}, not YCbCr")
    y = (a[..., 0] - 16.0) * (255.0 / 219.0)
    cb = (a[..., 1] - 128.0) * (255.0 / 224.0)
    cr = (a[..., 2] - 128.0) * (255.0 / 224.0)
    r = y + 1.402 * cr
    g = y - 0.344136 * cb - 0.714136 * cr
    b = y + 1.772 * cb
    bgr = np.stack([b, g, r], -1)
    return np.clip(bgr, 0.0, 255.0) if clip else bgr


def read_bgr_u8(path):
    img = read_bgr(path)
    return None if img is None else np.clip(img + 0.5, 0, 255).astype(np.uint8)


if __name__ == "__main__":
    import sys
    for p in sys.argv[1:]:
        print(f"{p}: {'nvjpegenc limited range' if is_nvjpeg(p) else 'JFIF/other full range'}")
