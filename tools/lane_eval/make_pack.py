#!/usr/bin/env python3
"""make_pack.py — build a lane replay pack from recorded footage.

Runs INSIDE the l4t-ml-gpio container (ffmpeg, cv2, numpy, PIL):

    python3 tools/lane_eval/make_pack.py --out /packs/night_csi_1003 --camera csi \\
        --frames 300 --condition night /footage/csi/p00_000{10,20,30}.mkv

A pack is what `lane_test --replay --pack DIR` reads: full-size frames as
lossless PNG (BGR, decoded the way the live camera path delivers them) and
pack.csv:

    # camera=csi
    # src_w=1456
    # src_h=1088
    # crop_top=0.5            (only with --crop-top/--crop-bottom/--ref-y;
    # crop_bottom=0.2059       lane_test uses the <Detection> defaults otherwise)
    # source=...              provenance lines, ignored by lane_test
    file,src_file,src_pts_ns,condition
    00000.png,p00_00010.mkv,1234000000,night

Frames are spread evenly over the inputs (--frames total, split by segment
length).  MJPEG inputs are demuxed with a bitstream copy (no re-encode) and
each JPEG is decoded with tools/isp_tuning/csi_decode.read_bgr_u8, which
handles the CSI's limited-range nvjpegenc stills as well as ordinary JFIF
(UGREEN).  Other codecs (H.264 example clips) are decoded by OpenCV.
--flip applies a 180° rotation (the upside-down 2026-09-26 morning footage).

The pack must not go inside the repo (public): the output path is refused if
it is.  Packs hold vehicle footage — keep them on /media/jetson/backup.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.realpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "isp_tuning"))
import csi_decode  # noqa: E402

REPO_ROOTS = [REPO, "/user/dashcam", "/home/jetson/Documents/github_repos/dashcam"]


def inside_repo(path):
    real = os.path.realpath(path)
    for r in REPO_ROOTS:
        rr = os.path.realpath(r)
        if real == rr or real.startswith(rr + os.sep):
            return True
    return False


def probe(path):
    """(codec, width, height, [packet pts in ns]) of the first video stream."""
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=codec_name,width,height,time_base",
         "-show_entries", "packet=pts", "-of", "json", path],
        check=True, capture_output=True, text=True).stdout
    j = json.loads(out)
    st = j["streams"][0]
    num, den = (int(x) for x in st["time_base"].split("/"))
    pts = sorted(int(p["pts"]) for p in j.get("packets", []) if p.get("pts") not in (None, "N/A"))
    pts_ns = [p * num * 1_000_000_000 // den for p in pts]
    return st["codec_name"], int(st["width"]), int(st["height"]), pts, pts_ns


def pick(n_total, n_want):
    if n_want >= n_total:
        return list(range(n_total))
    return [int(round(i * (n_total - 1) / max(1, n_want - 1))) for i in range(n_want)]


def frames_mjpeg(path, pts, wanted, tmp):
    """Yield (index, BGR uint8) for the wanted packet indices — bitstream copy.

    A wanted packet that is not a decodable JPEG (the first packet of many
    v0.4 UGREEN segments is not) is replaced by the nearest decodable one
    within 5 packets that is not wanted itself."""
    d = tempfile.mkdtemp(dir=tmp)
    try:
        # -frame_pts names each JPEG by its pts in stream time base units;
        # -copyts keeps the container's pts (splitmuxsink segments start far
        # from 0, and ffmpeg would otherwise rebase them to 0).
        subprocess.run(["ffmpeg", "-v", "error", "-copyts", "-i", path, "-map", "0:v:0", "-c:v", "copy",
                        "-f", "image2", "-frame_pts", "1", os.path.join(d, "%d.jpg")],
                       check=True)
        taken = set(wanted)

        def load(i):
            jpg = os.path.join(d, f"{pts[i]}.jpg")
            return csi_decode.read_bgr_u8(jpg) if os.path.exists(jpg) else None

        for i in wanted:
            img = load(i)
            if img is None:
                for j in sorted(range(max(0, i - 5), min(len(pts), i + 6)), key=lambda j: abs(j - i)):
                    if j not in taken and (img := load(j)) is not None:
                        print(f"NOTE: {os.path.basename(path)} packet {i} not a JPEG; took {j}",
                              file=sys.stderr)
                        taken.add(j)
                        i = j
                        break
            yield i, img
    finally:
        shutil.rmtree(d, ignore_errors=True)


def frames_cv(path, wanted):
    cap = cv2.VideoCapture(path)
    want = set(wanted)
    i = 0
    while want:
        ok, img = cap.read()
        if not ok:
            break
        if i in want:
            want.discard(i)
            yield i, img
        i += 1
    cap.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera", required=True, help="csi | usb | file (label only)")
    ap.add_argument("--frames", type=int, default=300, help="total frames over all inputs")
    ap.add_argument("--condition", default="", help="day | dusk | night (label)")
    ap.add_argument("--crop-top", type=float)
    ap.add_argument("--crop-bottom", type=float)
    ap.add_argument("--ref-y", type=float)
    ap.add_argument("--flip", action="store_true", help="rotate 180 degrees")
    ap.add_argument("--tmp", default=None, help="scratch dir for demuxed JPEGs")
    ap.add_argument("inputs", nargs="+")
    a = ap.parse_args()

    if inside_repo(a.out):
        sys.exit(f"ERROR: --out {a.out} is inside the repo; refusing")
    if a.tmp and inside_repo(a.tmp):
        sys.exit(f"ERROR: --tmp {a.tmp} is inside the repo; refusing")
    os.makedirs(a.out, exist_ok=True)
    if os.path.exists(os.path.join(a.out, "pack.csv")):
        sys.exit(f"ERROR: {a.out}/pack.csv exists; use a new directory")

    probes = [(p, *probe(p)) for p in a.inputs]
    total_pk = sum(len(pr[4]) for pr in probes)
    sizes = {(pr[2], pr[3]) for pr in probes}
    if len(sizes) != 1:
        sys.exit(f"ERROR: inputs differ in size: {sizes}")
    (w, h), = sizes

    rows = []
    k = 0
    total_wanted = 0
    for path, codec, _, _, pts, pts_ns in probes:
        share = max(1, round(a.frames * len(pts) / total_pk)) if total_pk else 0
        wanted = pick(len(pts), share)
        total_wanted += len(wanted)
        gen = frames_mjpeg(path, pts, wanted, a.tmp) if codec == "mjpeg" else frames_cv(path, wanted)
        for i, img in gen:
            if img is None:
                print(f"WARN: {os.path.basename(path)} frame {i} unreadable; skipped", file=sys.stderr)
                continue
            if img.shape[:2] != (h, w):
                sys.exit(f"ERROR: {path} frame {i} is {img.shape[1]}x{img.shape[0]}, expected {w}x{h}")
            if a.flip:
                img = cv2.rotate(img, cv2.ROTATE_180)
            name = f"{k:05d}.png"
            cv2.imwrite(os.path.join(a.out, name), np.ascontiguousarray(img))
            rows.append((name, os.path.basename(path), pts_ns[i] if i < len(pts_ns) else ""))
            k += 1
        print(f"{os.path.basename(path)}: {codec} {w}x{h}, {len(pts)} packets, took {len(wanted)}")

    with open(os.path.join(a.out, "pack.csv"), "w") as f:
        f.write(f"# camera={a.camera}\n# src_w={w}\n# src_h={h}\n")
        for key, val in (("crop_top", a.crop_top), ("crop_bottom", a.crop_bottom), ("ref_y", a.ref_y)):
            if val is not None:
                f.write(f"# {key}={val}\n")
        f.write(f"# flip={int(a.flip)}\n# source={' '.join(os.path.realpath(p) for p in a.inputs)}\n")
        f.write("file,src_file,src_pts_ns,condition\n")
        for name, src, pts in rows:
            f.write(f"{name},{src},{pts},{a.condition}\n")
    print(f"pack: {len(rows)} frames -> {a.out}")
    if len(rows) < total_wanted:
        print(f"WARN: pack has {len(rows)} of the {total_wanted} frames selected "
              f"(unreadable ones skipped)", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
