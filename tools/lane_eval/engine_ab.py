#!/usr/bin/env python3
"""engine_ab.py — compare UFLD v2 TensorRT engines on identical inputs.

Runs INSIDE the l4t-ml-gpio container (TensorRT + torch/CUDA + onnxruntime):

    python3 tools/lane_eval/engine_ab.py --frames /c/val_bgr_320x1600.raw \
        --onnx /ufldw/ufldv2_culane_res18_320x1600.onnx \
        E0=models/culane_res18_fp16.engine E1=/user/lane_engines/E1_fp16_forced.engine

Frames are raw BGR uint8 at the model size (320x1600, N frames back to back),
normalised exactly like liblanedetector_preprocess.cu at identity geometry:
BGR->RGB, /255, ImageNet mean/std, NCHW float32.  The reference is the ONNX
model run in FP32 by onnxruntime on the CPU (--onnx; optional).  Without it
the first engine is the reference.

Reports, per engine vs the reference:
  - raw output max|d| and mean|d| per head;
  - exist decision agreement (argmax over the 2 classes, per anchor x lane);
  - loc argmax agreement where the reference says "exists";
  - lane-slot detection agreement, using the library's thresholds
    (row head slots 1,2: count > K/2; col head slots 0,3: count > K/4).
Writes nothing; exit 0.
"""
import argparse
import sys

import numpy as np

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
OUTS = ["loc_row", "loc_col", "exist_row", "exist_col"]
H, W = 320, 1600


def load_frames(path, limit):
    a = np.fromfile(path, np.uint8)
    n = a.size // (H * W * 3)
    a = a[: n * H * W * 3].reshape(n, H, W, 3)[:limit]
    rgb = a[..., ::-1].astype(np.float32) / 255.0
    x = (rgb - MEAN) / STD
    return np.ascontiguousarray(x.transpose(0, 3, 1, 2))   # N,3,H,W


def run_trt(path, x):
    import tensorrt as trt
    import torch
    logger = trt.Logger(trt.Logger.ERROR)
    with open(path, "rb") as f:
        engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
    if engine is None:
        sys.exit(f"cannot load {path}")
    ctx = engine.create_execution_context()
    names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
    inp = [n for n in names if engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT][0]
    bufs = {n: torch.empty(tuple(engine.get_tensor_shape(n)), dtype=torch.float32,
                           device="cuda") for n in names}
    for n, b in bufs.items():
        ctx.set_tensor_address(n, b.data_ptr())
    stream = torch.cuda.Stream()
    res = {o: [] for o in OUTS}
    for i in range(x.shape[0]):
        bufs[inp].copy_(torch.from_numpy(x[i:i + 1]))
        torch.cuda.synchronize()
        ctx.execute_async_v3(stream.cuda_stream)
        stream.synchronize()
        for o in OUTS:
            res[o].append(bufs[o].cpu().numpy()[0])
    return {o: np.stack(v) for o, v in res.items()}


def run_onnx(path, x):
    import onnxruntime as ort
    so = ort.SessionOptions()
    sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name
    res = {o: [] for o in OUTS}
    for i in range(x.shape[0]):
        out = sess.run(OUTS, {iname: x[i:i + 1]})
        for o, v in zip(OUTS, out):
            res[o].append(v[0])
    return {o: np.stack(v) for o, v in res.items()}


def slot_detected(exist, rowhead):
    # exist: N,2,K,L -> per frame/slot "lane present" like the library
    present = exist[:, 1] > exist[:, 0]                   # N,K,L
    K = exist.shape[2]
    cnt = present.sum(axis=1)                             # N,L
    return cnt * 2 > K if rowhead else cnt * 4 > K


def compare(ref, got):
    out = {}
    for o in OUTS:
        d = np.abs(ref[o] - got[o])
        out[o] = (float(d.max()), float(d.mean()))
    for head in ("row", "col"):
        er, eg = ref["exist_" + head], got["exist_" + head]
        pr, pg = er[:, 1] > er[:, 0], eg[:, 1] > eg[:, 0]
        out["exist_agree_" + head] = float((pr == pg).mean())
        lr, lg = ref["loc_" + head].argmax(1), got["loc_" + head].argmax(1)
        out["loc_argmax_agree_" + head] = float((lr == lg)[pr].mean()) if pr.any() else float("nan")
        out["loc_argmax_maxcell_" + head] = int(np.abs(lr - lg)[pr].max()) if pr.any() else 0
    sr = np.concatenate([slot_detected(ref["exist_col"], False)[:, [0, 3]],
                         slot_detected(ref["exist_row"], True)[:, [1, 2]]], 1)
    sg = np.concatenate([slot_detected(got["exist_col"], False)[:, [0, 3]],
                         slot_detected(got["exist_row"], True)[:, [1, 2]]], 1)
    out["slot_agree"] = float((sr == sg).mean())
    out["slots_detected_ref"] = int(sr.sum())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    ap.add_argument("--limit", type=int, default=20)
    ap.add_argument("--onnx")
    ap.add_argument("engines", nargs="+", help="NAME=path.engine")
    a = ap.parse_args()

    x = load_frames(a.frames, a.limit)
    print(f"frames: {x.shape[0]} from {a.frames}")
    results = {}
    if a.onnx:
        results["onnx_fp32"] = run_onnx(a.onnx, x)
    for spec in a.engines:
        name, path = spec.split("=", 1)
        results[name] = run_trt(path, x)
    names = list(results)
    ref = names[0]
    print(f"reference: {ref}")
    for n in names[1:]:
        c = compare(results[ref], results[n])
        print(f"\n== {n} vs {ref}")
        for o in OUTS:
            print(f"  {o:10s} max|d| {c[o][0]:9.4f}  mean|d| {c[o][1]:.5f}")
        for k in ("exist_agree_row", "exist_agree_col", "loc_argmax_agree_row",
                  "loc_argmax_agree_col", "loc_argmax_maxcell_row",
                  "loc_argmax_maxcell_col", "slot_agree", "slots_detected_ref"):
            print(f"  {k:24s} {c[k]}")


if __name__ == "__main__":
    main()
