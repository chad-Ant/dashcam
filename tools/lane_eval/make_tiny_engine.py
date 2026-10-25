#!/usr/bin/env python3
"""make_tiny_engine.py — tiny UFLD-v2-signature TensorRT engines for lane_test --gpu.

Runs INSIDE the l4t-ml-gpio container (python3 + onnx + tensorrt + trtexec;
engines are locked to the TensorRT version that built them):

    docker run --rm --runtime nvidia \\
        -v /home/jetson/dashcam:/user/dashcam \\
        -v /home/jetson/drive_logs/lane_engines:/user/lane_engines \\
        -w /user/dashcam l4t-ml-gpio \\
        python3 tools/lane_eval/make_tiny_engine.py --out /user/lane_engines

Writes, into OUT (refused when inside the repo — the repo is public):

    tiny.onnx / tiny.engine        float32 IO, the real engine's signature
    tiny_fp16in.engine             same, input bound as fp16:chw    (E2, plan bug B10)
    tiny_fp16out.engine            same, all 4 outputs fp16:chw     (E2 control)

Signature (= models/culane_res18_fp16.engine, engine_check facts.json):
    input      "input"     (1,3,320,1600)
    loc_row    (1,200,72,4)     loc_col    (1,100,81,4)
    exist_row  (1,2,72,4)       exist_col  (1,2,81,4)

Every output is Const + 0 * ReduceMean(input), broadcast: it depends on the
input, so the parser keeps the input binding, and its value is exactly the
constant for any finite input.  The constants are lane_test_unit.cpp's
fourLanes() fixture (also rebuilt in lane_test_gpu.cpp, which compares the
TensorRT result bit-for-bit against a CPU decode of it):
    slot 1  row head, cell 69 on all 72 anchors     (x 558.79 px at W 1600)
    slot 2  row head, cell 129 on all 72 anchors    (x 1041.21 px)
    slot 0  col head, anchors 0..29,  y = 0.4 + 0.6 (1 - k/29)
    slot 3  col head, anchors 51..80, y = 0.4 + 0.6 (k - 51)/29
    absent anchors: exist (1, 0); present: exist (0, 1) and location
    logits -10 except +10 at the chosen cell.
Decoded: 4 boundaries -> 3 lanes, ego lane 1, lateral offset ~0.

After each build the engine is deserialized and its IO checked: exactly one
input, named "input", with the expected shape and dtype, and the four
outputs with the expected shapes and dtypes.  Exit 0 = all built and
checked, 1 = usage / guard error, 2 = a build or check failed.
"""

import argparse
import math
import os
import subprocess
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

TRTEXEC = "/usr/src/tensorrt/bin/trtexec"
REPO_ROOTS = ("/user/dashcam", "/home/jetson/Documents/github_repos/dashcam")

IN_NAME = "input"
IN_SHAPE = (1, 3, 320, 1600)
CELL_ROW, ROW_ANCH, CELL_COL, COL_ANCH, LANES = 200, 72, 100, 81, 4
OUTS = {
    "loc_row":   (1, CELL_ROW, ROW_ANCH, LANES),
    "loc_col":   (1, CELL_COL, COL_ANCH, LANES),
    "exist_row": (1, 2, ROW_ANCH, LANES),
    "exist_col": (1, 2, COL_ANCH, LANES),
}


def lround(v):
    """C lround for the non-negative values used here (half away from zero)."""
    return int(math.floor(v + 0.5)) if v >= 0 else -int(math.floor(-v + 0.5))


def fixture():
    """The fourLanes() tensors of lane_test_unit.cpp, batch dimension added."""
    loc_row = np.zeros(OUTS["loc_row"], np.float32)
    loc_col = np.zeros(OUTS["loc_col"], np.float32)
    exist_row = np.zeros(OUTS["exist_row"], np.float32)
    exist_col = np.zeros(OUTS["exist_col"], np.float32)
    exist_row[0, 0, :, :] = 1.0   # everything absent: exist[0] = 1 > exist[1] = 0
    exist_col[0, 0, :, :] = 1.0

    def row_point(lane, k, g):
        exist_row[0, 0, k, lane] = 0.0
        exist_row[0, 1, k, lane] = 1.0
        loc_row[0, :, k, lane] = -10.0
        loc_row[0, g, k, lane] = 10.0

    def col_point(lane, k, g):
        exist_col[0, 0, k, lane] = 0.0
        exist_col[0, 1, k, lane] = 1.0
        loc_col[0, :, k, lane] = -10.0
        loc_col[0, g, k, lane] = 10.0

    def col_line(lane, k0, k1, y):
        for k in range(k0, k1):
            g = lround(y(k) * (CELL_COL - 1) - 0.5)
            col_point(lane, k, max(0, min(CELL_COL - 1, g)))

    for k in range(ROW_ANCH):
        row_point(1, k, 69)
        row_point(2, k, 129)
    col_line(0, 0, 30, lambda k: 0.4 + 0.6 * (1.0 - k / 29.0))
    col_line(3, 51, 81, lambda k: 0.4 + 0.6 * ((k - 51) / 29.0))
    return {"loc_row": loc_row, "loc_col": loc_col,
            "exist_row": exist_row, "exist_col": exist_col}


def build_onnx(path):
    consts = fixture()
    nodes = [
        helper.make_node("ReduceMean", [IN_NAME], ["mean"], axes=[1, 2, 3], keepdims=1),
        helper.make_node("Mul", ["mean", "zero"], ["zeroed"]),
    ]
    inits = [numpy_helper.from_array(np.zeros((1, 1, 1, 1), np.float32), "zero")]
    outputs = []
    for name, shape in OUTS.items():
        inits.append(numpy_helper.from_array(consts[name], name + "_const"))
        nodes.append(helper.make_node("Add", [name + "_const", "zeroed"], [name]))
        outputs.append(helper.make_tensor_value_info(name, TensorProto.FLOAT, list(shape)))
    graph = helper.make_graph(
        nodes, "ufld_v2_tiny",
        [helper.make_tensor_value_info(IN_NAME, TensorProto.FLOAT, list(IN_SHAPE))],
        outputs, initializer=inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)],
                              producer_name="make_tiny_engine.py")
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, path)


def trtexec(onnx_path, engine_path, extra, log_path):
    cmd = [TRTEXEC, "--onnx=" + onnx_path, "--saveEngine=" + engine_path,
           "--skipInference"] + extra
    with open(log_path, "w") as log:
        rc = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
    print(("  built " if rc == 0 else "  FAILED ") + engine_path + "  (" + " ".join(extra) + ")")
    return rc == 0


def check_engine(path, in_dtype, out_dtype):
    """Deserialize and verify the IO contract; returns a list of problems."""
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.ERROR)
    with open(path, "rb") as f:
        engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
    if engine is None:
        return ["does not deserialize"]
    want_dt = {"float": trt.DataType.FLOAT, "half": trt.DataType.HALF}
    problems = []
    inputs = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)
              if engine.get_tensor_mode(engine.get_tensor_name(i)) == trt.TensorIOMode.INPUT]
    if inputs != [IN_NAME]:
        problems.append("inputs %s, want exactly ['%s']" % (inputs, IN_NAME))
    else:
        if tuple(engine.get_tensor_shape(IN_NAME)) != IN_SHAPE:
            problems.append("input shape %s" % (tuple(engine.get_tensor_shape(IN_NAME)),))
        if engine.get_tensor_dtype(IN_NAME) != want_dt[in_dtype]:
            problems.append("input dtype %s, want %s" % (engine.get_tensor_dtype(IN_NAME), in_dtype))
    for name, shape in OUTS.items():
        try:
            mode = engine.get_tensor_mode(name)
        except Exception:  # noqa: BLE001 — older bindings raise on unknown names
            mode = None
        if mode != trt.TensorIOMode.OUTPUT:
            problems.append("no output " + name)
            continue
        if tuple(engine.get_tensor_shape(name)) != shape:
            problems.append("%s shape %s" % (name, tuple(engine.get_tensor_shape(name))))
        if engine.get_tensor_dtype(name) != want_dt[out_dtype]:
            problems.append("%s dtype %s, want %s" % (name, engine.get_tensor_dtype(name), out_dtype))
    if engine.num_io_tensors != 1 + len(OUTS):
        problems.append("%d IO tensors, want %d" % (engine.num_io_tensors, 1 + len(OUTS)))
    return problems


def inside_repo(path):
    real = os.path.realpath(path)
    roots = list(REPO_ROOTS) + [os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))]
    for r in roots:
        rr = os.path.realpath(r)
        if real == rr or real.startswith(rr + os.sep):
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="output directory (outside the repo)")
    args = ap.parse_args()
    out = os.path.realpath(args.out)
    if inside_repo(out):
        print("ERROR: --out %s is inside the repo; refusing" % out, file=sys.stderr)
        return 1
    if not os.access(TRTEXEC, os.X_OK):
        print("ERROR: %s not found (run in the container)" % TRTEXEC, file=sys.stderr)
        return 1
    os.makedirs(out, exist_ok=True)
    st = os.stat(out)

    onnx_path = os.path.join(out, "tiny.onnx")
    build_onnx(onnx_path)
    print("  wrote " + onnx_path)

    fp16x4 = ",".join(["fp16:chw"] * len(OUTS))
    variants = [
        ("tiny.engine",         [],                                "float", "float"),
        ("tiny_fp16in.engine",  ["--inputIOFormats=fp16:chw"],     "half",  "float"),
        ("tiny_fp16out.engine", ["--outputIOFormats=" + fp16x4],   "float", "half"),
    ]
    rc = 0
    made = [onnx_path]
    for name, extra, in_dt, out_dt in variants:
        engine = os.path.join(out, name)
        log = engine[:-len(".engine")] + "_build.log"
        made += [engine, log]
        if not trtexec(onnx_path, engine, extra, log):
            rc = 2
            continue
        problems = check_engine(engine, in_dt, out_dt)
        if problems:
            print("  CHECK FAILED %s: %s" % (name, "; ".join(problems)))
            rc = 2
        else:
            print("  checked %s: one input '%s' %s, outputs %s, shapes ok"
                  % (name, IN_NAME, in_dt, out_dt))
    # Results belong to whoever owns OUT (the container runs as root).
    for p in made:
        if os.path.exists(p):
            try:
                os.chown(p, st.st_uid, st.st_gid)
            except OSError:
                pass
    print("RESULT: " + ("PASS" if rc == 0 else "FAIL"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
