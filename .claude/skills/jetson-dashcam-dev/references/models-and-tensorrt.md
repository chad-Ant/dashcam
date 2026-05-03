# Models and TensorRT on JetPack 6.2

## TensorRT version on JetPack 6.2

JetPack 6.2 ships **TensorRT 10.3** with **CUDA 12.6** and **cuDNN 9.x**. Library SONAMEs:
- `libnvinfer.so.10`
- `libnvonnxparser.so.10`
- `libnvinfer_plugin.so.10`

Python TensorRT bindings ship as part of the JetPack apt packages, not via pip. Find them at `/usr/lib/python3.10/dist-packages/tensorrt*/`. If you use a venv, create it with `--system-site-packages` so it sees them — `pip install tensorrt` pulls x86 wheels and won't work.

## Recommended model conversion path

Train / download a model in PyTorch (or any framework) → export to ONNX → convert ONNX to a TensorRT engine on the device → use the engine in DeepStream or a custom inference app.

**Build engines on the target device.** TensorRT engines are not portable across compute capabilities, hardware modules, or TRT versions. An engine built on Orin NX won't load on Orin Nano even though both are Ampere. An engine built on the host x86 GPU definitely won't load on Jetson.

## `trtexec` quick reference

`trtexec` is at `/usr/src/tensorrt/bin/trtexec` (on host) or in the DeepStream container.

Convert an ONNX model to a serialized engine:
```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=yolov8s.onnx \
  --saveEngine=yolov8s_fp16.engine \
  --fp16 \
  --workspace=2048 \
  --minShapes=images:1x3x640x640 \
  --optShapes=images:1x3x640x640 \
  --maxShapes=images:1x3x640x640 \
  --verbose
```

Flag notes:
- `--fp16` — use FP16 where supported. ~2× throughput vs FP32 with negligible accuracy loss for most CV models. Default choice on Jetson.
- `--int8` — INT8 quantization. ~2× more throughput vs FP16. Requires a calibration dataset; quality varies by model. Worth pursuing for production after FP16 baseline works.
- `--workspace=2048` — MB. Bumps the max workspace TRT can use during build. 2 GB is reasonable on Orin Nano (don't go past 4 GB or you'll OOM during build).
- `--shapes` flags — required for dynamic-shape ONNX models. For dashcam, you typically use fixed batch=1 fixed input size, so all three (`min`/`opt`/`max`) match.
- Don't pass `--useDLACore=N` on Orin Nano (no DLA).

Inspect an engine:
```bash
/usr/src/tensorrt/bin/trtexec --loadEngine=yolov8s_fp16.engine --verbose
```

## YOLOv8 → TensorRT for dashcam

For the dashcam's primary detector (cars, lanes, signs), the user has a YOLOv8 model. Workflow:

1. Export ONNX from PyTorch:
   ```python
   from ultralytics import YOLO
   model = YOLO("yolov8s.pt")
   model.export(format="onnx", imgsz=640, opset=17, simplify=True, dynamic=False)
   # Produces yolov8s.onnx
   ```
2. Copy to Jetson, run `trtexec --onnx=... --saveEngine=... --fp16`.
3. Plug into DeepStream `nvinfer` config:
   ```
   onnx-file=/models/yolov8s.onnx
   model-engine-file=/models/yolov8s.onnx_b1_gpu0_fp16.engine
   network-mode=2
   ```
   (DeepStream 7.1 auto-builds the engine from ONNX on first run if `model-engine-file` doesn't exist; subsequent runs use the cache.)

### Custom YOLO output parser

YOLOv8 outputs one tensor `[1, 84, 8400]` (4 box coords + 80 classes). DeepStream doesn't natively understand this; you need a custom output parser. Two paths:

- **Use NVIDIA's `nvdsinfer_custom_impl_Yolo` library** — supports many YOLO variants. Build from `https://github.com/marcoslucianops/DeepStream-Yolo` (community, well-maintained), point `parse-bbox-func-name` and `custom-lib-path` at it.
- **Write your own parser** — implement `NvDsInferParseCustomYoloV8` in C++. More code, more control. Necessary if you trained on a custom class set with non-default output shape.

For dashcam v0.1, use DeepStream-Yolo. Saves a week.

## INT8 calibration

For INT8, you need a calibration set: ~500–1000 representative images saved to disk, plus a calibration cache.

```bash
trtexec \
  --onnx=yolov8s.onnx \
  --saveEngine=yolov8s_int8.engine \
  --int8 \
  --calib=calib_cache.bin \
  --workspace=2048
```

`trtexec` will look for image inputs to calibrate with via the network's input bindings. For non-trivial calibration (the usual case), use the Python API with a custom `IInt8EntropyCalibrator2` that yields preprocessed batches from your dataset. Search for "TensorRT INT8 calibrator example" in NVIDIA's `TensorRT/samples/python/` directory.

Important: calibration data must look like real driving data (same lighting, same camera, same crops). Calibrate with cherry-picked daytime sunny clips and nighttime / rain accuracy will tank.

## Common conversion gotchas

- **ONNX opset too high** — TRT 10.3 supports up to opset 20-ish. PyTorch-exported opset 17 is safe; opset 21 may not be.
- **Unsupported ONNX op** — error like `getPluginCreator could not find Plugin: ...`. Check the TRT release notes for supported ops; otherwise rewrite the offending layer in PyTorch (e.g., custom upsample → standard Resize).
- **Dynamic shapes you don't need** — leaving `dynamic=True` in the export creates an engine that's hard to optimize. For fixed-input dashcam pipelines, export with `dynamic=False`.
- **Building the engine inside DeepStream's first run** — takes 5–15 min on Orin Nano for a small YOLO. Subsequent runs read the `.engine` cache instantly. Always pre-build on the device once and ship the `.engine` with your container/image.
- **Engine vs ONNX path mismatch** — DeepStream's `model-engine-file` is silently ignored if it doesn't match the requested batch size / FP mode; it then rebuilds from `onnx-file`. Look for "Building engine" messages on startup if confused.

## Inference outside DeepStream (raw TensorRT)

For the rangefinder USB camera pair (stereo) or driver-monitoring camera, you may not want DeepStream. Plain TensorRT in Python:

```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

logger = trt.Logger(trt.Logger.WARNING)
runtime = trt.Runtime(logger)

with open("/models/sign_classifier.engine", "rb") as f:
    engine = runtime.deserialize_cuda_engine(f.read())
context = engine.create_execution_context()

# Allocate buffers based on engine bindings (omitted for brevity)
# ...
context.execute_v2(bindings=bindings)
```

For Python the easier wrapper is `cuda-python` (newer, supported) or `polygraphy` (NVIDIA's debugging tool that also runs inference). Avoid the older deprecated `pycuda` for new code.

## Model size budgeting on Orin Nano

| Model | Engine size (FP16) | Inference time (ms) on Orin Nano MAXN_SUPER |
|---|---|---|
| YOLOv8n @ 640 | ~6 MB | ~12 ms |
| YOLOv8s @ 640 | ~22 MB | ~25 ms |
| YOLOv8m @ 640 | ~50 MB | ~55 ms |
| YOLOv8s @ 1280 | ~22 MB | ~85 ms |
| ResNet50 classifier @ 224 | ~50 MB | ~8 ms |

(These are rough — benchmark on the device. NVIDIA hasn't published official numbers for every combination.)

For 30 fps realtime: budget per frame is 33 ms. After overhead (decode, preprocess, postprocess, DeepStream plumbing) you have maybe 20-25 ms for the detector. **YOLOv8s @ 640 fits, YOLOv8m doesn't, YOLOv8n leaves headroom for a secondary classifier.**
