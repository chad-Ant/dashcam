# Debugging Jetson Dashcam Issues

When something is broken, work through this triage list in order. Each step takes seconds and rules out a major class of problem before drilling into specifics.

## Step 0: Confirm the basics

```bash
cat /etc/nv_tegra_release          # L4T version → JetPack version
sudo nvpmodel -q                    # current power mode (look for MAXN_SUPER for dashcam workloads)
free -h                              # RAM and swap usage
df -h /data /                        # disk space — full disk silently breaks recording
```

If swap is being used heavily during inference, that alone explains most "it's slow" problems.

## Step 1: `tegrastats` / `jtop`

`tegrastats` prints a one-line periodic snapshot:
```bash
tegrastats --interval 1000
# Example output:
# RAM 4521/7757MB SWAP 0/16384MB CPU [12%@1728,8%@1728,...] GR3D_FREQ 80%@624 CPU@52.5C SOC@51C
```

What to look for:
- `RAM` near max → memory pressure; check for leaks, reduce batch size, reduce model.
- `SWAP` non-zero during steady-state → swapping; will cause frame stutter.
- `CPU` cores at 100% → likely software encoding (no NVENC) saturating.
- `GR3D_FREQ` (GPU) at 99% → inference is the bottleneck; smaller model or reduced input size.
- `CPU@`/`SOC@` near 87 °C → thermal throttling imminent, fan / cooling problem.

`jtop` is a TUI on top of `tegrastats` — `sudo pip install -U jetson-stats`, then `jtop`. Easier to read.

## Step 2: kernel log

```bash
dmesg | tail -50
sudo dmesg -w           # tail -f equivalent
```

Camera issues, USB resets, OOM kills, voltage warnings show up here. Critical messages to grep for:
- `imx219` / `imx477` / your sensor model — driver loading / failing
- `tegra-vi` / `tegra-camrtc` — camera subsystem
- `nvgpu` — GPU driver
- `oom-killer` — process killed for memory
- `usb 1-N: device descriptor read/64, error -110` — USB camera disconnect / bad cable

## Step 3: Argus daemon

CSI camera weirdness almost always starts here.
```bash
sudo systemctl status nvargus-daemon
sudo journalctl -u nvargus-daemon -n 100
sudo systemctl restart nvargus-daemon
```

If a previous client crashed mid-capture, Argus can be wedged until restarted. Symptoms: `nvarguscamerasrc` opens but produces no buffers, or "no cameras available."

## Step 4: GStreamer pipeline negotiation

For pipeline construction failures, run with debug:
```bash
GST_DEBUG=3 gst-launch-1.0 ...                  # warnings + errors
GST_DEBUG=4 gst-launch-1.0 ...                  # + info
GST_DEBUG=*:3,nvarguscamerasrc:5,nvinfer:5 gst-launch-1.0 ...
```

Look for "could not link X to Y" or "no compatible caps" messages — those identify exactly which two elements failed to negotiate. The fix is usually a `capsfilter` or `nvvidconv` between them.

## Step 5: Library / linkage problems

```bash
ldd /path/to/your/binary | grep -i 'not found'
ldd /path/to/your/binary | grep -E 'nvinfer|cuda|cudnn'
```

JetPack 6.2 ships these SONAMEs:
- `libnvinfer.so.10`, `libnvonnxparser.so.10`, `libnvinfer_plugin.so.10` (TensorRT 10.3)
- `libcudart.so.12` (CUDA 12.6)
- `libcudnn.so.9` (cuDNN 9.x)

If `ldd` reports any of these "not found", the typical causes — in order of likelihood:

1. **Wrong Python wheel source.** Plain `pip install torch` pulls x86 wheels that don't run on aarch64 at all, or aarch64 wheels for the wrong CUDA version. Use the Jetson AI Lab index for JP 6.2 / CUDA 12.6:
   ```bash
   pip install --extra-index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 \
       torch torchvision onnxruntime-gpu
   ```
2. **`LD_LIBRARY_PATH` doesn't include Tegra paths.** Should include `/usr/lib/aarch64-linux-gnu/tegra` and `/usr/local/cuda/lib64`. Most l4t images set this up automatically.
3. **Stale venv** — venvs created without `--system-site-packages` won't see the system TensorRT Python bindings (which ship in the JetPack apt packages, not via pip). Either use `--system-site-packages` or recreate the venv.
4. **Custom C++ binaries** — if you compiled `.so` plugins or app binaries yourself, make sure they're linked against TRT 10 headers at `/usr/include/aarch64-linux-gnu/NvInfer*.h`.

## Step 6: Container-specific issues

If it works on the host but not in the container:

| Symptom | Cause | Fix |
|---|---|---|
| `nvarguscamerasrc` not found | Wrong base image or `--runtime=nvidia` missing | Use `dustynv/l4t-ml:r36.4.0` or NGC `l4t-jetpack`; add `--runtime=nvidia` |
| `nvarguscamerasrc` exists, no buffers | `argus_socket` not bind-mounted | Add `-v /tmp/argus_socket:/tmp/argus_socket` |
| USB camera not visible | Device not passed in | `--device /dev/video2` (etc.) |
| GPU not accessible | Runtime not nvidia or NVIDIA_VISIBLE_DEVICES | Add `--runtime=nvidia` and `-e NVIDIA_VISIBLE_DEVICES=all` |
| `libnvinfer.so.10: cannot open` (host has .10) | Library path not exposed | Most l4t images handle this; if custom image, ensure `/usr/lib/aarch64-linux-gnu/tegra` is in `LD_LIBRARY_PATH` |

## Step 7: Pipeline-level performance debugging

If the pipeline runs but is slow / drops frames:

1. Add a `fpsdisplaysink` or `identity` with `silent=false` after the inference stage to measure actual fps.
2. Run with `GST_DEBUG="GST_PERFORMANCE:5"` to see scheduling delays.
3. Check `nvinfer` output: it logs the per-frame inference time at startup if you set `interval=0`.
4. `trtexec --loadEngine=...` benchmarks the model in isolation — gives you a ceiling.
5. If inference time × source count > frame interval, you're inference-bound. Reduce model, lower input resolution, or skip frames (`nvinfer interval=1` skips every other frame).

## Common error → cause table

| Error | Likely cause |
|---|---|
| `WARNING: erroneous pipeline: no element "nvarguscamerasrc"` | Container missing nv plugins, or running on non-l4t image |
| `Could not initialize Argus camera provider` | nvargus-daemon crashed; restart it |
| `libnvinfer.so.10: cannot open shared object file` | Tegra library path not exposed, or wrong-arch / wrong-CUDA Python wheel — verify `LD_LIBRARY_PATH` includes `/usr/lib/aarch64-linux-gnu/tegra`; reinstall Python deps from `https://pypi.jetson-ai-lab.dev/jp6/cu126` |
| `libnvds_amqp_proto.so: cannot open` | DeepStream protocol adapter missing; usually harmless if you don't use Kafka/AMQP — set `disable-prebuilt-init=1` or remove the broker config |
| `NvDsInfer Error: NVDSINFER_CUDA_ERROR` during engine build | Often OOM. Verify swap is on, drop `--workspace` to 1024, ensure no other heavy processes |
| `nvbuf_utils: Failed to create NvBufSurface` | Out of NVMM buffer pool; usually means the pipeline isn't releasing buffers — check for accidentally held refs in your pad probes |
| `Failed to query video capabilities: Inappropriate ioctl for device` | Wrong device node (e.g., trying v4l2-ctl on a `/dev/video*` that's a CSI sub-device, not a capture node) |
| `gst-stream-error-quark: Internal data stream error` | Generic GStreamer error; rerun with `GST_DEBUG=3` to find the real culprit |
| Pipeline runs once then hangs on next start | Argus daemon needs restart after unclean shutdown |
| TensorRT engine build hangs at 100% RAM | Building too large a workspace; reduce `--workspace=2048`, ensure swap is on |

## Useful one-liners

```bash
# All cameras and their formats:
for d in /dev/video*; do echo "=== $d ==="; v4l2-ctl -d $d --list-formats-ext 2>/dev/null; done

# Live FPS of a running pipeline (find your gst process):
gst-stats-1.0 -n 100 -p   # if your pipeline prints stats

# Show the running power mode and clocks:
sudo nvpmodel -q && sudo jetson_clocks --show

# Inspect a serialized engine's expected inputs/outputs:
/usr/src/tensorrt/bin/trtexec --loadEngine=model.engine --verbose 2>&1 | grep -E "Input|Output"

# Check container can see the GPU:
docker run --rm --runtime=nvidia dustynv/l4t-ml:r36.4.0 \
  bash -c "nvcc --version && python3 -c 'import torch; print(torch.cuda.is_available())'"
```

## When to ask the user for `dmesg` / `journalctl` output

If after going through Steps 0–4 the cause isn't obvious, ask the user to paste:
- `cat /etc/nv_tegra_release`
- `sudo nvpmodel -q`
- Output of the failing command with `GST_DEBUG=3` or `--verbose`
- Last 50 lines of `dmesg`
- Last 50 lines of `sudo journalctl -u nvargus-daemon`

These five together solve maybe 80% of Jetson capture / inference bugs.
