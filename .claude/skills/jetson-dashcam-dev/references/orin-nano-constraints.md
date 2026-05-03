# Orin Nano Constraints

Orin Nano 8GB has a specific shape that diverges from the rest of the Orin family. Misjudging this is the #1 source of bad Jetson advice.

## NVENC: not present

The Orin Nano (4GB and 8GB) does **not** contain an NVENC hardware video encoder. Confirmed in NVIDIA's "Software Encode in Orin Nano" application note. Every encoded frame is produced on the CPU.

What this rules out on Orin Nano:
- `nvv4l2h264enc`, `nvv4l2h265enc` — these GStreamer elements either won't be present or will fail to negotiate.
- `omxh264enc` — deprecated everywhere on JP6 anyway.
- `ffmpeg -c:v h264_nvenc` — no NVENC means no `h264_nvenc`.

What works:
- `x264enc` (libx264) — the standard recommendation. Use `tune=zerolatency speed-preset=ultrafast` for realtime; `speed-preset=fast` if you have CPU headroom and care about size.
- `x265enc` (libx265) — better compression ratio but ~2-3× CPU. Usually not worth it on Orin Nano for realtime.
- `ffmpeg -c:v libx264 -preset ultrafast -tune zerolatency` — equivalent in ffmpeg.
- `jpegenc` (software JPEG) — for low-rate timelapse-style recording or per-frame snapshots.

### Realistic encode budgets

Single-source 1080p**30** software encode with `x264enc ultrafast` consumes roughly 2–3 of the 6 CPU cores under MAXN_SUPER. At 1080p**60** it's roughly 2× the work — borderline sustainable on Orin Nano alongside inference and IO.

**Recommendation for the dashcam's IMX296 1080p60 stream**: capture at 60 fps for the inference branches (lane / sign detection benefit from temporal density), but `videorate` down to 30 fps **before** the encoder. The 60 → 30 drop happens after a `tee`, so other branches keep the full 60 fps. See `deepstream-pipelines.md` for the tee+videorate pattern.

For multi-camera recording, prefer one of:
1. **Smart recording** — only encode in response to events (see `event-recording.md`).
2. **Lower res / lower fps** for non-primary cameras. 720p15 software encode is much cheaper than 1080p30.
3. **Round-robin** — encode each camera briefly and rotate. Useful for evidence cams; bad for primary safety cam.

Always benchmark with `tegrastats --interval 1000` running in another shell while the pipeline executes.

## NVDEC: present and useful

Orin Nano has NVDEC. Hardware-accelerated decode of H.264/H.265 is available via `nvv4l2decoder`. Useful for:
- Replaying recorded clips for analysis
- Streaming previously-encoded files into a DeepStream pipeline (`uridecodebin` will pick `nvv4l2decoder` automatically)
- Testing inference pipelines without live cameras

## DLA: not present

Orin Nano has 0 DLA cores. DLA-related flags fail silently or fall back to GPU. Compare:

| Module | DLA cores |
|---|---|
| Orin Nano (4GB/8GB) | 0 |
| Orin NX (8GB/16GB) | 1 |

Don't suggest `--useDLACore=N`, `precisionMode=int8 --useDLACore` in `trtexec`, or DeepStream `nvinfer` configs with `enable-dla=1` on Orin Nano.

## GPU

Ampere architecture, 1024 CUDA cores split across 8 SMs, 32 tensor cores, ~625 MHz max. About 40 INT8 TOPS theoretical. Plenty for one or two simultaneous TensorRT models at moderate input size (e.g., YOLOv8s @ 640×640 + a small classifier). Will struggle with large transformer-based vision models in realtime.

## RAM and swap

8 GB unified DRAM, shared CPU + GPU.

- TensorRT engine builds for medium models can momentarily spike to 4–6 GB. Set up swap on NVMe before building large engines. (User has 16 GB swap configured — good.)
- At runtime, avoid swapping. Watch `tegrastats` `RAM` field — if you go past ~7 GB used and swap starts taking hits, performance falls off a cliff.
- Consider reducing TensorRT workspace: `--workspace=2048` to `trtexec` keeps build memory in check.

## Power modes (`nvpmodel`)

JetPack 6 introduced **MAXN_SUPER** (~15 W boosted) for Orin Nano 8GB, in addition to the original 7 W and 15 W modes. Check available modes:

```bash
sudo nvpmodel -p --verbose
```

For dashcam workloads, run on MAXN_SUPER:
```bash
sudo nvpmodel -m 2      # mode index for MAXN_SUPER on JP 6.x; verify with -q
sudo jetson_clocks      # lock clocks to max for sustained performance (no DVFS)
```

`jetson_clocks` does not persist across reboots. Add to `/etc/rc.local` or a systemd unit if you want it always on.

## Thermal

Orin Nano dev kit has a small fan and modest passive heatsink. Under sustained MAXN_SUPER + active inference + software encoding, expect SoC temps to climb toward 80 °C. Throttling kicks in around 87 °C.

For automotive deployment (sealed enclosure, no airflow), thermal is the dominant constraint. Recommendations:
- Active cooling is mandatory — passive in a hot car cabin will throttle.
- Monitor `tegrastats` field `CPU@` and `GPU@` for tj temps.
- If throttling is observed, drop to standard 15 W mode (`nvpmodel -m 1`) or downscale workload before increasing thermal mass.

## When to push the user toward different hardware

If their plan needs **any** of:
- Continuous H.264 record of 3+ 1080p30 streams
- DLA offload
- More than ~2 simultaneous large TensorRT models
- Sustained operation at >40 °C ambient without active cooling

…recommend Orin NX 16 GB — drop-in module replacement on most carriers, has NVENC + 1 DLA + double the RAM. Be explicit about why; don't just say "you need more power."
