# Orin Nano Constraints

Orin Nano 8GB has a specific shape that diverges from the rest of the Orin family. Misjudging this is the #1 source of bad Jetson advice.

## NVENC: not present

The Orin Nano (4GB and 8GB) does **not** contain an NVENC hardware video encoder. Confirmed in NVIDIA's "Software Encode in Orin Nano" application note. Every encoded frame is produced on the CPU.

Quick hardware-level check (confirmed on this device): `ls /dev | grep -iE 'nvdec|nvenc'` shows **`v4l2-nvdec`** (decode present) and **no nvenc/msenc node** — the module physically lacks the encoder. If a build ever tries to open an NVENC device here, that's the bug.

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

Single-source 1456×1088**@30** (roughly 1080p-class) software encode with `x264enc ultrafast` consumes roughly 2–3 of the 6 CPU cores under MAXN_SUPER. At **@60** it's roughly 2× the work — borderline sustainable on Orin Nano alongside inference and IO. There is no NVENC to offload to.

**Recommendation for the dashcam's IMX296 1456×1088@60 stream** (native res — the IMX296 has no 1080p mode; see `cameras.md`): capture at 60 fps for the inference branches (lane / sign detection benefit from temporal density), but `videorate` down to 30 fps **before** the encoder. The 60 → 30 drop happens after a `tee`, so other branches keep the full 60 fps. See `deepstream-pipelines.md` for the tee+videorate pattern.

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

Ampere architecture, 1024 CUDA cores split across 8 SMs, 32 tensor cores. The **original** Orin Nano capped the GPU at ~635 MHz for ~40 sparse-INT8 TOPS; **JetPack 6.2 Super Mode (MAXN_SUPER) raises the GPU to ~1020 MHz for ~67 sparse-INT8 TOPS** (plus CPU 1.5→1.7 GHz and ~50% more memory bandwidth). This project runs MAXN_SUPER, so budget against the Super figures, not the original ones. Plenty for one or two simultaneous TensorRT models at moderate input size (e.g., YOLOv8s @ 640×640 + a small classifier). Will struggle with large transformer-based vision models in realtime.

## RAM and swap

8 GB unified DRAM, shared CPU + GPU.

- TensorRT engine builds for medium models can momentarily spike to 4–6 GB. **Confirmed: 16 GB swap on a dedicated NVMe** (`/dev/nvme1n1p1`), physically separate from the ~1 TB system NVMe (`/dev/nvme0n1p1`, rootfs at `/`) — so swap I/O never contends with rootfs/footage I/O. A good build-time safety net. *Accuracy note:* `swapon` shows this swap at priority 5, but with a **single** swap area, priority only matters for ordering *between* swap devices — **when** swap engages is governed by `vm.swappiness` (a low value = don't page until RAM is nearly full), not the priority. Confirm the intended behavior with `cat /proc/sys/vm/swappiness`.
- At runtime, avoid swapping. With low swappiness, **any** non-zero swap-in during steady state means RAM is genuinely near-full — treat it as a hard signal to shrink the model/batch/input size, not opportunistic paging. Watch the `tegrastats` `RAM` field; past ~7 GB used, performance falls off a cliff.
- Consider reducing the TensorRT workspace pool: `--memPoolSize=workspace:2048` to `trtexec` keeps build memory in check. (In TRT 10 the old `--workspace=N` is deprecated/removed — see `models-and-tensorrt.md`.)

## Power modes (`nvpmodel`)

JetPack 6.2 introduced a **25 W mode and an uncapped MAXN_SUPER** for Orin Nano 8GB, on top of the original 15 W mode. **Confirmed on this device** (`nvpmodel -p --verbose`, JP 6.2.2):

| ID | Name | CPU max (6× A78AE) | GPU max | EMC max | Note |
|---|---|---|---|---|---|
| 0 | 15W | **1.4976 GHz** | 612 MHz | 2133 MHz | |
| 1 | 25W | **1.344 GHz** | 918 MHz | 3199 MHz | **config default** |
| 2 | MAXN_SUPER | uncapped (~1.7 GHz) | uncapped (~1020 MHz) | uncapped | **active on this rig** |

**Non-obvious quirk (verified in the config):** the 25 W mode caps the CPU *lower* than the 15 W mode (1.344 vs 1.4976 GHz) — it trades CPU for GPU+EMC. On a box whose encode path is **CPU-bound x264**, switching 15 W → 25 W can *reduce* encode throughput while helping inference. If you must run capped and encode matters, benchmark both; if you can, run MAXN_SUPER (`-m 2`), which uncaps everything. Check available modes:

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

Orin Nano dev kit has a small fan and modest passive heatsink. Under sustained MAXN_SUPER + active inference + software encoding, SoC temps climb. **Confirmed trip points on this device** (`trip_point_0` per zone): **99 °C** on the `cpu`/`gpu`/`cv0-2`/`soc0-2` zones (the throttle trips; Tj max ~100 °C) and **35 °C** on `tj-thermal` (a low active-cooling/fan trip). So hardware throttling is configured near **~99 °C** — but treat the 90s °C as "already thermally limited," not headroom.

For automotive deployment (sealed enclosure, no airflow), thermal is the dominant constraint. Recommendations:
- Active cooling is mandatory — passive in a hot car cabin will throttle.
- Monitor `tegrastats` field `CPU@` and `GPU@` for tj temps.
- If throttling is observed, drop to a lower-power mode (e.g. the 15 W mode) or downscale the workload before adding thermal mass. Don't assume a fixed index — the Super devkit `nvpmodel.conf` renumbers modes; list them with `sudo nvpmodel -p --verbose` and confirm with `-q`.

## When to push the user toward different hardware

If their plan needs **any** of:
- Continuous H.264 record of 3+ 1080p30 streams
- DLA offload
- More than ~2 simultaneous large TensorRT models
- Sustained operation at >40 °C ambient without active cooling

…recommend Orin NX 16 GB — drop-in module replacement on most carriers, has NVENC + 1 DLA + double the RAM. Be explicit about why; don't just say "you need more power."
