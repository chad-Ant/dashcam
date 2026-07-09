# Containers on Jetson (jetson-containers)

The user runs inside the `l4t-ml` container from [`dusty-nv/jetson-containers`](https://github.com/dusty-nv/jetson-containers). This is the de-facto standard for ML on Jetson and saves a huge amount of dependency pain.

## What's where

| Image | Source | Contains |
|---|---|---|
| `l4t-ml` | jetson-containers (community) | PyTorch, TensorFlow, ONNX Runtime, OpenCV (with CUDA), NumPy, JupyterLab. **Does NOT include DeepStream.** |
| `l4t-jetpack` | NVIDIA NGC (`nvcr.io/nvidia/l4t-jetpack`) | Full JetPack: CUDA, cuDNN, TensorRT, VPI, Multimedia API. No PyTorch. |
| `deepstream` (NGC) | `nvcr.io/nvidia/deepstream:7.1-samples-multiarch` | DeepStream 7.1 + samples/configs. **No Triton.** Multiarch (works on Jetson; Jetson-specific alias is `deepstream-l4t:7.1-samples-multiarch`). For Triton use `7.1-triton-multiarch` (Triton 24.08, larger). |
| `deepstream` (jetson-containers) | jetson-containers | DeepStream built from source on top of `l4t-jetpack`. Has known build issues on JP 6.2 (PYDS_VERSION). |

For this dashcam project: ML work + camera testing → `l4t-ml`. DeepStream pipelines → either NGC `deepstream:7.1-samples-multiarch` (recommended for getting started) or build a custom multi-package image with both PyTorch and DeepStream via jetson-containers.

## NVIDIA Container Runtime: prerequisite

Already installed by JetPack 6 by default. Confirm:
```bash
docker info | grep -i runtime
# Should list: nvidia
```

If missing:
```bash
sudo apt install -y nvidia-container nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## Running with cameras: the canonical command

```bash
jetson-containers run \
  --device /dev/video0 \
  --device /dev/video1 \
  --device /dev/video2 \
  --device /dev/video3 \
  --device /dev/video4 \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /etc/enctune.conf:/etc/enctune.conf \
  --volume /data:/data \
  $(autotag l4t-ml)
```

Equivalent raw `docker run` (without the `jetson-containers` wrapper):
```bash
docker run --rm -it --runtime=nvidia --network=host \
  --device /dev/video0 --device /dev/video1 \
  --device /dev/video2 --device /dev/video3 --device /dev/video4 \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /etc/enctune.conf:/etc/enctune.conf \
  -v /data:/data \
  dustynv/l4t-ml:r36.4.0   # intended L4T 36.4.0 runtime base; host 36.5.0 Tegra libs mount over it (see note below)
```

### Why each flag matters

- `--runtime=nvidia` — gives the container access to the GPU and Tegra-specific device nodes. Without this, `nvidia-smi` (well, on Jetson there isn't one) won't work, and CUDA APIs fail.
- `--device /dev/videoN` — explicitly grants the container access to each camera. `--privileged` would also work but is overkill.
- `-v /tmp/argus_socket:/tmp/argus_socket` — **the secret sauce for CSI cameras inside containers**. The Argus daemon runs on the host; in-container `nvarguscamerasrc` connects to it via this Unix socket. Without this bind-mount, `nvarguscamerasrc` instantiates but capture silently fails or produces no buffers.
- `-v /etc/enctune.conf:/etc/enctune.conf` — encoder tuning; harmless to include, helps if you do attempt encoding.
- `--network=host` — lets X11 / display, GPS UDP feeds, NTP etc. work without port-mapping headaches. Optional but common.
- `-v /data:/data` — host-mounted persistent storage. Critical for dashcam recordings — don't write into the container's union FS or you lose data on `docker rm`.

## "no element nvarguscamerasrc" inside the container — what to check

In order:

1. **Are you using an l4t-* image** (not generic Ubuntu)? Generic ARM Ubuntu images don't have NVIDIA's GStreamer plugins.
2. **Is `--runtime=nvidia` set**? Without it, the GStreamer NV elements are unloadable.
3. **Verify inside the container:**
   ```bash
   gst-inspect-1.0 nvarguscamerasrc      # should show element details
   gst-inspect-1.0 | grep -i nv          # should list a bunch of nv* plugins
   ```
4. **Image tag vs host L4T (intended skew here).** This project **deliberately runs in the L4T 36.4.0 container** on a 36.5.0 host. `autotag l4t-ml` warns `Unknown L4T_VERSION: 36.5.0`, maps the stack correctly (`JETPACK_VERSION=6.2 CUDA_VERSION=12.6`), and resolves to **`dustynv/l4t-ml:r36.4.0`** — which is exactly the base you want. That's not a fallback to work around; it's the runtime base. The NVIDIA container runtime mounts the host's 36.5.0 Tegra userspace (Argus/`nvvidconv`/`NvBufSurface` plugins, driver, multimedia libs via CSV) over the 36.4.0 image, so Tegra bits are the host's and CUDA/TRT/cuDNN are the image's — all JP-6.2.x-compatible. So `$(autotag l4t-ml)` and a hardcoded `dustynv/l4t-ml:r36.4.0` are the same image here. **Don't** hand-edit the tag to `r36.5.x` (doesn't exist) or force a host-matching tag — the 36.4.0 base is correct. Build the app, custom GStreamer plugins, and TensorRT engines *inside this container* so they link against its userspace and this Orin Nano's GPU. If you ever hit a driver/mount mismatch, `cd ~/jetson-containers && git pull` or build locally with `jetson-containers build`.

## DeepStream container quickstart (NGC)

```bash
docker pull nvcr.io/nvidia/deepstream:7.1-samples-multiarch

docker run --rm -it --runtime=nvidia --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --device /dev/video0 --device /dev/video1 \
  --device /dev/video2 --device /dev/video3 --device /dev/video4 \
  -v /data:/data \
  nvcr.io/nvidia/deepstream:7.1-samples-multiarch
```

Inside, run a sample to verify:
```bash
cd /opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app
deepstream-app -c source1_csi_dec_infer_resnet_int8.txt
```

## Building a combined image (DeepStream + PyTorch + your code)

For production, you usually want **one** image with everything. Approach:

1. Start from `nvcr.io/nvidia/deepstream:7.1-samples-multiarch` (you can't easily add DeepStream to `l4t-ml` later — DeepStream's apt repo and lib paths matter).
2. Layer PyTorch via Jetson wheels:
   ```dockerfile
   FROM nvcr.io/nvidia/deepstream:7.1-samples-multiarch
   RUN pip install --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
       torch torchvision
   COPY . /app
   WORKDIR /app
   ```
3. Build with `jetson-containers` for cache reuse:
   ```bash
   jetson-containers build --base=deepstream --packages=pytorch,opencv,gstreamer my-dashcam
   ```

## Known JetPack 6.2 build issues

### `jetson-containers build deepstream` PYDS build errors on JP 6.2

For DeepStream 7.1 the Python bindings are **`pyds` 1.2.0+**, which moved to a PyPA-compliant build system to support pip 24.2+ (pip deprecated the old `setup.py` command line). Build recipes or steps that still assume `python3 setup.py install` can fail on JP 6.2 as a result.

Fixes, in order:
- Build/install the bindings the current way — follow the **`bindings/README`** in `NVIDIA-AI-IOT/deepstream_python_apps` (it uses the updated PyPA flow), or install the prebuilt `pyds` wheel from that repo's releases.
- If an older `jetson-containers` recipe still shells out to `setup.py`, pin a compatible build toolchain (e.g. `pip3 install "setuptools<69"`) before building, then rerun `jetson-containers build --name=my-deepstream deepstream`.
- Check the current [jetson-containers issues](https://github.com/dusty-nv/jetson-containers/issues) for the live state of the DeepStream recipe on JP 6.2 before sinking time into a source build.

If still failing, fall back to NGC's prebuilt `deepstream:7.1-samples-multiarch` (or `deepstream-l4t:7.1-samples-multiarch`) instead of building. **Note:** from DeepStream 9.0 the Python bindings are deprecated (NVIDIA points to PyServiceMaker) — not your concern on 7.1, but relevant if you ever move up.

### `tritonserver` build fails

DeepStream depends on Triton; Triton's JP 6 build path has changed. Either pin to the version of jetson-containers that worked for your JP, or use NGC.

## Storage strategy for dashcam recordings

Don't write to the container overlay. Mount host paths — point `/data` at the **footage directory on the SD card** (`mmcblk0`, mounted at `/media/jetson/backup`; the exact footage subdirectory is TBD), e.g. `-v /media/jetson/backup/dashcam:/data`. The card is removable (evidence retrieval) and isolated from the system rootfs on NVMe. Use a high-endurance card + power-loss-safe segmenting (see `event-recording.md`):
- `/data/recordings/<date>/<time>.mp4` — primary recordings
- `/data/events/<event-id>/{video.mp4, gps.json, imu.csv}` — event clips with metadata

Plan for log rotation: at 4 Mbps continuous record that's ~1.8 GB/hr per stream, and the ~167 GB SD card fills in ~11 days at 8 h/day (scale by stream count) — so aging matters *more* here than on the roomier NVMe. Implement an aging job that deletes oldest non-event clips when the SD card's free space drops below ~10%, so a full card never blocks recording (and never touches the OS on the NVMe).

## Container vs host: when to drop out

Some things don't work cleanly inside containers and are less painful on the host:
- **GPS receivers via gpsd** — gpsd inside a container is doable but X11/serial passthrough is finicky. Run gpsd on host, expose via TCP (`gpsd -G`), connect from container.
- **CAN bus** for OBD-II — needs `--cap-add=NET_ADMIN --network=host` and `socketcan` kernel module loaded on host.
- **udev rules** for stable device naming — these belong on the host.

A reasonable split: GPS, CAN, udev, systemd unit for autostart, log rotation → host. Application code, ML models, GStreamer pipelines → container.
