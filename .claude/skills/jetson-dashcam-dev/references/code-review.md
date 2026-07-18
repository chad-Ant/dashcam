# Code Review — Jetson headless multi-camera / dashcam

Use this when the user asks you to review, audit, "find bugs in", or sanity-check code (C/C++, Python/pyds, or GStreamer pipelines) for this project. Produce **severity-ranked findings** plus a short prioritized improvement list. The bar is safety-critical embedded discipline (NASA/JPL *Power of Ten* spirit) adapted to Jetson realities.

## Output format

For each finding:
```
[SEVERITY] <file:line or code region> — <what's wrong>
  Why it bites here: <the Orin-Nano / headless / camera-specific reason>
  Fix: <concrete change, with the corrected line/snippet>
```
Then: **Top improvements** — a numbered list, highest-leverage first.

Group findings by severity (BLOCKER first). If the code is fine, say so plainly and still list any improvements. Don't invent problems to fill space; don't soften a BLOCKER into a suggestion.

## Severity definitions

- **BLOCKER** — will not run on this hardware, crashes, or loses data. Ship-stopper.
- **HIGH** — real defect that will bite under load or over time (race, leak, wrong-target artifact) even if it "works on my desk once".
- **MEDIUM** — performance / thermal / robustness footgun; correct but fragile or wasteful on a constrained box.
- **LOW / NIT** — style, naming, missing asserts, magic numbers, docs.

---

## Jetson-specific checklist (the ones LLM-written code gets wrong)

### Hardware-fit (usually BLOCKER)
- **Hardware video encoder on Orin Nano** — `nvv4l2h264enc` / `nvv4l2h265enc` / `nvv4l2av1enc` / `omxh264enc` / `ffmpeg -c:v h264_nvenc`. No NVENC exists. → `x264enc` / `x265enc` / libx264 C++ (`multimedia-api.md`).
- **DLA usage** — `--useDLACore=N`, `enable-dla=1`, `precisionMode …--useDLACore`. Orin Nano has 0 DLA. → GPU only.
- **Display sink in a headless service** — `nv3dsink` / `nveglglessink` / `nvdrmvideosink` / `nvoverlaysink` / `xvimagesink` / `NvEglRenderer`, or `export DISPLAY=:0`. → `appsink` / `filesink` / `splitmuxsink` / `udpsink` / `fakesink` + probe.
- **Container: missing `--volume /tmp/argus_socket:/tmp/argus_socket`** with CSI capture, or missing `--device /dev/videoN` / `--runtime=nvidia`. → CSI works bare-metal, fails in-container. (`containers.md`)
- **TensorRT engine built off-target** — `.engine` from x86 or a different Jetson module shipped as-is. Engines aren't portable across compute capability / module / TRT version. → build on the target, or auto-build from ONNX on first launch.

### Buffers & memory (BLOCKER/HIGH)
- **`NvBufSurface` / DMABUF fd leak** — acquired surfaces not released on every path (incl. errors). Finite pool → `Failed to create NvBufSurface`, stall. → release on all exits; pre-allocate pools. (`multimedia-api.md` §4)
- **Block-linear NVMM handed to CPU / libx264 without `NvBufSurfaceCopy`** — garbled/striped output. → copy (or `NvBufSurfTransform` for format).
- **Missing `(memory:NVMM)` caps** between `nvargus*` / `nvvidconv` / `nvinfer` when buffers must stay on GPU — forces silent CPU copies or negotiation failure.
- **appsink readout copying frame bytes without honoring stride** — `gst_buffer_extract` / raw `memcpy` of `gst_buffer_get_size()` bytes assumes tightly-packed `w×h×C`. `nvvidconv`/`videoconvert` output can be row-padded (stride ≠ `width×bytes`), so any width that isn't stride-aligned comes out sheared. Some widths (e.g. IMX296's 1456) are safe by luck; odd USB modes are not. → map with `gst_video_frame_map` and copy per-row using `GST_VIDEO_FRAME_PLANE_STRIDE`, or at minimum assert `size == w×h×C` and fail loudly.
- **Unified-RAM blindness** — large batch/model + TRT build during runtime; swapping mid-inference falls off a cliff (`tegrastats` RAM). → size models to the 8 GB budget.

### Error handling (HIGH — Power of Ten rule: check every return)
- **Unchecked returns**: Argus `Status` / `Argus::Status`; V4L2 ioctls (`-1` + `errno`); CUDA (`cudaError_t` / `cudaGetLastError`); libav (`avcodec_*` `< 0`; `avcodec_receive_packet` `EAGAIN`/`EOF` are normal control flow, not errors); `NvBufSurface*` returns (nullptr); GStreamer `gst_element_link*` / state-change returns; `Gst.Element.make(...)` returning `None` in pyds. → check and handle every one.
- **No pipeline bus handler** (pyds/GLib) — errors/EOS silently swallowed; no recovery, no clean shutdown.

### Concurrency (HIGH — this app is inherently multi-threaded)
- **Shared state without a guard** — GPS/speed snapshot, IMU/event flags, ring-buffer indices read by a probe/consumer and written by another thread with no mutex/atomic → torn reads, missed events. → mutex or lock-free SPSC queue; publish immutable snapshots.
- **Blocking inside a pad probe or Argus consumer callback** — file IO, network, or heavy compute on the streaming thread → backpressure stalls every branch. → copy out, hand to a worker, return fast.
- **No `queue` between `tee` branches** — a slow branch (the software encoder) backpressures the others onto one thread. → `queue` at the head of each branch (MEDIUM if perf-only, HIGH if it can deadlock).
- **Lifecycle methods mutating shared handles/maps outside the lock that other methods guard** — e.g. `start()` publishes element pointers, or `teardown()` clears a valve/pad map, *without* the mutex, while `captureFrame` / `setBranchEnabled` / attribute writes guard their access *with* it. A guarded reader/mutator does **not** synchronize-with an unguarded writer, so you get a data race (torn/stale handle on weakly-ordered ARM) or a use-after-free (mutating a valve teardown just freed). The **optimistic-`RUNNING`-claim** pattern is the usual culprit: status flips to RUNNING under the lock, then the handles publish *after* it without the lock — the read-side guard alone can't save you. → assign/clear shared pipeline handles under the same mutex so status and pointers publish together, and gate runtime-control methods (branch enable/disable) on `status == RUNNING` *inside* the lock.

### Correctness footguns (MEDIUM/LOW)
- **`trtexec --workspace=N`** — deprecated/removed in TRT 10. → `--memPoolSize=workspace:N`.
- **Claiming IMX296 does 1920×1080** — it's 1456×1088 native; 1080p is upscaled. → correct caps / expectations. (`cameras.md`)
- **Encoding 60 fps when 30 suffices** for recorded evidence — ~2× CPU on a box with no NVENC. → `videorate` to 30 before the encoder.
- **Hardcoded `/dev/videoN`** for USB cams — numbering isn't stable across reboots. → udev symlink by serial/path. (`cameras.md`)
- **Stock apt OpenCV** (`python3-opencv`) — no CUDA / limited GStreamer. → jetson-containers `opencv` or build with `WITH_CUDA=ON WITH_GSTREAMER=ON`.

---

## Power of Ten — applied to this codebase

Holzmann's 10 rules, framed for Jetson multi-camera firmware/apps:
1. **Simple control flow** — no `goto`/recursion; bounded, obvious flow in capture/encode loops.
2. **Bound every loop** — capture, drain, retry, and reconnect loops need a fixed upper bound (or a provable termination), not `while(true)` with no exit.
3. **No dynamic allocation on the hot path** — pre-allocate buffer pools, `AVFrame`/`AVPacket`, and inference IO once; per-frame `malloc`/`new` invites fragmentation and jitter.
4. **Short functions** — one job each; a 300-line capture-encode-record-IMU function is a review finding.
5. **Assert liberally** — validate frame dims/format, buffer non-null, index in range, sensor-id valid; asserts are cheap vs a wedged dashcam.
6. **Smallest scope** — don't hoist camera handles / GPS state to globals when a member or local will do.
7. **Check every return / validate every parameter** — see the error-handling section; this is the rule most violated in generated Jetson code.
8. **Limit preprocessor / macro cleverness** — keep pinmux/device-tree and build macros legible.
9. **Restrict pointer use** — one level of dereference where practical; be explicit about `NvBufSurface`/fd ownership and who releases.
10. **Compile clean at max warnings, and run the analyzers** — `-Wall -Wextra` clean; for the C/C++ pieces, run a static analyzer and treat warnings as defects.

---

## Review procedure

1. Identify the language and where it runs (host vs container; C++ Multimedia API vs pyds vs GStreamer string).
2. Read the matching references first (`multimedia-api.md`, `deepstream-pipelines.md`, `containers.md`, `orin-nano-constraints.md`) so findings cite the right platform reason.
3. Walk the checklist top-down: hardware-fit → buffers/memory → error handling → concurrency → footguns → Power of Ten.
4. Emit severity-ranked findings in the output format above, each with a concrete fix.
5. Close with the prioritized improvement list — and, if the code is a demo derived from NVIDIA samples, note anything that diverges from the installed `common/classes` / API reference (the demos have shipped bugs).
