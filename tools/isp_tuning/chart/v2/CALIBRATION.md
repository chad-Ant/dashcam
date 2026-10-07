# V2 calibration analyser

`calibrate.py` analyses **saved captures** of the new three-sheet pack. It does
not open cameras, stop the recorder, change controls, write `.isp` files, or
install a profile. The offline Docker wrapper uses the existing OpenCV image,
mounts captures read-only and gives write access only to a fresh result folder.
It needs no sudo, GPU runtime, camera devices, network or Argus socket.

## Quick start on this rig

Use absolute paths and quote the wildcard. Replace `YOUR_CAPTURE` below with
your actual session directory under `/home/jetson/drive_logs`. Never combine
different sheets, lighting conditions, runtime variants or profiles in a glob.
The wrapper chooses a new timestamped directory below `v2/results` each time.

### 1. Colour / USB reference

Photograph **sheet 1** with both cameras under unchanged lighting. Prefer five
stationary images from each camera with fixed exposure/gain/white balance; a
single pair is accepted for an initial check. Input limit is 16 per camera.

```bash
bash /home/jetson/drive_logs/tools/isp_tuning/chart/v2/run_calibration.sh colour \
  --images '/home/jetson/drive_logs/YOUR_CAPTURE/colour_*.csi.jpg' \
  --reference '/home/jetson/drive_logs/YOUR_CAPTURE/colour_*.ug.jpg'
```

Outputs:

- `csi_rois.png` and `reference_rois.png`: check these **before trusting the fit**.
- `report.json`: patch means, spatial/temporal statistics, clipping, repeated
  neutral differences, input hashes, decoder, registration and capture-quality gates.
- `colour_comparison.png`: measured CSI / proposed correction / USB patch swatches.
- `REPORT.md`: concise status and colour-fit scores.

Omit `--reference` for CSI patch/neutral diagnostics only. The analyser deliberately
does **not** fit against nominal chart RGB: a home-printed chart is uncharacterized.
CSI and USB bursts are averaged independently; they are not timestamp-matched.
Only use the comparison with a static chart and stable illumination across both
bursts. This is unsuitable for moving scenes or changing light.

The fit first matches mid-grey exposure and channel ratios to the USB reference,
then estimates a regularized, neutral-preserving 3x3 matrix. C03, C06, C09, C12,
C15 and C18 are held out. The remaining 12 chromatic patches train the matrix;
greys are not used as chromatic training samples. The matrix is **not refit on
the holdouts** after evaluation. It is marked `provisional_candidate` only when
holdout mean dE76 improves by >0.2 over grey matching alone, no evaluated patch
falls outside gamut, and both capture-quality checks pass. Otherwise it is
`not_recommended`; no colour-comparison image should be interpreted as proof
of a physical improvement.

**Important: the matrix acts on inverse-sRGB-linearized, already-processed RGB.**
Both cameras' tone curves are only approximated by sRGB. This is a relative
USB-matching model, not a sensor-space CCM or absolute colour calibration. Its
orientation is explicit in `apply_formula` and `matrix_rows`. **Do not paste it
into `camera_overrides.isp` or multiply it into the existing ISP CCM.** Printed
grey is not guaranteed neutral; the script matches the USB's grey, including any
USB cast, rather than pretending it measured a neutral physical reference.

Use an independent capture under a second light before considering a separate
ISP experiment. Preserve the installed profile and use base/candidate/base
trials. This analyser never generates an automatically installable profile.

### 2. Noise / shadow response

Photograph **sheet 2** in one stationary session: **64 frames recommended**,
32 minimum, 256 maximum. Exposure, gain, white balance, sharpening and temporal
denoising must stay fixed. Use zero-padded sequential filenames in capture order.

```bash
bash /home/jetson/drive_logs/tools/isp_tuning/chart/v2/run_calibration.sh noise \
  --images '/home/jetson/drive_logs/YOUR_CAPTURE/noise_*.csi.jpg'
```

For already-correct full-range PNG captures use `--decoder standard`. A PNG
made by incorrectly decoding a limited-range CSI JPEG is **not** corrected
automatically; go back to the original JPEG instead. Prefer true lossless
captures when possible: JPEG-based results include codec quantization/artifacts.

The script measures unresampled native pixels inside the geometry's inner ROIs:

- `temporal_rms_std_rgb`: square root of the mean per-pixel unbiased temporal
  variance (RGB code units). Welford accumulation avoids storing the entire burst.
- `spatial_std_mean_image_rgb`: spatial variation of the mean image, kept
  separate because it includes printing, illumination and fixed-pattern effects.
- `pair_difference_rms_rgb`: difference-image standard deviation divided by
  sqrt(2), RMS-averaged across pairs at lags 1, 4 and 8 input frames. Difference
  image DC is removed, whereas temporal variance includes frame-level changes.
- `frame_mean_peak_to_peak_rgb`: an illumination/exposure/WB drift check.
- `neutral_response_rgb`: measured noise tiles and shadow-step means.

These are **output-pipeline noise**, not raw sensor characterization. ISP temporal
filtering correlates frames; compare the lag estimates, and do not equate a low
lag-1 value with low independent sensor noise. Dropped frames, timing intervals,
USB synchronization and control locks cannot be proven from still files. Record
PTS/timestamps and controls when capturing; lag units here are **file intervals**.
Do not build a noise burst from independently restarted one-shot Argus sessions.
The old `tune_session.py` records only five frames per variant and is therefore
not sufficient for this noise command; do not combine its variants to reach 32.

### 3. Detail / denoising trade-off

Photograph **sheet 3** once per profile, with matched geometry, focus, illumination
and exposure. Analyse each repeat separately:

```bash
bash /home/jetson/drive_logs/tools/isp_tuning/chart/v2/run_calibration.sh detail \
  --images '/home/jetson/drive_logs/YOUR_CAPTURE/detail.csi.jpg'
```

The report contains four quarter-pixel-binned edge-spread profiles, their 10-90%
width in native pixels, and overshoot/undershoot. The edge position/orientation
comes from the chart homography. A failed edge is marked invalid rather than
silently skipped. Texture RMS contrast is reported at four pattern periods.

These are **relative gamma-encoded detail diagnostics**, not ISO MTF or absolute
lens resolution. Print quality, distortion, focus, ringing and aliasing all
affect results. Low noise is not a win if edge width increases markedly or fine
texture disappears. Follow stationary tests with a moving-scene ghosting check.

## Capture gates and failure behaviour

- All four correct sheet markers, no repeated expected IDs; legacy IDs 0-3 are
  rejected. The maximum marker reprojection error is 2.5 pixels.
- Main colour inner ROIs must have sides >=64 pixels; noise tiles >=96 pixels.
  The full sheet should fill about 70-80% of CSI width. Smaller repeated greys
  and shadow strips are diagnostic only.
- Maximum detected-corner displacement within a burst: **1 pixel**. Registration
  is used to detect movement; images are never warped for flat/noise measurement.
- RGB-channel mean drift <=2 codes peak-to-peak, and <=1% near-clipped pixels
  (code <=1 or >=254) in each fitted/noise region. Intentionally dark shadow steps
  are excluded from those quality gates. Repeated left/right greys differing by
  >5 dE76 also block a colour fit.
- Duplicate paths and exact duplicate decoded images are rejected. This may reject
  a truly identical static quantized burst too; do not force it into a noise result.
- Per-burst dimensions and decoding convention must match; input images are
  restricted to 8-bit RGB/grey, <=12 MP. A single ROI is limited to 250,000 pixels.
- A failed quality gate returns exit code **2**. Successfully measured data may
  remain in `report.json` but `valid=false`; **do not use it as calibration**.
  An existing output directory is refused. Failed runs are retained for diagnosis.
- Exit code 0 means acquisition/measurement gates passed, **not** that the fitted
  colour correction is recommended. Inspect `fit.status` separately.

For investigation only, `--max-motion-px` can be set in (0,3] and
`--max-drift-codes` in (0,10]. Defaults are conservative heuristics, not a standard.
Prefer improving capture conditions instead of loosening them to obtain a pass.

## Decoder selection

`--decoder auto` uses the existing rig-specific `../../csi_decode.py` JPEG-header
test and limited-range BT.601 correction. The detected convention is logged.
`--decoder nvjpeg` requires a recognized rig JPEG and refuses other files.
`--decoder standard` bypasses that heuristic for known full-range images. The
USB reference defaults to `--reference-decoder standard`. The heuristic is specific
to this rig's existing nvJPEG output, not a universal JPEG colour-space detector.

## Direct invocation and tests

Inside an environment with OpenCV+ArUco, NumPy and Pillow:

```bash
python3 /home/jetson/drive_logs/tools/isp_tuning/chart/v2/calibrate.py --help
```

Direct invocation requires `--out NEW_DIRECTORY`; unlike the wrapper it can
use paths outside `drive_logs`. The script needs its sibling `output` geometry
and the existing decoder two directories above. No installation is necessary.

Run the synthetic tests in the existing container without camera access:

```bash
docker run --rm --runtime=runc --network=none \
  --env PYTHONDONTWRITEBYTECODE=1 --env OPENBLAS_NUM_THREADS=1 \
  -v /home/jetson/drive_logs/tools/isp_tuning:/tuning:ro \
  l4t-ml-gpio:latest python3 /tuning/chart/v2/test_calibrate.py
```

Tests cover all three sheets, wrong/missing markers, pixel coverage, known matrix
recovery, held-out colour improvement, correlated and independent noise, a full
32-frame noise run, known Gaussian blur, drift, clipping, duplicates, motion,
decoder selection, failure reports and overwrite prevention. Passing synthetic
tests is not a live validation of the printed chart or installed ISP.

Methods background:
[OpenCV ArUco](https://docs.opencv.org/4.10.0/d9/d6a/group__aruco.html),
[Imatest multi-frame temporal noise](https://imatest.atlassian.net/wiki/spaces/KB/pages/11416109743/Measuring+temporal+noise).
