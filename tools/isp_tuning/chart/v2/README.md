# IMX296 calibration chart v2

Three **A4 landscape vector sheets**, for controlled *relative* ISP tuning with
the existing CSI/USB setup. No ISP setting, camera service, or original chart
has been changed. A chart makes measurements possible; it does not itself
eliminate colour offset or noise. Printed ink is not a calibrated reference.

## Files

- `output/imx296_chart_v2_a4.pdf`: print this three-page pack.
- `output/01_colour.pdf`: 24 labelled colour patches and three left/right pairs
  of strictly equal-channel grey patches for field-consistency checks.
- `output/02_noise_tone.pdf`: six 80 x 50 mm flat patches (nominal RGB codes
  16, 32, 64, 96, 160, 224), plus ten shadow steps.
- `output/03_detail.pdf`: vertical/horizontal 5-degree slanted edges, in both
  polarities, plus four spatial scales of low-contrast checker texture.
- `output/*_geometry.json`: page, marker, patch and inner-ROI geometry in mm.
- `output/*.svg`: editable vector copies; `*_300dpi.png`: lossless raster copies.
- `output/preview.png`: screen overview only, **not a print target**.
- `make_chart.py`: deterministic artwork generator, using host pycairo/Pillow.
- `validate_chart.py`: offline generated-artifact checks, using OpenCV/NumPy.
- `calibrate.py`: v2 saved-capture colour/noise/detail analyser; see
  [CALIBRATION.md](CALIBRATION.md) for commands and measurement limits.
- `run_calibration.sh`: offline Docker wrapper with read-only captures and a fresh result folder.

## What changed and why

The original PDF embedded a JPEG of the whole page. V2 PDFs use vector fills,
paths and text, so the artwork introduces no JPEG ringing or block noise.
Markers are now 26 mm, 10 mm from paper edges, with at least one module of
white quiet space around each marker. Original artwork and metadata remain
unchanged one directory up.

Separating the tasks gives noise measurements much larger uniform regions and
gives denoising a detail-retention check. Colour tiles are 32 mm instead of the
original 36 mm to accommodate marker clearance and repeated neutrals; **move
the chart closer** to gain measurement pixels. Layout alone does not improve
sampling if you keep the old distant placement.

The RGB 96/180 detail patterns have approximately 3.9:1 luminance contrast
*under an ideal sRGB transfer function*. The actual printed contrast is
unknown until measured. These are relative sharpness patterns, not a claim
of ISO/EMVA compliance or an absolute lens MTF measurement. The fine texture
is intentionally susceptible to optical blur/aliasing; never use it as a flat
noise ROI or judge it from a downscaled screen preview.

## Print and position

1. Print the PDF single-sided, A4 landscape, **actual size / 100%**. Disable
   fit-to-page, photo enhancement, auto contrast and sharpening. Use one
   consistent printer/paper/ICC workflow; avoid double colour management.
2. Use flat, matte stock. Check the ruler is **100 mm**, all four markers are
   intact, and the paper is not curled. Printer halftoning and gamut limits
   remain; some shadow steps may merge on paper and cannot diagnose the camera
   in that case. Do not laminate with glossy film.
3. Mount one sheet at a time, nearly perpendicular to the optical axis, under
   even, stable light without glare. Get the page to span about **70-80% of the
   native 1456-pixel CSI width**, with all markers visible in both cameras.
   At 70% fill, the central colour sampling ROIs are about 66 x 66 pixels,
   compared with roughly 20 x 20 in the earlier distant CSI capture.
   Repeated side greys are smaller: useful for colour means, not noise estimates.
4. Capture at native resolution, keep focus/placement fixed within an A/B
   trial, and remove strong nearby colour reflections. Record illumination,
   printer/paper, distance, profile and all runtime controls with each session.

## Measurement procedure

### Colour

- For absolute calibration, use a characterized physical colour target or
  measure the actual printed patches. Nominal sRGB values are **not measured
  reflectance**. The USB camera is a comparative reference, not ground truth.
- Use a genuinely neutral grey reference to set white balance. Ordinary white
  paper and printer grey can have a cast. L1/R1, L2/R2, L3/R3 contain identical
  artwork; differences in camera means can reveal uneven illumination,
  printing, reflections or field shading, not uniquely an ISP fault.
- Fix exposure, gain and white balance separately on CSI and USB after settling.
  Use timestamps and stable light for pairing. Avoid clipping dark or bright
  patches. Change one tuning variable at a time and use base/candidate/base.
- Fit on only part of the colour data and validate held-out patches plus a
  second lighting condition. A better in-sample USB match is not sufficient
  evidence to install a system-wide CCM.

### Noise and tone

- Capture **64 stationary frames** per profile/lighting setting, with fixed
  exposure, gain, white balance and denoising/sharpening controls. Retain native
  samples; prefer lossless decoded frames over JPEG when the capture path permits.
- Map the inner rectangles in the JSON into the original camera image and use
  those masks. Do not warp/resample the image before estimating noise; that
  changes noise statistics. Use one stable registration for a stationary burst
  and reject camera/chart movement, flicker, dropped/duplicate frames and drift.
- Report per-pixel temporal standard deviation across frames, aggregated per
  ROI/channel, **separately** from spatial variation in the mean image. Spatial
  variation includes paper, halftone, uneven light, lens shading and fixed-pattern
  effects. A single image's spatial standard deviation is not sensor noise.
- Pair-difference noise, `std(frame_a-frame_b)/sqrt(2)`, is only valid under
  equal variance and independent noise. ISP temporal denoising correlates frames;
  compare several frame lags and report correlation/scene flicker instead of
  claiming raw sensor noise. Temporal output noise includes the whole pipeline.
- Use the shadow strip for clipping/banding visibility, not an assumption that
  dark printer codes represent distinct known scene luminances.

### Detail retention

- Capture the detail sheet under the same controls and comparable lighting as
  the noise sheet for each A/B profile. Check both edge orientations/polarities
  for halos, edge spread and retained checker contrast. Do not accept a lower
  noise number bought by unacceptable blur or smeared moving detail.
- Use inner edge ROIs in the geometry, excluding patch boundaries/text. Proper
  quantitative slanted-edge analysis needs oversampling and a measured/qualified
  print; marker edges are only positioning features, not sharpness references.
- After stationary tests, use a separate moving-scene check for temporal ghosting.

## Compatibility and decoding: important

The legacy `chartcmp.py`, `chartfit.py`, `chartnoise.py`, `chartsharp.py` and
`analyse_session.py` are **not v2-compatible**. They assume IDs 0-3 and/or the
old 6x4 geometry. V2 deliberately uses different IDs so those scripts cannot
silently fit the wrong patches:

| Sheet | IDs in TL, TR, BR, BL order |
| --- | --- |
| Colour | 10, 11, 12, 13 |
| Noise/tone | 14, 15, 16, 17 |
| Detail | 18, 19, 20, 21 |

All markers are upright `DICT_4X4_50`, with a one-cell black border; coordinates
describe the outer black square (not the quiet zone). Each flat patch has a
central **60% width/height** ROI. RGB values in metadata describe source artwork
only. Use the separate `calibrate.py` for v2 captured images, with native-pixel
masks, burst temporal statistics and held-out colour fitting. It does not capture
images or install a profile; its colour matrix is a post-render relative model,
not an Argus ISP CCM. See [CALIBRATION.md](CALIBRATION.md).
`validate_chart.py` still validates artwork, **not real camera calibration**.

For real CSI JPEG captures, retain the existing
`/home/jetson/drive_logs/tools/isp_tuning/csi_decode.py` range-aware decode path:
earlier nvJPEG images carried limited-range BT.601 without the usual JFIF marker.
Do not compare an ordinary full-range decode of those files against USB and
interpret the range error as camera colour error. Do not use the special CSI
decoder on these chart PNGs or other ordinary images.

## Regenerate and validate (offline)

The generator refuses an existing output directory; choose a new destination
to compare revisions. Existing captures and originals are never overwritten.

```bash
python3 /home/jetson/drive_logs/tools/isp_tuning/chart/v2/make_chart.py --out /tmp/imx296-chart-v2-new
```

Use an unused path above. On this rig OpenCV/ArUco is available in the existing
container image; no GPU, camera device, Argus socket or network is needed:

```bash
docker run --rm --runtime=runc --network=none \
  -v /home/jetson/drive_logs/tools/isp_tuning/chart/v2:/chart:ro \
  l4t-ml-gpio:latest python3 /chart/validate_chart.py
```

For independent PDF rendering, `pdftoppm -r 100 -png` the pack to a temporary
prefix, then pass `--pdf-prefix` to the validator with the rendered files mounted
read-only in the same container. `pdfinfo` should report 3 A4 pages;
`pdfimages -list` should contain no image objects. Validation covers IDs,
corner order/positions, all flat ROI colours, 70%-fill 1456x1088 views and a
simulated perspective view. It does not validate an actual print or live sensor.

## References

- [Imatest colour/tone measurements](https://docs.imatest.com/docs/color-tone/):
  measured target reference data and colour/tone analysis.
- [Imatest temporal noise](https://imatest.atlassian.net/wiki/spaces/KB/pages/11416109743/Measuring+temporal+noise):
  multi-frame and difference-image approaches.
- [Imatest slanted-edge/star comparison](https://www.imatest.com/docs/slant_edge_star_comparison/):
  lower-contrast detail targets and processing sensitivity.
- [Calibrite grey-balance reference](https://calibrite.com/us/product/colorchecker-gray-balance/?noredirect=us-US):
  a spectrally neutral reference differs from ordinary paper.
