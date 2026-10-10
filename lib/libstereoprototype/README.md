# Stereo rangefinding prototype — 100 m feasibility study

The requested target is **moving cars ahead, up to 100 m, about +/-1 m error,
with at most 1 m between cameras**. The original proposal was two ESP32-CAMs
and a software trigger. That is **not a defensible architecture for this target**.
The Orin-side prototype here is camera-independent so it can be evaluated with
properly synchronized industrial cameras. No ESP32 firmware, hardware capture
adapter, SDK installation or production integration is included in this pass.

## Feasibility and hardware recommendation

For rectified stereo, `Z = f_px * B_m / disparity_px`. Ideal first-order error is
`delta_Z ~= Z^2 * delta_disparity / (f_px * B_m)`. The `budget` command uses the
exact inverse relation for a specified disparity interval rather than that
first-order approximation. It is a sensitivity calculation, **not an accuracy
guarantee**, confidence interval, or proof that the matcher achieves the assumed
subpixel error. Wrong matches can be arbitrarily wrong.

Examples at **100 m, 1 m baseline, +/-0.5 pixel assumed disparity error**, with
no calibration/motion error:

| Optics / image width | Focal length in pixels | Ideal worst-direction depth error |
| --- | ---: | ---: |
| 640 px, assumed 60-degree horizontal FOV | 554 | about 9.9 m |
| 1600 px, assumed 60-degree horizontal FOV | 1386 | about 3.7 m |
| 4096 px, 2.74 um pixels, 25 mm lens | 9124 | about 0.55 m |
| 4096 px, 2.74 um pixels, 35 mm lens | 12774 | about 0.39 m |

The FOVs above are assumptions, **not measured ESP32-CAM lens specifications**.
An OV2640's 1600x1200 output and JPEG buffering are documented by
[Espressif](https://github.com/espressif/esp32-camera). Its framebuffer timestamp
is the start of DMA, not a certified simultaneous exposure timestamp; separate
ESP32 boot clocks also cannot be subtracted as if synchronized. Requesting two
JPEGs concurrently therefore does not establish exposure alignment.
[Driver timestamp/buffering definition](https://github.com/espressif/esp32-camera/blob/master/driver/include/esp_camera.h).

**Recommended starting point to quote/evaluate, not an unconditional purchase:**

- Two identical **Basler ace 2 a2A4096-30umBAS** monochrome USB3 cameras
  (Sony IMX545, global shutter, 4096x3000, 2.74 um pixels). A comparable 4K-wide
  global-shutter industrial camera with documented external triggering is also
  suitable. [Product specifications](https://www.baslerweb.com/en/shop/a2a4096-30umbas/).
- Matched, locking **25-35 mm C-mount lenses** rated for the full 1/1.1-inch image
  circle and this pixel pitch. Prefer 35 mm for the 100 m trial; nominal HFOV is
  about 18 degrees, so retain the existing wide-angle road camera for near/wide
  coverage. Lens focal length/FOV must be calibrated, not copied from a label.
- A rigid, thermally stable **1 m optical-centre baseline**, stable focus, matched
  exposure/gain and short exposures. Avoid flexible dashboard mounts and
  different windshield regions without measuring their distortion/reflections.
- **One electrical trigger edge distributed to both cameras.** Software may
  request that *one shared pulse*, but two separate USB/Wi-Fi software-trigger
  calls are not the same thing. Measure exposure-active timing/skew with a scope
  and verify the camera's trigger-delay settings. The prototype rejects a
  declared skew bound above 100 us; this is a design gate, not a measured rig value.
  [Basler trigger control](https://docs.baslerweb.com/triggered-image-acquisition),
  [acquisition timing](https://docs.baslerweb.com/acquisition-timing-information).

Use a proper trigger distributor/level driver. Camera opto-input current/voltage
requirements are not necessarily compatible with directly driving two inputs
from a Jetson GPIO. Check exact camera electrical specifications before wiring;
do not connect automotive supply voltage to Jetson pins.

The geometry leaves little margin: at 35 mm, adding a **0.2% scale-error bound**
already raises the ideal 100 m error to about 0.59 m. An illustrative combined
0.2 ms timing/blur interval, 10 m/s transverse motion and 5 degrees/s yaw raises
it to about **0.97 m** before the small forward-motion allowance and unmodelled
effects. These numbers are conditional assumptions, not attainable performance
claims. The disparity-error assumption must include residual alignment/optical
errors, not just the SGBM fractional-pixel quantization. Vibration, temperature,
defocus, low texture, shiny car panels, headlights, rain and night exposure can
break the budget. A common trigger by itself does not eliminate motion blur.

If +/-1 m at 100 m is a hard operational requirement over varying conditions,
evaluate **automotive long-range radar as the primary range channel**, with vision
for association. TI's AWR2944 reference design targets vehicle tracking to 200 m;
this is an alternative evaluation path, not a claim that an EVM automatically
meets your error/availability requirements. [TI TIDEP-01027](https://www.ti.com/tool/TIDEP-01027).
This prototype is not for braking, collision avoidance or vehicle actuation.

## Transferring images to the Orin

Recommended data path:

```text
                         one shared electrical trigger
                              /               \
                      left camera          right camera
                       Mono8 USB3           Mono8 USB3
                              \               /
                         Orin capture adapter
                  serial IDs + trigger IDs + exposure metadata
                              |
               calibrated rectification at native pixel scale
                              |
                    target-region stereo matching
                              |
                  depth + validity + conditional error budget
```

Use the vendor's **Linux ARM64 pylon SDK / pypylon**, delivering uncompressed
Mono8 buffers directly to NumPy/OpenCV, not JPEG over Wi-Fi. Basler documents
Linux ARM64 support and recommends Jetson-class ARM processors; compatibility
with this rig's exact R36.5.2/container stack still needs a bench check.
[pylon support](https://docs.baslerweb.com/pylon-software-suite).
No vendor SDK has been installed by this work.

Payload arithmetic (not measured link throughput): two full 4096x3000 Mono8
images at 10 pairs/s need **245.8 MB/s** before protocol overhead. Two 4096x1024
native-pixel sensor bands at 10 pairs/s need **83.9 MB/s**. Shared USB controllers,
cables, camera power and existing recorder traffic must be checked. Start at
low frame rate; use a calibrated sensor ROI or a host-side crop, **not downsampling
that throws away long-range disparity precision**. A changed sensor crop needs
corrected intrinsics/offset metadata and validation. Raw 16-bit containers double
these payloads. Two full-resolution 30 fps streams need about 737 MB/s, beyond
one shared nominal 5 Gbit/s USB link even before overhead.

For longer vehicle cabling consider an industrial GigE/5GigE or GMSL variant,
with matching NIC/capture hardware and supported drivers. A camera's rated frame
rate does not imply this Orin can calculate dense full-frame depth at that rate.

For an **ESP32-CAM static short-range bench experiment only**, the transfer would
instead be on-sensor JPEG -> binary HTTP/TCP over a local Wi-Fi AP -> Orin JPEG
decode. Prefer Orin Ethernet to that AP, bounded requests, no base64, and send
pair ID, camera/boot ID, frame counter, dimensions and DMA timestamp with each
frame. Reject pre-request buffered frames and duplicate/late/missing pairs.
Those checks improve freshness but still do not prove synchronized exposure;
the moving-scene API deliberately does not accept such timestamps as proof.
That firmware/HTTP adapter was not implemented because it cannot satisfy the
clarified 100 m moving-car requirement.

## Implemented library

- `stereoprototype/calibration.py`: metric pinhole calibration loader and
  checkerboard fitting, with every fourth view held out for vertical residual
  checking. Horizontal positive-disparity rigs only; distinct camera IDs,
  dimensions, distortion and rotation are validated. No automatic resizing.
- `core.py`: OpenCV SGBM left/right consistency, invalid-rectification masking,
  texture rejection, range/search limits and robust target-ROI median. Broad
  depth populations are rejected rather than averaged into one object.
- Long-range computation is **native-resolution ROI processing**. The example
  supports about 30-120 m with 448 disparities at the illustrative 35 mm/1 m
  geometry. That is not a claim of near-range coverage. More than 1 MP of dense
  matching is refused; rectification may still process up to 4096x3000.
- `timing.py`: moving scenes require same trigger sequence, a validated shared
  timing domain, global shutters, uncertainty/exposure durations and conservative
  transverse/forward/yaw motion bounds. Host receipt times and independent board
  clocks are refused. Metadata is a **caller assertion backed by external
  measurement**, not something software can certify from two images.
- `geometry.py`: range/error-budget calculator. Timing and blur are approximated
  conservatively for the central field; this is not a rolling-shutter model or
  arbitrary 3D motion compensation.
- CLI for design budgets, saved-pair ranging, checkerboard calibration and a
  reproducible synthetic 100 m plane. JSON reports, depth NPZ and diagnostic PNGs.
  `valid=true` means a usable matcher result, **not** that +/-1 m is established.
  `accuracy_verified` remains false; `within_assumed_budget` is conditional only.

This is a separate Python prototype, **not a replacement for `lib/libstereocam`**.
That existing C++ class couples capture to `iCamera`/VPI and uses capture-return
times rather than exposure synchronization. It was left unchanged. The new
prototype uses CPU OpenCV in the existing container, no display, NVENC or DLA.
It is single-worker/non-thread-safe and not a realtime production implementation.

## Run without hardware

```bash
cd /home/jetson/Documents/github_repos/dashcam
docker run --rm --runtime=runc --network=none \
  -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 \
  -v "$PWD/lib/libstereoprototype:/work:ro" -w /work \
  l4t-ml-gpio:latest python3 -m unittest discover -s tests -v
```

With this package directory on `PYTHONPATH` (or as working directory) and a
working NumPy/OpenCV environment:

```bash
python3 -m stereoprototype budget --focal-px 12774 --baseline-m 1 \
  --disparity-error-px .5 --scale-error-fraction .002 --distance 30 50 100
python3 -m stereoprototype demo --out /tmp/stereo-demo-new
python3 -m stereoprototype range --pair capture/pair.json \
  --calibration calibration.json --settings config/long_range.example.json \
  --roi 1700 180 500 150 --out result-new
```

Directories/files named as outputs must not already exist. The demo is
synthetic and explicitly marked as such; **never use its calibration on cameras**.
The moving-pair template intentionally contains unverified/null values and
refuses to run until real measurements are supplied. A static saved-pair manifest
contains `schema: stereo-prototype-pair-v1`, distinct `left`/`right` paths,
matching `left_id`/`right_id`, and explicit `static_scene: true`. Do not mark a
moving car or moving rig static just to bypass the gate.

The API accepts uint8 gray or BGR arrays from any future capture adapter:

```python
from stereoprototype.calibration import Calibration
from stereoprototype.core import Rangefinder, Settings

finder = Rangefinder(Calibration.load("calibration.json"),
                    Settings(30, 120, .5, scale_error_fraction=.002,
                             num_disparities=448))
# Only a truly stationary bench scene:
result = finder.compute(left_image, right_image, static_scene=True,
                        region=[1700, 180, 500, 150])
estimate = finder.measure_roi(result, [1700, 180, 500, 150])
```

For moving scenes supply `ExposurePair`, `lateral_speed_bound_mps`,
`yaw_rate_bound_rad_s` and `forward_speed_bound_mps`, leaving `static_scene=False`.
ROI coordinates are in the **full rectified left image**. Cropped outputs carry
`origin_xy`; depth is optical-axis Z, not Euclidean slant range or bumper-to-bumper
distance. A vehicle detector/segmenter and mounting-offset compensation are not
included. A box that includes road/background is not automatically a car range.

## Calibration and validation before hardware claims

1. Choose/quote the cameras, lenses, trigger distributor and mounting arrangement.
   Confirm software support and input/output electrical timing with the vendor.
2. Write the capture adapter for the selected SDK. Select by serial, lock identical
   formats/exposure, arm both cameras, issue one common trigger, match hardware
   sequence IDs, reject lost frames and record bounded exposure timing. Receiving
   a left image followed by a right image does not identify a synchronized pair.
3. Calibrate at the final focus/resolution/mount/crop. Supply 16-100 static board
   pairs at diverse positions, tilts and distances, measured square size in metres.
   `calibrate --help` gives arguments. The ISP colour chart is **not** a stereo
   intrinsic/extrinsic target. Check checkerboard order: a symmetric board can
   have a 180-degree ambiguity; all views must have consistent physical origins.
   The prototype does not automatically solve that ambiguity or calibrate fisheye.
4. Verify optical-centre baseline independently. Low checkerboard reprojection
   error and held-out vertical residuals do not validate horizontal scale at 100 m.
5. Use independently surveyed target distances at 10/20/30/50/75/100 m on a closed
   test area; the long-range example should report unavailable outside its coverage.
   Separate calibration/tuning from held-out runs. Measure error percentiles,
   outlier rate and **availability**, including textureless and reflective cars.
6. Repeat with relative motion, turns, vibration, warmed/cold rig, glare and reduced
   light. Verify trigger skew on a scope and image motion blur at actual exposures.
   Add independent radar/LiDAR/RTK reference as appropriate. Do not validate
   against another uncharacterized vision estimate.

The raw 30-test suite and the synthetic demo validate software behaviour only.
No new cameras were connected, captured, flashed or deployed by this work.
