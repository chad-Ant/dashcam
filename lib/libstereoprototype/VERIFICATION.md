# Prototype verification — 2026-10-07

Ran on the Orin in the existing `l4t-ml-gpio:latest` container, using the default
`runc` runtime, no network, devices, Argus socket, GPU access or display. Host
reports L4T R36.5.2. No firmware flash, dependency installation, camera capture,
service stop/restart or change to the existing stereo library was performed.

- **30 tests passed** with OpenCV/NumPy: metric calibration roundtrip and
  left/right sign, known projected-checkerboard calibration/held-out residuals,
  disparity/range calculation, left-right consistency, blank-scene rejection,
  native cropping/coordinate offsets, search/ROI limits, moving-scene timing
  gates, sequence/skew/clock faults, motion-budget sensitivity and full CLI demo.
  The checkerboard calibration test supplies exact projected corners to the real
  OpenCV optimizers; physical corner detection on the recommended camera/lens
  combination is not tested.
- 4096x512 **synthetic textured plane** at 100 m, 1 m baseline and illustrative
  `fx = 35 mm / 2.74 um = 12773.72 px`: median result **99.7947 m**, p10/p90
  **99.7947 / 99.8435 m**, all pixels valid in the selected 500x150 target ROI.
  The texture, calibration and stereo shift are generated, not photographed.
- One saved run's `compute` call took **0.381 s** with two OpenCV threads for
  this ROI. This excludes initialization, camera acquisition/transfer and disk
  output; it is not a sustained end-to-end FPS benchmark or road performance.
- The model's example error budget was about 0.59 m for that static synthetic
  result, conditional on a 0.5 px disparity bound and 0.2% scale bound. Neither
  assumption has been measured on physical cameras. `accuracy_verified=false`.
- Python compile checks and `git diff --check` passed. No v0.4 regression suite
  was run: the new prototype is isolated and not linked into the application.

Saved synthetic artifacts stay local:
`/home/jetson/drive_logs/stereo_prototype_check.LL2juQ/demo/`.
`result/report.json` records the result, assumptions and input/calibration hashes;
the paths starting `/evidence` are container-relative to that evidence directory.

Not verified: camera/SDK compatibility, actual lens FOV/quality, achievable
exposure synchronization, power/USB throughput, night imaging, thermal/mechanical
stability, real-object disparity bounds, moving-target range accuracy, target
association or production compute budgets. A vendor capture adapter, common
trigger implementation and measured 100 m validation are still required.
