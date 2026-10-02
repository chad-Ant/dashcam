# IMX296 (CSI) image tuning — state on 2026-10-02 morning

**Installed now (since 2026-10-02 06:53, verified live 06:56):** `/var/nvidia/nvcam/settings/camera_overrides.isp`
= `c8_sh15.isp` (sha256 43ec0c81…). It is c5_rpi100T's colour (black level 50, the transposed IMX296 colour
matrix) plus the sharpness table at its weakest index, so the ISP's default output matches ee-mode=0. The vendor
original is `vendor_v1.1.isp` (sha256 c85a9342…). The c5 base is `c5_rpi100T.isp` (20b4c94c…). Earlier c1/c2
notes below are historical, not the current selection.

## Corrections and review, 23:45 (read first)

The repo's `HANDOVER.md` has the full text, under "Review of the c7 work…". The review outputs are in
`reviews_20261001/`.

- **Wrong decode in every CSI number below.** The CSI JPEGs are limited-range BT.601 with no JFIF marker; the
  scripts decoded them as full range. Corrected trial: colour ΔE to USB **11.71 (c5) vs 6.69 (c7)**; CSI paper
  L* 74.9 vs USB 72.6.
  - **Fixed 2026-10-02:** `csi_decode.py` expands the range, decided from the header. Every analysis script
    uses it; JFIF (UGREEN) results are unchanged.
- **c7 saturation=1.0 is withdrawn.** With the correct decode, c7 scores 5.24 / 6.71 / 9.27 at 0.9 / 1.0 / 1.1.
  An explicit value replaces the ISP's own; unset measures about 0.75 of an explicit 1.0.
- **c7 is closer to the UGREEN, not shown to be better.** The trial is in-sample; against the chart's nominal
  values the gain is about zero; and the matrix bakes in a lamp-specific tint (a* −1.6, 4 % darker). Do not
  install it as it is. The neutral-preserving variant c7n is in the review.
- **sharpness.v2: lower = stronger** (0 = strongest). `c6_sh1` and `c6_sh0` sharpened MORE.
  - On R36.5.2, `sharpness.v2` and `sharpness.v5.tab` are one table (last line wins), and `sharpness.v2.MaxValue`
    is discarded.
  - Candidates to test: `c8_sh15.isp` (table all 15) and `c8_v5off.isp`. `c8_v5tab15` was identical to
    `c8_sh15` and moved to `reviews_20261001/superseded/`.
- **`trial_c7.sh` is safe for the file on ordinary paths.** Its weak points: Ctrl-C is deferred until the
  capture ends; the candidate is hashed at the start but installed later; a second Ctrl-C can interrupt the
  restore. See the HANDOVER.
- **`pair.sh`'s `[exposure gain]` readout is the CSI's (`/dev/video0`), not the UGREEN's.**
- **c8 result (2026-10-02 05:45, `trial_20261002_054515/report.json`): `c8_sh15` PASSES.** At the ISP default it
  gives ee-mode=0-like output: rim 14.5 %, halo 7.2 %, MTF 1.01, noise 1.04× ee-off (c5: 82 %, 68 %, 3.2). Manual
  ee-strength still works, and colour is unchanged. `c8_v5off` passes too, but disables ee-strength.
  **Installed 2026-10-02 06:53.** Verified with a no-property capture (what the app does), which measures like
  ee-mode=0: rim 19.5 % vs 21.2 %, halo 8.5 % vs 5.6 %, MTF 1.07 vs 1.10. There were no loader errors.
  Back to c5:
  `sudo install -m 0644 ~/drive_logs/tools/isp_tuning/c5_rpi100T.isp /var/nvidia/nvcam/settings/camera_overrides.isp && sudo systemctl restart nvargus-daemon`
- **Marker detection:** `chartcmp.find_markers` retries missed markers on an unsharp-masked copy. ee-mode=0 frames
  lost marker 0; frames that already found all four are unchanged.
- **The A/B kit (2026-10-02, reviewed):** `trial_ab.sh` (sudo; base, cand, base, …; restores on every exit),
  `ab_report.py` (pass/fail) and `tune_session.py --with-unset/--auto`. Use them instead of `cycle.sh` and
  `trial_c7.sh`. Next run:
  ```bash
  sudo bash ~/drive_logs/tools/isp_tuning/trial_ab.sh c8_sh15 c8_v5off
  docker run --rm --user 1000:1000 -v ~/drive_logs/tools/isp_tuning:/w -w /w l4t-ml-gpio:latest python3 ab_report.py trial_<stamp> --json
  ```

## Live continuation, 21:41–21:46

### C7 A/B result, user-run trial at 21:54

The user ran `trial_c7.sh`. Both sessions completed with 50 pairs each and the
correct distinct ISP hashes. Exposure/gain were 675/184 at every step; USB
control dumps before/after were identical. The original c5 profile was restored
(verified sha256 20b4c94c…), Argus is active, and the recorder remains stopped.
Recovery copy: `/var/backups/csi-trial.c4MsRq/camera_overrides.isp`.

Measurements are in `trial_20261001_215416_{base,candidate}/metrics.json`.
Of the 100 pairs, 97 had all four markers in both images; 3 were excluded.
All five pairs at the main comparison setting were valid. At `ee-mode=0`,
`tnr-mode=1`, `saturation=1.0`:

| Metric (five paired samples) | Existing c5 | Trial c7 |
|---|---:|---:|
| Mean colour ΔE76 to USB (18 patches; lower is better) | 8.56 | 4.79 |
| Mean grey ΔE76 to USB (patches 19–22) | 5.36 | 4.03 |
| CSI paper L* | 71.15 | 70.23 |
| USB paper L* | 72.41 | 72.56 |

The improvement persisted at the end of the sweeps (colour ΔE 8.60 / 4.75).
Across the five main-comparison USB frames per session, the reference's mean
patch colour drift between sessions was only ΔE 0.35 (maximum 0.76).
C7 is about 0.9 L* darker at the same sensor exposure; it still has a residual
grey tint (mean a*=+3.88, b*=-9.12), and the USB reference itself renders blue
greys. This validates relative matching under this lamp, NOT absolute colour
accuracy or daylight performance.

**Updated recommendation for c7:** `ee-mode=0 tnr-mode=1 saturation=1.0`.
C7 saturation 0.9 scores worse (5.59), as does 1.1 (5.68). Thus the earlier
0.9 recommendation applies only to c5, not c7. Default sharpening still produces
large halos (~63% versus ~5% off, representative frames). The preset is recorded
in `c7_runtime.json`; no application configuration has been changed. Installing
the ISP file alone does not disable runtime edge enhancement.

C7 has NOT been selected permanently. Next: optionally select it for this bench
setup, then repeat under daylight and with auto exposure/white balance before
production use. Do not copy the bench's manual shutter/gain or locks into the app.

### Earlier sweep and candidate derivation

Capture tool: `tune_session.py`, using host GStreamer/Argus (not Docker for capture).
Offline measurements: `analyse_session.py` in `l4t-ml-gpio`, with no camera access.
Both tools reject reused output directories or incomplete sessions as appropriate;
the old `pair.sh`/`cycle.sh` have NOT been repaired and were not used.

- `session_20261001_2142`: initial lock-only sweep. Significant brightness drift;
  do not use it for a final colour fit.
- `session_20261001_2146`: explicit fixed exposure/gain sweep, five pairs per setting,
  all four markers detected in both cameras on all 50 pairs. Full metadata and
  measurements in `session.json` and `metrics.json`.
- Vendor-driver-specific bench settings: Argus exposure range `675000 675000`
  produces **675 lines ≈10 ms**, NOT 675 microseconds; gain range `11.5 11.5`
  produces code 184 (18.4 dB); ISP digital gain fixed to 1. Verified V4L2 exposure
  675 / gain 184 at every variant. These are **not road-use defaults**.
- CSI paper L* mean: 71.08 initially / 71.10 finally. USB reference approximately
  72.35. USB controls were never written; before/after dumps are identical. Its
  automatic processing remains enabled, with paired images retained to check drift.
- `ee-mode=0`: representative spatial luma std 1.11 versus 2.80 with default
  sharpening; low-contrast bright overshoot ~5.6% versus ~64.3%. The std includes
  print texture and JPEG artefacts, not solely sensor noise. Mild strength 0.05
  is an option (~9.4% overshoot), but off is the cleanest tested baseline.
- Colour ΔE76 to USB (18 coloured patches, mean over five pairs): saturation
  0.9 = 8.32, 1.0 = 8.66, 1.1 = 10.18, 1.2 = 12.46, 1.3 = 15.19.
  Recommend `ee-mode=0 tnr-mode=1 saturation=0.9` as a **provisional runtime
  preset for this light**, not a validated all-light production preset.
- `session_20261001_2146/c7_usb_half.isp`: uninstalled half-strength USB-match
  candidate. Brightness-normalized leave-one-colour-out ΔE on 22 usable patches
  predicts 8.48 → 5.43. This is an offline approximation from rendered JPEGs,
  not proof the ISP will improve by that amount; it also inherits USB rendering.
  Do not confuse these cross-validation numbers with the raw 18-patch scores above.
- Administrator access is unavailable to the agent. To do the real A/B test:
  `sudo bash /home/jetson/drive_logs/tools/isp_tuning/trial_c7.sh`
  It checks the current profile and candidate, backs up the current profile,
  captures a fresh c5 baseline and c7 trial, and restores c5 on normal exit/error/
  interrupt. It never starts the recorder. Power loss/SIGKILL cannot be trapped;
  the printed `/var/backups/csi-trial.*/camera_overrides.isp` is the recovery copy.
  Review the two new trial directories before selecting or installing c7 permanently.

No system ISP changes were made during this continuation; dashcam-v04 remains
stopped. Daylight/motion validation and application integration are still pending.

To go back to the vendor file:
```bash
sudo install -m 0644 ~/drive_logs/tools/isp_tuning/vendor_v1.1.isp /var/nvidia/nvcam/settings/camera_overrides.isp && sudo systemctl restart nvargus-daemon
```

## Findings

Measured against the UGREEN, which pointed at roughly the same office scene, under warm room lights.

- **Edge enhancement.** `ee-mode=0` (or `ee-strength=0`) cuts the measured noise about 4×
  (σ 3.5 → 0.9) at the same brightness. It is the largest visible gain.
- **Temporal noise reduction.** `tnr-mode=2` gives a flat green frame (at strength 1 and 0.5).
  Keep the default mode 1.
- **Brightness.** `exposurecompensation=0.5` matches the UGREEN (wall L* 71–73 vs 72).
  The vendor AE target is 80 (`ae.MeanAlg.*Target`).
- **White balance presets.** All of them are worse than auto, and every one is warmer. The
  vendor's AWB tables come from RidgeRun's IMX477 file (the file's header says "experimental
  candidate … not production tuning").
- **Saturation.** Raising `saturation` only amplifies the remaining cast. Leave it at 1 until
  the white balance is right.
- **Black level (c1).** The driver programs BLKLEVEL 0x032 (50) for colour, but the vendor ISP
  subtracted 60, which crushed the shadows and tinted them green. With 50:
  - the cardboard's b* is 21 (UGREEN 20; it was 43);
  - the blue flag now reads blue;
  - the mean region ΔE fell from 27.8 to 23.7 (25.9 to 20.4 at +0.5 EV).
- **Remaining wall cast.** The wall is still magenta-warm: a* +14…18 vs 0. Two changes did
  not fix it:
  - The AWB grey-line intercept moves the wall along a diagonal: −0.10 gives Δa −3 and Δb +3.
    So c2a, c2b and c2c are not keepers.
  - A CSI→UGREEN colour fit (`fitccm.py`) asks for a fixed red cut of 0.53, which is not
    plausible. The Raspberry Pi calibrations (`rpi/imx296.json` vs `rpi/imx477.json`) show
    nearly the same raw R/G for the two sensors; IMX296 B/G is about 15% lower. Part of that
    "cast" is the UGREEN's own rendering and the mixed light, so nothing has been baked in.
- **Driver units.** The vendor/frc971 driver writes Argus exposure as **lines** (14.815 µs each)
  and Argus gain as **0.1 dB** codes. Argus "1 ms max" = 14.8 ms real; "12.5×" = 10× real.
  Not changed.

## Next: the printed chart

`chart/chart_a4.pdf`:
- 24 ColorChecker sRGB patches, six of them greys;
- paper white;
- ArUco markers DICT_4X4_50, ids 0–3, in the corners;
- the geometry is in `chart/chart_geometry.json`.

To do:
1. Print it at A4 landscape, 100%, on matte paper, with no colour enhancement.
2. Hang it flat, so that both cameras see all four markers; about 30–50 cm from the CSI lens,
   near the middle of its view, evenly lit, without glare.
3. Write a chart comparison: detect the markers in both images, map the patch centres through
   the homography, sample the inner part of each patch.
   - Set the white balance from paper white and the greys.
   - Then the colour matrix (neutral-preserving), the AE target and the saturation.
4. Check in daylight before anything becomes the default.

## Files

| Path | What |
|---|---|
| `cycle.sh <cand>…` | Run with sudo. For each candidate: install `<cand>.isp`, restart nvargus-daemon, capture (`pair.sh`) at EV 0 and +0.5. At the end, put back `KEEP` (default `c1_black50`). |
| `pair.sh <name> [props]` | One IMX296 still (Argus, after 150 frames) and one UGREEN still (MJPEG 1920×1080, read only), at the same time, into `shots/`. |
| `regions.py`, `match.py`, `fitccm.py` | The comparisons: hand-picked regions; SIFT-matched patches (run in the l4t-ml-gpio container: host cv2 is broken by numpy 2); the colour-matrix fit. |
| `c1_black50.isp`, `c2a…`, `c2b…`, `c2c…` | Candidates. c1 is the keeper so far. |
| `shots/` | Captures of c1 and c2*. `shots_vendor_and_gst/`: vendor tuning, the wbmode, EV and saturation sweeps. `noise_study/`: the edge-enhancement, TNR and gain stills, and the overlay dts. |
