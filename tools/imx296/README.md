# IMX296 tools: driver rebuild, L4T upgrade/rollback, ISP tuning

A snapshot, taken 2026-10-02, of the tools behind the IMX296 CSI camera work in `HANDOVER.md` (branch `camera`).
They were written and run on the Jetson in `~/drive_logs/tools/`, which stays the working copy. Paths in messages
and docs (`~/drive_logs/tools/...`) refer to it. The layout here mirrors it: the upgrade script expects
`imx296_driver/out-<kernel>/imx296.ko` next to itself.

| Path | What |
|---|---|
| `imx296_driver/` | Rebuilds the vendor's IMX296 driver for any L4T kernel from their own (briefly published) source plus FRC 971's mode table. Reproduces the vendor's 5.15.185 module byte-identically in code, data and strings. The 5.15.199 build runs on R36.5.2. `README.md` has the provenance and checks. Includes the two built `imx296.ko` files the upgrade/rollback install. |
| `l4t_upgrade_36_5_2.sh`, `l4t_rollback_36_5_0.sh` | Move the rig between L4T R36.5.0 and R36.5.2 (65 NVIDIA packages, `*.pairs`), installing the matching `imx296.ko`. Modes: `--list`, `--check`, `--simulate`, `--apply`. Both re-hold the packages after apt, because `--allow-change-held-packages` clears holds. |
| `l4t_checks.sh`, `test_l4t_checks.sh` | The pre-reboot checks both scripts share: the extlinux default entry read the way NVIDIA's UEFI launcher reads it, the initrd unpacked the way the kernel does, and the package holds. Covered by a 141-check test. |
| `verify_platform.sh` | Test after any platform change. It changes nothing. |
| `dev/` | How the checks were attacked: 56 mutants (`run_mutants.sh`), the dry-run harness, initrd tools, NVIDIA's L4TLauncher source and a Python port of its parser. See `dev/README.md`. |
| `isp_tuning/` | Argus ISP override tuning against a printed ColorChecker/ArUco chart and a UGREEN USB reference. See `isp_tuning/README.md`, read top-down; the history is in `HANDOVER.md`. |

What `isp_tuning/` holds:
- **Profiles:** `c1`–`c8` `.isp` files. `c5_rpi100T` is installed; `c8_sh15` passed the sharpening A/B.
- **Capture tools:** `tune_session.py`, plus `trial_ab.sh`, the reviewed A-B-A trial that restores on every exit.
- **Analysis:** `chartcmp.py`, `chartsharp.py`, `ab_report.py`, and `csi_decode.py`, which decodes the nvjpegenc
  stills' limited range correctly.
- **The chart:** `chart/`.
- **Results:** the small JSON and log files of each session and trial.

## Prerequisites: NVIDIA material that is not in this repo

These are NVIDIA's, redistributable only under NVIDIA's terms. Fetch them from NVIDIA as shown.

**1. Header and DTB packages, to build `imx296.ko` for a kernel that is not installed.** Only needed when building
ahead of an upgrade: `build.sh` with no argument uses the running kernel's installed headers. They come from
NVIDIA's L4T apt repository, which a JetPack 6 system already has configured
(`repo.download.nvidia.com/jetson`, `r36.5`). These are the exact versions used on 2026-10-01:

| Package (`=version`) | sha256 of the .deb | Used for |
|---|---|---|
| `nvidia-l4t-kernel-headers=5.15.199-tegra-36.5.2-20260716114719` | `7a9f3bb7f118439d1257aa71dbe50f97cfd81dca6b80dae839a820fbf3dc48b8` | kernel headers for `build.sh 5.15.199-tegra` |
| `nvidia-l4t-kernel-oot-headers=5.15.199-tegra-36.5.2-20260716114719` | `fab664727b2d5542edc54664003c8321bf11d75c2e780aa75ff16b5b1515939c` | nvidia-oot headers (tegracam, camera_common) and `Module.symvers` |
| `nvidia-l4t-kernel-dtbs=5.15.199-tegra-36.5.2-20260716114719` | `007e10ea1bef5ff3dabf30adfa3e483db1d9146e007d0e87592ebcf715470bfc` | checking that the cam1 overlay applies to the 36.5.2 base DTB (`fdtoverlay`) |
| `nvidia-l4t-jetson-io=36.5.2-20260716114719` | `46b9dcaaa4d774134bf154a1a690fbb238608c09f5030e0f10de48d0f3b1dab1` | inspected before the upgrade: how jetson-io writes the JetsonIO extlinux entry |

```bash
cd imx296_driver && mkdir -p debs && cd debs
apt-get download nvidia-l4t-kernel-headers=5.15.199-tegra-36.5.2-20260716114719 \
                 nvidia-l4t-kernel-oot-headers=5.15.199-tegra-36.5.2-20260716114719 \
                 nvidia-l4t-kernel-dtbs=5.15.199-tegra-36.5.2-20260716114719
sha256sum *.deb                                   # compare with the table
mkdir -p ../headers-5.15.199-tegra ../dtbs-5.15.199
for d in nvidia-l4t-kernel-*headers_*.deb; do dpkg-deb -x "$d" ../headers-5.15.199-tegra; done
dpkg-deb -x nvidia-l4t-kernel-dtbs_*.deb ../dtbs-5.15.199
cd .. && ./build.sh 5.15.199-tegra                # -> out-5.15.199-tegra/imx296.ko
```
`imx296_driver/.gitignore` keeps `debs/`, `headers-*/`, `dtbs-*/` and the build intermediates out of git.

**2. The 65 L4T packages themselves**, for the upgrade or rollback. `--apply` installs them from the same apt
repository; `l4t_36_5_2.pairs` and `l4t_36_5_0.pairs` pin their versions. R36.5.0 must still be offered by the
repository for a rollback.

**3. Jetson Linux R36.5 public sources** (`public_sources.tbz2` from NVIDIA's Jetson Linux R36.5 download page;
sha256 starts `d0acb2187786d6ba9f01`). Two files in it were references:
- `kernel/kernel-jammy-src/init/initramfs.c`: the unpacking rules that `l4t_checks.sh`'s `initrd_check` models;
- `gst-nvarguscamera_src.tbz2`: nvarguscamerasrc applies `saturation`, `ee-mode`, `ee-strength` and `tnr-*` only
  when the property is set.

**4. NVIDIA's closed camera libraries on the Jetson** (part of L4T, e.g.
`/usr/lib/aarch64-linux-gnu/nvidia/libnvscf.so`, `libnvargus.so`, `libnvodm_imager.so`).
- The ISP findings were established on the device. Examples: `colorCorrection.srgbMatrix[i]` is a column;
  `sharpness.v2` values are indices, lower = stronger; `sharpness.v2` and `sharpness.v5.tab` are one table;
  `MaxValue` is discarded.
- The conclusions are in `isp_tuning/README.md` and `HANDOVER.md`.
- The raw analysis is not published.

**5. Not NVIDIA, also needed.**
- **The analysis container.** `l4t-ml-gpio`, built by `docker_dev/build.sh` from `dustynv/l4t-ml:r36.4.0`. The
  host's OpenCV is broken by numpy 2, so the analysis runs in it. It needs OpenCV with ArUco, numpy and Pillow.
- **Host tools:**
  - GStreamer with `nvarguscamerasrc` and `nvjpegenc`, and `python3-gi`, for `tune_session.py`;
  - `v4l-utils`;
  - `device-tree-compiler` (`fdtoverlay`);
  - `cpio`, `gzip` and `python3`, for the checks.
- **Hardware:**
  - the InnoMaker CAM-IMX296RAW (colour) in CAM1;
  - the UGREEN USB camera, at the by-id path in `tune_session.py`;
  - the chart `isp_tuning/chart/chart_a4.pdf`, printed at 100 % on matte paper.

## Kept on the Jetson only

- **Camera captures.** The JPEGs under `isp_tuning/shots*/`, `noise_study/`, `session_*/` and `trial_*/`, about
  240 MB: photos of the room. Their JSON and log results are here.
- **`isp_tuning/reviews_20261001/`.** The agent reviews' raw outputs (they quote disassembly of NVIDIA's
  `libnvscf`), the pre-fix copies of the analysis scripts, and superseded profiles. Only the chart crops and
  `make_c8.py` are here.
- **Build intermediates** in `imx296_driver/out-*/`. The `.ko` files are here.

## Licences

- **`imx296_driver/src`, `build_src`, `out-*`:** GPL-2.0. NVIDIA's tegracam template, modified by InnoMaker and
  FRC team 971 (the frc971 repository itself is MIT); see the file headers and `MODULE_LICENSE`.
- **`dev/launcher_src/`:** NVIDIA edk2-nvidia, BSD-2-Clause-Patent.
- **`dev/initrd_tools/nv-update-initrd.sim`:** NVIDIA, BSD-3-Clause; a redirected copy for a dry run.
- **`isp_tuning/rpi/*.json`:** Raspberry Pi camera tuning files from libcamera.
- **`isp_tuning/vendor_v1.1.isp`:** InnoMaker's ISP override as published in their repository, derived from
  NVIDIA's IMX477 tuning. The `c*.isp` profiles are edits of it.
- **Everything else:** this repository's MIT licence.
