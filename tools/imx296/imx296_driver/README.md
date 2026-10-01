# imx296.ko for any L4T kernel, from the vendor's own source

The IMX296 colour camera (CAM-IMX296RAW, InnoMaker, `github.com/INNO-MAKER/cam-imx296raw-trigger`) ships its Jetson
driver only as prebuilt modules for 5.15.148 and 5.15.185. A module loads only on the kernel it was built for. This
folder rebuilds the same driver for any kernel. On 2026-10-01 it reproduced the vendor's working 5.15.185 module
exactly, and built a 5.15.199 one (L4T R36.5.2).

## Inputs (all in `src/` and `build_src/`)

| File | Where it comes from | sha256 |
|---|---|---|
| `src/imx296_source_working_20260615.tar.gz` | The vendor's source package. Published in their repo in commit `523d689` (2026-06-16 09:37 +0800) and deleted the same day in `92143cf`. Recovered from the local clone with `git show 523d689:1-1jetson_orin_nano_driver/imx296_source_working_20260615.tar.gz` | `e4e86894c1808b9c…5b263` |
| `src/imx296_mode_tbls.h` | FRC team 971's open driver, the vendor's upstream: `github.com/frc971/jetson-orin-kernel-builder`, `patches/imx296_mode_tbls.h`. The vendor's tarball lacks this header, which `imx296.c` includes. | `d27ae376b7130924…4fab3b` |
| `src/imx296_vendor_v1.1_5.15.185.ko` | The vendor's v1.1 prebuilt, as installed and working on this rig. Kept as the reference for the identity check. | `7061c58f12dd1b62…cee172` |
| `build_src/imx296.c` | The tarball's `source/imx296.c` plus `build_src/v1.1.patch` and `build_src/build-fix.patch` | — |

The two patches:
- **`v1.1.patch`** makes the 06-15 source match the vendor's later v1.1 binary.
  - The mono black level becomes `0x03c` instead of `0x070`.
  - The probe line becomes `found IMX%uL%c`, with `L` for mono and `Q` for colour.
  - Neither touches the colour path.
- **`build-fix.patch`** drops `#include "../platform/tegra/camera/camera_gpio.h"`. That path exists only inside the
  nvidia-oot source tree, and nothing from it is used: the driver calls only `<linux/gpio.h>`.

## Build

```bash
./build.sh                    # for the running kernel, from its installed headers
./build.sh 5.15.199-tegra     # from extracted header debs in headers-5.15.199-tegra/
```

For a kernel that is not installed, extract its two header debs first. No root is needed:
```bash
cd debs && apt-get download nvidia-l4t-kernel-headers=<ver> nvidia-l4t-kernel-oot-headers=<ver>
mkdir -p ../headers-<kver> && for d in *<kver>*.deb; do dpkg-deb -x $d ../headers-<kver>; done
```

`build.sh` refuses to report success unless every symbol the module imports carries the CRC that the target
kernel exports. It then compares the module's `.text`, `.rodata`, `.rodata.str1.8` and `.data` with the vendor's
5.15.185 build.

## Checked on 2026-10-01

| | Built for 5.15.185-tegra (R36.5.0) | Built for 5.15.199-tegra (R36.5.2) |
|---|---|---|
| vermagic | `5.15.185-tegra SMP preempt mod_unload modversions aarch64` | `5.15.199-tegra SMP preempt mod_unload modversions aarch64` |
| Symbol CRCs | 41/41 match the installed kernel and nvidia-oot | 41/41 match the 36.5.2 `Module.symvers` (kernel and nvidia-oot) |
| Against the vendor's 5.15.185 prebuilt | code, data, strings, `.modinfo` and symbol sizes byte-identical | code, data and strings byte-identical |

Other facts checked for 36.5.2:
- The nvidia-oot `include/` tree (tegracam, camera_common) is byte-identical between 36.5.0 and 36.5.2.
- The base DTB `tegra234-p3768-0000+p3767-0005-nv-super.dtb` is byte-identical too, and `imx296-cam1.dtbo` applies
  to the 36.5.2 one with `fdtoverlay`.
- The vendor's 5.15.185 prebuilt cannot be forced onto 5.15.199. Besides the vermagic mismatch, 32 of its 40
  imported symbols changed CRC (every tegracam_* and camera_common_* one).

**Run on hardware 2026-10-01:** after the upgrade (`../l4t_upgrade_36_5_2.sh`), the 5.15.199 module loads on the
5.15.199 kernel and the IMX296 streams through Argus: `verify_platform.sh` 24 pass / 0 fail / 1 skip, `csi_test`
5/5, CSI RTP and Camera_GST pass.

## Notes

- **Every kernel change needs a rebuild.** Even a point bump like .185 → .199 changes the symbol CRCs. No package
  owns `imx296.ko`, so apt never replaces it: copy the new build into `/lib/modules/<kver>/kernel/drivers/media/i2c/`
  and run `depmod -a <kver>`.
- **Stock `tegra-camera.ko` is right for this colour sensor.** The vendor's Y10-patched `tegra-camera.ko` is only for
  the mono IMX296LLR, and its source was never published.
- **Licence.** The driver is GPL v2 (file header and `MODULE_LICENSE`). The frc971 repository is MIT.
