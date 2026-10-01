## Done 2026-10-01 (Jetson coder): the IMX296 driver rebuilt from source, and tools to move between R36.5.0 and R36.5.2

**Why.** On R36.5.0 everything works, but the platform is held one release back, and the QSPI bootloader is already
36.5.2. UEFI refused the 36.5.0 capsule: `fwupdmgr` reports `Current version 2360578`, `Minimum Version 2360578`,
which is 36.5.2. So the question was whether the IMX296 can run on R36.5.2 (kernel 5.15.199). It can: the driver
rebuilds for any kernel from the vendor's own source. **Nothing below has been applied yet.** The machine is still on
R36.5.0 as of 2026-09-30, and the upgrade is the user's call.

**The driver source.**
- InnoMaker (`github.com/INNO-MAKER/cam-imx296raw-trigger`, cloned at `~/Downloads/IMX296_driver/cam-imx296raw-trigger`)
  publishes only prebuilt modules. At HEAD (`69a6afc`, 2026-07-05), the 5.15.185 folder holds only a file named
  `contact sales@inno-maker.com for the driver.txt`. There is nothing for 5.15.199.
- Their source package `imx296_source_working_20260615.tar.gz` was added in `523d689` (2026-06-16 09:37 +0800) and
  deleted in `92143cf` four hours later. It is recoverable from the clone's history (`git show 523d689:…`).
- It lacks `imx296_mode_tbls.h`. That header comes from FRC team 971's open IMX296 tegracam driver
  (`github.com/frc971/jetson-orin-kernel-builder`, `patches/`), which the vendor's code derives from (per the
  research; the vendor pointed to it in their issue #1).
- Two small patches turn the 06-15 source into the vendor's v1.1 binary. One sets the mono black level to `0x03c`;
  the other changes the probe line to `found IMX%uL%c`. Neither touches the colour path. A build fix drops an
  include of `camera_gpio.h`, which nothing uses.

**Verified on the Jetson.**

| Check | Result |
|---|---|
| Built for 5.15.185, against the vendor's working v1.1 module | `.text`, `.rodata`, `.rodata.str1.8`, `.data`, `.modinfo` and symbol sizes **byte-identical**; 41/41 symbol CRCs identical |
| Built for 5.15.199, from the 36.5.2 `nvidia-l4t-kernel-headers` and `-oot-headers` (downloaded, not installed) | vermagic `5.15.199-tegra`; **41/41 CRCs match** the 36.5.2 kernel and nvidia-oot `Module.symvers`; code and data identical to the vendor module |
| nvidia-oot `include/` (tegracam, camera_common), 36.5.0 against 36.5.2 | byte-identical |
| Base DTB `tegra234-p3768-0000+p3767-0005-nv-super.dtb`, 36.5.0 against 36.5.2 | byte-identical; `imx296-cam1.dtbo` applies to the 36.5.2 one (`fdtoverlay`) |
| The vendor's 5.15.185 prebuilt, forced onto 5.15.199 | not possible: besides the vermagic, 32 of its 40 imported symbols changed CRC, including every tegracam_* and camera_common_* (research) |

**Not yet run:** loading the 5.15.199 module on a 5.15.199 kernel. That is step 4 of the procedure below.

**The tools.** They live outside the repo, in `~/drive_logs/tools/` on the Jetson. A cloud session cannot see them.

| Tool | What it does |
|---|---|
| `imx296_driver/` | The rebuild. `src/` holds the recovered vendor tarball (sha256 `e4e86894…`), the frc971 header (`d27ae376…`) and the vendor's 5.15.185 prebuilt as the reference (`7061c58f…`). `build_src/` holds the patched `imx296.c`, `v1.1.patch`, `build-fix.patch` and the Makefile. `README.md` records the provenance and the checks. |
| `imx296_driver/build.sh [kver]` | Builds `out-<kver>/imx296.ko`, for the running kernel or from extracted header debs in `headers-<kver>/`. It **fails on any symbol-CRC mismatch**, then compares the code with the vendor's prebuilt. Run it at every kernel change: no package owns `imx296.ko`. |
| `l4t_upgrade_36_5_2.sh` | R36.5.0 → R36.5.2. Modes: `--list`, `--check`, `--simulate`, `--apply` (root). Details below. |
| `l4t_rollback_36_5_0.sh` | R36.5.2 → R36.5.0, the state verified on 2026-09-30. Same modes and the same checks. Rewritten on 2026-10-01: the 09-30 version could no longer run, because the apt history had rotated and the packages are held. |
| `l4t_36_5_2.pairs`, `l4t_36_5_0.pairs` | The 65 `package=version` pairs for each direction, frozen on 2026-10-01, because `/var/log/apt/history.log` rotates. The 36.5.2 list equals the `dpkg -l` that the rollback backed up before running. The 36.5.0 list equals what is installed now. |
| `verify_platform.sh [--quick]` | The test after any platform change. It changes nothing and stops no service. |

What `l4t_upgrade_36_5_2.sh --apply` does:
1. Installs the 65 packages with `--allow-change-held-packages`. The holds stay.
2. Puts the 5.15.199 `imx296.ko` in place *before* apt, then runs `depmod`.
3. Restores `DEFAULT JetsonIO`, which the kernel package resets to `primary`.

Before it says "All checks passed", it checks:
- that `/boot/Image` is 5.15.199;
- the USB, UVC, CDC, imx296 and tegra-camera modules, and that `modprobe -n` resolves imx296;
- that the initrd carries nvme, nvme-core, pcie-tegra194 and phy-p2u for 5.15.199 (the root file system is on NVMe,
  and these are modules);
- that `dpkg --audit` is clean;
- `DEFAULT JetsonIO` and the cam1 overlay;
- the holds.

Any failure ends with "do NOT reboot". Re-running it is safe.

`--check` reports `ready`, and `--simulate` shows 65 Inst, 65 Conf and 0 Remv.

What `verify_platform.sh` covers:
- **Platform:** kernel, release and UEFI; that the packages are held and at one release; the boot entry and overlay;
  imx296 built for the running kernel, loaded and probed, with no I2C or capture errors; Argus and the reload unit;
  both cameras.
- **Tests, in the dev container:** `csi_test`, `csi_rtp_test`, `camera_gst_test` (with the CSI dictionary against
  live Argus), `scan_cameras`, `usb_test` with a control diff (skipped while `dashcam-v04` runs), CUDA and TensorRT.
- **Output:** logs to `~/drive_logs/platform_checks/<timestamp>/`.
- **Baseline on R36.5.0 (2026-10-01):** 23 passed, 0 failed (`platform_checks/20261001-081029`). TensorRT 14.1 ms
  mean.

**Review.** A workflow reviewed the upgrade script adversarially against the real 36.5.2 maintainer scripts. Every
finding went to a skeptic. Fixed:
- a failure mid-apt could leave an initrd without NVMe, and the old script did not say "do not reboot";
- a re-run after a partial run (an SSH drop, a dpkg error) silently skipped the driver and boot-entry steps;
- the rollback could not run.

Documented:
- the first boot re-applies the same-version 36.5.2 capsule, which takes a few minutes plus one extra automatic
  restart;
- after the upgrade there is no 5.15.185 kernel to fall back to.

Checked fine:
- hold marks survive;
- no conffile prompts;
- nothing ships its own imx296 that would override this one;
- `nv-update-extlinux` keeps the JetsonIO entry and its FDT/OVERLAYS lines;
- the A_kernel fallback holds a bootable stock 5.15.199 (without the camera overlay).

**Upgrade procedure** (the user, on the bench, with HDMI and a keyboard or the serial console attached):
1. `bash ~/drive_logs/tools/l4t_upgrade_36_5_2.sh --check`, then `--simulate`.
2. `sudo systemctl stop dashcam-v04`. Then run the apply at the console, or with nohup over SSH, because tmux is not
   installed:
   `sudo -v && sudo nohup bash ~/drive_logs/tools/l4t_upgrade_36_5_2.sh --apply > ~/l4t_upgrade.log 2>&1 &`. The log
   must end with `All checks passed.`
3. `sudo reboot` on bench power. The first boot writes the capsule and restarts once by itself; do not cut power. If
   it hangs at the camera driver, choose "primary kernel" in the boot menu, which boots without the overlay.
4. `bash ~/drive_logs/tools/verify_platform.sh`. Expect kernel 5.15.199, R36.5.2, UEFI 36.5.2, `Detected IMX296LQ`,
   `csi_test` 5/5, both cameras, CUDA and TensorRT.
5. If the camera fails: `l4t_rollback_36_5_0.sh --check`, `--simulate`, `sudo bash … --apply`, reboot, then
   `verify_platform.sh`. The bootloader stays 36.5.2 either way.

**Licence.** The driver is GPL v2 (file header and `MODULE_LICENSE`). The frc971 repository is MIT. Keeping
`imx296_driver/` in this repo would let a cloud session see it, and turn future kernel updates into a routine rebuild.
That is the user's call.

