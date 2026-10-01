# tools/dev — what was used to build and attack l4t_checks.sh (2026-10-01)

Saved from the session scratchpad (/tmp, wiped at boot) just before the reboot into
R36.5.2. Nothing here is needed to run the upgrade, rollback or verify scripts.

| Path | What |
|---|---|
| `run_mutants.sh [filter]` | Mutation test for `../l4t_checks.sh`: 56 mutants, each removing or weakening one rule; all must be killed by `../test_l4t_checks.sh`. Run after changing either file. |
| `harness/mutate.py` | Used by `run_mutants.sh`: replaces one text in a copy of a file. |
| `harness/stubs.sh` | For a dry run of `--apply` without root: copy a script with `BACKUP_DIR` pointed somewhere writable, then `bash -c ". ./stubs.sh; bash ./copy.sh --apply"`. Stubs `id -u` (0), `systemctl is-active` (inactive), `apt-get` (except `-s`), `install`, `depmod`, `sed -i` and `apt-mark hold`. The checks themselves run for real, read-only. |
| `harness/shim/apt-mark` | Put first on PATH to make `apt-mark showhold` drop `nvidia-l4t-kernel` and add an unrelated package (the count stays 65): the evening review's hold scenario. |
| `launcher_src/<tag>/L4TLauncher.{c,h}` | NVIDIA edk2-nvidia UEFI launcher source at tag r36.5, r36.5.1-updates and r36.5-updates (identical); `ProcessExtLinuxConfig`/`CleanExtLinuxLine`/`CheckCommandString` are what `l4t_checks.sh` copies for extlinux.conf. |
| `launcher_src/launcher_emu.py` | The afternoon attacker's Python port of that parser (and `ExtLinuxBoot`), used to show which entry/overlays the launcher would really boot. |
| `initrd_tools/drop.py` | `drop.py <initrd> <out> <regex>`: re-packs an initrd without the entries whose names match (how the evening review's missing-directory and missing-/init cases were made). |
| `initrd_tools/newc.py`, `parse_newc.py`, `strict_probe.sh`, `nv-update-initrd.sim` | The afternoon attacker's newc writer/parser, its strict probe, and the redirected copy of `nv-update-initrd` it ran under fakeroot to build a 36.5.2 initrd before the upgrade. |
| `notes/handover_1001.md`, `notes/upgrade_review.txt` | Drafts: the HANDOVER section of 2026-10-01 morning, and the first review of the upgrade script. |

The kernel's own `init/initramfs.c` (what `initrd_check` models) is in
`~/Downloads/r36.5/public_sources/Linux_for_Tegra/source/kernel_src/kernel/kernel-jammy-src/init/`.
The review workflows' transcripts are under `~/.claude/projects/-home-jetson-Documents-github-repos-dashcam/`.
