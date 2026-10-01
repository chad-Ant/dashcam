#!/bin/bash
#
# build.sh — build the IMX296 tegracam sensor driver (imx296.ko) for a Jetson
# L4T kernel, from the vendor's own source.  See README.md for where every input
# comes from and how the result was checked.
#
# Usage:
#   ./build.sh                 build for the RUNNING kernel (its installed headers)
#   ./build.sh 5.15.199-tegra  build for that kernel from extracted header debs in
#                              headers-<kver>/ (download them first, see README.md)
# Output: out-<kver>/imx296.ko
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KVER="${1:-$(uname -r)}"

if [ "$KVER" = "$(uname -r)" ] && [ -d "/lib/modules/$KVER/build" ]; then
    KDIR="/lib/modules/$KVER/build"
    NVOOT="/usr/src/nvidia/nvidia-oot"
else
    H="$HERE/headers-$KVER"
    KDIR="$(ls -d "$H"/usr/src/linux-headers-"$KVER"-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source 2>/dev/null || true)"
    NVOOT="$H/usr/src/nvidia/nvidia-oot"
fi
[ -n "$KDIR" ] && [ -d "$KDIR" ]          || { echo "no kernel headers for $KVER (see README.md)" >&2; exit 1; }
[ -f "$NVOOT/Module.symvers" ]            || { echo "no nvidia-oot Module.symvers under $NVOOT" >&2; exit 1; }

OUT="$HERE/out-$KVER"
rm -rf "$OUT"; mkdir -p "$OUT"
cp "$HERE"/build_src/{imx296.c,imx296_mode_tbls.h,Makefile} "$OUT"/
make -C "$KDIR" M="$OUT" NVOOT="$NVOOT" KBUILD_EXTRA_SYMBOLS="$NVOOT/Module.symvers" modules

echo
echo "built: $OUT/imx296.ko"
modinfo -F vermagic "$OUT/imx296.ko"

# Every symbol the module imports must have the CRC the target kernel exports.
SYMS="$(mktemp)"; trap 'rm -f "$SYMS"' EXIT
cat "$KDIR/Module.symvers" "$NVOOT/Module.symvers" | awk '{print $2, $1}' > "$SYMS"
bad=0; n=0
while read -r crc sym; do
    n=$((n + 1))
    want="$(awk -v s="$sym" '$1 == s {print $2; exit}' "$SYMS")"
    if [ -z "$want" ] || [ "$((crc))" != "$((want))" ]; then
        echo "CRC MISMATCH: $sym (module $crc, kernel ${want:-missing})"; bad=$((bad + 1))
    fi
done < <(/sbin/modprobe --dump-modversions "$OUT/imx296.ko")
[ "$bad" -eq 0 ] || { echo "$bad of $n symbol CRCs do not match $KVER: do not install" >&2; exit 2; }
echo "all $n symbol CRCs match $KVER"

# Code identity with the vendor's prebuilt (InnoMaker v1.1, built for 5.15.185),
# kept in src/: the sections that carry code and data must be byte-identical
# (whole files differ only in ELF layout, which follows the build path).
REF="$HERE/src/imx296_vendor_v1.1_5.15.185.ko"
if [ -f "$REF" ]; then
    T1="$(mktemp)"; T2="$(mktemp)"; same=1
    for sec in .text .rodata .rodata.str1.8 .data; do
        objcopy -O binary --only-section="$sec" "$REF" "$T1"
        objcopy -O binary --only-section="$sec" "$OUT/imx296.ko" "$T2"
        cmp -s "$T1" "$T2" || { echo "$sec differs from the vendor build"; same=0; }
    done
    rm -f "$T1" "$T2"
    [ "$same" -eq 1 ] && echo "code and data identical to the vendor's 5.15.185 build"
fi
