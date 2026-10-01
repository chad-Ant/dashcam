#!/bin/bash
#
# test_l4t_checks.sh — runs the checks in l4t_checks.sh against a good boot setup
# and against broken ones built in a temporary directory, so that a check which
# stops catching a fault shows up here before it matters on a reboot.
#
# Each case says whether the check must pass or fail, and a failing case also
# names the problem it must report (-m), so that it cannot fail for some other
# reason and hide a rule that no longer works.  For the faults the 2026-10-01
# reviews found, it also runs the checks they replaced, and requires that the
# old check really did accept the fault (the case then tests the fix).  The last
# cases run the checks on this machine's own /boot (read-only).
#
# Usage:  bash test_l4t_checks.sh        (no root; changes nothing outside a temp dir)
# Needs:  python3, dtc and fdtoverlay (device-tree-compiler), gzip, cpio.
#
# Written 2026-10-01 by Claude; extended the same day after the adversarial
# review (launcher-style extlinux parsing, DTB content, kernel-style initrd).

set -uo pipefail

HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"   # also when run through a symlink
# shellcheck source=l4t_checks.sh
. "$HERE/l4t_checks.sh" || { echo "cannot load $HERE/l4t_checks.sh"; exit 1; }
for t in python3 dtc fdtoverlay gzip cpio; do command -v "$t" > /dev/null || { echo "needs $t"; exit 1; }; done

T="$(mktemp -d)" || { echo "mktemp failed"; exit 1; }
trap 'rm -rf "${T:?}"' EXIT
NPASS=0; NFAIL=0
ok()  { echo "PASS  $*"; NPASS=$((NPASS + 1)); }
bad() { echo "FAIL  $*"; NFAIL=$((NFAIL + 1)); }

# expect <pass|fail> <case name> [-m <text the failure must report>] <command...>
expect() {
    local want="$1" name="$2" must="" out rc; shift 2
    if [ "${1:-}" = -m ]; then must="$2"; shift 2; fi
    out="$("$@" 2>&1)"; rc=$?
    if [ "$want" = pass ] && [ "$rc" -eq 0 ]; then
        ok "$name: passes (${out%%$'\n'*})"
    elif [ "$want" = fail ] && [ "$rc" -ne 0 ] && [[ "$out" == *"$must"* ]]; then
        ok "$name: fails (${out%%$'\n'*})"
    elif [ "$want" = fail ] && [ "$rc" -ne 0 ]; then
        bad "$name: fails, but without '$must': ${out//$'\n'/; }"
    else
        bad "$name: should ${want}, rc $rc: ${out//$'\n'/; }"
    fi
}

# The checks the morning's fix replaced, verbatim apart from the arguments.
old_overlay_ok() { awk '/^LABEL JetsonIO/{f=1} f && /OVERLAYS/{print; exit}' "$1" | grep -q "$2"; }
old_initrd_ok() {   # <initrd> <kver> <module>...
    local lst k i="$1" kv="$2"; shift 2
    lst="$(zcat "$i" 2>/dev/null | cpio -t --quiet 2>/dev/null || true)"
    for k in "$@"; do grep -q "lib/modules/$kv/kernel/drivers/$k" <<< "$lst" || return 1; done
}
# old_accepted <case> <command...>: the case only tests the fix if the old check passed it
old_accepted() {
    local name="$1"; shift
    if "$@" > /dev/null 2>&1; then ok "$name: the old check accepted it"; else bad "$name: the old check already rejected it, so this case does not test the fix"; fi
}

OVL="tegra234-p3767-camera-p3768-imx296-cam1.dtbo"
OTHERCAM="tegra234-p3767-camera-p3768-imx219-C.dtbo"
PU="11111111-2222-3333-4444-555555555555"

# --- the boot entry ----------------------------------------------------------
# A root with real device-tree blobs: the overlays apply to the base DTB.
R="$T/root"
mkdir -p "$R/boot/dtb" "$T/dts"
for f in Image Image.backup initrd initrd.img; do head -c 4096 /dev/urandom > "$R/boot/$f"; done
printf '/dts-v1/;\n/ { compatible = "test,board"; cam { status = "disabled"; }; };\n' > "$T/dts/base.dts"
ovl_dts() { printf '/dts-v1/;\n/plugin/;\n/ { fragment@0 { target-path = "%s"; __overlay__ { %s; }; }; };\n' "$1" "$2"; }
ovl_dts /cam 'status = "okay"' > "$T/dts/cam.dts"
ovl_dts / 'test-prop = <1>' > "$T/dts/other.dts"
ovl_dts /nonexistent 'status = "okay"' > "$T/dts/badtarget.dts"
dtc -q -I dts -O dtb -o "$R/boot/dtb/kernel_board.dtb" "$T/dts/base.dts"
dtc -q -I dts -O dtb -o "$R/boot/$OVL" "$T/dts/cam.dts"
cp "$R/boot/$OVL" "$R/boot/$OVL.bak"
cp "$R/boot/$OVL" "$R/boot/$OTHERCAM"
dtc -q -I dts -O dtb -o "$R/boot/other.dtbo" "$T/dts/other.dts"
dtc -q -I dts -O dtb -o "$R/boot/badtarget.dtbo" "$T/dts/badtarget.dts"
head -c "$(stat -c %s "$R/boot/$OVL")" /dev/urandom > "$R/boot/garbage.dtbo"
head -c "$(stat -c %s "$R/boot/$OVL")" /dev/zero > "$R/boot/zeros.dtbo"
printf 'not a dtb\n' > "$R/boot/dtb/notadtb.dtb"
: > "$R/boot/empty.dtbo"
# a file on another file system (sysfs), reached through a symlink
SYSF=/sys/class/dmi/id/bios_version
[ -r "$SYSF" ] && ln -s "$SYSF" "$R/boot/elsewhere.dtbo"

# The layout of this machine's extlinux.conf (primary entry, commented backup
# entry, the JetsonIO entry jetson-io wrote, tab-indented).
good_conf() {
    printf '%s\n' 'TIMEOUT 30' 'DEFAULT JetsonIO' '' 'MENU TITLE L4T boot options' '' \
        'LABEL primary' '      MENU LABEL primary kernel' '      LINUX /boot/Image' '      INITRD /boot/initrd' \
        "      APPEND \${cbootargs} root=PARTUUID=$PU rw rootwait" '' \
        '# LABEL backup' '#    MENU LABEL backup kernel' '#    LINUX /boot/Image.backup' '#    INITRD /boot/initrd' '' \
        'LABEL JetsonIO' $'\tMENU LABEL Custom Header Config: <CSI Camera IMX296-C Cam1>' $'\tLINUX /boot/Image' \
        $'\tFDT /boot/dtb/kernel_board.dtb' $'\tINITRD /boot/initrd' $'\tAPPEND ${cbootargs} root=PARTUUID='"$PU"' rw rootwait' \
        $'\tOVERLAYS /boot/'"$OVL"
}
# the JetsonIO entry alone, under another label line
entry_block() { good_conf | sed -n '/^LABEL JetsonIO/,$p' | sed "1s/.*/$1/"; }
C="$T/conf"; mkdir -p "$C"
good_conf > "$C/good"
conf() { good_conf | sed "$1" > "$C/$2"; }   # conf <sed script> <name>: a variant of the good file
entry() { local w="$1" n="$2"; shift 2; expect "$w" "entry/$n" "$@" extlinux_entry_check "$C/$n" JetsonIO "$OVL" "$R" "$PU"; }
dflt()  { local w="$1" n="$2"; shift 2; expect "$w" "default/$n" "$@" extlinux_default_check "$C/$n" JetsonIO; }

entry pass good
dflt  pass good

# The morning review's three cases, each also shown to have passed the old check,
# and two more the old check accepted (a longer name, a label that only starts with JetsonIO).
sed '/^\tOVERLAYS/d' "$C/good" > "$C/overlay-in-other-entry"
printf '\nLABEL other\n\tLINUX /boot/Image\n\tOVERLAYS /boot/%s\n' "$OVL" >> "$C/overlay-in-other-entry"
conf 's/^\tOVERLAYS/\t# OVERLAYS/' overlay-commented
conf "s#OVERLAYS /boot/#OVERLAYS /boot/nonexistent/#" overlay-file-missing
conf "s#$OVL#$OVL.bak#" overlay-name-suffix
conf 's/^LABEL JetsonIO$/LABEL JetsonIO-old/' label-prefix
entry fail overlay-in-other-entry -m "0 OVERLAYS lines"
entry fail overlay-commented      -m "0 OVERLAYS lines"
entry fail overlay-file-missing   -m "does not exist"
entry fail overlay-name-suffix    -m "does not name $OVL"
entry fail label-prefix           -m "0 entries labelled exactly"
for c in overlay-in-other-entry overlay-commented overlay-file-missing overlay-name-suffix label-prefix; do
    old_accepted "entry/$c" old_overlay_ok "$C/$c" "$OVL"
done

# The afternoon review: lines the launcher reads differently from how they look.
sed '/^LABEL JetsonIO/,$ {/^\tAPPEND/d}' "$C/good" > "$C/append-missing"; entry fail append-missing -m "0 APPEND lines"
sed '/^LABEL JetsonIO/,$ s/^\tAPPEND/\tappend/' "$C/good" > "$C/append-lower-case"; entry fail append-lower-case -m "not a plain 'APPEND"
sed "/^LABEL JetsonIO/,\$ s/root=PARTUUID=$PU/root=PARTUUID=0/" "$C/good" > "$C/append-other-root"; entry fail append-other-root -m "not root=PARTUUID=$PU"
sed '/^LABEL JetsonIO/,$ s#rootwait#rootwait root=/dev/sda1#' "$C/good" > "$C/append-two-roots"; entry fail append-two-roots -m "not root=PARTUUID=$PU"
{ cat "$C/good"; printf 'label other\n\tLINUX /boot/Image.backup\n\tOVERLAYS /boot/other.dtbo\n'; } > "$C/lower-case-label-after"
entry fail lower-case-label-after -m "not a plain 'LABEL"
{ cat "$C/good"; printf '\tFDTOVERLAYS /boot/%s\n' "$OVL"; } > "$C/fdtoverlays-in-entry";  entry fail fdtoverlays-in-entry -m "not a plain 'FDT"
{ cat "$C/good"; printf '\tOVERLAYS/boot/other.dtbo\n'; } > "$C/overlays-no-space";       entry fail overlays-no-space -m "not a plain 'OVERLAYS"
{ good_conf | sed '/^LABEL JetsonIO/,$d'; entry_block 'LABEL JetsonIO # the old one'; good_conf | sed -n '/^LABEL JetsonIO/,$p'; } > "$C/hash-label-before"
entry fail hash-label-before -m "2 entries labelled exactly"
{ good_conf | sed '/^LABEL JetsonIO/,$d'; entry_block $'LABEL JetsonIO\r'; good_conf | sed -n '/^LABEL JetsonIO/,$p'; } > "$C/cr-label-before"
entry fail cr-label-before -m "2 entries labelled exactly"   # what the launcher sees (it drops CRs); "not plain ASCII" too
{ good_conf | sed '/^LABEL JetsonIO/,$d'; echo 'LABEL JetsonIO'; good_conf | sed -n '/^LABEL JetsonIO/,$p'; } > "$C/empty-label-before"
entry fail empty-label-before -m "2 entries labelled exactly"
{ good_conf | sed '/^LABEL JetsonIO/,$d'; for i in 1 2 3 4 5 6 7 8 9; do printf 'LABEL test%s\n\tLINUX /boot/Image\n' $i; done
  good_conf | sed -n '/^LABEL JetsonIO/,$p'; } > "$C/jetsonio-11th"
entry fail jetsonio-11th -m "0 entries labelled exactly"
{ good_conf | sed '/^LABEL JetsonIO/,$d'; for i in 1 2 3 4 5 6 7 8; do printf 'LABEL test%s\n\tLINUX /boot/Image\n' $i; done
  good_conf | sed -n '/^LABEL JetsonIO/,$p'; printf 'LABEL eleventh\n\tOVERLAYS /boot/other.dtbo\n'; } > "$C/jetsonio-10th-of-11"
entry fail jetsonio-10th-of-11 -m "11 LABEL lines"
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/other.dtbo\x00,/boot/$OVL#" nul-in-overlays;      entry fail nul-in-overlays -m "not plain ASCII"
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/other.dtbo,/../boot/$OVL#" dotdot-after-comma;  entry fail dotdot-after-comma -m "'..' or '//'"
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot//$OVL#" double-slash;                            entry fail double-slash -m "'..' or '//'"
if [ -L "$R/boot/elsewhere.dtbo" ]; then
    conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/$OVL,/boot/elsewhere.dtbo#" overlay-on-other-fs; entry fail overlay-on-other-fs -m "not on the file system"
fi
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/garbage.dtbo,/boot/$OVL#" overlay-garbage;      entry fail overlay-garbage -m "do not apply"
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/zeros.dtbo,/boot/$OVL#" overlay-zero-filled;    entry fail overlay-zero-filled -m "do not apply"
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/$OVL,/boot/badtarget.dtbo#" overlay-bad-target; entry fail overlay-bad-target -m "do not apply"
conf 's#FDT /boot/dtb/kernel_board.dtb#FDT /boot/dtb/notadtb.dtb#' fdt-not-a-dtb;         entry fail fdt-not-a-dtb -m "do not apply"
conf "s#^\tOVERLAYS .*#\tOVERLAYS /boot/$OVL,/boot/$OTHERCAM#" other-camera-overlay;     entry fail other-camera-overlay -m "another camera overlay"

# The rest of the entry rules, each case failing for its own reason.
{ cat "$C/good"; echo; good_conf | sed -n '/^LABEL JetsonIO/,$p'; } > "$C/entry-twice"; entry fail entry-twice -m "2 entries labelled exactly"
{ cat "$C/good"; printf '\tOVERLAYS /boot/other.dtbo\n'; } > "$C/overlays-twice";        entry fail overlays-twice -m "2 OVERLAYS lines"
conf "s#OVERLAYS .*#OVERLAYS /boot/other.dtbo,/boot/$OVL#" overlay-list;                  entry pass overlay-list
conf "s#OVERLAYS .*#OVERLAYS /boot/$OVL,/boot/missing.dtbo#" overlay-list-one-missing;     entry fail overlay-list-one-missing -m "does not exist"
conf "s#OVERLAYS .*#OVERLAYS /boot/other.dtbo, /boot/$OVL#" overlay-list-space;            entry fail overlay-list-space -m "has a space"
conf "s#OVERLAYS .*#OVERLAYS /boot/$OVL,#" overlay-list-trailing-comma;                    entry fail overlay-list-trailing-comma -m "empty item"
conf "s#OVERLAYS .*#OVERLAYS#" overlays-empty;                                             entry fail overlays-empty -m "empty item"
conf "s#OVERLAYS .*#OVERLAYS /boot/empty.dtbo,/boot/$OVL#" overlay-empty-file;             entry fail overlay-empty-file -m "does not exist or is empty"
conf "s#OVERLAYS /boot/#OVERLAYS boot/#" overlay-relative;                                 entry fail overlay-relative -m "not an absolute path"
conf "s#\(OVERLAYS .*\)#\1  \# cam1#" overlay-inline-comment;                              entry pass overlay-inline-comment   # the launcher cuts at '#' too
conf 's/^\tOVERLAYS/\toverlays/' keyword-lower-case;                                       entry fail keyword-lower-case -m "not a plain 'OVERLAYS"
conf '/^\tFDT /d' fdt-line-missing;                                                        entry fail fdt-line-missing -m "0 FDT lines"
conf 's#FDT /boot/dtb/kernel_board.dtb#FDT /boot/dtb/gone.dtb#' fdt-file-missing;          entry fail fdt-file-missing -m "does not exist"
conf 's#^\tFDT .*#&\n\tFDTDIR /boot/dtb#' fdtdir;                                          entry fail fdtdir -m "not a plain 'FDT"
conf '/^LABEL JetsonIO/,$ s#LINUX /boot/Image#LINUX /boot/Image.backup#' linux-other-image; entry fail linux-other-image -m "not /boot/Image"
conf '/^LABEL JetsonIO/,$ s#INITRD /boot/initrd#INITRD /boot/initrd.img#' initrd-other-file; entry fail initrd-other-file -m "not /boot/initrd"
conf 's/\t/    /' spaces-not-tabs;                                                          entry pass spaces-not-tabs
conf 's/$/\r/' crlf;                                                                       entry fail crlf -m "not plain ASCII"
{ printf 'OVERLAYS /boot/%s\n' "$OVL"; sed '/^\tOVERLAYS/d' "$C/good"; } > "$C/overlay-before-any-label"; entry fail overlay-before-any-label -m "0 OVERLAYS lines"
cp "$C/good" "$C/initrd-file-missing-on-disk"
mv "$R/boot/initrd" "$R/boot/initrd.moved"; entry fail initrd-file-missing-on-disk -m "does not exist"; mv "$R/boot/initrd.moved" "$R/boot/initrd"

conf 's/^DEFAULT JetsonIO$/DEFAULT primary/' default-primary;              dflt fail default-primary -m "DEFAULT is 'primary'"
conf 's/^DEFAULT JetsonIO$/DEFAULT primary\nDEFAULT JetsonIO/' default-twice; dflt fail default-twice -m "2 DEFAULT lines"
conf 's/^DEFAULT JetsonIO$/# DEFAULT JetsonIO\nDEFAULT primary/' default-commented-good; dflt fail default-commented-good -m "DEFAULT is 'primary'"
conf 's/^DEFAULT JetsonIO$/#DEFAULT JetsonIO/' default-only-commented;      dflt fail default-only-commented -m "0 DEFAULT lines"
conf 's/^DEFAULT JetsonIO$/default JetsonIO/' default-lower-case;           dflt fail default-lower-case -m "not a plain 'DEFAULT"
conf 's/^DEFAULT JetsonIO$/DEFAULT JetsonIO-old/' default-other-label;      dflt fail default-other-label -m "DEFAULT is 'JetsonIO-old'"
conf 's/^DEFAULT JetsonIO$/DEFAULT JetsonIO # the camera/' default-inline-comment; dflt pass default-inline-comment
conf 's/^DEFAULT JetsonIO$/   DEFAULT JetsonIO/' default-indented;          dflt pass default-indented

# --- the initrd --------------------------------------------------------------
KV="9.9.9-test"
MODS=(nvme/host/nvme.ko nvme/host/nvme-core.ko pci/controller/dwc/pcie-tegra194.ko phy/tegra/phy-tegra194-p2u.ko)
D="$T/installed/lib/modules/$KV/kernel/drivers"
S="$T/src"
for m in "${MODS[@]}"; do
    mkdir -p "$D/$(dirname "$m")" "$S/usr/lib/modules/$KV/kernel/drivers/$(dirname "$m")"
    head -c $((20000 + RANDOM)) /dev/urandom > "$D/$m"
    cp "$D/$m" "$S/usr/lib/modules/$KV/kernel/drivers/$m"
done
ln -s usr/lib "$S/lib"
mkdir -p "$S/usr/share"; head -c 2000000 /dev/urandom > "$S/usr/share/filler"   # incompressible, after the modules

# What /init needs to reach the root, in miniature: /init (#!/bin/bash), bash,
# the commands it runs, the ELF loader and libc.  The programs are stand-ins:
# ELF headers with the real loader path and DT_NEEDED entries, nothing to run.
cat > "$T/kit.py" <<'PY'
import os, stat, struct, sys
LD = "/lib/ld-linux-aarch64.so.1"
DIR, REG, LNK = 0o40755, 0o100755, 0o120777
def elf(interp=None, needed=()):
    strtab, offs = b"\0", []
    for n in needed:
        offs.append(len(strtab)); strtab += n.encode() + b"\0"
    nph = 1 + bool(interp) + bool(needed)
    ib = interp.encode() + b"\0" if interp else b""
    ioff = 64 + 56 * nph
    soff = ioff + len(ib)
    doff = (soff + len(strtab) + 7) & ~7
    dyn = b"".join(struct.pack("<qQ", 1, o) for o in offs) + struct.pack("<qQ", 5, soff) + struct.pack("<qQ", 0, 0)
    total = doff + (len(dyn) if needed else 0)
    ph = struct.pack("<IIQQQQQQ", 1, 5, 0, 0, 0, total, total, 0x1000)           # PT_LOAD: vaddr = offset
    if interp:
        ph += struct.pack("<IIQQQQQQ", 3, 4, ioff, ioff, ioff, len(ib), len(ib), 1)
    if needed:
        ph += struct.pack("<IIQQQQQQ", 2, 6, doff, doff, doff, len(dyn), len(dyn), 8)
    eh = b"\x7fELF" + bytes([2, 1, 1, 0]) + b"\0" * 8 + struct.pack(
        "<HHIQQQIHHHHHH", 3, 183, 1, 0, 64, 0, 0, 64, 56, nph, 64, 0, 0)
    body = eh + ph + ib + strtab
    body += b"\0" * (doff - len(body))
    return body + (dyn if needed else b"")
def entries(skip=(), init_mode=0o755, init_body=b"#!/bin/bash\nmount -t proc proc /proc\nexec chroot . /sbin/init 2\n",
            bash_cut=False):
    libc = ["libc.so.6"]
    e = [("bin", LNK, b"usr/bin"), ("sbin", LNK, b"usr/sbin"), ("init", 0o100000 | init_mode, init_body),
         ("usr/bin", DIR, b""), ("usr/sbin", DIR, b""), ("usr/lib/aarch64-linux-gnu", DIR, b""),
         ("usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1", REG, elf()),
         ("usr/lib/ld-linux-aarch64.so.1", LNK, b"aarch64-linux-gnu/ld-linux-aarch64.so.1"),
         ("usr/lib/aarch64-linux-gnu/libc.so.6", REG, elf(needed=["ld-linux-aarch64.so.1"])),
         ("usr/lib/aarch64-linux-gnu/libtinfo.so.6", 0o100644, elf(needed=libc)),
         ("usr/bin/bash", REG, elf(LD, ["libtinfo.so.6"] + libc)[:100 if bash_cut else None])]
    e += [("usr/bin/" + c, REG, elf(LD, libc)) for c in ("mount", "cat", "grep", "sed", "tail", "ln", "kmod", "sleep", "expr")]
    e += [("usr/sbin/chroot", REG, elf(LD, libc)), ("usr/sbin/modprobe", LNK, b"/bin/kmod")]
    return [x for x in e if x[0] not in skip]
if __name__ == "__main__":                     # kit.py <dir>: write it there
    for name, mode, body in entries():
        p = os.path.join(sys.argv[1], name)
        if stat.S_ISDIR(mode): os.makedirs(p, exist_ok=True)
        elif stat.S_ISLNK(mode): os.symlink(body.decode(), p)
        else:
            open(p, "wb").write(body); os.chmod(p, mode & 0o7777)
PY
python3 "$T/kit.py" "$S"
I="$T/initrd"; mkdir -p "$I"
# pack <src dir> <out> <file list on stdin>: what nv-update-initrd does, in a fixed order
pack() { (cd "$1" && cpio -H newc -o --quiet) | gzip -9 -n > "$2"; }
list_usr() { echo .; echo lib; echo bin; echo sbin; echo init; echo usr; find usr/lib usr/bin usr/sbin -print; echo usr/share; echo usr/share/filler; }
(cd "$S" && list_usr) | pack "$S" "$I/good"
initrd() { local w="$1" n="$2"; shift 2; expect "$w" "initrd/$n" "$@" initrd_check "$I/$n" "$KV" "$D" "${MODS[@]}"; }

initrd pass good
SZ=$(stat -c %s "$I/good")
for cut in 99 75 50; do head -c $((SZ * cut / 100)) "$I/good" > "$I/cut-to-$cut%"; done
head -c $((SZ - 1)) "$I/good" > "$I/minus-1-byte"
head -c $((SZ - 8)) "$I/good" > "$I/minus-gzip-trailer"
for c in cut-to-99% cut-to-75% cut-to-50% minus-1-byte minus-gzip-trailer; do
    initrd fail "$c" -m "cut off"; old_accepted "initrd/$c" old_initrd_ok "$I/$c" "$KV" "${MODS[@]}"
done
# invert one byte (not overwrite it: the byte is random, and could already be the new value)
cp "$I/good" "$I/byte-flipped"
B=$(od -An -tu1 -j $((SZ / 2)) -N1 "$I/good" | tr -d ' ')
printf "\\$(printf %03o $((B ^ 0xff)))" | dd of="$I/byte-flipped" bs=1 seek=$((SZ / 2)) conv=notrunc status=none
if cmp -s "$I/good" "$I/byte-flipped"; then bad "initrd/byte-flipped: the fixture did not change"; else initrd fail byte-flipped -m "does not decompress"; fi
{ cat "$I/good"; echo junk; } > "$I/trailing-garbage";                    initrd fail trailing-garbage -m "after its gzip stream"
{ cat "$I/good"; head -c 512 /dev/zero; } > "$I/trailing-zeros";          initrd pass trailing-zeros   # gzip and the kernel skip zero padding
(cd "$S" && list_usr) | (cd "$S" && cpio -H newc -o --quiet) > "$I/raw.cpio"
head -c $(($(stat -c %s "$I/raw.cpio") * 3 / 4)) "$I/raw.cpio" | gzip -9 -n > "$I/whole-gzip-of-cut-cpio"
initrd fail whole-gzip-of-cut-cpio -m "the cpio archive is cut off at offset"
old_accepted "initrd/whole-gzip-of-cut-cpio" old_initrd_ok "$I/whole-gzip-of-cut-cpio" "$KV" "${MODS[@]}"
cp "$I/raw.cpio" "$I/not-gzip";                                           initrd fail not-gzip -m "not a gzip file"
: > "$I/empty";                                                           initrd fail empty -m "does not exist or is empty"
expect fail "initrd/no-such-file" -m "does not exist" initrd_check "$I/no-such-file" "$KV" "$D" "${MODS[@]}"
(cd "$S" && list_usr | grep -v 'nvme-core.ko$') | pack "$S" "$I/module-missing"; initrd fail module-missing -m "lacks nvme/host/nvme-core.ko"
expect fail "initrd/other-kernel-version" -m "lacks" initrd_check "$I/good" "9.9.8-test" "$D" "${MODS[@]}"
# same size, other bytes
S2="$T/src2"; cp -a "$S" "$S2"
head -c "$(stat -c %s "$D/nvme/host/nvme.ko")" /dev/urandom > "$S2/usr/lib/modules/$KV/kernel/drivers/nvme/host/nvme.ko"
(cd "$S2" && list_usr) | pack "$S2" "$I/module-other-bytes";              initrd fail module-other-bytes -m "bytes differ"
# a symlink in place of the module
S3="$T/src3"; cp -a "$S" "$S3"
ln -sf nvme-core.ko "$S3/usr/lib/modules/$KV/kernel/drivers/nvme/host/nvme.ko"
(cd "$S3" && list_usr) | pack "$S3" "$I/module-is-symlink";               initrd fail module-is-symlink -m "bytes differ"   # modprobe would load nvme-core.ko
# lib/ as a real directory holding the modules and libraries; and nv-update-initrd's
# own "find . | cpio" order (GNU cpio drops the "./" from the names)
S4="$T/src4"; cp -a "$S" "$S4"; unlink "${S4:?}/lib"; mv "$S4/usr/lib" "$S4/lib"
(cd "$S4" && { echo .; find lib -print; echo bin; echo sbin; echo init; echo usr; find usr -print; }) | pack "$S4" "$I/lib-layout"; initrd pass lib-layout
(cd "$S" && find . -print) | pack "$S" "$I/find-dot-order";               initrd pass find-dot-order
# The 2026-10-01 evening review: a module whose parent directory is not in the
# archive (or comes after it) is never created; the old check only read names.
(cd "$S" && list_usr | grep -vxE "usr/lib/modules/$KV/kernel/drivers/nvme(/host)?") | pack "$S" "$I/module-parent-dirs-missing"
initrd fail module-parent-dirs-missing -m "would not create"
old_accepted "initrd/module-parent-dirs-missing" old_initrd_ok "$I/module-parent-dirs-missing" "$KV" "${MODS[@]}"
(cd "$S" && { list_usr | grep -vxE "usr/lib/modules/$KV/kernel/drivers/nvme(/host)?"; echo "usr/lib/modules/$KV/kernel/drivers/nvme"; echo "usr/lib/modules/$KV/kernel/drivers/nvme/host"; }) \
    | pack "$S" "$I/dirs-after-their-files";                              initrd fail dirs-after-their-files -m "its parent directory does not exist at that point"
(cd "$S" && list_usr | grep -vx init) | pack "$S" "$I/no-init";          initrd fail no-init -m "lacks /init"
old_accepted "initrd/no-init" old_initrd_ok "$I/no-init" "$KV" "${MODS[@]}"
(cd "$S" && list_usr | grep -vx "usr/lib/modules/$KV/kernel/drivers/pci/controller/dwc") | pack "$S" "$I/one-module-parent-missing"
initrd fail one-module-parent-missing -m "would not create 1 entry"
mv "$D/phy/tegra/phy-tegra194-p2u.ko" "$T/p2u.moved"
expect fail "initrd/installed-module-missing" -m "missing or empty" initrd_check "$I/good" "$KV" "$D" "${MODS[@]}"
mv "$T/p2u.moved" "$D/phy/tegra/phy-tegra194-p2u.ko"

# Archives GNU cpio would never write, made byte by byte, as the afternoon
# review did: the kernel unpacks them differently from what cpio -t shows.
cat > "$T/mk.py" <<'PY'
import gzip, io, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kit
out, variant, D, KV, filler = sys.argv[1:6]
MODS = ["nvme/host/nvme.ko", "nvme/host/nvme-core.ko", "pci/controller/dwc/pcie-tegra194.ko", "phy/tegra/phy-tegra194-p2u.ko"]
DIR, REG, LNK = 0o40755, 0o100644, 0o120777
ino = [100]
def ent(name, mode, body=b"", nlink=1, magic=b"070701"):
    ino[0] += 1
    nb = name.encode() + b"\0"
    rec = magic + b"".join(b"%08X" % v for v in (ino[0], mode, 0, 0, nlink, 0, len(body), 0, 0, 0, 0, len(nb), 0)) + nb
    rec += b"\0" * (-len(rec) % 4)
    return rec + body + b"\0" * (-len(body) % 4)
def archive(entries, magic=b"070701", trailer=True):
    b = b"".join(ent(*e, magic=magic) if len(e) < 4 else ent(e[0], e[1], e[2], e[3], magic) for e in entries)
    if not trailer:
        return b                                             # ends right after the last entry
    b += ent("TRAILER!!!", 0, magic=magic)
    return b + b"\0" * (-len(b) % 512)                     # cpio -o pads to 512
mod = lambda k: open(os.path.join(D, k), "rb").read()
base = f"usr/lib/modules/{KV}/kernel/drivers/"
def tree(lib="link", rename=lambda n: n, extra=(), nlink_nvme=1, nvme_dir=False, kit_opts={}, before_modules=(), after_dirs=()):
    dirs = ["usr", "usr/lib", "usr/lib/modules", f"usr/lib/modules/{KV}", f"usr/lib/modules/{KV}/kernel", base.rstrip("/")]
    for k in MODS:
        p = base.rstrip("/")
        for part in k.split("/")[:-1]:
            p += "/" + part
            if p not in dirs: dirs.append(p)
    e = [(".", DIR)]
    if lib == "link": e.append(("lib", LNK, b"usr/lib"))
    if lib == "dir": e.append(("lib", DIR))
    e += [(d, DIR) for d in dirs[:6]] + list(before_modules) + [(d, DIR) for d in dirs[6:]] + list(after_dirs)
    for k in MODS:
        if k == MODS[0] and nvme_dir: e.append((base + k, DIR))
        else: e.append((base + k, REG, mod(k), nlink_nvme if k == MODS[0] else 1))
    e += list(extra)
    e += kit.entries(**kit_opts)
    e += [("usr/share", DIR), ("usr/share/filler", REG, open(filler, "rb").read())]
    return [(rename(n) if n != "." else n,) + tuple(r) for n, *r in e]
nvme = base + MODS[0]
def junk_before(name, a):       # 8 bytes that are no header, just before <name>'s header
    i = a.index(name.encode() + b"\0") - 110
    return a[:i] + b"JUNKJUNK" + a[i:]
V = {
    "newc-good":        lambda: archive(tree()),
    "dot-slash-names":  lambda: archive(tree(rename=lambda n: "./" + n)),
    "absolute-names":   lambda: archive(tree(rename=lambda n: "/" + n)),
    "lib-is-dir":       lambda: archive(tree(lib="dir")),
    "no-lib":           lambda: archive(tree(lib=None)),
    "dup-good-then-empty": lambda: archive(tree(extra=[(nvme, REG, b"")])),
    "alias-via-lib":    lambda: archive(tree(extra=[(nvme.replace("usr/lib", "lib", 1), REG, b"junk" * 750)])),
    "alias-dot-slash":  lambda: archive(tree(extra=[("./" + nvme, REG, b"junk" * 750)])),
    "hard-linked-module": lambda: archive(tree(nlink_nvme=2)),
    "module-is-dir":    lambda: archive(tree(nvme_dir=True)),
    "crc-format":       lambda: archive(tree(), magic=b"070702"),
    "junk-between-entries": lambda: junk_before("usr/share", archive(tree())),
    "after-trailer":    lambda: archive(tree()) + archive([(nvme, REG, b"")]),
    "no-trailer":       lambda: archive(tree(), trailer=False),
    # the evening review: extraction order, and what /init needs
    "file-replaced-by-later-dir": lambda: archive(tree(before_modules=[(base + "nvme", REG, b"x")])),
    "file-replaces-empty-dir": lambda: archive(tree(after_dirs=[(base + "nvme/host", REG, b"x")])),
    "file-over-full-dir": lambda: archive(tree(extra=[(base + "nvme", REG, b"x")])),
    "dir-entry-with-data": lambda: archive(tree(extra=[("usr/lib/extra", DIR, b"data")])),
    "init-not-executable": lambda: archive(tree(kit_opts={"init_mode": 0o644})),
    "init-neither-elf-nor-script": lambda: archive(tree(kit_opts={"init_body": b"mount -t proc proc /proc\n"})),
    "no-bash":          lambda: archive(tree(kit_opts={"skip": {"usr/bin/bash"}})),
    "no-loader":        lambda: archive(tree(kit_opts={"skip": {"usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1"}})),
    "no-libtinfo":      lambda: archive(tree(kit_opts={"skip": {"usr/lib/aarch64-linux-gnu/libtinfo.so.6"}})),
    "no-chroot":        lambda: archive(tree(kit_opts={"skip": {"usr/sbin/chroot"}})),
    "no-kmod":          lambda: archive(tree(kit_opts={"skip": {"usr/bin/kmod"}})),
    "no-bin-symlink":   lambda: archive(tree(kit_opts={"skip": {"bin"}})),
    "bash-cut-off-elf": lambda: archive(tree(kit_opts={"bash_cut": True})),
}
if variant == "second-gzip-member":
    data = gzip.compress(archive(tree()), 9, mtime=0) + gzip.compress(archive([(nvme, REG, b"")]), 9, mtime=0)
elif variant == "gzip-comment":
    g = gzip.compress(archive(tree()), 9, mtime=0)
    data = g[:3] + bytes([0x10]) + g[4:10] + b"a comment\0" + g[10:]
elif variant == "gzip-file-name":
    buf = io.BytesIO()
    with gzip.GzipFile(filename="initrd", mode="wb", fileobj=buf, compresslevel=9, mtime=0) as f:
        f.write(archive(tree()))
    data = buf.getvalue()
else:
    data = gzip.compress(V[variant](), 9, mtime=0)
open(out, "wb").write(data)
PY
craft() { python3 "$T/mk.py" "$I/$1" "$1" "$D" "$KV" "$S/usr/share/filler"; }
for v in newc-good dot-slash-names absolute-names gzip-file-name; do craft $v; initrd pass $v; done
craft lib-is-dir;           initrd fail lib-is-dir -m "no lib -> usr/lib symlink"
craft no-lib;               initrd fail no-lib -m "no lib -> usr/lib symlink"
craft dup-good-then-empty;  initrd fail dup-good-then-empty -m "more than once"
craft alias-via-lib;        initrd fail alias-via-lib -m "more than once"
craft alias-dot-slash;      initrd fail alias-dot-slash -m "more than once"
craft hard-linked-module;   initrd fail hard-linked-module -m "hard-linked"
craft module-is-dir;        initrd fail module-is-dir -m "not as a regular file"
craft crc-format;           initrd fail crc-format -m "no newc cpio header"
craft junk-between-entries; initrd fail junk-between-entries -m "no newc cpio header"
craft no-trailer;           initrd fail no-trailer -m "has no trailer"
craft after-trailer;        initrd fail after-trailer -m "after the cpio trailer"
old_accepted "initrd/after-trailer" old_initrd_ok "$I/after-trailer" "$KV" "${MODS[@]}"
craft second-gzip-member;   initrd fail second-gzip-member -m "after its gzip stream"
old_accepted "initrd/second-gzip-member" old_initrd_ok "$I/second-gzip-member" "$KV" "${MODS[@]}"
craft gzip-comment;         initrd fail gzip-comment -m "gzip header flags 0x10"
craft file-replaced-by-later-dir; initrd pass file-replaced-by-later-dir   # the later directory entry unlinks the file
craft file-replaces-empty-dir; initrd fail file-replaces-empty-dir -m "would not create"
craft file-over-full-dir;   initrd fail file-over-full-dir -m "a directory of that name is in the way"   # rmdir fails: not empty
craft dir-entry-with-data;  initrd fail dir-entry-with-data -m "a non-file entry with data"
craft init-not-executable;  initrd fail init-not-executable -m "lacks /init as an executable file"
craft init-neither-elf-nor-script; initrd fail init-neither-elf-nor-script -m "neither an ELF program nor a #! script"
craft no-bash;              initrd fail no-bash -m "lacks /bin/bash as an executable file (the interpreter of /init)"
craft no-loader;            initrd fail no-loader -m "lacks /lib/ld-linux-aarch64.so.1 as an executable file (the loader of"
craft no-libtinfo;          initrd fail no-libtinfo -m "needs libtinfo.so.6, which is not in the initrd"
craft no-chroot;            initrd fail no-chroot -m "/init runs chroot, but no chroot is on its PATH"
craft no-kmod;              initrd fail no-kmod -m "/init runs modprobe, but no modprobe is on its PATH"
craft no-bin-symlink;       initrd fail no-bin-symlink -m "lacks /bin/bash"
craft bash-cut-off-elf;     initrd fail bash-cut-off-elf -m "is a damaged ELF file"

# --- the package list ----------------------------------------------------------
# 65 target packages, as in l4t_36_5_*.pairs; the kernel is one of them.
P="$T/pkgs"; mkdir -p "$P"
{ echo "nvidia-l4t-kernel=5.15.199-tegra-36.5.2-1"; for i in $(seq -w 1 64); do echo "nvidia-l4t-pkg$i=36.5.2-1"; done; } > "$P/target"
cut -d= -f1 "$P/target" > "$P/held-all"
{ grep -vx nvidia-l4t-kernel "$P/held-all"; echo nvidia-unrelated; } > "$P/held-kernel-swapped"
sed 's/$/:arm64/' "$P/held-all" > "$P/held-arch-qualified"
: > "$P/held-none"
old_holds_ok() { [ "$(grep -c '^nvidia-' "$1")" -ge 65 ]; }    # the count check the evening review found, on a list
hold() { local w="$1" n="$2"; shift 2; expect "$w" "holds/$n" "$@" holds_check "$P/target" "$P/$n"; }
hold pass held-all
hold fail held-kernel-swapped -m "not on hold: nvidia-l4t-kernel"
old_accepted "holds/held-kernel-swapped" old_holds_ok "$P/held-kernel-swapped"
hold pass held-arch-qualified
hold fail held-none -m "65 of the 65 target packages are not on hold"
sed 's/=/\thi\t/' "$P/target" > "$P/inst-all"                   # held packages show dpkg status "hi"
sed 's/=/\tii\t/' "$P/target" > "$P/inst-unheld"
sed 's/^\(nvidia-l4t-kernel\)\thi\t.*/\1\thi\t5.15.185-tegra-36.5.0-1/' "$P/inst-all" > "$P/inst-kernel-old"
sed 's/^\(nvidia-l4t-kernel\)\thi\t/\1\tiiR\t/' "$P/inst-all" > "$P/inst-kernel-reinst-required"   # installed, but flagged
sed 's/^\(nvidia-l4t-kernel\)\thi\t/\1\trc\t/' "$P/inst-all" > "$P/inst-kernel-removed"
grep -v '^nvidia-l4t-kernel' "$P/inst-all" > "$P/inst-kernel-missing"
ver() { local w="$1" n="$2"; shift 2; expect "$w" "versions/$n" "$@" versions_check "$P/target" "$P/$n"; }
ver pass inst-all
ver pass inst-unheld
ver fail inst-kernel-old -m "nvidia-l4t-kernel 5.15.185-tegra-36.5.0-1, not 5.15.199-tegra-36.5.2-1"
ver fail inst-kernel-reinst-required -m "dpkg status iiR"
ver fail inst-kernel-removed -m "dpkg status rc"
ver fail inst-kernel-missing -m "nvidia-l4t-kernel not installed"

# --- this machine (read-only) --------------------------------------------------
# The kernel /boot/initrd was built for is the one in /boot/Image: between an
# --apply and the reboot that is not the running one.
K="$(grep -aoE -m1 'Linux version [^ ]+' /boot/Image 2>/dev/null | awk '{print $3}')"
K="${K:-$(uname -r)}"
if [ -r /boot/extlinux/extlinux.conf ]; then
    expect pass "live/extlinux DEFAULT" extlinux_default_check /boot/extlinux/extlinux.conf JetsonIO
    expect pass "live/extlinux JetsonIO entry" extlinux_entry_check /boot/extlinux/extlinux.conf JetsonIO "$OVL"
fi
if [ -r /boot/initrd ] && [ -d "/lib/modules/$K" ]; then
    expect pass "live/initrd for $K (the kernel in /boot/Image)" initrd_check /boot/initrd "$K" "/lib/modules/$K/kernel/drivers" "${MODS[@]}"
fi
# the release's own package list: every one of its 65 packages held, at its version
REL="$(head -1 /etc/nv_tegra_release 2>/dev/null | sed -nE 's/.*R([0-9]+) \(release\), REVISION: ([0-9.]+),.*/\1.\2/p')"
if [ -n "$REL" ] && [ -r "$HERE/l4t_${REL//./_}.pairs" ]; then
    expect pass "live/holds (R$REL)" holds_check "$HERE/l4t_${REL//./_}.pairs"
    expect pass "live/versions (R$REL)" versions_check "$HERE/l4t_${REL//./_}.pairs"
fi

echo
echo "== $NPASS passed, $NFAIL failed"
[ "$NFAIL" -eq 0 ]
