#!/bin/bash
# strict_probe.sh <initrd>: the extra conditions proposed in the report, evaluated alone.
set -uo pipefail
f="$1"; err="$(mktemp -p "$(dirname "$0")")"
flg=$(od -An -tu1 -j3 -N1 "$f" | tr -d ' ')
total=$(gzip -dc -- "$f" | wc -c)
blocks=$(gzip -dc -- "$f" | cpio -t 2>&1 >/dev/null | sed -n 's/^\([0-9]*\) blocks\{0,1\}$/\1/p')
lst="$(gzip -dc -- "$f" | cpio -t --quiet 2>"$err")"
dups=$(sed -E 's#^\./##; s#^/##; s#//+#/#g; s#^lib/#usr/lib/#' <<< "$lst" | sort | uniq -d | wc -l)
links=$(gzip -dc -- "$f" | cpio -tv --quiet 2>/dev/null | awk '$1 ~ /^-/ && $2 > 1' | wc -l)
libsym=$(gzip -dc -- "$f" | cpio -tv --quiet 2>/dev/null | awk '$9 == "lib" && $10 == "->" && $11 == "usr/lib"' | wc -l)
v=ok
[ "$flg" = 0 ] || v="bad-flg"
[ -s "$err" ] && v="$v,cpio-stderr"
[ "$((blocks * 512))" = "$total" ] || v="$v,after-trailer($((total - blocks * 512)))"
[ "$dups" = 0 ] || v="$v,dups=$dups"
[ "$links" = 0 ] || v="$v,hardlinks=$links"
[ "$libsym" = 1 ] || v="$v,no-lib-symlink"
echo "$v"; rm -f "$err"
