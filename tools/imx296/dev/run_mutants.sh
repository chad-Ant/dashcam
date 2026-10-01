#!/bin/bash
#
# run_mutants.sh — mutation test for l4t_checks.sh: each mutant removes or weakens
# one rule in a copy of l4t_checks.sh, then test_l4t_checks.sh runs against the
# copy.  Every mutant must be "killed" (the suite fails).  A "SURVIVED" line means
# a rule no test depends on: add a case whose -m text only that rule produces.
#
# Usage:  bash ~/drive_logs/tools/dev/run_mutants.sh [name-filter]   (about 15 s per mutant)
# The live/holds and live/versions cases are skipped in the copies (no pairs file
# next to them), so a copy's total is two below the real suite's.
#
# Written 2026-10-01 by Claude (the 56 mutants run that day, all killed).

set -uo pipefail
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
T="$(dirname "$HERE")"                      # ~/drive_logs/tools
W="$(mktemp -d)" || exit 1
trap 'rm -rf "${W:?}"' EXIT
FILTER="${1:-}"
SURV=0

run() {   # run <name> <old text> <new text>
    [ -z "$FILTER" ] || [[ "$1" == *"$FILTER"* ]] || return 0
    mkdir -p "$W/$1"; cp "$T/test_l4t_checks.sh" "$W/$1/"
    if ! python3 "$HERE/harness/mutate.py" "$T/l4t_checks.sh" "$W/$1/l4t_checks.sh" "$2" "$3" 2> /dev/null; then
        echo "NOPATTERN $1 (the code changed: update this mutant)"; return
    fi
    local r; r="$(bash "$W/$1/test_l4t_checks.sh" 2>&1 | tail -1)"
    case "$r" in *" 0 failed"*) echo "SURVIVED  $1: $r"; SURV=$((SURV + 1)) ;; *) echo "killed    $1: $r" ;; esac
}

# extlinux.conf
run ascii           'if odd:' 'if False:'
run prefix-keyword  'if not (line.startswith(k) and' 'if False and not (line.startswith(k) and'
run hash-cut        'i = s.find("#")' 'i = -1'
run cr-drop         'raw.replace("\r", "")' 'raw'
run max-entries     'if len(cfg["entries"]) < MAX_ENTRIES:' 'if True:'
run labels-count    'if cfg["labels"] > MAX_ENTRIES:' 'if False:'
run label-exactly1  'if len(idx) != 1:' 'if len(idx) == 0:'
run key-count       'if len(got) == 1:' 'if len(got) >= 1:'
run linux-path      '("LINUX", "/boot/Image"), ' ''
run initrd-path     ', ("INITRD", "/boot/initrd")' ''
run append-root     'elif roots != [want_root]:' 'elif False:'
run ovl-space       'if any(c in v for c in " \t"):' 'if False:'
run ovl-name        'if overlay not in names:' 'if False:'
run other-camera    'if others:' 'if False:'
run abs-path        'if not p.startswith("/"):' 'if False:'
run dot-path        'elif any(c in ("", ".", "..") for c in p.split("/")[1:]):' 'elif False:'
run exists          'elif not os.path.isfile(fp) or os.path.getsize(fp) == 0:' 'elif not os.path.exists(fp):'
run same-fs         'elif os.stat(fp).st_dev != confdev:' 'elif False:'
run fdtoverlay      'if content_ok:' 'if False:'
run default-count   'if len(d) != 1:' 'if len(d) == 0:'
run default-label   'elif d[0] != label:' 'elif False:'
# initrd: gzip and newc
run gz-magic        'if data[:3] != b"\x1f\x8b\x08":' 'if False:'
run gz-flags        'if data[3] & ~0x08:' 'if False:'
run gz-eof          'if not d.eof:' 'if False:'
run gz-unused       'if d.unused_data.strip(b"\0"):' 'if False:'
run newc-magic      'if h[:6] != b"070701":' 'if False:'
run cpio-cutoff     'if namesize == 0 or dstart + size > len(out) or out[nend - 1] != 0:' 'if namesize == 0:'
run trailer         'if not trailer:' 'if False:'
run after-trailer   'if out[pos:].strip(b"\0"):' 'if False:'
# initrd: the kernel's unpacking, the modules, /init
run skipped-rule    'if skipped:' 'if False:'
run missing-parent  'return p if last else None               # ENOENT' 'return p  # '
run enotdir         'if node[0] != "dir":
                return None                              # ENOTDIR' 'if False:
                return None                              # ENOTDIR'
run follow-middle   'if node[0] == "lnk" and (follow_last or not last):' 'if node[0] == "lnk" and follow_last and last:'
run rmdir-nonempty  'elif not any(q.startswith(p + "/") for q in self.n):' 'else:'
run clean-off       'node = None if p is None else self.n.get(p)
        if node and node[0] != kind:' 'node = None
        if node and node[0] != kind:'
run dir-with-data   'if kind not in ("reg", "lnk") and body:' 'if False:'
run dir-in-the-way  'if old is not None and old[0] == "dir":
                return "a directory' 'if False:
                return "a directory'
run rewrite-flag    'return "rewrite" if old is not None and old[0] == "reg" else None' 'return None'
run hardlinks       'if links:' 'if False:'
run reg-kind        'elif node[0] != "reg":' 'elif False:'
run lib-hint        'if fs.lookup("/usr" + want) is not None else ""' 'if False else ""'
run bytes           'elif open(inst, "rb").read() != node[2]:' 'elif False:'
run installed-mod   'elif not os.path.isfile(inst) or os.path.getsize(inst) == 0:' 'elif False:'
run xbit            'or not node[1] & 0o111 or' 'or'
run shebang         'need_exec(interp[0].decode("utf-8", "replace"), f"the interpreter of {p}")' 'pass'
run elf-magic       'if not body.startswith(b"\x7fELF"):' 'if False:'
run elf-loader      'if interp:
            need_exec(interp' 'if False:
            need_exec(interp'
run elf-needed      'if found is None:' 'if False:'
run elf-damaged     'probs.append(f"{path}: {p} is a damaged ELF file ({ex})")' 'pass'
run need-init       'need_exec("/init",' 'probs.extend([]) or (lambda *a: None)("/init",'
run init-commands   'INIT_COMMANDS = ["mount", "cat", "grep", "sed", "tail", "ln", "kmod", "modprobe", "sleep", "expr", "chroot"]' 'INIT_COMMANDS = []'
# packages
run holds-names     'missing = [n for n in want if n not in held]' 'missing = []'
run holds-arch      'held = {h.split(":")[0] for h in held}' 'held = set(held)'
run ver-state       'installed = len(st) >= 2 and st[1] == "i" and not st[2:].strip()' 'installed = len(st) >= 2 and not st[2:].strip()'
run ver-errflag     'installed = len(st) >= 2 and st[1] == "i" and not st[2:].strip()' 'installed = len(st) >= 2 and st[1] == "i"'
run ver-version     'if not installed or have != v:' 'if not installed:'

echo
echo "== $SURV survived"
[ "$SURV" -eq 0 ]
