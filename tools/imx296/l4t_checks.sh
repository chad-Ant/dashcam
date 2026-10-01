# l4t_checks.sh — boot-file checks shared by l4t_upgrade_36_5_2.sh,
# l4t_rollback_36_5_0.sh and verify_platform.sh.  Sourced, not run.
#
# Each check prints a single summary line and returns 0 when everything holds;
# otherwise it prints one line per problem and returns 1.  test_l4t_checks.sh
# (next to this file) runs them against good and broken inputs.
#
# They read the files the way the programs that use them at boot do:
#   - extlinux.conf as NVIDIA's UEFI launcher does (edk2-nvidia L4TLauncher.c,
#     ProcessExtLinuxConfig and ExtLinuxBoot; identical at tag r36.5, r36.5.1
#     and r36.5-updates, which covers UEFI 36.5.x).  Each line is cut at the
#     first '#', CRs are dropped, spaces/tabs trimmed; a keyword is matched
#     case-sensitively as a PREFIX of the line (so "FDTDIR x" is read as FDT
#     "DIR x"); within an entry the last line of a kind wins; only the first 10
#     LABELs are entries (MAX_EXTLINUX_OPTIONS), later lines go to entry 10;
#     DEFAULT picks the first entry with exactly that label, else entry 0.
#     OVERLAYS is split at ',' only.  A missing or broken overlay, a DTB that is
#     not one, or an overlay that does not apply makes the launcher give up on
#     extlinux and boot the kernel partition, without the camera overlay.
#   - the initrd as the kernel does (init/initramfs.c, lib/decompress_inflate.c):
#     it unpacks every archive in the image, including anything after the first
#     cpio trailer, the last copy of a name wins, and its gunzip skips only the
#     FNAME header field.
# The checks refuse anything they cannot read the same way as the boot path,
# even where the boot path might cope: they guard a reboot.
#
# Written 2026-10-01 by Claude.  First version the same morning, after a review
# found that the old inline checks passed a truncated initrd and an overlay under
# another entry, commented out, or naming a missing file.  Rewritten that
# afternoon after an adversarial review against the launcher and kernel sources
# found more ways past it (APPEND unchecked, prefix keywords, '#' inside lines,
# the 10-entry limit, DTB/overlay content, data after the cpio trailer).

_l4t_py() {
    python3 - "$@" <<'PY'
import os, stat, struct, subprocess, sys, zlib

def done(problems, summary):
    text = "\n".join(problems) if problems else summary
    # a value quoted from a broken file may hold a NUL or CR: show it, escaped
    print("".join(c if c.isprintable() or c == "\n" else f"\\x{ord(c):02x}" for c in text))
    sys.exit(1 if problems else 0)

# ---------------------------------------------------------------- extlinux.conf
KEYS = ["TIMEOUT", "DEFAULT", "MENU TITLE", "LABEL",
        "MENU LABEL", "LINUX", "INITRD", "FDT", "OVERLAYS", "APPEND"]
ENTRY_KEYS = ["MENU LABEL", "LINUX", "INITRD", "FDT", "OVERLAYS", "APPEND"]
MAX_ENTRIES = 10            # MAX_EXTLINUX_OPTIONS, L4TLauncher.h

def clean(s):               # CleanExtLinuxLine
    i = s.find("#")
    if i >= 0:
        s = s[:i]
    return s.strip(" \t")

def parse_extlinux(path):
    """ProcessExtLinuxConfig, plus a list of lines it reads differently from how they look."""
    probs = []
    with open(path, "rb") as f:
        data = f.read()
    odd = [i for i, b in enumerate(data) if not (b in (9, 10) or 32 <= b <= 126)]
    if odd:
        probs.append(f"{path} has {len(odd)} byte(s) that are not plain ASCII text (the first at offset "
                     f"{odd[0]}: a CR, NUL, BOM or non-ASCII character)")
    cfg = {"defaults": [], "entries": [], "labels": 0}
    for n, raw in enumerate(data.decode("ascii", "replace").split("\n"), 1):
        line = clean(raw.replace("\r", ""))
        if not line:
            continue
        for k in KEYS:      # anything that looks like a keyword must be exactly one
            if line.upper().startswith(k):
                if not (line.startswith(k) and (len(line) == len(k) or line[len(k)] in " \t")):
                    probs.append(f"{path} line {n}: '{line[:48]}' is not a plain '{k} <value>' line "
                                 f"(the boot loader matches keywords case-sensitively by prefix)")
                break
        take = lambda k: clean(line[len(k):]) if line.startswith(k) else None
        if take("TIMEOUT") is not None:
            continue
        v = take("DEFAULT")
        if v is not None:
            cfg["defaults"].append(v)
            continue
        if take("MENU TITLE") is not None:
            continue
        if line.startswith("LABEL"):
            cfg["labels"] += 1
            if len(cfg["entries"]) < MAX_ENTRIES:
                cfg["entries"].append({"label": take("LABEL"), "keys": {}})
                continue
        if 0 < len(cfg["entries"]) <= MAX_ENTRIES:
            for k in ENTRY_KEYS:
                v = take(k)
                if v is not None:
                    cfg["entries"][-1]["keys"].setdefault(k, []).append(v)
                    break
    return cfg, probs

def cmd_default(conf, label):
    cfg, probs = parse_extlinux(conf)
    d = cfg["defaults"]
    if len(d) != 1:
        probs.append(f"{conf} has {len(d)} DEFAULT lines (need exactly 1)")
    elif d[0] != label:
        probs.append(f"DEFAULT is '{d[0]}', not {label}")
    done(probs, f"DEFAULT {label}")

def cmd_entry(conf, label, overlay, root, partuuid):
    cfg, probs = parse_extlinux(conf)
    ents = cfg["entries"]
    if cfg["labels"] > MAX_ENTRIES:
        probs.append(f"{conf} has {cfg['labels']} LABEL lines; the boot loader reads only the first "
                     f"{MAX_ENTRIES} and adds the lines of the rest to entry {MAX_ENTRIES}")
    idx = [i for i, e in enumerate(ents) if e["label"] == label]
    if len(idx) != 1:
        probs.append(f"{conf} has {len(idx)} entries labelled exactly '{label}' among the first "
                     f"{MAX_ENTRIES} (need exactly 1)")
        done(probs, "")
    e = ents[idx[0]]
    val = {}
    for k in ("LINUX", "INITRD", "FDT", "OVERLAYS", "APPEND"):
        got = e["keys"].get(k, [])
        if len(got) == 1:
            val[k] = got[0]
        else:
            probs.append(f"the {label} entry has {len(got)} {k} lines (need 1)")
    for k, want in (("LINUX", "/boot/Image"), ("INITRD", "/boot/initrd")):
        if k in val and val[k] != want:
            probs.append(f"the {label} entry's {k} is '{val[k]}', not {want} (the file the packages "
                         f"write and these checks examine)")
    want_root = f"root=PARTUUID={partuuid}"
    if "APPEND" in val:
        roots = [t for t in val["APPEND"].split() if t.startswith("root=")]
        if not partuuid:
            probs.append("cannot tell the PARTUUID of the running root file system (findmnt), "
                         "so the APPEND line's root= cannot be checked")
        elif roots != [want_root]:
            probs.append(f"the {label} entry's APPEND has '{' '.join(roots) or 'no root='}', not "
                         f"{want_root} (the running root file system)")
    files = [(k, val[k]) for k in ("LINUX", "INITRD", "FDT") if k in val]
    ovls = []
    if "OVERLAYS" in val:
        v = val["OVERLAYS"]
        if any(c in v for c in " \t"):
            probs.append(f"the {label} entry's OVERLAYS has a space in it: '{v}' (the boot loader splits "
                         f"at ',' only and trims nothing)")
        for p in v.split(","):
            if p:
                ovls.append(p)
                files.append(("OVERLAYS", p))
            else:
                probs.append(f"the {label} entry's OVERLAYS has an empty item: '{v}'")
        names = [p.rsplit("/", 1)[-1] for p in ovls]
        if overlay not in names:
            probs.append(f"the {label} entry's OVERLAYS does not name {overlay}: '{v}'")
        others = [n for n in names if n != overlay and "-camera-" in n]
        if others:          # applied after (or before) ours, it may rewire the same CSI port
            probs.append(f"the {label} entry's OVERLAYS also names another camera overlay: {', '.join(others)}")
    confdev = os.stat(conf).st_dev
    content_ok = "FDT" in val and bool(ovls)
    for k, p in files:
        bad = None
        fp = root + p
        if not p.startswith("/"):
            bad = f"{k} '{p}' (in the {label} entry) is not an absolute path"
        elif any(c in ("", ".", "..") for c in p.split("/")[1:]):
            bad = (f"{k} '{p}' (in the {label} entry) has '.', '..' or '//' in it (the boot loader "
                   f"cleans those up across the whole OVERLAYS list, not per item)")
        elif not os.path.isfile(fp) or os.path.getsize(fp) == 0:
            bad = f"{k} {p} (in the {label} entry) does not exist or is empty"
        elif os.stat(fp).st_dev != confdev:
            bad = (f"{k} {p} (in the {label} entry) is not on the file system that holds {conf}; "
                   f"the boot loader reads only that one")
        if bad:
            probs.append(bad)
            if k in ("FDT", "OVERLAYS"):
                content_ok = False
    if content_ok:          # the DTB must be one, and the overlays must apply to it, in order
        cmd = ["fdtoverlay", "-i", root + val["FDT"], "-o", os.devnull] + [root + p for p in ovls]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
            if r.returncode != 0:
                msg = " ".join(r.stderr.split())[:240]
                probs.append(f"the {label} entry's overlays do not apply to its FDT {val['FDT']} "
                             f"(fdtoverlay: {msg or 'rc ' + str(r.returncode)})")
        except FileNotFoundError:
            probs.append("fdtoverlay is not installed (apt install device-tree-compiler), so the FDT "
                         "and overlays could not be checked")
    done(probs, f"the {label} entry (entry {idx[0] + 1} of {len(ents)}): OVERLAYS {val.get('OVERLAYS')}, "
                f"{want_root}; its files exist on the root file system, and the overlays apply to its FDT")

# ---------------------------------------------------------------------- initrd
def norm(name):
    while name.startswith("./") or name.startswith("/"):
        name = name[2:] if name.startswith("./") else name[1:]
    while "//" in name:
        name = name.replace("//", "/")
    return name.rstrip("/")

def cmd_initrd(path, kver, drivers, mods):
    probs = []
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        done([f"{path} does not exist or is empty"], "")
    with open(path, "rb") as f:
        data = f.read()
    if data[:3] != b"\x1f\x8b\x08":
        done([f"{path} is not a gzip file"], "")
    if data[3] & ~0x08:
        done([f"{path}: gzip header flags 0x{data[3]:02x}; the kernel's gunzip skips only a file name "
              f"(nv-update-initrd writes 0x00)"], "")
    d = zlib.decompressobj(31)
    try:
        out = d.decompress(data) + d.flush()
    except zlib.error as ex:
        done([f"{path} does not decompress (truncated or damaged): {ex}"], "")
    if not d.eof:
        done([f"{path} is cut off: the gzip stream does not end"], "")
    if d.unused_data.strip(b"\0"):
        done([f"{path} has {len(d.unused_data)} bytes after its gzip stream that are not zero padding "
              f"(the kernel would unpack them too)"], "")
    # newc ("070701"), as cpio -H newc writes it and the kernel's do_header reads it
    pos, entries, trailer = 0, [], False
    while pos + 110 <= len(out):
        h = out[pos:pos + 110]
        if h[:6] != b"070701":
            done([f"{path}: no newc cpio header at offset {pos} of the archive (the kernel stops there)"], "")
        try:
            ino, mode, _, _, nlink, _, size, _, _, _, _, namesize, _ = (
                int(h[6 + 8 * i:14 + 8 * i], 16) for i in range(13))
        except ValueError:
            done([f"{path}: a damaged cpio header at offset {pos}"], "")
        nend = pos + 110 + namesize
        dstart = (nend + 3) & ~3
        if namesize == 0 or dstart + size > len(out) or out[nend - 1] != 0:
            done([f"{path}: the cpio archive is cut off at offset {pos}"], "")
        name = out[pos + 110:nend - 1].decode("utf-8", "surrogateescape")
        pos = (dstart + size + 3) & ~3
        if name == "TRAILER!!!":
            trailer = True
            break
        entries.append((norm(name), mode, nlink, out[dstart:dstart + size]))
    if not trailer:
        done([f"{path}: the cpio archive has no trailer (cut off)"], "")
    if out[pos:].strip(b"\0"):
        done([f"{path} has {len(out) - pos} bytes after the cpio trailer that are not zero padding "
              f"(the kernel would unpack them as another archive)"], "")
    # Unpack it into a model of the kernel's rootfs (init/initramfs.c, do_name/do_symlink/
    # clean_path), starting from the built-in initramfs (usr/default_cpio_list, as
    # CONFIG_INITRAMFS_SOURCE=""): an entry whose parent directory does not exist (yet)
    # is not created, a later entry replaces an earlier one, and paths go through symlinks.
    fs = Rootfs()
    skipped, links, rewritten = [], 0, set()
    for name, mode, nlink, body in entries:
        if stat.S_ISREG(mode) and nlink > 1:
            links += 1
        why = fs.unpack(name, mode, body)
        if why == "rewrite":
            rewritten.add(name)
        elif why:
            skipped.append(f"{name} ({why})")
    if skipped:
        probs.append(f"{path}: the kernel would not create {len(skipped)} entr{'y' if len(skipped) == 1 else 'ies'}, "
                     f"e.g. {skipped[0]}")
    if rewritten:
        probs.append(f"{path} holds {len(rewritten)} name(s) more than once, and the kernel keeps the last "
                     f"copy (e.g. {sorted(rewritten)[0]})")
    if links:
        probs.append(f"{path} has {links} hard-linked file(s) (nv-update-initrd never writes those)")
    names = []
    for k in mods:
        want = f"/lib/modules/{kver}/kernel/drivers/{k}"     # where modprobe will look
        node = fs.lookup(want)
        inst = os.path.join(drivers, k)
        if node is None:
            hint = " (it is under usr/lib, but /lib does not lead there: no lib -> usr/lib symlink)" \
                if fs.lookup("/usr" + want) is not None else ""
            probs.append(f"{path} lacks {k} for {kver}{hint}")
        elif node[0] != "reg":
            probs.append(f"{path} has {k} for {kver}, but not as a regular file")
        elif not os.path.isfile(inst) or os.path.getsize(inst) == 0:
            probs.append(f"{path} has {k}, but the installed {inst} is missing or empty")
        elif open(inst, "rb").read() != node[2]:
            probs.append(f"{path} has {k} for {kver}, but its bytes differ from {inst}")
        else:
            names.append(k.rsplit("/", 1)[-1])
    probs += startup_problems(fs, path)
    done(probs, f"{path} intact ({len(entries)} entries, one gzip stream, one cpio archive, every entry "
                f"created in order); /init and what it needs to reach the root are there; "
                f"{' '.join(names)} for {kver} match the installed modules")

class Rootfs:
    """The rootfs the kernel unpacks into: path -> (kind, mode, body); kind dir/reg/lnk/other."""
    def __init__(self):
        self.n = {"": ("dir", 0o755, b""), "dev": ("dir", 0o755, b""),
                  "dev/console": ("other", 0o600, b""), "root": ("dir", 0o700, b"")}

    def resolve(self, path, follow_last, depth=0):
        """The canonical path, with its parent an existing directory; None if it cannot be reached."""
        if depth > 40:
            return None                                  # ELOOP
        todo = [c for c in path.split("/") if c not in ("", ".")]
        cur = []
        while todo:
            c = todo.pop(0)
            if c == "..":
                cur = cur[:-1]
                continue
            p = "/".join(cur + [c])
            node = self.n.get(p)
            last = not todo
            if node is None:
                return p if last else None               # ENOENT for a missing middle part
            if node[0] == "lnk" and (follow_last or not last):
                t = node[2].decode("utf-8", "surrogateescape")
                rest = "/".join(todo)
                base = t if t.startswith("/") else "/".join(cur + [t])
                return self.resolve(base + ("/" + rest if rest else ""), follow_last, depth + 1)
            if last:
                return p
            if node[0] != "dir":
                return None                              # ENOTDIR
            cur.append(c)
        return "/".join(cur)

    def lookup(self, path):
        p = self.resolve(path, True)
        return None if p is None else self.n.get(p)

    def clean(self, name, kind):                         # clean_path
        p = self.resolve(name, False)
        node = None if p is None else self.n.get(p)
        if node and node[0] != kind:
            if node[0] != "dir":
                del self.n[p]
            elif not any(q.startswith(p + "/") for q in self.n):
                del self.n[p]                            # init_rmdir: empty directories only

    def unpack(self, name, mode, body):
        """Apply one entry; returns None, "rewrite" (a file written twice), or why it was not created."""
        if name in ("", ".", "/", "./"):
            return None
        ft = stat.S_IFMT(mode)
        kind = {stat.S_IFREG: "reg", stat.S_IFDIR: "dir", stat.S_IFLNK: "lnk"}.get(ft, "other")
        if kind not in ("reg", "lnk") and body:
            return "a non-file entry with data, which the kernel skips"   # do_header
        self.clean(name, kind if kind != "lnk" else None)               # do_symlink: clean_path(.., 0)
        p = self.resolve(name, False)
        if p is None:
            return "its parent directory does not exist at that point"
        old = self.n.get(p)
        if kind == "reg":
            if old is not None and old[0] == "dir":
                return "a directory of that name is in the way"
            self.n[p] = ("reg", mode & 0o7777, body)
            return "rewrite" if old is not None and old[0] == "reg" else None
        if old is not None:
            return None if kind == "dir" and old[0] == "dir" else f"a {old[0]} of that name is in the way"
        self.n[p] = (kind, mode & 0o7777, body)
        return None

# What NVIDIA's /init (L4T R36.5) runs on its way to a root=PARTUUID NVMe root: it
# mounts /proc, /dev and /sys, reads /proc/cmdline (cat, grep, sed, tail), links
# depmod to kmod (ln), loads pcie-tegra194, phy-tegra194-p2u and nvme (modprobe),
# mounts the root (mount, sleep, expr), then "exec chroot . /sbin/init".  The
# kernel starts /init with no PATH, so bash uses its built-in default.
INIT_COMMANDS = ["mount", "cat", "grep", "sed", "tail", "ln", "kmod", "modprobe", "sleep", "expr", "chroot"]
BASH_PATH = ["/usr/local/bin", "/usr/local/sbin", "/usr/bin", "/usr/sbin", "/bin", "/sbin"]
LIB_DIRS = ["/lib/aarch64-linux-gnu", "/usr/lib/aarch64-linux-gnu", "/lib", "/usr/lib"]   # no ld.so.cache in the initrd

def elf_info(body):
    """(interpreter, [DT_NEEDED], [DT_RUNPATH/RPATH]) of a 64-bit little-endian ELF; raises ValueError."""
    if body[4] != 2 or body[5] != 1:
        raise ValueError("not a 64-bit little-endian ELF")
    phoff, = struct.unpack_from("<Q", body, 0x20)
    phentsize, phnum = struct.unpack_from("<HH", body, 0x36)
    interp, dyn, loads = None, None, []
    for i in range(phnum):
        ptype, _, off, vaddr, _, filesz = struct.unpack_from("<IIQQQQ", body, phoff + i * phentsize)
        if off + filesz > len(body):
            raise ValueError("cut off")
        if ptype == 3:
            interp = body[off:off + filesz].split(b"\0")[0].decode()
        elif ptype == 2:
            dyn = (off, filesz)
        elif ptype == 1:
            loads.append((vaddr, off, filesz))
    needed, runpath, strtab = [], [], None
    if dyn:
        for j in range(dyn[1] // 16):
            tag, val = struct.unpack_from("<qQ", body, dyn[0] + 16 * j)
            if tag == 0:
                break
            if tag == 1:
                needed.append(val)
            elif tag in (15, 29):
                runpath.append(val)
            elif tag == 5:
                strtab = val
    def string(o):
        for vaddr, off, filesz in loads:
            if vaddr <= strtab < vaddr + filesz:
                start = off + (strtab - vaddr) + o
                return body[start:body.index(b"\0", start)].decode()
        raise ValueError("string table outside the file")
    if (needed or runpath) and strtab is None:
        raise ValueError("no string table")
    return interp, [string(o) for o in needed], [d for o in runpath for d in string(o).split(":") if d]

def startup_problems(fs, path):
    probs, seen = [], set()

    def need_exec(p, why):                   # p must resolve to an executable regular file; then check what it needs
        node = fs.lookup(p)
        if node is None or node[0] != "reg" or not node[1] & 0o111 or not node[2]:
            probs.append(f"{path} lacks {p} as an executable file ({why})")
            return
        check_binary(p, node[2], why)

    def check_binary(p, body, why):
        canon = fs.resolve(p, True)
        if canon in seen:
            return
        seen.add(canon)
        if body.startswith(b"#!"):
            interp = body[2:].split(b"\n")[0].split()
            if not interp:
                probs.append(f"{path}: {p} has an empty #! line")
            else:
                need_exec(interp[0].decode("utf-8", "replace"), f"the interpreter of {p}")
            return
        if not body.startswith(b"\x7fELF"):
            probs.append(f"{path}: {p} is neither an ELF program nor a #! script ({why})")
            return
        try:
            interp, needed, runpath = elf_info(body)
        except (ValueError, IndexError, UnicodeDecodeError, struct.error) as ex:
            probs.append(f"{path}: {p} is a damaged ELF file ({ex})")
            return
        if interp:
            need_exec(interp, f"the loader of {p}")
        for lib in needed:
            dirs = [d.replace("$ORIGIN", os.path.dirname(canon) or "/") for d in runpath] + LIB_DIRS
            hits = [d + "/" + lib for d in dirs] if "/" not in lib else [lib]
            found = next((h for h in hits if (fs.lookup(h) or ("",))[0] == "reg"), None)
            if found is None:
                probs.append(f"{path}: {p} needs {lib}, which is not in the initrd")
            else:
                check_binary(found, fs.lookup(found)[2], f"needed by {p}")

    need_exec("/init", "the kernel runs it; without it, it tries to mount root= itself, and nvme is a module")
    for c in INIT_COMMANDS:
        hit = next((d + "/" + c for d in BASH_PATH
                    if (fs.lookup(d + "/" + c) or ("",))[0] == "reg"), None)
        if hit is None:
            probs.append(f"{path}: /init runs {c}, but no {c} is on its PATH")
        else:
            need_exec(hit, f"/init runs {c}")
    return probs

# -------------------------------------------------------------------- packages
def read_pairs(path):
    out = []
    for line in open(path):
        line = line.strip()
        if line and not line.startswith("#"):
            name, _, ver = line.partition("=")
            out.append((name, ver))
    if not out:
        done([f"{path} lists no packages"], "")
    return out

def cmd_holds(pairs_file, held_file=""):
    want = [n for n, _ in read_pairs(pairs_file)]
    if held_file:
        held = open(held_file).read().split()
    else:
        r = subprocess.run(["apt-mark", "showhold"], capture_output=True, text=True)
        if r.returncode != 0:
            done([f"apt-mark showhold failed: {r.stderr.strip()}"], "")
        held = r.stdout.split()
    held = {h.split(":")[0] for h in held}                 # pkg:arch for a foreign architecture
    missing = [n for n in want if n not in held]
    done([f"{len(missing)} of the {len(want)} target packages are not on hold: {' '.join(missing)}"]
         if missing else [], f"all {len(want)} target packages are on hold")

def cmd_versions(pairs_file, inst_file=""):
    pairs = read_pairs(pairs_file)
    if inst_file:
        text = open(inst_file).read()
    else:                                # exits 1 if a package is unknown; that one is just missing below
        text = subprocess.run(["dpkg-query", "-W", "-f=${Package}\t${db:Status-Abbrev}\t${Version}\n"]
                              + [n for n, _ in pairs], capture_output=True, text=True).stdout
    inst = {}
    for line in text.splitlines():
        f = line.split("\t")
        if len(f) == 3:
            inst[f[0].split(":")[0]] = (f[1].strip(), f[2])
    bad = []
    for n, v in pairs:
        st, have = inst.get(n, ("", ""))
        # Status-Abbrev: wanted action (i install, h hold, ...), state (i = installed), error flag
        installed = len(st) >= 2 and st[1] == "i" and not st[2:].strip()
        if not installed or have != v:
            bad.append(f"{n} {have or 'not installed'}{'' if installed or not st else ' (dpkg status ' + st + ')'}, not {v}")
    done([f"{len(bad)} of the {len(pairs)} target packages are not installed at their target version: "
          + "; ".join(bad[:8]) + (" ..." if len(bad) > 8 else "")] if bad else [],
         f"all {len(pairs)} target packages are installed at their target versions")

cmd, args = sys.argv[1], sys.argv[2:]
try:
    if cmd == "default":
        cmd_default(*args)
    elif cmd == "entry":
        cmd_entry(*args)
    elif cmd == "initrd":
        cmd_initrd(args[0], args[1], args[2], args[3:])
    elif cmd == "holds":
        cmd_holds(*args)
    elif cmd == "versions":
        cmd_versions(*args)
except OSError as ex:
    done([f"cannot read: {ex}"], "")
PY
}

# extlinux_default_check <extlinux.conf> <label>
# Passes only if the file is plain ASCII, every keyword-like line is a plain
# "KEYWORD value" line, and there is exactly one DEFAULT, naming <label> exactly.
extlinux_default_check() { _l4t_py default "$1" "$2"; }

# extlinux_entry_check <extlinux.conf> <label> <overlay.dtbo file name> [root] [partuuid]
# Passes only if, besides the file rules above:
#   - exactly one of the first 10 entries is labelled exactly <label>, and the
#     file has at most 10 LABELs;
#   - that entry has exactly one LINUX, INITRD, FDT, OVERLAYS and APPEND line;
#   - LINUX is /boot/Image and INITRD is /boot/initrd: the files the kernel and
#     initrd packages write, and the files the scripts check;
#   - APPEND's only root= is root=PARTUUID=<partuuid> (default: the running
#     root file system's, from findmnt);
#   - OVERLAYS (comma-separated, no spaces, as jetson-io writes it) names
#     <overlay> by its exact file name, and no other "*-camera-*" overlay;
#   - every path is absolute and plain (no '.', '..' or '//'), and is an
#     existing, non-empty file on the file system that holds extlinux.conf;
#   - fdtoverlay applies the overlays, in order, to the FDT (necessary, not
#     sufficient: NVIDIA's own apply also filters by board).
# [root] is prepended to the paths before they are checked (for tests).
extlinux_entry_check() {
    _l4t_py entry "$1" "$2" "$3" "${4:-}" "${5:-$(findmnt -no PARTUUID / 2>/dev/null)}"
}

# initrd_check <initrd> <kver> <installed drivers dir> <module under drivers/>...
# Passes only if:
#   - the file is exactly one gzip stream (header flags 0 or FNAME, CRC and length
#     right, nothing after it but zero padding) holding exactly one newc cpio
#     archive (every header valid, a trailer, nothing after it but zero padding);
#   - unpacked the way the kernel does it (init/initramfs.c), into the rootfs its
#     built-in initramfs starts with, every entry is created: its parent directory
#     exists by then, nothing of another kind is in the way; and no file is
#     written twice, and there are no hard links;
#   - each module is then at /lib/modules/<kver>/kernel/drivers/<module>, through
#     any symlinks, where modprobe will look, as a regular file with exactly the
#     bytes of the installed one in <installed drivers dir>;
#   - /init is an executable file, and so is everything it needs to reach an NVMe
#     root (INIT_COMMANDS, found on bash's default PATH as the kernel gives /init
#     none): each #! interpreter, each ELF loader and each DT_NEEDED library, found
#     where the loader looks (there is no ld.so.cache in the initrd).
initrd_check() { _l4t_py initrd "$@"; }

# holds_check <target list: pkg=version (or pkg) per line> [file of held names, for tests]
# Passes only if every target package is on hold (apt-mark showhold): the names,
# not a count, so an unheld kernel cannot hide behind some other held package.
holds_check() { _l4t_py holds "$1" "${2:-}"; }

# versions_check <target list: pkg=version per line> [file of "pkg<TAB>status<TAB>version", for tests]
# Passes only if every target package is installed (dpkg state "i", no error
# flag; held packages show "hi") at exactly its version.
versions_check() { _l4t_py versions "$1" "${2:-}"; }
