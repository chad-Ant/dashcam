#!/usr/bin/env python3
"""Port of the extlinux handling in edk2-nvidia L4TLauncher (r36.5 == r36.5.1 ==
r36.5-updates head, identical files) plus the edk2 helpers it calls.
Usage: launcher_emu.py <extlinux.conf> <root dir prefix for /boot paths>
Prints which entry the launcher boots and what happens to FDT/OVERLAYS/LINUX.
Headless: ExtLinuxBootMenu returns DefaultBootEntry (timeout, no key)."""
import os, sys

MAX_EXTLINUX_OPTIONS = 10

def read_lines(data):
    # FileHandleReadLine (ASCII mode, no FF FE tag): ends at '\n', drops every
    # '\r' (CrCount++; continue), stores other bytes as CHAR16 -> a NUL byte
    # terminates the C string.  Loop: while (!FileHandleEof) ReturnLine.
    out, cur, i = [], [], 0
    while i < len(data):
        b = data[i]; i += 1
        if b == 0x0A:
            out.append(bytes(cur)); cur = []
            continue
        if b == 0x0D:
            continue
        cur.append(b)
    if cur:
        out.append(bytes(cur))
    res = []
    for l in out:
        s = l.decode('latin-1')
        if '\x00' in s:
            s = s[:s.index('\x00')]
        res.append(s)
    return res

def clean(s):
    # CleanExtLinuxLine: cut at the first '#' anywhere; skip leading ' '/'\t';
    # cut after the last char that is not ' '/'\t'.
    k = s.find('#')
    if k >= 0:
        s = s[:k]
    j = 0
    while j < len(s) and s[j] in ' \t':
        j += 1
    s = s[j:]
    if s:
        last = 0
        for n, c in enumerate(s):
            if c not in ' \t':
                last = n
        s = s[:last + 1]
    return s

def check(line, key):
    # CheckCommandString: StrnCmp(CommandLine, Key, StrLen(Key)) == 0 -> prefix,
    # case-sensitive; value = CleanExtLinuxLine(CommandLine + StrLen(Key))
    if line[:len(key)] == key:
        return clean(line[len(key):])
    return None

def path_remove_last_item(p):
    last = None
    for i, c in enumerate(p):
        if c == '\\' and i + 1 < len(p):
            last = i + 1
        elif c == ':' and i + 1 < len(p) and p[i + 1] != '\\':
            last = i + 1
    if last is not None:
        return p[:last], True
    return p, False

def path_cleanup(p):
    # PathCleanUpDirectories (MdePkg/Library/BaseLib/FilePaths.c)
    p = p.replace('/', '\\')
    while '\\\\' in p:
        k = p.index('\\\\'); p = p[:k] + p[k + 1:]
    while '\\.\\' in p:
        k = p.index('\\.\\'); p = p[:k] + p[k + 2:]
    if len(p) >= 2 and p.endswith('\\.'):
        p = p[:-1]
    while True:
        k = p.find('\\..')
        if k < 0:
            break
        nxt = p[k + 3] if k + 3 < len(p) else ''
        if not (nxt == '\\' or nxt == ''):
            break
        tail = p[k + 4:] if nxt else None
        head, _ = path_remove_last_item(p[:k + 1])
        p = head + (tail if tail is not None else '')
    return p

def process(lines):
    opts, default, timeout = [], None, 0
    for fl in lines:
        cl = clean(fl)
        if cl == '':
            continue
        v = check(cl, 'TIMEOUT')
        if v is not None:
            timeout = v; continue
        v = check(cl, 'DEFAULT')
        if v is not None:
            default = v; continue
        v = check(cl, 'MENU TITLE')
        if v is not None:
            continue
        if len(opts) < MAX_EXTLINUX_OPTIONS:
            v = check(cl, 'LABEL')
            if v is not None:
                opts.append({'Label': v}); continue
        if len(opts) <= MAX_EXTLINUX_OPTIONS and len(opts) != 0:
            cur = opts[-1]
            for key, field in (('MENU LABEL', 'MenuLabel'), ('LINUX', 'LinuxPath'),
                               ('INITRD', 'InitrdPath'), ('FDT', 'DtbPath'),
                               ('OVERLAYS', 'Overlays'), ('APPEND', 'BootArgs')):
                v = check(cl, key)
                if v is not None:
                    cur[field] = v; break
    dflt = 0
    if default is not None:
        for i, o in enumerate(opts):
            if o['Label'] == default:
                dflt = i; break
    for o in opts:
        for f in ('DtbPath', 'InitrdPath', 'LinuxPath', 'Overlays'):
            if f in o:
                o[f] = path_cleanup(o[f])
    return opts, default, dflt

def efi_file(root, p):
    # files are opened on the APP partition root; '\' -> '/'
    q = root + '/' + p.replace('\\', '/').lstrip('/')
    return q if os.path.isfile(q) else None

def is_fdt(path):
    with open(path, 'rb') as f:
        return f.read(4) == b'\xd0\x0d\xfe\xed'

def boot(opt, root):
    r = []
    if 'BootArgs' not in opt:
        return r + ['APPEND missing: ExtLinuxBoot does StrSize (BootOption->BootArgs) on NULL '
                    '-> StrLen dereferences NULL (no NULL check) -> crash / undefined']
    if 'InitrdPath' in opt and not efi_file(root, opt['InitrdPath']):
        return r + ['INITRD %r cannot be read -> return error -> fallback to kernel partition (no overlays)' % opt['InitrdPath']]
    overlays_applied = []
    if 'DtbPath' in opt:
        f = efi_file(root, opt['DtbPath'])
        if not f:
            r.append('FDT %r cannot be read -> goto LoadKernel: firmware DTB, NO overlays' % opt['DtbPath'])
        elif not is_fdt(f):
            return r + ['FDT %r is not a DTB -> fdt_open_into fails -> goto Exit -> fallback to kernel partition (no overlays)' % opt['DtbPath']]
        else:
            if 'Overlays' in opt:
                ov = opt['Overlays']
                for item in ov.split(','):
                    g = efi_file(root, item) if item else None
                    if not g:
                        return r + ['overlay %r cannot be loaded -> goto Exit -> fallback to kernel partition (no overlays)' % item]
                    if not is_fdt(g):
                        return r + ['overlay %r bad header -> goto Exit -> fallback to kernel partition (no overlays)' % item]
                    overlays_applied.append(item)
            r.append('FDT %r loaded; overlays applied: %s' % (opt['DtbPath'], overlays_applied or 'none'))
    else:
        r.append('no FDT line: firmware DTB, overlays never applied')
    if 'LinuxPath' not in opt:
        return r + ['no LINUX line: nothing to start -> fallback to kernel partition']
    if not efi_file(root, opt['LinuxPath']):
        return r + ['LINUX %r cannot be loaded -> goto Exit -> fallback to kernel partition' % opt['LinuxPath']]
    r.append('starts LINUX %r INITRD %r' % (opt['LinuxPath'], opt.get('InitrdPath')))
    return r

def main():
    conf, root = sys.argv[1], sys.argv[2]
    with open(conf, 'rb') as f:
        lines = read_lines(f.read())
    opts, default, dflt = process(lines)
    print('  launcher: %d entries %s; DEFAULT %r -> boots entry %d (%r)' %
          (len(opts), [o['Label'] for o in opts], default, dflt, opts[dflt]['Label'] if opts else None))
    if opts:
        o = opts[dflt]
        print('  launcher entry: ' + ', '.join('%s=%r' % (k, o[k]) for k in ('LinuxPath', 'InitrdPath', 'DtbPath', 'Overlays') if k in o) +
              ('' if 'BootArgs' in o else ', BootArgs=NULL'))
        for l in boot(o, root):
            print('  launcher: ' + l)

if __name__ == '__main__':
    main()
