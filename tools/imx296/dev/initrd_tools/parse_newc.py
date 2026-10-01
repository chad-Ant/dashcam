import sys, gzip
d = gzip.open(sys.argv[1]).read() if sys.argv[1] != '-' else sys.stdin.buffer.read()
off = 0; arch = 1; n = 0; verbose = len(sys.argv) > 2
names = {}
while off < len(d):
    if d[off:off+1] == b'\0':
        off += 1; continue
    if d[off:off+6] not in (b'070701', b'070702'):
        print('junk at', off, d[off:off+20]); break
    h = d[off:off+110]
    f = [int(h[6+8*i:14+8*i], 16) for i in range(13)]
    ino, mode, uid, gid, nlink, mtime, fsize, dmaj, dmin, rmaj, rmin, nsize, chk = f
    name = d[off+110:off+110+nsize-1].decode('latin1')
    p = (off + 110 + nsize + 3) & ~3
    dend = p + fsize
    nxt = (dend + 3) & ~3
    if name == 'TRAILER!!!':
        print(f'archive {arch}: {n} entries, trailer at {off}, ends {nxt}')
        arch += 1; n = 0
    else:
        n += 1
        if verbose and any(s in name for s in sys.argv[2:]): print(arch, oct(mode), nlink, fsize, name)
        names.setdefault(name, []).append(arch)
    off = nxt
print('end at', off, 'of', len(d))
dups = {k: v for k, v in names.items() if len(v) > 1}
print('names appearing more than once:', len(dups))
for k, v in list(dups.items())[:40]: print('  dup', k, v)
