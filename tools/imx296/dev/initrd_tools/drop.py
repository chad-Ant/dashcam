import gzip, sys, re
src, out, pattern = sys.argv[1], sys.argv[2], re.compile(sys.argv[3])
data = gzip.decompress(open(src, "rb").read())
pos, keep, dropped = 0, [], []
while True:
    h = data[pos:pos+110]; ns = int(h[94:102], 16); size = int(h[54:62], 16)
    nend = pos + 110 + ns; dstart = (nend + 3) & ~3; end = (dstart + size + 3) & ~3
    name = data[pos+110:nend-1].decode()
    if name == "TRAILER!!!":
        keep.append(data[pos:end]); break
    (dropped if pattern.fullmatch(name) else keep).append(data[pos:end] if not pattern.fullmatch(name) else name)
    pos = end
b = b"".join(keep); b += b"\0" * (-len(b) % 512)
open(out, "wb").write(gzip.compress(b, 9, mtime=0))
print("dropped:", dropped)
