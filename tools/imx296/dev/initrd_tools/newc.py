# newc.py: write a newc cpio stream from a spec (JSON list) to stdout, the way the
# kernel and GNU cpio read it.  Entry: {"name":..., "type":"f|d|l", "data": file path or "", "target": for l,
#   "ino": int, "nlink": int, "raw": bytes-as-latin1 to inject verbatim (junk)}
import sys, json, os
spec = json.load(open(sys.argv[1]))
out = sys.stdout.buffer
pos = 0
def w(b):
    global pos; out.write(b); pos += len(b)
def pad4():
    if pos % 4: w(b'\0' * (4 - pos % 4))
ino = 1000
for e in spec + [{"name": "TRAILER!!!", "type": "trailer"}]:
    if 'raw' in e:
        w(e['raw'].encode('latin1')); continue
    t = e.get('type', 'f')
    if t == 'f':
        data = open(e['data'], 'rb').read() if e.get('data') else b''
        mode = 0o100644
    elif t == 'd':
        data = b''; mode = 0o40755
    elif t == 'l':
        data = e['target'].encode(); mode = 0o120777
    else:
        data = b''; mode = 0
    if 'size_override' in e: data = data[:e['size_override']]
    ino += 1
    i = e.get('ino', ino if t != 'trailer' else 0)
    nlink = e.get('nlink', 2 if t == 'd' else 1 if t != 'trailer' else 1)
    name = e['name'].encode() + b'\0'
    hdr = '070701' + ''.join('%08X' % v for v in [i, mode, 0, 0, nlink, 0, len(data), 0, 0, 0, 0, len(name), 0])
    w(hdr.encode()); w(name); pad4(); w(data); pad4()
if pos % 512: w(b'\0' * (512 - pos % 512))
