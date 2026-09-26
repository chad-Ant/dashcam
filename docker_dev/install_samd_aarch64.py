#!/usr/bin/env python3
"""Install arduino:samd@<ver> for aarch64 Linux by hand: arduino-cli refuses the
whole platform because one tool (avrdude, unused by the MKR boards, which upload
with bossac) has no aarch64 build.  Everything else comes from Arduino's own
package index, checksum-verified.  Usage: install_samd_aarch64.py <arduino data dir> [version]"""
import hashlib, json, os, shutil, sys, tarfile, tempfile, time, urllib.request
data = sys.argv[1]; ver = sys.argv[2] if len(sys.argv) > 2 else "1.8.14"
idx = json.load(open(os.path.join(data, "package_index.json")))
pk = next(p for p in idx["packages"] if p["name"] == "arduino")
plat = next(p for p in pk["platforms"] if p["architecture"] == "samd" and p["version"] == ver)
tools = {(t["name"], t["version"]): t for t in pk["tools"]}
def fetch(url, checksum, dest_dir, strip_top=True):
    algo, want = checksum.split(":", 1)
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "a")
        # A dropped connection is retried with backoff (2, 4, 8, 16 s); five
        # failures end the build with the reason, not a traceback further on.
        for attempt in range(5):
            try:
                urllib.request.urlretrieve(url, f); break
            except Exception as e:
                if attempt == 4: sys.exit(f"download failed after 5 attempts: {url}: {e}")
                wait = 2 ** (attempt + 1)
                print(f"  download failed ({e}); retry {attempt + 1}/4 in {wait} s")
                time.sleep(wait)
        h = hashlib.new(algo.lower().replace("-", ""))
        h.update(open(f, "rb").read())
        if h.hexdigest() != want: sys.exit(f"checksum mismatch for {url}")
        out = os.path.join(tmp, "x"); os.makedirs(out)
        with tarfile.open(f) as t: t.extractall(out)
        entries = os.listdir(out)
        src = os.path.join(out, entries[0]) if strip_top and len(entries) == 1 else out
        if os.path.exists(dest_dir): shutil.rmtree(dest_dir)
        os.makedirs(os.path.dirname(dest_dir), exist_ok=True)
        shutil.copytree(src, dest_dir, symlinks=True)
base = os.path.join(data, "packages", "arduino")
print(f"core samd {ver}")
fetch(plat["url"], plat["checksum"], os.path.join(base, "hardware", "samd", ver))
for d in plat["toolsDependencies"]:
    t = tools[(d["name"], d["version"])]
    sysent = next((s for s in t["systems"] if s["host"].startswith("aarch64")), None)
    if not sysent:
        print(f"skip {d['name']}@{d['version']} (no aarch64 build; not used by the MKR boards)")
        continue
    print(f"tool {d['name']}@{d['version']}")
    fetch(sysent["url"], sysent["checksum"], os.path.join(base, "tools", d["name"], d["version"]))
print("done")
