import sys
src, dst, old, new = sys.argv[1:5]
s = open(src).read()
assert s.count(old) >= 1, f"pattern not found: {old!r}"
open(dst, "w").write(s.replace(old, new, 1))
