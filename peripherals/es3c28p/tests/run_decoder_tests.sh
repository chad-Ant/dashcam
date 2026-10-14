#!/bin/bash
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JPEGDEC_SRC="${JPEGDEC_SRC:-/root/Arduino/libraries/JPEGDEC/src}"
TEST_BUILD="$(mktemp -d)"
trap 'rm -rf "$TEST_BUILD"' EXIT
python3 - "$TEST_BUILD" <<'PY'
import sys
from PIL import Image, ImageDraw
from pathlib import Path
out = Path(sys.argv[1])
image = Image.new('RGB', (320, 180))
draw = ImageDraw.Draw(image)
for left, right, color in [(0, 105, 'red'), (106, 212, 'lime'), (213, 319, 'blue')]:
    draw.rectangle((left, 0, right, 179), fill=color)
image.save(out/'color420.jpg', quality=90, subsampling=2)
image.save(out/'color422.jpg', quality=90, subsampling=1)
image.resize((320, 224)).save(out/'color444map.jpg', quality=80, subsampling=0)
image.resize((319, 179)).save(out/'odd.jpg', quality=80)
image.convert('L').save(out/'gray.jpg')
image.save(out/'progressive.jpg', progressive=True)
image.resize((640, 360)).save(out/'large.jpg')
(out/'broken.jpg').write_bytes(b'\xff\xd8broken\xff\xd9')
PY
# JPEGDEC's architecture-specific SIMD uses intentional unaligned reads and
# wrapping arithmetic. ASan covers vendor memory access; strict UBSan also
# covers our wrapper/test without changing the pinned upstream implementation.
c++ -std=c++11 -D__LINUX__ -I"$JPEGDEC_SRC" -fsanitize=address -g \
    -c "$JPEGDEC_SRC/JPEGDEC.cpp" -o "$TEST_BUILD/jpegdec.o"
c++ -std=c++11 -D__LINUX__ -I"$JPEGDEC_SRC" -fsanitize=address,undefined -fno-sanitize-recover=all -g \
    "$TEST_DIR/jpeg_decoder_test.cpp" "$TEST_BUILD/jpegdec.o" -o "$TEST_BUILD/codec"
"$TEST_BUILD/codec" "$TEST_BUILD"
