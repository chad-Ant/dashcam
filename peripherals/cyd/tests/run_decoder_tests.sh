#!/bin/bash
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JPEGDEC_SRC="${JPEGDEC_SRC:-/root/Arduino/libraries/JPEGDEC/src}"
TEST_BUILD="$(mktemp -d)"
trap 'rm -rf "$TEST_BUILD"' EXIT
python3 - "$TEST_BUILD" <<'PY'
from PIL import Image, ImageDraw
from pathlib import Path
import sys
out=Path(sys.argv[1]); im=Image.new('RGB',(320,224)); draw=ImageDraw.Draw(im)
for left,right,color in [(0,105,'red'),(106,212,'lime'),(213,319,'blue')]:
    draw.rectangle((left,0,right,223),fill=color)
for name,sub in [('420',2),('422',1),('444',0)]: im.save(out/(name+'.jpg'),quality=90,subsampling=sub)
im.convert('L').save(out/'gray.jpg')
im.save(out/'progressive.jpg',progressive=True)
im.resize((512,384)).save(out/'large.jpg')
(out/'broken.jpg').write_bytes(b'\xff\xd8broken\xff\xd9')
PY
# Upstream SIMD intentionally uses unaligned/wrapping arithmetic: ASan vendor,
# ASan + UBSan on our wrapper, as in the ES3C28P decoder tests.
c++ -std=c++11 -D__LINUX__ -I"$JPEGDEC_SRC" -fsanitize=address -g \
    -c "$JPEGDEC_SRC/JPEGDEC.cpp" -o "$TEST_BUILD/jpegdec.o"
c++ -std=c++11 -D__LINUX__ -I"$TEST_DIR/stubs" -I"$JPEGDEC_SRC" \
    -fsanitize=address,undefined -fno-sanitize-recover=all -g \
    "$TEST_DIR/map_decoder_test.cpp" "$TEST_BUILD/jpegdec.o" -o "$TEST_BUILD/decoder"
"$TEST_BUILD/decoder" "$TEST_BUILD"
c++ -std=c++17 -D__LINUX__ -I"$TEST_DIR/stubs" -I"$JPEGDEC_SRC" \
    -I"$TEST_DIR/../lib/cydNavigation" -fsanitize=address,undefined -fno-sanitize-recover=all -g \
    "$TEST_DIR/navigation_link_test.cpp" "$TEST_DIR/../lib/cydNavigation/cydNavigation.cpp" \
    "$TEST_BUILD/jpegdec.o" -o "$TEST_BUILD/link"
"$TEST_BUILD/link" "$TEST_BUILD"
