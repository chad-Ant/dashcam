#!/bin/bash
# pair.sh <name> [nvarguscamerasrc properties...]
# One still from the IMX296 (Argus, 60 fps, after 150 frames of AE/AWB settling) and,
# at the same time, one from the UGREEN (MJPEG 1920x1080, read only: no control is written).
D="${SHOTS:-$(dirname "$(readlink -f "$0")")/shots}"; mkdir -p "$D"
UG=/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0
name="$1"; shift
gst-launch-1.0 -q -e nvarguscamerasrc sensor-id=0 num-buffers=150 "$@" ! \
    'video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1' ! \
    nvjpegenc quality=95 ! multifilesink location="$D/$name.csi-%03d.jpg" max-files=1 > "$D/$name.csi.log" 2>&1 &
c=$!
if [ "${UG_TOO:-1}" = 1 ]; then
    gst-launch-1.0 -q -e v4l2src device="$UG" num-buffers=75 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! \
        multifilesink location="$D/$name.ug-%03d.jpg" max-files=1 > "$D/$name.ug.log" 2>&1 &
    u=$!
fi
sleep 2
ctl="$(v4l2-ctl -d /dev/video0 --get-ctrl=exposure,gain 2>&1 | tr '\n' ' ')"
wait $c; rc=$?
[ -n "${u:-}" ] && wait $u
mv "$D/$name".csi-*.jpg "$D/$name.csi.jpg" 2>/dev/null
mv "$D/$name".ug-*.jpg "$D/$name.ug.jpg" 2>/dev/null
echo "$name: csi rc=$rc [$ctl] $(ls "$D/$name".*.jpg 2>/dev/null | xargs -n1 basename | tr '\n' ' ')"
sleep 1
