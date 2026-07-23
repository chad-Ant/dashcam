#!/bin/bash

IMAGE_NAME="l4t-ml-gpio"
INIT_DIR="/user/dashcam"
SRC_DIR="/home/$USER/dashcam"        # repo/source + build tree (working dir)
MEDIA_DIR="/media/$USER/backup"      # backup drive holding footage/logs/configs

echo "Booting Machine Vision Environment: $IMAGE_NAME..."

docker run -it --rm --network=host --privileged --ipc=host \
    --runtime nvidia \
    -w $INIT_DIR \
    -e DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket \
    -v /dev:/dev \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /run/dbus:/run/dbus \
    -v /run/NetworkManager:/run/NetworkManager \
    -v $SRC_DIR:/user/dashcam \
    -v $MEDIA_DIR/configs:/user/output/configs \
    -v $MEDIA_DIR/footage:/user/output/footage \
    -v $MEDIA_DIR/logs:/user/output/logs \
    -v /etc/localtime:/etc/localtime:ro \
    -v /etc/timezone:/etc/timezone:ro \
    $IMAGE_NAME
