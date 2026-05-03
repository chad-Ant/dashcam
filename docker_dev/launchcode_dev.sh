#!/bin/bash

IMAGE_NAME="l4t-ml-gpio"
INIT_DIR="/user/dashcam"
OUTPUT_DIR="/media/$USER/backup"

echo "Booting Machine Vision Environment: $IMAGE_NAME..."

docker run -it --rm --network=host --privileged --ipc=host \
    --runtime nvidia \
    -w $INIT_DIR \
    -v /dev:/dev \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v $OUTPUT_DIR:/user/output \
    -v /home/$USER/dashcam:/user/dashcam \
    $IMAGE_NAME
