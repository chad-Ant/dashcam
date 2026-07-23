# build.sh always injects this clean upstream base.  Never use a prior app image
# as the next base: removed packages would survive in lower layers.
ARG BASE_IMAGE=dustynv/l4t-ml:r36.4.0
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# plugins-good / plugins-bad / gstreamer-tools ship with the NVIDIA l4t-ml base
# image via nvidia-l4t-gstreamer.  Re-installing the standard Ubuntu packages
# causes duplicate GStreamer type registrations (GstRtpSrc, GstRtpSink, …).
# plugins-ugly is genuinely absent: x264enc is excluded from the NVIDIA image
# (patent licensing).  v0.3 writes an ASS telemetry sidecar and does not use the
# pango/textoverlay runtime plugin, so do not install another plugins-base copy.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    pkg-config \
    libunwind-dev \
    libglib2.0-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-ugly \
    libcairo2-dev \
    ffmpeg \
    v4l-utils \
    libv4l-dev \
    libgpiod-dev \
    gpiod \
    libpugixml-dev \
    libspdlog-dev \
    libasound2-dev \
    && rm -rf /var/lib/apt/lists/*

# nmcli (from network-manager) lets the dashcam ask the OS to connect to WiFi at
# startup (libnetwork connectWifi).  It talks to the HOST's NetworkManager over
# the D-Bus socket the launchcode_*.sh scripts mount (-v /run/dbus, -v
# /run/NetworkManager); the container does NOT run its own NM service.  Kept in a
# separate layer so it does not invalidate the large build-tooling layer above.
RUN apt-get update && apt-get install -y --no-install-recommends \
    network-manager \
    && rm -rf /var/lib/apt/lists/*

# NVIDIA's plugins-bad build contains an experimental rtpmanagerbad plugin
# that registers GstRtpSrc/GstRtpSink types also owned by the stable
# plugins-good rtpmanager.  Keep the package (other Jetson plugins depend on
# it), but move only that conflicting module outside GStreamer's scan path.
RUN install -d /usr/lib/aarch64-linux-gnu/gstreamer-1.0-disabled \
    && mv /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstrtpmanagerbad.so \
          /usr/lib/aarch64-linux-gnu/gstreamer-1.0-disabled/

WORKDIR /home/app

CMD ["/bin/bash"]
