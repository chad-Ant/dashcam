# BASE_IMAGE is injected by build.sh:
#   - First build:      falls back to dustynv/l4t-ml:r36.4.0
#   - Subsequent builds: the previous l4t-ml-gpio:latest (archived as build_<timestamp>)
# Direct `docker build` invocations still work; they use the ARG default.
ARG BASE_IMAGE=dustynv/l4t-ml:r36.4.0
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# plugins-good / plugins-bad / gstreamer-tools ship with the NVIDIA l4t-ml base
# image via nvidia-l4t-gstreamer.  Re-installing the standard Ubuntu packages
# causes duplicate GStreamer type registrations (GstRtpSrc, GstRtpSink, …).
# plugins-ugly and plugins-base are genuinely absent or incomplete:
#   - plugins-ugly: x264enc is excluded from the NVIDIA image (patent licensing).
#   - plugins-base: the NVIDIA image omits the pango plugin (libgstpango.so),
#     so textoverlay is unavailable without explicitly installing this package.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    pkg-config \
    libunwind-dev \
    libglib2.0-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-ugly \
    libcairo2-dev \
    ffmpeg \
    v4l-utils \
    libv4l-dev \
    libgpiod-dev \
    gpiod \
    libpugixml-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/app

CMD ["/bin/bash"]