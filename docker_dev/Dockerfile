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

# VPI is distributed through NVIDIA's Jetson repository rather than Ubuntu's
# repositories.  Keep this on the r36.4 line to match the l4t-ml base image.
# libstereocam and the production `dashcam` target need the VPI headers and
# unversioned linker entry supplied by vpi3-dev.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        gnupg \
    && apt-key adv --fetch-key \
        https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
    && printf '%s\n' \
        'deb https://repo.download.nvidia.com/jetson/common r36.4 main' \
        > /etc/apt/sources.list.d/nvidia-jetson-common.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        libnvvpi3 \
        vpi3-dev \
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

# NVIDIA's container startup checks loaded kernel modules with lsmod.
RUN apt-get update && apt-get install -y --no-install-recommends \
        kmod \
    && rm -rf /var/lib/apt/lists/*

# ESP32-C3 bridge tooling: build and flash peripherals/esp32-c3 from the Jetson
# (peripherals/esp32-c3/build_and_upload.sh) and inspect the chip.
#   - arduino-cli + the esp32 core: the verified firmware build (same FQBN as
#     BuildAndUpload.cmd).  Core pinned to 3.2.0 (ESP-IDF 5.4.1), the version
#     the bridge firmware was last built with; the core bundles the RISC-V
#     toolchain and its own esptool for uploads.
#   - esptool (pip): chip_id / read_flash / erase diagnostics.  The base image
#     points pip at the Jetson wheel mirror, which does not carry it, so this
#     one install uses public PyPI.
#   - picocom: serial console for bring-up.
# The core is ~1.2 GB from GitHub: the install retries, keeping finished
# downloads, so one dropped connection does not fail the build.
# Kept last so the layers above stay cached.
ARG ARDUINO_ESP32_CORE=3.2.0
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        picocom \
    && rm -rf /var/lib/apt/lists/* \
    && curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        | BINDIR=/usr/local/bin sh \
    && arduino-cli config init --overwrite \
    && arduino-cli config add board_manager.additional_urls \
        https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json \
    && arduino-cli core update-index \
    && for i in 1 2 3; do \
           arduino-cli core install "esp32:esp32@${ARDUINO_ESP32_CORE}" && break; \
           echo "core install attempt $i failed; retrying (finished downloads are kept)"; sleep 10; \
       done \
    && arduino-cli core list | grep -q "^esp32:esp32 *${ARDUINO_ESP32_CORE}" \
    && PIP_INDEX_URL=https://pypi.org/simple pip3 install esptool \
    && arduino-cli version && esptool.py version

# MKR Zero telemetry-master tooling: build and flash peripherals/mkr_zero from
# the Jetson (peripherals/mkr_zero/build_and_upload.sh) over the MKR's
# development USB port.
#   - arduino:samd pinned to 1.8.14: vendor/Wire is a patched copy of that
#     core's Wire and uses its private SERCOM API (the build script refuses any
#     other core).  arduino-cli cannot install it on aarch64: one tool
#     dependency, avrdude 6.3.0-arduino9, has no aarch64 build — the MKR boards
#     upload with bossac, never avrdude — so install_samd_aarch64.py places the
#     core and its other tools from Arduino's own index, checksum-verified.
#   - The sketch's external libraries, pinned to the versions it was verified
#     with on the Jetson (2026-09-25), without their example-only dependencies.
#     downloads.arduino.cc drops connections now and then (2026-10-12: EOF on
#     the last library), so every download from it retries; the gates are the
#     checks that follow (core list, every library name@version).  RUN uses the
#     base image's SHELL (/bin/bash -c); the snippets stay POSIX anyway.
ARG ARDUINO_SAMD_CORE=1.8.14
COPY install_samd_aarch64.py /tmp/install_samd_aarch64.py
RUN for i in 1 2 3; do arduino-cli core update-index && break; sleep 10; done \
    && python3 /tmp/install_samd_aarch64.py "$(arduino-cli config get directories.data)" "${ARDUINO_SAMD_CORE}" \
    && arduino-cli core list | grep -q "^arduino:samd *${ARDUINO_SAMD_CORE}" \
    && for i in 1 2 3; do arduino-cli lib update-index && break; sleep 10; done \
    && set -- \
        "Time@1.6.1" \
        "SparkFun u-blox GNSS Arduino Library@2.2.29" \
        "Servo@1.3.0" \
        "RTCZero@1.6.0" \
        "Adafruit BusIO@1.17.4" \
        "Adafruit GFX Library@1.12.6" \
        "Adafruit SSD1306@2.5.17" \
        "Adafruit LED Backpack Library@1.5.1" \
    && for i in 1 2 3; do \
           arduino-cli lib install --no-deps "$@" && break; \
           echo "library install attempt $i failed; retrying"; sleep 10; \
       done \
    && for lib in "$@"; do \
           arduino-cli lib list | grep -q "^${lib%@*} *${lib#*@} " \
               || { echo "library $lib is not installed"; exit 1; }; \
       done \
    && rm /tmp/install_samd_aarch64.py

# CYD (ESP32-2432S028R "Cheap Yellow Display") dashboard tooling: build and
# flash peripherals/cyd from the Jetson (peripherals/cyd/build_and_upload.sh).
# The CYD is a classic ESP32: the esp32 core above already carries its Xtensa
# toolchain.  It uploads over a CH340 USB-serial chip, whose host driver the
# Tegra kernel does not ship (peripherals/cyd/README.md: build it with
# host/ch341_module.sh).  Adafruit ILI9341 is pinned to the version the
# bring-up sketch was verified with (2026-10-12); --no-deps skips the
# touch-controller libraries only its examples use.  GFX/BusIO come from the
# MKR layer.
RUN for i in 1 2 3; do \
           arduino-cli lib update-index && arduino-cli lib install --no-deps "Adafruit ILI9341@1.6.4" "JPEGDEC@1.8.4" && break; \
           echo "library install attempt $i failed; retrying"; sleep 10; \
       done \
    && arduino-cli lib list | grep -q "^Adafruit ILI9341 *1.6.4 " \
    && arduino-cli lib list | grep -q "^JPEGDEC *1.8.4 "

WORKDIR /home/app

CMD ["/bin/bash"]
