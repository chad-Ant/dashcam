FROM dustynv/l4t-ml:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    v4l-utils \
    libv4l-dev \
    libgpiod-dev \
    gpiod \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/app

CMD ["/bin/bash"]