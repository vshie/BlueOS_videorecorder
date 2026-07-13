FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Python, build tools, and Pillow native dependencies.
# swig is required to build the lgpio C extension from source (no prebuilt
# wheel exists for this base image's Python 3.8 on arm).
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    swig \
    libjpeg-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# GStreamer
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-alsa \
    alsa-utils \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# ffmpeg for still capture and ffprobe
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# USB filesystem support (exFAT, NTFS, FAT32 is built-in)
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    exfat-fuse exfat-utils ntfs-3g \
    && rm -rf /var/lib/apt/lists/*

# Build pigpio daemon from source (not in Ubuntu/Debian repos -- Raspbian only).
# ADD downloads via the Docker daemon, bypassing container SSL cert issues.
ADD https://github.com/joan2937/pigpio/archive/master.tar.gz /tmp/pigpio.tar.gz
RUN cd /tmp && tar xf pigpio.tar.gz \
    && cd pigpio-master && make -j"$(nproc)" && make install \
    && ldconfig \
    && rm -rf /tmp/pigpio*

# Build the lg archive (liblgpio + Python lgpio) from source. lgpio is the
# Pi 5 servo/PWM backend; the lgpio pip wrapper only links against liblgpio
# (absent on Ubuntu), so we build the full library here. `make install` also
# installs the Python lgpio module (via swig, installed above).
#
# lg needs the GPIO v2 char-device uAPI (GPIO_V2_*), which only exists in
# linux/gpio.h from kernel >= 5.10. The ubuntu:20.04 base ships 5.4 headers,
# so drop in the v6.6 uapi gpio.h (matching the Pi 5's runtime kernel) before
# building. The compiled binary runs fine on the Pi's 6.6 kernel.
ADD https://raw.githubusercontent.com/torvalds/linux/v6.6/include/uapi/linux/gpio.h /usr/include/linux/gpio.h
ADD https://github.com/joan2937/lg/archive/master.tar.gz /tmp/lg.tar.gz
RUN cd /tmp && tar xf lg.tar.gz \
    && cd lg-master && make -j"$(nproc)" && make install \
    && ldconfig \
    && rm -rf /tmp/lg*

WORKDIR /app

# Python dependencies (before COPY so app-only edits don't rebuild this layer).
# Servo/PWM: on the DeckHand PCB every servo output goes through a PCA9685
# over I2C (smbus2), so Pi 4 and Pi 5 share one code path. pigpio (Pi 4 DMA)
# + lgpio (Pi 5 / RP1) remain as fallbacks for legacy direct-PWM boards;
# lgpio is installed by the lg source build above, not pip. WS2812 LED:
# rpi_ws281x (Pi 4) + a self-contained SPI driver (Pi 5) that only needs
# spidev (gpio_backend.py). smbus2 talks to the PCA9685 and the MCP7940N
# RTC (rtc_sync.py).
RUN pip3 install flask requests pigpio rpi_ws281x spidev smbus2 Pillow dalybms pyserial

RUN mkdir -p /app/videorecordings

COPY app/ .

ENV PYTHONUNBUFFERED=1
ENV FLASK_APP=main.py

EXPOSE 5423

LABEL version="1.0"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "5423/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": [\
      "/usr/blueos/extensions/videorecorder:/app/videorecordings",\
      "/dev/snd:/dev/snd",\
      "/dev:/dev"\
    ],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "5423/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    },\
    "NetworkMode": "host",\
    "Privileged": true\
  }\
}'

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[\
    {\
        "name": "Tony White",\
        "email": "tonywhite@bluerobotics.com"\
    }\
]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='\
{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="tool"

ARG REPO
ARG OWNER
LABEL readme=''
LABEL links='\
{\
        "source": "https://github.com/vshie/BlueOS_videorecorder"\
    }'
LABEL requirements="core >= 1.1"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
