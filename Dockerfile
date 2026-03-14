FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Python, build tools, and Pillow native dependencies
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
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
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# ffmpeg for still capture and ffprobe
RUN apt-get update && apt-get install -y --no-install-recommends --no-install-suggests \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Build pigpio daemon from source (not in Ubuntu/Debian repos -- Raspbian only).
# ADD downloads via the Docker daemon, bypassing container SSL cert issues.
ADD https://github.com/joan2937/pigpio/archive/master.tar.gz /tmp/pigpio.tar.gz
RUN cd /tmp && tar xf pigpio.tar.gz \
    && cd pigpio-master && make -j"$(nproc)" && make install \
    && ldconfig \
    && rm -rf /tmp/pigpio*

WORKDIR /app
COPY app/ .

# Python dependencies
RUN pip3 install flask requests pigpio rpi_ws281x Pillow

RUN mkdir -p /app/videorecordings

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
      "/dev/video2:/dev/video2"\
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

VOLUME ["/dev/video2"]

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
