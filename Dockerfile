FROM python:3.11-slim

# Set Python to run in unbuffered mode
ENV PYTHONUNBUFFERED=1

WORKDIR /app

# Install required packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create necessary directories
RUN mkdir -p /app/static /app/videorecordings

# Copy the application code and static files
COPY app/main.py /app/
COPY app/static/* /app/static/

# Install Python dependencies
RUN pip install --no-cache-dir flask requests

# Expose the port that the application runs on
EXPOSE 5423

# Command to run the application
CMD ["python", "main.py"]

LABEL version="0.9"

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
        "source": ""\
    }'
LABEL requirements="core >= 1.1"

# Mark /dev/video2 as a volume so that it can be bound from the host at runtime.
VOLUME ["/dev/video2"]

ENTRYPOINT ["python", "-u", "/app/main.py"]
