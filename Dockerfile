FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

# Copy application and install Python package
COPY app /app
RUN python -m pip install /app --extra-index-url https://www.piwheels.org/simple

EXPOSE 59002/tcp

LABEL version="0.9"

ARG IMAGE_NAME

# Merged and corrected permissions JSON
LABEL permissions='{"ExposedPorts": {"5420/tcp": {}}, "HostConfig": {"Binds": ["/usr/blueos/extensions/videorecorder:/app/videorecordings", "/usr/blueos/extensions/videorecorder:/app", "/dev:/dev"], "ExtraHosts": ["host.docker.internal:host-gateway"], "PortBindings": {"59002/tcp": [{"HostPort": ""}]}, "Privileged": true}}'

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[{"name": "Tony White", "email": "tonywhite@bluerobotics.com"}]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='{"about": "", "name": "Blue Robotics", "email": "support@bluerobotics.com"}'
LABEL type="tool"
ARG REPO
ARG OWNER
LABEL readme=''
LABEL links='{"source": ""}'
LABEL requirements="core >= 1.1"

ENTRYPOINT ["python", "-u", "/app/main.py"]
