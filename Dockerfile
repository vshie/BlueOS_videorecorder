FROM python:3.11-slim

# RUN apt-get update && \
#    apt-get -y install gcc && \
#    rm -rf /var/lib/apt/lists/*

COPY app /app
RUN python -m pip install /app --extra-index-url https://www.piwheels.org/simple
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*


EXPOSE 59002/tcp

LABEL version="0.9"

ARG IMAGE_NAME

LABEL permissions='\
{\
  "ExposedPorts": {\
    "5420/tcp": {}\
  },\
  "HostConfig": {\
    "Binds":["/usr/blueos/extensions/videorecorder:/app/videorecordings"],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "59002/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
  "HostConfig": {\
  "Privileged": true,\
  "Binds":[\
    "/usr/blueos/extensions/videorecorder:/app",\
    "/dev:/dev"\
  ]\
}
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
LABEL company='{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="tool"
ARG REPO
ARG OWNER
LABEL readme=''
LABEL links='{\
        "source": ""\
    }'
LABEL requirements="core >= 1.1"

ENTRYPOINT ["python", "-u", "/app/main.py"]

#GPT generated dockerfile ### Dockerfile
# ```dockerfile
# # Use a lightweight base image for Python and Node.js
# FROM python:3.9-slim AS backend

# # Install required system packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     curl gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
#     && rm -rf /var/lib/apt/lists/*

# # Install Python dependencies
# COPY requirements.txt /app/requirements.txt
# WORKDIR /app
# RUN pip install --no-cache-dir -r requirements.txt

# # Frontend build stage
# FROM node:14 AS frontend

# # Install dependencies and build frontend
# WORKDIR /frontend
# COPY frontend /frontend
# RUN npm install && npm run build

# # Final image
# FROM python:3.9-slim

# # Copy backend and built frontend
# WORKDIR /app
# COPY --from=backend /app /app
# COPY --from=frontend /frontend/dist /app/static

# # Expose port and run server
# EXPOSE 59002
# CMD ["python", "app.py"]
# ```
