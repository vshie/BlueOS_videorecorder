from flask import Flask, request, jsonify, send_from_directory, send_file
import os
import threading
import subprocess
import glob

app = Flask(__name__, static_folder="static")
VIDEO_DIRECTORY = "./videos"

if not os.path.exists(VIDEO_DIRECTORY):
    os.makedirs(VIDEO_DIRECTORY)

recording = False
process = None

@app.route("/", methods=["GET"])
def index():
    return send_file(os.path.join(app.static_folder, "index.html"))

@app.route("/devices", methods=["GET"])
def list_devices():
    devices = []
    # Find all video device nodes
    video_paths = glob.glob("/dev/video*")
    for video_path in video_paths:
        # Extract the basename (e.g. "video0")
        device_basename = os.path.basename(video_path)
        sys_path = f"/sys/class/video4linux/{device_basename}/name"
        if os.path.exists(sys_path):
            try:
                with open(sys_path, "r") as f:
                    device_name = f.read().strip()
            except Exception as e:
                device_name = "Error reading name"
        else:
            device_name = "Unknown"
        devices.append({"device": video_path, "name": device_name})
    return jsonify({"devices": devices})


@app.route("/start", methods=["POST"])
def start_recording():
    global recording, process

    if recording:
        return jsonify({"error": "Recording already in progress"}), 400

    device = request.json.get("device", "/dev/video2")
    max_duration = int(request.json.get("max_duration", 60)) * 1_000_000_000  # Convert seconds to nanoseconds
    split_duration = int(request.json.get("split_duration", 10)) * 1_000_000_000  # Convert seconds to nanoseconds

    command = [
        "gst-launch-1.0", "-e", "v4l2src", f"device={device}",
        "!", "video/x-h264,width=1920,height=1080,framerate=30/1",
        "!", "h264parse",
        "!", "splitmuxsink",
        f"location={VIDEO_DIRECTORY}/video_%05d.mp4",
        f"max-size-time={split_duration}"
    ]

    try:
        recording = True
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except Exception as e:
        recording = False
        return jsonify({"error": str(e)}), 500

    return jsonify({"message": "Recording started"})

@app.route("/stop", methods=["POST"])
def stop_recording():
    global recording, process

    if not recording:
        return jsonify({"error": "No recording in progress"}), 400

    recording = False
    if process:
        process.terminate()
        stdout, stderr = process.communicate()
        process = None
        if stderr:
            return jsonify({"error": stderr.decode("utf-8")}), 500

    return jsonify({"message": "Recording stopped"})

@app.route("/list", methods=["GET"])
def list_videos():
    files = [f for f in os.listdir(VIDEO_DIRECTORY) if os.path.isfile(os.path.join(VIDEO_DIRECTORY, f))]
    return jsonify({"videos": files})

@app.route("/download/<filename>", methods=["GET"])
def download_video(filename):
    return send_from_directory(VIDEO_DIRECTORY, filename, as_attachment=True)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=59002)
