from flask import Flask, request, jsonify, send_from_directory, send_file
import os
import threading
import subprocess
import glob
import logging

app = Flask(__name__, static_folder="static")
app.config["DEBUG"] = True  # Set False in production

# Set up logging to include timestamps and log level
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(levelname)s: %(message)s'
)

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
    video_paths = glob.glob("/dev/video*")
    for video_path in video_paths:
        device_basename = os.path.basename(video_path)
        sys_path = f"/sys/class/video4linux/{device_basename}/name"
        if os.path.exists(sys_path):
            try:
                with open(sys_path, "r") as f:
                    device_name = f.read().strip()
            except Exception as e:
                app.logger.error(f"Error reading device name for {video_path}: {e}", exc_info=True)
                device_name = f"Error: {str(e)}"
        else:
            device_name = "Unknown"
        devices.append({"device": video_path, "name": device_name})
    return jsonify({"devices": devices})

@app.route("/start", methods=["POST"])
def start_recording():
    global recording, process

    if recording:
        return jsonify({"error": "Recording already in progress"}), 400

    # Get parameters from JSON body with defaults
    device = request.json.get("device", "/dev/video2")
    try:
        max_duration = int(request.json.get("max_duration", 60)) * 1_000_000_000  # seconds to ns
        split_duration = int(request.json.get("split_duration", 10)) * 1_000_000_000
    except (ValueError, TypeError) as e:
        app.logger.error("Invalid duration parameters", exc_info=True)
        return jsonify({"error": "Invalid duration parameters"}), 400

    command = [
        "gst-launch-1.0", "-e", "v4l2src", f"device={device}",
        "!", "video/x-h264,width=1920,height=1080,framerate=30/1",
        "!", "h264parse",
        "!", "splitmuxsink",
        f"location={VIDEO_DIRECTORY}/video_%05d.mp4",
        f"max-size-time={split_duration}"
    ]

    try:
        app.logger.info(f"Starting recording with command: {' '.join(command)}")
        recording = True
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except Exception as e:
        recording = False
        app.logger.error("Error starting recording", exc_info=True)
        return jsonify({"error": f"Error starting recording: {str(e)}"}), 500

    return jsonify({"message": "Recording started"})

@app.route("/stop", methods=["POST"])
def stop_recording():
    global recording, process

    if not recording:
        return jsonify({"error": "No recording in progress"}), 400

    recording = False
    if process:
        try:
            process.terminate()
            stdout, stderr = process.communicate(timeout=10)
            app.logger.info(f"Recording process stdout: {stdout.decode('utf-8')}")
            if stderr:
                error_msg = stderr.decode("utf-8")
                app.logger.error(f"Recording process error: {error_msg}")
                return jsonify({"error": error_msg}), 500
        except Exception as e:
            app.logger.error("Error stopping recording", exc_info=True)
            return jsonify({"error": f"Error stopping recording: {str(e)}"}), 500
        finally:
            process = None

    return jsonify({"message": "Recording stopped"})

@app.route("/list", methods=["GET"])
def list_videos():
    try:
        files = [f for f in os.listdir(VIDEO_DIRECTORY)
                 if os.path.isfile(os.path.join(VIDEO_DIRECTORY, f))]
    except Exception as e:
        app.logger.error("Error listing video files", exc_info=True)
        return jsonify({"error": f"Error listing videos: {str(e)}"}), 500
    return jsonify({"videos": files})

@app.route("/download/<filename>", methods=["GET"])
def download_video(filename):
    try:
        return send_from_directory(VIDEO_DIRECTORY, filename, as_attachment=True)
    except Exception as e:
        app.logger.error(f"Error sending file {filename}", exc_info=True)
        return jsonify({"error": f"Error sending file: {str(e)}"}), 500

# Global error handler: returns detailed errors in debug mode,
# but only a generic message otherwise.
@app.errorhandler(Exception)
def handle_exception(e):
    app.logger.error("Unhandled Exception", exc_info=e)
    response = {
        "error": str(e) if app.debug else "An unknown error occurred."
    }
    return jsonify(response), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=59002)
