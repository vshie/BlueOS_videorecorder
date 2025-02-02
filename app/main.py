from flask import Flask, request, jsonify, send_from_directory, send_file
import os
import threading
import subprocess
import glob
import logging
print("hello we are running main.py, hello world")

# Configure Flask to serve static files from the root URL.
app = Flask(__name__, static_url_path="/static", static_folder="static") #setup flask app


# Enable logging with a basic configuration.
logging.basicConfig(level=logging.DEBUG,
                    format="%(asctime)s %(levelname)s: %(message)s")

VIDEO_DIRECTORY = "./videos"
if not os.path.exists(VIDEO_DIRECTORY):
    os.makedirs(VIDEO_DIRECTORY)

recording = False
process = None

@app.route('/')
def index():
    try:
        return send_file("static/index.html")
    except Exception as e:
        app.logger.error(f"Error serving index.html: {e}", exc_info=True)
        return jsonify({"error": "Error serving index page"}), 500

@app.route('/devices')
def list_devices():
    devices = []
    try:
        import subprocess
        # Use v4l2-ctl to get detailed device information
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        current_device_name = None
        
        for line in result.stdout.split('\n'):
            if ':' in line:  # This is a device name
                current_device_name = line.split('(')[0].strip()
            elif 'video' in line:  # This is a device path
                device_path = line.strip()
                devices.append({
                    "device": device_path,
                    "name": f"{current_device_name} ({device_path})"
                })
    except Exception as e:
        app.logger.error(f"Error listing devices: {e}", exc_info=True)
        # Fallback to basic device listing
        video_paths = glob.glob("/dev/video*")
        for video_path in video_paths:
            devices.append({
                "device": video_path,
                "name": f"Camera Device {video_path}"
            })
    
    return jsonify({"devices": devices})

@app.route('/start', methods=['POST'])
def start_recording():
    global recording, process

    if recording:
        return jsonify({"error": "Recording already in progress"}), 400

    device = request.json.get("device", "/dev/video2")
    try:
        max_duration = int(request.json.get("max_duration", 60)) * 1_000_000_000
        split_duration = int(request.json.get("split_duration", 30)) * 1_000_000_000
    except (ValueError, TypeError) as e:
        app.logger.error("Invalid duration parameters", exc_info=True)
        return jsonify({"error": "Invalid duration parameters"}), 400

    command = [
        "gst-launch-1.0", "-e",
        "v4l2src", f"device={device}",
        "!", "video/x-h264,width=1920,height=1080,framerate=30/1",  # Explicitly use H.264
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

@app.route('/stop', methods=['POST'])
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

    return 'Stopped'

@app.route('/list')
def list_videos():
    try:
        files = [f for f in os.listdir(VIDEO_DIRECTORY)
                 if os.path.isfile(os.path.join(VIDEO_DIRECTORY, f))]
    except Exception as e:
        app.logger.error("Error listing video files", exc_info=True)
        return jsonify({"error": f"Error listing videos: {str(e)}"}), 500
    return jsonify({"videos": files})

@app.route('/download/<filename>')
def download_video(filename):
    try:
        return send_from_directory(VIDEO_DIRECTORY, filename, as_attachment=True)
    except Exception as e:
        app.logger.error(f"Error sending file {filename}", exc_info=True)
        return jsonify({"error": f"Error sending file: {str(e)}"}), 500


# Global error handler
@app.errorhandler(Exception)
def handle_exception(e):
    app.logger.error("Unhandled Exception", exc_info=e)
    response = {
        "error": str(e) if app.debug else "An unknown error occurred."
    }
    return jsonify(response), 500

if __name__ == '__main__':
    # For production use a proper WSGI server; this is just for development.
    app.run(host='0.0.0.0', port=5423
            , debug=True)
