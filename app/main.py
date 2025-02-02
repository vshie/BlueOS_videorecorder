from flask import Flask, request, jsonify, send_from_directory, send_file
import os
import threading
import subprocess
import glob
import logging
import signal
from datetime import datetime, timezone

# Get the directory containing the current file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, 'static')
VIDEO_DIRECTORY = "/app/videorecordings"

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Turn off Flask development server warning
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__, static_folder='static')

# Global variables
recording = False
process = None
start_time = None
record_lock = threading.Lock()

# Make sure directories exist
os.makedirs(STATIC_DIR, exist_ok=True)
os.makedirs(VIDEO_DIRECTORY, exist_ok=True)

# Remove the custom /app.js route since Flask will handle static files
# @app.route('/app.js')
# def serve_js():
#     return send_file("static/app.js")  # Remove this

@app.route('/')
def index():
    try:
        return send_file("static/index.html")
    except Exception as e:
        logger.error(f"Error serving index.html: {e}", exc_info=True)
        return jsonify({"error": "Error serving index page"}), 500

@app.route('/favicon.ico')
def favicon():
    return send_file("static/favicon.ico")

@app.route('/docs')
def docs():
    return jsonify({
        "openapi": "3.0.0",
        "info": {
            "title": "Video Recorder API",
            "version": "1.0.0"
        },
        "paths": {}
    })

@app.route('/v1.0/ui/')
def ui():
    return jsonify({
        "message": "UI endpoint"
    })

@app.route('/register_service')
def register_service():
    return jsonify({
        "name": "Video Recorder",
        "description": "Record video from connected cameras. Supports splitting recordings into manageable chunks and downloading recorded files.",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "0.9",
        "webpage": "",
        "api": "https://github.com/bluerobotics/BlueOS-docker"
    })

@app.route('/devices')
def list_devices():
    devices = []
    try:
        # Simpler approach using glob
        video_paths = glob.glob("/dev/video*")
        for video_path in video_paths:
            devices.append({
                "device": video_path,
                "name": f"Camera Device {video_path}"
            })
    except Exception as e:
        logger.error(f"Error listing devices: {e}", exc_info=True)
        return jsonify({"error": f"Error listing devices: {str(e)}"}), 500
    
    return jsonify({"devices": devices})

@app.route('/start', methods=['GET', 'POST'])
def start_recording():
    global recording, process, start_time

    # Use lock to ensure thread safety
    with record_lock:
        if recording:
            return jsonify({"error": "Recording already in progress"}), 400

        # Kill any existing process
        if process:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            except:
                if process:  # Double-check process still exists
                    try:
                        process.kill()  # Force kill if necessary
                        process.wait(timeout=2)
                    except:
                        pass
            process = None

        # Handle both GET and POST parameters
        if request.method == 'POST':
            data = request.json
            split_duration = data.get("split_duration", 30)
        else:  # GET
            split_duration = request.args.get("split_duration", 30)

        try:
            split_duration = int(split_duration) * 1_000_000_000
        except (ValueError, TypeError) as e:
            logger.error("Invalid duration parameter", exc_info=True)
            return jsonify({"error": "Invalid duration parameter"}), 400

        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_pattern = f"{VIDEO_DIRECTORY}/video_{timestamp}_%03d.mp4"

        command = [
            "gst-launch-1.0", "-e",
            "v4l2src", "device=/dev/video2",  # Hardcoded to video2
            "!", "video/x-h264,width=1920,height=1080,framerate=30/1",
            "!", "h264parse",
            "!", "splitmuxsink",
            f"location={filename_pattern}",
            f"max-size-time={split_duration}"
        ]

        try:
            logger.info(f"Starting recording with command: {' '.join(command)}")
            recording = True
            start_time = datetime.now().isoformat()  # Store start time
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except Exception as e:
            recording = False
            start_time = None
            process = None
            logger.error("Error starting recording", exc_info=True)
            return jsonify({"error": f"Error starting recording: {str(e)}"}), 500

        return 'Started'

@app.route('/stop', methods=['GET', 'POST'])
def stop_recording():
    global recording, process, start_time

    with record_lock:
        if not recording:
            return jsonify({"error": "No recording in progress"}), 400

        recording = False
        start_time = None
        if process:
            try:
                # Send EOS (End Of Stream) signal to gstreamer pipeline
                process.send_signal(signal.SIGINT)
                # Wait for the process to finish gracefully
                stdout, stderr = process.communicate(timeout=10)
                logger.info(f"Recording process stdout: {stdout.decode('utf-8')}")
                if stderr:
                    error_msg = stderr.decode("utf-8")
                    logger.error(f"Recording process error: {error_msg}")
                    return jsonify({"error": error_msg}), 500
            except Exception as e:
                logger.error("Error stopping recording", exc_info=True)
                if process:  # Double-check process still exists
                    try:
                        process.kill()  # Force kill if necessary
                        process.wait(timeout=2)
                    except:
                        pass
                return jsonify({"error": f"Error stopping recording: {str(e)}"}), 500
            finally:
                process = None

        return 'Stopped'

@app.route('/list')
def list_videos():
    try:
        videos = []
        for file in os.listdir(VIDEO_DIRECTORY):
            if file.endswith('.mp4'):
                videos.append(file)
        return jsonify({"videos": sorted(videos)})
    except Exception as e:
        logger.error(f"Error listing videos: {e}", exc_info=True)
        return jsonify({"error": "Error listing videos"}), 500

@app.route('/download/<filename>')
def download_video(filename):
    try:
        return send_from_directory(VIDEO_DIRECTORY, filename, as_attachment=True)
    except Exception as e:
        logger.error(f"Error sending file {filename}", exc_info=True)
        return jsonify({"error": f"Error sending file: {str(e)}"}), 500

@app.route('/docs.json')
@app.route('/openapi.json')
@app.route('/swagger.json')
def api_json():
    return jsonify({
        "openapi": "3.0.0",
        "info": {
            "title": "Video Recorder API",
            "description": "API for controlling video recording and managing recorded files",
            "version": "1.0.0"
        },
        "paths": {
            "/start": {
                "get": {
                    "summary": "Start video recording",
                    "parameters": [
                        {
                            "name": "split_duration",
                            "in": "query",
                            "description": "Duration in seconds for each video segment",
                            "required": False,
                            "schema": {
                                "type": "integer",
                                "default": 30
                            }
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Recording started successfully"
                        }
                    }
                }
            },
            "/stop": {
                "get": {
                    "summary": "Stop video recording",
                    "responses": {
                        "200": {
                            "description": "Recording stopped successfully"
                        }
                    }
                }
            },
            "/list": {
                "get": {
                    "summary": "List recorded videos",
                    "responses": {
                        "200": {
                            "description": "List of recorded video files",
                            "content": {
                                "application/json": {
                                    "schema": {
                                        "type": "object",
                                        "properties": {
                                            "videos": {
                                                "type": "array",
                                                "items": {
                                                    "type": "string"
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    })

@app.route('/status')
def get_status():
    return jsonify({
        "recording": recording,
        "start_time": start_time
    })

# Global error handler
@app.errorhandler(Exception)
def handle_exception(e):
    logger.error("Unhandled Exception", exc_info=e)
    response = {
        "error": str(e) if app.debug else "An unknown error occurred."
    }
    return jsonify(response), 500

if __name__ == '__main__':
    # Ensure we start with recording off
    recording = False
    process = None
    start_time = None
    
    # Run without debug mode
    app.run(host='0.0.0.0', port=5423, debug=False)
