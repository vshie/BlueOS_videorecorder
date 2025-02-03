from flask import Flask, request, jsonify, send_from_directory, send_file
import os
import threading
import subprocess
import glob
import logging
from datetime import datetime, timezone

# Get the directory containing the current file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, 'static')

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variables
recording = False
process = None
start_time = None

def start_recording():
    global recording, process, start_time
    try:
        if recording:
            return False
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"video_{timestamp}_%03d.mp4"
        filepath = os.path.join("/app/videorecordings", filename)
        
        command = [
            "gst-launch-1.0", "-e",
            "v4l2src", "device=/dev/video2",
            "!", "video/x-h264,width=1920,height=1080,framerate=30/1",
            "!", "h264parse",
            "!", "splitmuxsink",
            f"location={filepath}",
            "max-size-time=30000000000"
        ]
        
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        recording = True
        start_time = datetime.now(timezone.utc)
        return True
        
    except Exception as e:
        logger.error(f"Failed to start recording: {str(e)}")
        return False

def stop_recording():
    global recording, process, start_time
    try:
        if not recording:
            return False
            
        if process:
            process.terminate()
            process.wait(timeout=5)
            process = None
            
        recording = False
        start_time = None
        return True
        
    except Exception as e:
        logger.error(f"Failed to stop recording: {str(e)}")
        return False

@app.route('/')
def index():
    return send_from_directory(STATIC_DIR, 'index.html')

@app.route('/api/v1/recording/start', methods=['POST'])
def api_start_recording():
    if start_recording():
        return jsonify({"status": "success", "message": "Recording started"})
    return jsonify({"status": "error", "message": "Failed to start recording"}), 500

@app.route('/api/v1/recording/stop', methods=['POST'])
def api_stop_recording():
    if stop_recording():
        return jsonify({"status": "success", "message": "Recording stopped"})
    return jsonify({"status": "error", "message": "Failed to stop recording"}), 500

@app.route('/api/v1/recording/status', methods=['GET'])
def api_recording_status():
    return jsonify({
        "recording": recording,
        "start_time": start_time.isoformat() if start_time else None
    })

@app.route('/api/v1/recordings', methods=['GET'])
def list_recordings():
    recordings = []
    for file in sorted(glob.glob("/app/videorecordings/*.mp4")):
        filename = os.path.basename(file)
        size = os.path.getsize(file)
        mtime = os.path.getmtime(file)
        recordings.append({
            "filename": filename,
            "size": size,
            "mtime": mtime
        })
    return jsonify(recordings)

@app.route('/api/v1/recordings/<path:filename>', methods=['GET'])
def download_recording(filename):
    return send_file(
        os.path.join("/app/videorecordings", filename),
        as_attachment=True,
        download_name=filename
    )

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5423)
