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
            
        # Ensure the video directory exists
        os.makedirs("/app/videorecordings", exist_ok=True)
            
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
        recording = False
        process = None
        start_time = None
        return False

def stop_recording():
    global recording, process, start_time
    try:
        if not recording:
            return True  # Return success if already stopped
            
        if process:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()  # Force kill if terminate doesn't work
                process.wait()
            
        recording = False
        process = None
        start_time = None
        return True
        
    except Exception as e:
        logger.error(f"Failed to stop recording: {str(e)}")
        # Reset state even if there's an error
        recording = False
        process = None
        start_time = None
        return False

@app.route('/')
def index():
    return send_from_directory(STATIC_DIR, 'index.html')

@app.route('/status', methods=['GET'])
def get_status():
    try:
        if process and process.poll() is not None:
            global recording, start_time
            recording = False
            start_time = None
            
        return jsonify({
            "recording": recording,
            "start_time": start_time.isoformat() if start_time else None
        })
    except Exception as e:
        logger.error(f"Error in status endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/start', methods=['GET'])
def start():
    try:
        split_duration = request.args.get('split_duration', default=30, type=int)
        success = start_recording()
        if success:
            return jsonify({"success": True})
        else:
            return jsonify({"success": False, "message": "Failed to start recording"}), 500
    except Exception as e:
        logger.error(f"Error in start endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    try:
        success = stop_recording()
        if success:
            return jsonify({"success": True})
        else:
            return jsonify({"success": False, "message": "Failed to stop recording"}), 500
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/list', methods=['GET'])
def list_videos():
    recordings = []
    for file in sorted(glob.glob("/app/videorecordings/*.mp4")):
        filename = os.path.basename(file)
        recordings.append(filename)
    return jsonify({"videos": recordings})

@app.route('/download/<path:filename>', methods=['GET'])
def download_video(filename):
    return send_file(
        os.path.join("/app/videorecordings", filename),
        as_attachment=True,
        download_name=filename
    )

# Add error handlers
@app.errorhandler(500)
def internal_error(error):
    return jsonify({"error": "Internal server error"}), 500

@app.errorhandler(404)
def not_found_error(error):
    return jsonify({"error": "Not found"}), 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5423)
