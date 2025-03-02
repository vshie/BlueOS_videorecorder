from flask import Flask, jsonify, request, send_file
import os
import subprocess
from datetime import datetime
import logging
import signal
import time
import shlex

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variables
process = None
recording = False
start_time = None

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/register_service')
def register_service():
    return '''
    {
        "name": "Video Recorder",
        "description": "Record video from connected cameras",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "0.5",
        "webpage": "https://github.com/bluerobotics/blueos-video-recorder",
        "api": "https://github.com/bluerobotics/BlueOS-docker"
    }
    '''
@app.route('/start', methods=['GET'])
def start():
    global process, recording, start_time
    try:
        if recording:
            return jsonify({"success": False, "message": "Already recording"}), 400
            
        # Ensure the video directory exists
        os.makedirs("/app/videorecordings", exist_ok=True)
            
        # Add a small delay to allow camera to initialize
        time.sleep(1)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"video_{timestamp}.mp4"
        filepath = os.path.join("/app/videorecordings", filename)
        
        pipeline = ("v4l2src device=/dev/video2 ! "
            "video/x-h264,width=1920,height=1080,framerate=30/1 ! "
            f"h264parse ! mp4mux ! filesink location={filepath}")

        command = ["gst-launch-1.0", "-e"] + shlex.split(pipeline)

        process = subprocess.Popen(command,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE)
        
        logger.info(f"Starting recording with command: {' '.join(command)}")
        
        if process.poll() is not None:
            stdout, stderr = process.communicate()
            logger.error(f"Process failed to start. stdout: {stdout.decode()}, stderr: {stderr.decode()}")
            raise Exception(f"Failed to start recording: {stderr.decode()}")
            
        recording = True
        start_time = datetime.now()
        
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in start endpoint: {str(e)}")
        recording = False
        start_time = None
        if process:
            try:
                process.kill()
            except:
                pass
        process = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    global process, recording, start_time
    try:
        if not recording:
            return jsonify({"success": True})
        
        if process:
            logger.info("Stopping recording process gracefully...")
            
            # Send SIGINT (Ctrl+C) to GStreamer for EOS
            process.send_signal(signal.SIGINT)
            
            # Wait for the process to handle EOS
            try:
                process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                logger.warning("Process did not exit gracefully, force killing")
                process.kill()
                process.wait()
        
        recording = False
        start_time = None
        process = None
        
        logger.info("Recording stopped successfully")
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        recording = False
        start_time = None
        process = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/status', methods=['GET'])
def get_status():
    global process, recording, start_time
    try:
        if process and process.poll() is not None:
            recording = False
            start_time = None
            process = None
            
        return jsonify({
            "recording": recording,
            "start_time": start_time.isoformat() if start_time else None
        })
    except Exception as e:
        logger.error(f"Error in status endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/list', methods=['GET'])
def list_videos():
    try:
        video_dir = "/app/videorecordings"
        if not os.path.exists(video_dir):
            os.makedirs(video_dir)
            
        videos = [f for f in os.listdir(video_dir) if f.endswith('.mp4')]
        videos.sort(reverse=True)  # Most recent first
        return jsonify({"videos": videos})
    except Exception as e:
        logger.error(f"Error in list endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/download/<filename>')
def download(filename):
    try:
        return send_file(
            os.path.join("/app/videorecordings", filename),
            as_attachment=True
        )
    except Exception as e:
        logger.error(f"Error in download endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5423)
