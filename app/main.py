from flask import Flask, request, jsonify, send_from_directory, send_file
import os
import threading
import subprocess
import glob
import logging
from datetime import datetime, timezone
import signal
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Get the directory containing the current file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, 'static')

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize GStreamer
Gst.init(None)

# Global variables
pipeline = None
recording = False
start_time = None
mainloop = None

def on_eos(bus, message):
    global recording, pipeline, start_time
    logger.info("Received EOS")
    if pipeline:
        pipeline.set_state(Gst.State.NULL)
    recording = False
    start_time = None
    return True

def start_recording():
    global recording, pipeline, start_time
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
        
        pipeline_str = ' '.join(command)
        
        logger.info(f"Creating pipeline: {pipeline_str}")
        
        pipeline = Gst.parse_launch(pipeline_str)
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::eos', on_eos)
        
        # Start the pipeline
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise Exception("Failed to start pipeline")
            
        recording = True
        start_time = datetime.now(timezone.utc)
        return True
        
    except Exception as e:
        logger.error(f"Failed to start recording: {str(e)}")
        recording = False
        pipeline = None
        start_time = None
        return False

def stop_recording():
    global recording, pipeline, start_time
    try:
        if not recording:
            return True  # Return success if already stopped
            
        if pipeline:
            logger.info("Sending EOS event")
            pipeline.send_event(Gst.Event.new_eos())
            # Wait briefly for EOS to be processed
            time.sleep(1)
            # Force cleanup if EOS doesn't complete
            pipeline.set_state(Gst.State.NULL)
            pipeline = None
            
        recording = False
        start_time = None
        pipeline = None
        return True
        
    except Exception as e:
        logger.error(f"Failed to stop recording: {str(e)}")
        # Reset state even if there's an error
        recording = False
        pipeline = None
        start_time = None
        return False

@app.route('/')
def index():
    return send_from_directory(STATIC_DIR, 'index.html')

@app.route('/register_service')
def register_service():
    return '''
    {
        "name": "Video Recorder",
        "description": "Record video from connected cameras. Supports automatic file splitting and download of recorded videos.",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "0.5",
        "webpage": "https://github.com/bluerobotics/blueos-video-recorder",
        "api": "https://github.com/bluerobotics/BlueOS-docker"
    }
    '''

@app.route('/status', methods=['GET'])
def get_status():
    global pipeline, recording, start_time
    try:
        if pipeline:
            state = pipeline.get_state(0)[1]
            if state != Gst.State.PLAYING:
                recording = False
                start_time = None
                pipeline = None
            
        return jsonify({
            "recording": recording,
            "start_time": start_time.isoformat() if start_time else None
        })
    except Exception as e:
        logger.error(f"Error in status endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/start', methods=['GET'])
def start():
    global pipeline, recording, start_time, mainloop
    try:
        if recording:
            return jsonify({"success": False, "message": "Already recording"}), 400
            
        split_duration = request.args.get('split_duration', default=30, type=int)
        
        # Ensure the video directory exists
        os.makedirs("/app/videorecordings", exist_ok=True)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"video_{timestamp}_%03d.mp4"
        filepath = os.path.join("/app/videorecordings", filename)
        
        # Create GStreamer pipeline
        pipeline_str = f'''v4l2src device=/dev/video2 ! 
            video/x-h264,width=1920,height=1080,framerate=30/1 ! 
            h264parse ! 
            splitmuxsink location={filepath} max-size-time={split_duration * 1000000000}'''
        
        logger.info(f"Creating pipeline: {pipeline_str}")
        
        pipeline = Gst.parse_launch(pipeline_str)
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::eos', on_eos)
        
        # Start the pipeline
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise Exception("Failed to start pipeline")
            
        recording = True
        start_time = datetime.now()
        
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in start endpoint: {str(e)}")
        recording = False
        start_time = None
        if pipeline:
            pipeline.set_state(Gst.State.NULL)
            pipeline = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    global pipeline, recording, start_time
    try:
        if not recording:
            return jsonify({"success": True})
        
        if pipeline:
            logger.info("Sending EOS event")
            pipeline.send_event(Gst.Event.new_eos())
            # Wait briefly for EOS to be processed
            time.sleep(1)
            # Force cleanup if EOS doesn't complete
            pipeline.set_state(Gst.State.NULL)
            pipeline = None
        
        recording = False
        start_time = None
        
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        recording = False
        start_time = None
        if pipeline:
            pipeline.set_state(Gst.State.NULL)
            pipeline = None
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
