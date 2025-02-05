from flask import Flask, jsonify, request, send_file
import os
import subprocess
from datetime import datetime
import logging
import signal
import time

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
            
        # Get split duration in minutes (default 5 minutes), convert to seconds
        split_duration = request.args.get('split_duration', default=5, type=int) * 60
        
        # Ensure the video directory exists
        os.makedirs("/app/videorecordings", exist_ok=True)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"video_{timestamp}_%03d.mp4"
        filepath = os.path.join("/app/videorecordings", filename)
        
        # Construct the command as a proper list for subprocess
        command = [
            "gst-launch-1.0",
            "-e",
            f"v4l2src device=/dev/video2 ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! splitmuxsink location={filepath} max-size-time={split_duration * 1000000000}"
        ]
        
        logger.info(f"Starting recording with command: {' '.join(command)}")
        
        # Use shell=True to properly handle the GStreamer pipeline string
        process = subprocess.Popen(
            ' '.join(command),
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Check if the process started successfully
        if process.poll() is not None:
            # Process failed to start or terminated immediately
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

""" @app.route('/stop', methods=['GET'])
def stop():
    global process, recording, start_time
    try:
        if not recording:
            return jsonify({"success": True})
        
        if process:
            logger.info("Stopping recording process...")
            
            # Kill all gst-launch-1.0 processes
            try:
                #subprocess.run(['killall', '-9', 'gst-launch-1.0'], check=False)
                subprocess.run(['killall', '-2', 'gst-launch-1.0'], check=False)       
            except Exception as e:
                logger.warning(f"Error killing gst-launch: {str(e)}")
            
            # Also terminate our subprocess
            try:
                process.kill()
                process.wait(timeout=1)
            except:
                pass
        
        recording = False
        start_time = None
        process = None
        
        logger.info("Recording stopped successfully")
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        # One final attempt to kill everything
        try:
            subprocess.run(['killall', '-9', 'gst-launch-1.0'], check=False)
        except:
            pass
        recording = False
        start_time = None
        process = None
        return jsonify({"success": False, "message": str(e)}), 500 """

@app.route('/stop', methods=['GET'])
def stop():
    global process, recording, start_time
    try:
        if not recording:
            return jsonify({"success": True})
        
        if process:
            logger.info("Stopping recording process gracefully...")
            
            # Send SIGINT to the process so that gst-launch-1.0 can flush EOS.
            try:
                process.send_signal(signal.SIGINT)
                # Optionally, wait a bit longer for a graceful shutdown.
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("Process did not exit gracefully, force killing")
                process.kill()
                process.wait(timeout=1)
        
        recording = False
        start_time = None
        process = None
        
        logger.info("Recording stopped successfully")
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        # Fallback in case something goes wrong.
        try:
            subprocess.run(['killall', '-SIGINT', 'gst-launch-1.0'], check=False)
        except Exception as ex:
            logger.warning(f"Fallback error sending SIGINT: {str(ex)}")
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