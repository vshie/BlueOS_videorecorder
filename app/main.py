from flask import Flask, jsonify, request, send_file
import os
import subprocess
from datetime import datetime
import logging
import signal
import time
import shlex
import requests
import threading

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variables
process = None
recording = False
start_time = None
subtitle_thread = None
stop_subtitle_thread = False
current_subtitle_file = None

# Mavlink URLs
ahrs2_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/AHRS2'
vfr_hud_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/VFR_HUD'
baro_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/SCALED_PRESSURE2'

def create_subtitle_file(video_path):
    """Create a new .ass subtitle file and write the header"""
    subtitle_path = video_path.replace('.mp4', '.ass')
    
    # ASS subtitle format header
    header = """[Script Info]
Title: Telemetry Data
ScriptType: v4.00+
WrapStyle: 0
PlayResX: 1920
PlayResY: 1080
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,Arial,54,&H00FFFFFF,&H000000FF,&H00000000,&H80000000,-1,0,0,0,100,100,0,0,1,2,0,8,10,10,10,1
Style: Telemetry,Arial,48,&H00FFFFFF,&H000000FF,&H00000000,&H80000000,-1,0,0,0,100,100,0,0,1,2,1,8,10,10,50,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    
    with open(subtitle_path, 'w') as f:
        f.write(header)
    
    return subtitle_path

def update_subtitles():
    """Update subtitle file with current telemetry data"""
    global stop_subtitle_thread, current_subtitle_file, start_time
    
    subtitle_update_rate = 2  # Updates per second
    
    while not stop_subtitle_thread and recording and current_subtitle_file:
        try:
            # Get current timestamp relative to recording start
            if start_time:
                elapsed = (datetime.now() - start_time).total_seconds()
                start_timestamp = format_timestamp(elapsed)
                end_timestamp = format_timestamp(elapsed + 1/subtitle_update_rate)
                
                # Fetch telemetry data
                depth = get_depth_data()
                vfr_data = get_vfr_hud_data()
                baro_data = get_baro_data()
                
                # Format subtitle text - using alignment tag \an8 for top center
                subtitle_text = f"Dialogue: 0,{start_timestamp},{end_timestamp},Telemetry,,0,0,0,,{{\\an8}}Depth: {depth:.1f}m | Climb: {vfr_data:.2f}m/s | Temp: {baro_data:.1f}°C | Time: {datetime.now().strftime('%H:%M:%S')}"
                
                # Append to subtitle file
                with open(current_subtitle_file, 'a') as f:
                    f.write(subtitle_text + '\n')
                
            time.sleep(1/subtitle_update_rate)
        except Exception as e:
            logger.error(f"Error updating subtitles: {str(e)}")
            time.sleep(1)

def format_timestamp(seconds):
    """Format seconds into ASS timestamp format (H:MM:SS.cc)"""
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    seconds = seconds % 60
    centiseconds = int((seconds - int(seconds)) * 100)
    return f"{hours}:{minutes:02d}:{int(seconds):02d}.{centiseconds:02d}"

def get_depth_data():
    """Get depth data from AHRS2 message (altitude is negative underwater)"""
    try:
        response = requests.get(ahrs2_url)
        if response.status_code == 200:
            # In ArduSub, altitude is negative for depth underwater
            altitude = response.json()['message'].get('altitude', 0.0)
            # Convert altitude to depth (positive value for underwater)
            depth = -altitude if altitude < 0 else 0.0
            return depth
    except Exception as e:
        logger.error(f"Error fetching depth data: {str(e)}")
    return 0.0

def get_vfr_hud_data():
    """Get climb rate from VFR_HUD message"""
    try:
        response = requests.get(vfr_hud_url)
        if response.status_code == 200:
            climb = response.json()['message'].get('climb', 0.0)
            return climb
    except Exception as e:
        logger.error(f"Error fetching VFR_HUD data: {str(e)}")
    return 0.0

def get_baro_data():
    """Get temperature from SCALED_PRESSURE2 message"""
    try:
        response = requests.get(baro_url)
        if response.status_code == 200:
            temperature = response.json()['message'].get('temperature', 0.0) / 100.0  # Convert to degrees C
            return temperature
    except Exception as e:
        logger.error(f"Error fetching baro data: {str(e)}")
    return 0.0

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/register_service')
def register_service():
    return '''
    {
        "name": "Video Recorder",
        "description": "Record video from connected cameras with telemetry subtitles",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "0.5",
        "webpage": "https://github.com/bluerobotics/blueos-video-recorder",
        "api": "https://github.com/bluerobotics/BlueOS-docker"
    }
    '''

@app.route('/start', methods=['GET'])
def start():
    global process, recording, start_time, subtitle_thread, stop_subtitle_thread, current_subtitle_file
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
        
        # Create subtitle file
        current_subtitle_file = create_subtitle_file(filepath)
        
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
        
        # Start subtitle thread
        stop_subtitle_thread = False
        subtitle_thread = threading.Thread(target=update_subtitles)
        subtitle_thread.daemon = True
        subtitle_thread.start()
        logger.info(f"Started telemetry subtitle generation for {current_subtitle_file}")
        
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
    global process, recording, start_time, subtitle_thread, stop_subtitle_thread
    try:
        if not recording:
            return jsonify({"success": True})
        
        # Stop subtitle thread
        stop_subtitle_thread = True
        if subtitle_thread:
            subtitle_thread.join(timeout=2)
        
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

@app.route('/telemetry', methods=['GET'])
def get_telemetry():
    try:
        depth = get_depth_data()
        vfr_data = get_vfr_hud_data()
        baro_data = get_baro_data()
        
        return jsonify({
            "success": True,
            "depth": round(depth, 1),
            "climb": round(vfr_data, 2),
            "temperature": round(baro_data, 1),
            "timestamp": datetime.now().strftime('%H:%M:%S')
        })
    except Exception as e:
        logger.error(f"Error in telemetry endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5423)
