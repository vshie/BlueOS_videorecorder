from flask import Flask, jsonify, request, send_file
import os
import stat
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
rtsp_process = None
recording = False
start_time = None
subtitle_thread = None
stop_subtitle_thread = False
current_subtitle_file_h264 = None
current_subtitle_file_rtsp = None

# RTSP H.265 from RadCam — RTP/RTCP over UDP (explicit protocols=udp on rtspsrc).
RTSP_H265_ENDPOINT = "rtsp://admin:blue@192.168.2.10:554/stream_0"

# exploreHD USB H.264 — only record when this V4L2 device exists (RadCam is RTSP only).
USB_H264_DEVICE = "/dev/video2"


def _log_gst_process_exit(proc_name, proc):
    """Read and log GStreamer stdout/stderr after the child has exited (PIPE safe)."""
    if proc is None or proc.poll() is None:
        return
    try:
        stdout, stderr = proc.communicate(timeout=2)
    except subprocess.TimeoutExpired:
        logger.warning("%s: communicate() timed out while reading output", proc_name)
        return
    out = (stdout or b"").decode(errors="replace").strip()
    err = (stderr or b"").decode(errors="replace").strip()
    if out:
        logger.warning("%s stdout (exit %s): %s", proc_name, proc.returncode, out[-4000:])
    if err:
        logger.warning("%s stderr (exit %s): %s", proc_name, proc.returncode, err[-8000:])


def usb_h264_device_available():
    """True if the USB H.264 camera device node exists and is a character device."""
    try:
        st = os.stat(USB_H264_DEVICE)
        return stat.S_ISCHR(st.st_mode)
    except OSError:
        return False


# Mavlink URLs
ahrs2_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/AHRS2'
vfr_hud_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/VFR_HUD'
baro_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/SCALED_PRESSURE2'
rc_channels_url = 'http://host.docker.internal/mavlink2rest/mavlink/vehicles/1/components/1/messages/RC_CHANNELS'

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
    """Update subtitle files with current telemetry data for both video streams"""
    global stop_subtitle_thread, current_subtitle_file_h264, current_subtitle_file_rtsp, start_time
    
    subtitle_update_rate = 2  # Updates per second
    
    while not stop_subtitle_thread and recording and (current_subtitle_file_h264 or current_subtitle_file_rtsp):
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
                light_percentage = get_light_output()
                
                # Format subtitle text - using alignment tag \an1 for bottom left
                subtitle_text = f"Dialogue: 0,{start_timestamp},{end_timestamp},Telemetry,,0,0,0,,{{\\an1}}Depth: {depth:.1f}m | Climb: {vfr_data:.2f}m/s | Temp: {baro_data:.1f}°C | Lights: {light_percentage}% | Time: {datetime.now().strftime('%H:%M:%S')}"
                
                # Append to both subtitle files if they exist
                if current_subtitle_file_h264:
                    with open(current_subtitle_file_h264, 'a') as f:
                        f.write(subtitle_text + '\n')
                
                if current_subtitle_file_rtsp:
                    with open(current_subtitle_file_rtsp, 'a') as f:
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

def get_light_output():
    try:
        response = requests.get(rc_channels_url, timeout=1)
        data = response.json()
        
        if 'message' in data and 'chan9_raw' in data['message']:
            raw_value = data['message']['chan9_raw']
            
            # Convert from 1100-1900 range to 0-100%
            if raw_value <= 1100:
                percentage = 0
            elif raw_value >= 1900:
                percentage = 100
            else:
                percentage = round((raw_value - 1100) / 8.0)  # 800 range / 8 = percentage
                
            return percentage
        return 0  # Default to 0% if not available
    except Exception as e:
        logger.error(f"Error getting light output: {str(e)}")
        return 0

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
    global process, rtsp_process, recording, start_time, subtitle_thread, stop_subtitle_thread, current_subtitle_file_h264, current_subtitle_file_rtsp
    try:
        if recording:
            return jsonify({"success": False, "message": "Already recording"}), 400
            
        # Ensure the video directory exists
        os.makedirs("/app/videorecordings", exist_ok=True)
            
        # Add a small delay to allow cameras to initialize
        time.sleep(1)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_rtsp = f"video_rtsp_{timestamp}.mp4"
        filepath_rtsp = os.path.join("/app/videorecordings", filename_rtsp)

        record_usb_h264 = usb_h264_device_available()
        if not record_usb_h264:
            logger.info(
                "Skipping USB H.264 recording: %s not present or not a character device",
                USB_H264_DEVICE,
            )

        filepath_h264 = None
        current_subtitle_file_h264 = None
        if record_usb_h264:
            filename_h264 = f"video_h264_{timestamp}.mp4"
            filepath_h264 = os.path.join("/app/videorecordings", filename_h264)
            current_subtitle_file_h264 = create_subtitle_file(filepath_h264)

        current_subtitle_file_rtsp = create_subtitle_file(filepath_rtsp)
        
        # Set recording state and start time BEFORE starting video processes
        recording = True
        start_time = datetime.now()
        
        # Start subtitle thread immediately for perfect synchronization
        stop_subtitle_thread = False
        subtitle_thread = threading.Thread(target=update_subtitles)
        subtitle_thread.daemon = True
        subtitle_thread.start()
        
        # Log which subtitle files are being generated
        subtitle_files = []
        if current_subtitle_file_h264:
            subtitle_files.append(f"H264: {current_subtitle_file_h264}")
        if current_subtitle_file_rtsp:
            subtitle_files.append(f"RTSP: {current_subtitle_file_rtsp}")
        logger.info(f"Started telemetry subtitle generation: {'; '.join(subtitle_files)}")
        
        h264_command = None
        if record_usb_h264:
            h264_pipeline = (
                f"v4l2src device={USB_H264_DEVICE} ! "
                "video/x-h264,width=1920,height=1080,framerate=30/1 ! "
                f"h264parse ! mp4mux ! filesink location={filepath_h264}"
            )
            h264_command = ["gst-launch-1.0", "-e"] + shlex.split(h264_pipeline)

        # RTSP H.265 — towfish-style chain; transport is UDP only (not TCP interleaved).
        # h265parse config-interval=-1: re-insert VPS/SPS/PPS on each IDR (valid GStreamer API; see docs).
        # rtph265depay: omit wait-for-keyframe (towfish uses it) — property needs GStreamer >= 1.26; image has 1.16.
        mux_element = "mp4mux fragment-duration=5000"
        rtsp_pipeline = (
            f"rtspsrc location={RTSP_H265_ENDPOINT} protocols=udp is-live=true "
            "latency=5000 retry=5 timeout=5000000 "
            "! rtph265depay "
            "! h265parse config-interval=-1 "
            "! queue max-size-time=30000000000 max-size-bytes=0 max-size-buffers=0 "
            "leaky=downstream silent=true "
            f"! {mux_element} "
            f"! filesink location={filepath_rtsp} sync=false"
        )

        rtsp_command = ["gst-launch-1.0", "-e"] + shlex.split(rtsp_pipeline)

        # Start H264 recording process (USB exploreHD only — skipped if device missing)
        h264_started = False
        process = None
        if h264_command:
            try:
                process = subprocess.Popen(
                    h264_command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )

                logger.info("Starting H264 recording with command: %s", " ".join(h264_command))

                if process.poll() is not None:
                    _log_gst_process_exit("H264", process)
                    process = None
                    current_subtitle_file_h264 = None
                else:
                    h264_started = True
                    logger.info("H264 recording started successfully")
            except Exception as e:
                logger.error("Failed to start H264 recording: %s", str(e))
                process = None
                current_subtitle_file_h264 = None
        
        # Start RTSP recording process
        rtsp_started = False
        try:
            rtsp_process = subprocess.Popen(rtsp_command,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE)
            
            logger.info("Starting RTSP recording (UDP) with command: %s", " ".join(rtsp_command))
            
            if rtsp_process.poll() is not None:
                _log_gst_process_exit("RTSP", rtsp_process)
                rtsp_process = None
            else:
                rtsp_started = True
                logger.info("RTSP recording started successfully")
        except Exception as e:
            logger.error(f"Failed to start RTSP recording: {str(e)}")
            rtsp_process = None
        
        # Check if at least one stream started successfully
        if not h264_started and not rtsp_started:
            logger.error("Both video streams failed to start")
            return jsonify({"success": False, "message": "Both video streams failed to start"}), 500
        
        # Log which streams are active
        active_streams = []
        if h264_started:
            active_streams.append("H264")
        if rtsp_started:
            active_streams.append("RTSP")
        logger.info(f"Recording started successfully with streams: {', '.join(active_streams)}")
        
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
        if rtsp_process:
            try:
                rtsp_process.kill()
            except:
                pass
        process = None
        rtsp_process = None
        current_subtitle_file_h264 = None
        current_subtitle_file_rtsp = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/stop', methods=['GET'])
def stop():
    global process, rtsp_process, recording, start_time, subtitle_thread, stop_subtitle_thread, current_subtitle_file_h264, current_subtitle_file_rtsp
    try:
        if not recording:
            return jsonify({"success": True})
        
        # Stop subtitle thread
        stop_subtitle_thread = True
        if subtitle_thread:
            subtitle_thread.join(timeout=2)
        
        # Stop H264 recording process
        if process:
            logger.info("Stopping H264 recording process gracefully...")
            
            # Send SIGINT (Ctrl+C) to GStreamer for EOS
            process.send_signal(signal.SIGINT)
            
            # Wait for the process to handle EOS
            try:
                process.wait(timeout=7)
                logger.info("H264 recording process stopped successfully")
            except subprocess.TimeoutExpired:
                logger.warning("H264 process did not exit gracefully, force killing")
                process.kill()
                process.wait()
                logger.info("H264 recording process force killed")
        
        # Stop RTSP recording process
        if rtsp_process:
            logger.info("Stopping RTSP recording process gracefully...")
            
            # Send SIGINT (Ctrl+C) to GStreamer for EOS
            rtsp_process.send_signal(signal.SIGINT)
            
            # Wait for the process to handle EOS
            try:
                rtsp_process.wait(timeout=7)
                logger.info("RTSP recording process stopped successfully")
            except subprocess.TimeoutExpired:
                logger.warning("RTSP process did not exit gracefully, force killing")
                rtsp_process.kill()
                rtsp_process.wait()
                logger.info("RTSP recording process force killed")
        
        recording = False
        start_time = None
        process = None
        rtsp_process = None
        current_subtitle_file_h264 = None
        current_subtitle_file_rtsp = None
        
        logger.info("Recording stopped (all active streams finalized)")
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error in stop endpoint: {str(e)}")
        recording = False
        start_time = None
        if process:
            try:
                process.kill()
            except:
                pass
        if rtsp_process:
            try:
                rtsp_process.kill()
            except:
                pass
        process = None
        rtsp_process = None
        current_subtitle_file_h264 = None
        current_subtitle_file_rtsp = None
        return jsonify({"success": False, "message": str(e)}), 500

@app.route('/status', methods=['GET'])
def get_status():
    global process, rtsp_process, recording, start_time
    try:
        # Check if processes have died and clean up individually
        if process and process.poll() is not None:
            logger.warning("H264 recording process has died")
            _log_gst_process_exit("H264", process)
            try:
                process.kill()
            except Exception:
                pass
            process = None
            
        if rtsp_process and rtsp_process.poll() is not None:
            logger.warning("RTSP recording process has died")
            _log_gst_process_exit("RTSP", rtsp_process)
            try:
                rtsp_process.kill()
            except Exception:
                pass
            rtsp_process = None
        
        # Only stop recording if both processes are dead or None
        if (not process or process.poll() is not None) and (not rtsp_process or rtsp_process.poll() is not None):
            if recording:
                logger.info("All recording processes have stopped")
                recording = False
                start_time = None
            
        return jsonify({
            "recording": recording,
            "start_time": start_time.isoformat() if start_time else None,
            "h264_process_alive": process and process.poll() is None if process else False,
            "rtsp_process_alive": rtsp_process and rtsp_process.poll() is None if rtsp_process else False
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
        light_percentage = get_light_output()
        
        logger.info(f"Sending telemetry: depth={depth}, climb={vfr_data}, temp={baro_data}, lights={light_percentage}%")
        
        return jsonify({
            "success": True,
            "depth": round(depth, 1),
            "climb": round(vfr_data, 2),
            "temperature": round(baro_data, 1),
            "lights": light_percentage,
            "timestamp": datetime.now().strftime('%H:%M:%S')
        })
    except Exception as e:
        logger.error(f"Error in telemetry endpoint: {str(e)}")
        return jsonify({"success": False, "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5423)
