# BlueOS Video Recorder Extension

Onboard video recording extension for use with hovering AUV running BlueOS / ArduSub, featuring automated dive mission control with external switch triggering.

## Components

### 1. BlueOS Video Recorder Extension
A Python-based web service that provides HTTP endpoints for starting and stopping video recording from USB and IP camera video devices.

**Features:**
- HTTP API for remote recording control
- H.264 video recording with configurable quality settings
- Integration with ArduPilot Lua scripts via HTTP requests

### 2. HAUV Lua Script
An ArduPilot Lua script that controls automated dive missions for a hovering AUV with external switch-based triggering.

**Features:**
- External switch-based mission initiation (GPIO pin 27 Navigator Leak sensor input)
- Automated dive sequence: countdown → descent → hover → ascent → surface
- Configurable parameters for depth, timing, and throttle settings
- Water sampling integration with relay control
- Light control synchronized with dive phases
- Safety abort functionality via switch opening
- Video recording start/stop synchronized with dive phases

## Installation

### Prerequisites
- BlueOS running on compatible hardware
- ArduSub firmware with Lua scripting enabled
- USB video device providing H.264 stream on `/dev/video2`

### BlueOS Video Recorder Extension Setup


1. **Configure video device access:**
   - Navigate to BlueOS Video Streams page
   - **Disable any existing `/dev/video2` camera streams**
   - This ensures the extension has exclusive access to the video device

### HAUV Lua Script Setup

1. **Enable ArduPilot Lua scripting:**
   - Set parameter `SCR_ENABLE` to `1` (true)
   - Restart the autopilot for changes to take effect

2. **Upload the script:**
   - Copy `HAUV.lua` to the ArduPilot scripts directory
   - The script will automatically execute on autopilot startup

3. **Hardware connections:**
   - Connect external switch to GPIO pin 27 (PWM0/RGB port on Navigator)
   - Switch should connect to 3.3V and signal pin
   - Close switch to initiate dive mission
   - Open switch during mission to abort

## Configuration

### HAUV Script Parameters

The script includes configurable parameters accessible through Mission Planner or similar GCS:

- `HOVER_DELAY_S`: Countdown delay before dive (default: 30 seconds)
- `HOVER_LIGHT_D`: Depth to turn on lights (default: 7.0m)
- `HOVER_HOVER_M`: Hover duration in minutes (default: 1.0)
- `HOVER_SURF_D`: Surface threshold depth (default: 2.0m)
- `HOVER_MAX_AH`: Maximum amp-hours before abort (default: 12.0)
- `HOVER_MIN_V`: Minimum battery voltage (default: 13.0V)
- `HOVER_REC_DEPTH`: Depth to start video recording (default: 5.0m)
- `HOVER_T_DEPTH`: Target maximum depth (default: 60m)
- `HOVER_D_THRTL`: Descent throttle setting (default: 1750)
- `HOVER_A_THRTL`: Ascent throttle setting (default: 1460)
- `HOVER_SIM_MODE`: Simulation mode (0=normal, 1=simulation)

### Video Recorder Configuration

The extension uses HTTP endpoints:
- `GET /start` - Start video recording
- `GET /stop` - Stop video recording
- Default port: 5423

## Mission Flow

1. **Standby**: Vehicle waits with switch open
2. **Countdown**: Switch closed triggers countdown period
3. **Descent**: Vehicle descends to target depth with video recording
4. **Hover**: Vehicle hovers at target depth for specified duration
5. **Ascent**: Vehicle ascends to surface with lights controlled by depth
6. **Surface**: Vehicle completes mission and disarms

## Safety Features

- **External switch abort**: Opening switch during mission triggers immediate abort
- **Battery monitoring**: Mission aborts if voltage or amp-hours exceed limits
- **Timeout protection**: Missions have configurable time limits
- **Depth collision detection**: Automatic hover depth adjustment on impact detection
- **Automatic disarm**: Vehicle disarms on mission completion or abort

## Troubleshooting

### Common Issues

1. **Script not executing:**
   - Verify `SCR_ENABLE` parameter is set to `1`
   - Restart autopilot after parameter change
   - Check script file is in correct directory

2. **Video recording fails:**
   - Ensure `/dev/video2` streams are disabled in BlueOS Video Streams
   - Verify video device provides H.264 stream
   - Check extension web service is running on port 5423

3. **Switch not responding:**
   - Verify GPIO pin 27 connection
   - Check switch wiring to 3.3V and signal pin
   - Monitor GCS messages for switch state updates

## Development

This project is in active development. For issues, feature requests, or contributions, please refer to the project repository.

## License

See LICENSE file for details.
