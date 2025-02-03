-- Configuration
--LIGHTS1_SERVO = 13  -- Servo function for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1500  -- PWM value for medium brightness
WINCH_SERVO = 13
-- from https://ardupilot.org/rover/docs/parameters.html#servo14-function-servo-output-function
WINCH_FUNCTION = 88
winch_channel = SRV_Channels:find_channel(WINCH_FUNCTION)
if winch_channel == nil then
    gcs:send_text(6, "Set a SERVO_FUNCTION to WINCH and try restart vehicle")
end
-- States
STANDBY = 0
LIGHTS_ON = 1
RECORDING = 2
LIGHTS_OFF = 3
COMPLETE = 4

-- HTTP Configuration
HTTP_HOST = "localhost"
HTTP_PORT = 5423

-- Timing constants (in milliseconds)
LIGHTS_ON_DELAY = 10000
START_RECORDING_DELAY = 10000
LIGHTS_OFF_DELAY = 30000
STOP_RECORDING_DELAY = 92000

-- Global variables
local state = STANDBY
local timer = 0
local RC9 = rc:get_channel(9)
-- Function to control lights
function set_lights(on)
    if on then
        RC9:set_override(PWM_Lightmed)
        gcs:send_text(6, "Lights turned ON")
    else
        RC9:set_override(PWM_Lightoff)
        gcs:send_text(6, "Lights turned OFF")
    end
end

-- Function to start video recording
function start_video_recording()
    local sock = Socket(0)
    if not sock:bind("0.0.0.0", 9988) then
        gcs:send_text(6, "Failed to bind socket")
        sock:close()
        return false
    end

    if not sock:connect(HTTP_HOST, HTTP_PORT) then
        gcs:send_text(6, string.format("Failed to connect to %s:%d", HTTP_HOST, HTTP_PORT))
        sock:close()
        return false
    end

    local request = "GET /start?split_duration=90 HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    gcs:send_text(6, string.format("Sending request to http://%s:%d/start?split_duration=90", HTTP_HOST, HTTP_PORT))
    sock:send(request, string.len(request))
    sock:close()
    gcs:send_text(6, "Video recording started")
    return true
end

-- Function to stop video recording
function stop_video_recording()
    local sock = Socket(0)
    if not sock:bind("0.0.0.0", 9988) then
        gcs:send_text(6, "Failed to bind socket")
        sock:close()
        return false
    end

    if not sock:connect(HTTP_HOST, HTTP_PORT) then
        gcs:send_text(6, string.format("Failed to connect to %s:%d", HTTP_HOST, HTTP_PORT))
        sock:close()
        return false
    end

    local request = "GET /stop HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    gcs:send_text(6, string.format("Sending request to http://%s:%d/stop", HTTP_HOST, HTTP_PORT))
    sock:send(request, string.len(request))
    sock:close()
    gcs:send_text(6, "Video recording stopped")
    return true
end

function handle_sequence()
    if state == STANDBY then
        -- Start the sequence by turning on lights after delay
        if millis() > (timer + LIGHTS_ON_DELAY) then
            set_lights(true)
            state = LIGHTS_ON
            timer = millis()
            gcs:send_text(6, "State: LIGHTS ON")
        end
    elseif state == LIGHTS_ON then
        -- Start recording after lights have been on
        if millis() > (timer + START_RECORDING_DELAY) then
            if start_video_recording() then
                state = RECORDING
                timer = millis()
                gcs:send_text(6, "State: RECORDING")
            else
                gcs:send_text(6, "Failed to start recording - retrying in 5 seconds")
                timer = millis() - (START_RECORDING_DELAY - 5000)  -- Retry in 5 seconds
            end
        end
    elseif state == RECORDING then
        -- Turn off lights while still recording
        if millis() > (timer + LIGHTS_OFF_DELAY) then
            set_lights(false)
            state = LIGHTS_OFF
            timer = millis()
            gcs:send_text(6, "State: LIGHTS OFF")
        end
    elseif state == LIGHTS_OFF then
        -- Stop recording after lights have been off
        if millis() > (timer + STOP_RECORDING_DELAY) then
            if stop_video_recording() then
                state = COMPLETE
                gcs:send_text(6, "Test sequence complete!")
                return
            else
                gcs:send_text(6, "Failed to stop recording - retrying in 5 seconds")
                timer = millis() - (STOP_RECORDING_DELAY - 5000)  -- Retry in 5 seconds
            end
        end
    end
end

function loop()
    if state ~= COMPLETE then
        handle_sequence()
    end
    return loop, 100
end

function main()
    timer = millis()  -- Initialize timer
    gcs:send_text(6, "Starting lights and recording test sequence...")
    return loop, 100
end

return main, 5000 