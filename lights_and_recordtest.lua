-- Configuration
--LIGHTS1_SERVO = 13  -- Servo function for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1550  -- PWM value for medium brightness

-- States
STANDBY = 0
LIGHTS_ON = 1
RECORDING = 2
LIGHTS_OFF = 3
COMPLETE = 4
firstrun = 1
-- HTTP Configuration
HTTP_HOST = "localhost"
HTTP_PORT = 5423

-- Timing constants (in milliseconds)
LIGHTS_ON_DELAY = 2000
START_RECORDING_DELAY = 6000
LIGHTS_OFF_DELAY = 40000
STOP_RECORDING_DELAY = 50000

-- Global variables
local state = COMPLETE
local timer = 0
local RC9 = rc:get_channel(9)

-- Global variables for altitude and climb rate
global_altitude = 0
global_climb_rate = 0
local last_report_time = 0  -- Variable to track the last report time

-- Function to handle incoming MAVLink messages
function getvehicledata()
    -- try and get true position, but don't fail for no GPS lock

    global_altitude = baro:get_altitude()
    global_climb_rate =  -ahrs:get_velocity_NED():z()
    gcs:send_text(6, string.format("Altitude: %.2f meters, Climb Rate: %.2f m/s", global_altitude, global_climb_rate))
end

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


gpio:pinMode(18,0) --setup pwm0 pin for switch
function updateswitch()

    if gpio:read(18) and firstrun == 1 then 
        gcs:send_text(6, "starting!") 
        firstrun = 0
        state = STANDBY
    else
        state = COMPLETE
        gcs:send_text(6, "complete!")

    end
    return update, 1000
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

    local request = "GET /start?split_duration=1 HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    gcs:send_text(6, string.format("Sending request to http://%s:%d/start?split_duration=1", HTTP_HOST, HTTP_PORT))
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
            timer = millis()
            gcs:send_text(6, "State: LIGHTS ON")
            state = LIGHTS_ON
        end
    elseif state == LIGHTS_ON then
        -- Start recording after lights have been on
        if millis() > (timer + START_RECORDING_DELAY) then
            if start_video_recording() then
                state = RECORDING
                timer = millis()
                gcs:send_text(6, "State: RECORDING")
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
            end
        end
    end
end

function loop()
    if state ~= COMPLETE then
        handle_sequence()
    end

    -- Report altitude and climb rate every 15 seconds
    if millis() > (last_report_time + 15000) then
        getvehicledata()
        last_report_time = millis()  -- Update the last report time
    end
    updateswitch() -- run in loop
    -- Add any additional logic to handle MAVLink messages here
    return loop, 100
end

function main()
    timer = millis()  -- Initialize timer
    gcs:send_text(6, "Starting lights and recording test sequence...")
    return loop, 100
end

return main, 5000 
