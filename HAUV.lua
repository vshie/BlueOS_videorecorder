-- Configuration parameters
PARAM_TABLE_KEY = 91
PARAM_TABLE_PREFIX = 'HOVER_'

-- Parameter binding helper functions
function bind_param(name)
    p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Add parameter table
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 32), 'could not add param table')

-- Add configurable parameters with defaults
dive_delay_s = bind_add_param('DELAY_S', 1, 15)      -- Countdown before dive
light_depth = bind_add_param('LIGHT_D', 2, 10.0)     -- Depth to turn on lights
hover_time = bind_add_param('HOVER_M', 3, 1.0)       -- Minutes to hover
surf_depth = bind_add_param('SURF_D', 4, 2.0)        -- Surface threshold
max_ah = bind_add_param('MAX_AH', 5, 12.0)           -- Max amp-hours
min_voltage = bind_add_param('MIN_V', 6, 13.0)       -- Min battery voltage
recording_depth = bind_add_param('REC_D', 7, 5.0)    -- Depth to start recording

-- PID Controller parameters
local Kp = 1.0  -- Proportional gain
local Ki = 0.0  -- Integral gain
local Kd = 0.0  -- Derivative gain

-- PID Controller variables
local integral = 0
local previous_error = 0

-- Function to calculate PID output
function calculate_pid(setpoint, current_depth)
    local error = setpoint - current_depth
    integral = integral + error
    local derivative = error - previous_error
    previous_error = error
    
    -- PID formula
    local output = Kp * error + Ki * integral + Kd * derivative
    return output
end

-- States
STANDBY = 0
COUNTDOWN = 1
DESCENDING = 2
HOVERING = 3
SURFACING = 4
ABORT = 5

-- Initialize variables
state = STANDBY
timer = 0
hover_depth = 0
last_depth = 0
descent_rate = 0
descent_throttle = 0
depth_stable_count = 0
start_ah = 0
hover_start_time = 0  -- New variable to track hover start time
PWM_Lightoff = 1000
PWM_Lightmed = 1500
LIGHTS1_SERVO = 13
LIGHTS1_FUNCTION = 13

gpio:pinMode(27,0)
function updateleak()
    if gpio:read(27) then 
        gcs:send_text(6, "Leak Detected!") --replace with change to abort state, log fault
    end
    return update, 1000
end
updateleak() -- run in loop

gpio:pinMode(51,0) -- set AUX 2 to input
function updateswitch()
    if gpio:read(51) then 
        gcs:send_text(6, "Mission switch closed, starting countdown") --replace with change to countdown state, log fault
        state = COUNTDOWN
    end
    return update, 1000
end

-- Configuration for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1600  -- PWM value for medium brightness
local RC9 = rc:get_channel(9)  -- Get RC channel 9 for lights

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

-- Function to read mavlink depth, orientation, and battery voltage / consumed mah
function get_data() 
    local descent_rate =  -ahrs:get_velocity_NED():z()
    local mah = battery:consumed_mah()
    return depth, descent_rate, mah
end

-- Mode definitions
MODE_MANUAL = 19
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2
local RC5 = rc:get_channel(5)  -- Get RC channel 5 for left vertical motor
local RC6 = rc:get_channel(6)  -- Get RC channel 9 for right vertical motor

-- Function to control dive mission!
function control_dive_mission()
    if not switch_closed() and state ~= STANDBY then
        gcs:send_text(6, "Mission switch opened - aborting")
        state = ABORT
    end
    
    if state == STANDBY then
        set_lights(false)
        if updateswitch() then
            -- Arm vehicle and set to manual mode
            if not vehicle:arm() then
                gcs:send_text(6, "Failed to arm vehicle")
                return
            end
            if not vehicle:set_mode(MODE_MANUAL) then
                gcs:send_text(6, "Failed to set manual mode")
                return
            end
            gcs:send_text(6, "Vehicle armed and in manual mode")
            
            timer = millis()
            state = COUNTDOWN
            start_mah = battery:consumed_mah()
        end
    elseif state == COUNTDOWN then
        timer = millis()
        if millis() > (timer + dive_delay_s:get() * 1000) then
            state = DESCENDING
            gcs:send_text(6, "Starting descent")
        end
    elseif state == DESCENDING then
        RC5:set_override(1300)
        RC6:set_override(1300)
        local depth = get_data()
        if depth > light_depth:get() then
            set_lights(true)
        end
        if depth > recording_depth:get() then
            if start_video_recording() then
                gcs:send_text(6, "Video recording started at depth")
            end
        end
    elseif state == HOVERING then
        local depth, roll, mah = get_data()
        local setpoint = hover_depth  -- Assuming hover_depth is the desired depth

        -- Calculate PID output
        local pid_output = calculate_pid(setpoint, depth)

        -- Modulate RC5 and RC6 with PID output
        local throttle = 1500 + pid_output  -- Centered at 1500, adjust with PID
        RC5:set_override(throttle)
        RC6:set_override(throttle)

        -- Stop recording if above recording depth
        if depth < recording_depth:get() then
            if stop_video_recording() then
                gcs:send_text(6, "Video recording stopped during surfacing")
            end
        end

        -- Check if hover time has elapsed
        if millis() > (hover_start_time + hover_time:get() * 60000) then
            gcs:send_text(6, "Hover time elapsed, transitioning to SURFACING")
            state = SURFACING
        end
    elseif state == SURFACING then
        set_lights(false)
        if stop_video_recording() then
            gcs:send_text(6, "Video recording stopped during surfacing")
        end
    elseif state == ABORT then
        set_lights(false)
        stop_video_recording()
        vehicle:disarm()
    end
    
    gcs:send_named_float("State", state)
    gcs:send_named_float("Depth", depth)
    return update, 1000
end

-- Transition to HOVERING state
function transition_to_hovering()
    hover_start_time = millis()  -- Record the start time of hovering
    state = HOVERING
    gcs:send_text(6, "Transitioning to HOVERING state")
end

  
-- HTTP Configuration
HTTP_HOST = "localhost"
HTTP_PORT = 5423

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

    local request = "GET /start?split_duration=2 HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    gcs:send_text(6, string.format("Sending request to http://%s:%d/start?split_duration=2", HTTP_HOST, HTTP_PORT))
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
  

