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
light_depth = bind_add_param('LIGHT_D', 2, 10.0)     -- Depth to turn on lights (m)
hover_time = bind_add_param('HOVER_M', 3, 1.0)       -- Minutes to hover
surf_depth = bind_add_param('SURF_D', 4, 2.0)        -- Surface threshold
max_ah = bind_add_param('MAX_AH', 5, 12.0)           -- Max amp-hours
min_voltage = bind_add_param('MIN_V', 6, 13.0)       -- Min battery voltage
recording_depth = bind_add_param('REC_DEPTH', 7, 5.0)    -- Depth to start recording
video_split_duration = bind_add_param('REC_SPLIT', 8, 2)    -- Duration of each video split (m)
descent_speed = bind_add_param('DESC_SPEED',9,1) --Target speed to descend at (m/s)
target_depth = bind_add_param('TARGET_DEPTH',10,35) --Max depth 
hover_offset = bind_add_param('HOVER_OFF'.11,3) --hover this far above of target depth or impact (actual) max depth

predicted_divetime = dive_delay_s/60 +  hover_time + (target_depth/descend)/60 -- calculate dive time, if exceeded abort



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
hover_depth = target_depth-hover_offset -- this may be set shallower if bottom changes target depth (shallower than expected)
last_depth = 0 --used to track depth to detect collision with bottom
descent_rate = 0 --m/s
descent_throttle = 1650 -- initial guess
start_ah = 0 -- track power consumption
hover_start_time = 0  --  variable to track hover start time, determine duration


gpio:pinMode(27,0) --setup standard Navigator leak detection pin
function updateleak()
    if gpio:read(27) then 
        gcs:send_text(6, "Leak Detected!") --replace with change to abort state, log fault
        state = ABORT
    end
    return update, 1000
end
updateleak() -- run in loop

gpio:pinMode(51,0) -- set AUX 2 to input, used to connect external "arming" switch
function updateswitch()
    if gpio:read(51) then 
        gcs:send_text(6, "Mission switch closed, starting countdown") --replace with change to countdown state, log fault
        state = COUNTDOWN
    end
else
    return update, 1000
end

-- Configuration for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1600  -- PWM value for medium brightness
local RC9 = rc:get_channel(9)  -- Using Navigator output channel 9 for lights

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

-- Function to read inputs we control vehicle off of
function get_data() 
    -- need working depth code
    local descent_rate =  -ahrs:get_velocity_NED():z() --m/s (?)
    local mah = battery:consumed_mah()
    local batV = battery:voltage()
    return depth, descent_rate, mah
end

-- Mode definitions
MODE_MANUAL = 19
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2

--function to control motors - will this conflict with alt_hode mode? Does not require vehicle to be armed...
local RC5 = rc:get_channel(5)  -- Get RC channel 5 for left vertical motor
local RC6 = rc:get_channel(6)  -- Get RC channel 6 for right vertical motor
function motor_output(throttle)
    RC5:set_override(throttle)
    RC6:set_override(throttle)
    if state == ABORT then-- turn motors off!!
        RC5:set_override(1500)
        RC6:set_override(1500)
    end
end


-- State machine to control dive mission!
function control_dive_mission()
    if not updateswitch() and state ~= STANDBY then --@willian do I need to make that updateswitch function return true/false for this to work? 
        gcs:send_text(6, "Mission switch opened - aborting")
        state = ABORT
    end
    
    if state == STANDBY then
        set_lights(false)
        pdateswitch()
    end
    elseif state == COUNTDOWN then
        timer = millis()
        start_mah = battery:consumed_mah()
        if millis() > (timer + dive_delay_s:get() * 1000) then
            state = DESCENDING
            gcs:send_text(6, "Starting descent")
        end
    elseif state == DESCENDING then
        motor_output(descent_throttle)
        local depth = get_data()
        if depth > light_depth:get() then
            set_lights(on)
        end
        if depth > recording_depth:get() then
            if start_video_recording() then
                gcs:send_text(6, "Video recording started at depth")
            end
        end
        if climb_rate < 0.1 --@willian need to check signs here - you put a - in front of the code used to fetch. If descending # will be positive then? Maybe I shoul take abs value?
    elseif state == HOVERING then
        local depth, roll, mah = get_data()
        local setpoint = hover_depth  -- Assuming hover_depth is the desired depth

        -- Calculate PID output
        local pid_output = calculate_pid(setpoint, depth)

        -- Modulate RC5 and RC6 with PID output
        local throttle = 1500 + pid_output  -- Centered at 1500, adjust with PID
        RC5:set_override(throttle)
        RC6:set_override(throttle)

        -- Stop recording if back at surface depth
        if depth < surf_depth:get() then
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
  

