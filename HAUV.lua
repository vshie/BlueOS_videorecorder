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
dive_delay_s = bind_add_param('DELAY_S', 1, 30)      -- Countdown before dive
light_depth = bind_add_param('LIGHT_D', 2, 7.0)     -- Depth to turn on lights (m)
hover_time = bind_add_param('HOVER_M', 3, 1.0)       -- Minutes to hover
surf_depth = bind_add_param('SURF_D', 4, 2.0)        -- Surface threshold
max_ah = bind_add_param('MAX_AH', 5, 12.0)           -- Max amp-hours
min_voltage = bind_add_param('MIN_V', 6, 13.0)       -- Min battery voltage
recording_depth = bind_add_param('REC_DEPTH', 7, 5.0)    -- Depth to start recording
target_depth = 40
hover_offset = bind_add_param('HOVER_OFF',11,3) --hover this far above of target depth or impact (actual) max depth

-- States
STANDBY = 0
COUNTDOWN = 1
DESCENDING = 2
ASCEND_TOHOVER = 3
HOVERING = 4
SURFACING = 5
ABORT = 6
COMPLETE = 7

-- Initialize variables
state = STANDBY
timer = 0
hover_depth = target_depth-hover_offset:get() -- this may be set shallower if bottom changes target depth (shallower than expected)
last_depth = 0 --used to track depth to detect collision with bottom
descent_rate = 0 --m/s
--descent_throttle = 1700 -- initial guess
start_ah = 0 -- track power consumption
hover_start_time = 0  --  variable to track hover start time, determine duration
switch_state = 0
is_recording = 0
impact_threshold = 0.2-- in m/s, speed of descent is positive
dive_timeout = 5 --minutes

gpio:pinMode(27,0) -- set pwm0 to input, used to connect external "arming" switch
function updateswitch()
    --switch_state = 1-- for testing sitl
    switch_state = gpio:read(27)
    if not switch_state and state == DESCENDING or state == HOVERING then
        state = ABORT
    end
end

-- Configuration for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1600  -- PWM value for medium brightness
local RC9 = rc:get_channel(9)  -- Using Navigator input channel 9 for lights
local RC3 = rc:get_channel(3)  -- Using Navigator inpout channel 3 for vertical control

-- Function to control lights
function set_lights(on)
    if on then
        RC9:set_override(PWM_Lightmed)
        gcs:send_text(6, "Lights turned ON")
    else
        RC9:set_override(PWM_Lightoff)
        --gcs:send_text(6, "Lights turned OFF")
    end
end

-- Function to read inputs we control vehicle off of
function get_data() 
    depth = -baro:get_altitude() -- positive downwards
    velocity = ahrs:get_velocity_NED() 
    if velocity ~= nil then
        descent_rate =  ahrs:get_velocity_NED():z() --positive downward
    end
    --mah = battery:consumed_mah(0)
    batV = battery:voltage(0)
end

-- Mode definitions
MODE_MANUAL = 19
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2

--function to control motors - will this conflict with alt_hode mode? Does not require vehicle to be armed...

function motor_output()
    
    if state == ABORT then-- turn motors off!!
        SRV_Channels:set_output_pwm_chan_timeout(5-1, 1500, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, 1500, 100)
    else
        SRV_Channels:set_output_pwm_chan_timeout(5-1, 1700, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, 1300, 100) --opposite because reverse parameter doesn't carry over
    end
end

--timer = millis()-- remember to move this when leaving SITL!

-- State machine to control dive mission! When diving, we arm and go to alt_hold, and command a constant descent throttle determined experimentally. Then after detecting low descent rate from hitting bottom, we go to stabilize mode. When we cross hoverdepth, we revertback to alt_hold for hover time then manual mode to ascend to surface passively! 
function control_dive_mission()
    -- if  not switch_state and state ~= STANDBY or state ~=COMPLETE then
    --     state = ABORT
    -- end
    -- if batV < min_voltage:get() and state ~= STANDBY and state ~= COUNTDOWN then --or (mah - start_mah) >(max_ah:get() * 1000) and state ~= COMPLETE then
    --     gcs:send_text(6, "Battery low - aborting")
    --     state = ABORT
    -- end
  
    if state == STANDBY then
        set_lights(false)
    end
    
    if state == STANDBY and switch_state then
        state = COUNTDOWN
        --start_mah = battery:consumed_mah(0)
        timer = millis() -- start overall dive clock

    elseif state == COUNTDOWN then
        if millis() > (timer + dive_delay_s:get() * 1000) then
            state = DESCENDING
            gcs:send_text(6, "Starting descent")
        end
    elseif state == DESCENDING then
        arming:arm()
        vehicle:set_mode(MODE_MANUAL)
        motor_output()

        if depth > light_depth:get() then
            set_lights(true)
        end
        if depth > recording_depth:get() and is_recording == 0 then
            if start_video_recording() then
                is_recording = 1
                gcs:send_text(6, "Video recording started at depth")
            end
        end
        if millis() > (dive_timeout*60000) then 
            state = ABORT
            gcs:send_text(6, "Dive duration timeout")
        end
        if depth > 7 and descent_rate < impact_threshold then  
            transition_to_hovering()
        end

    elseif state == ASCEND_TOHOVER then
        if depth < (target_depth) then 
            vehicle:set_mode(MODE_ALT_HOLD)
            gcs:send_text(6, "Transitioning to HOVERING state")
            hover_start_time = millis()  -- Record the start time of hovering
            state = HOVERING
        end

    elseif state == HOVERING then       
        
        if millis() > (hover_start_time + hover_time:get() * 60000) then
            gcs:send_text(6, "Hover time elapsed, transitioning to SURFACING")
            vehicle:set_mode(MODE_MANUAL)
            state = SURFACING
        end
    elseif state == SURFACING then
        if depth < surf_depth:get() then
            if stop_video_recording() then
                gcs:send_text(6, "Video recording stopped during surfacing")
                set_lights(false)
                state = COMPLETE
            end
        end
    elseif state == ABORT then
        set_lights(false)
        motor_output()
        if is_recording == 1 then
            stop_video_recording()
            is_recording = 0
        end
        vehicle:set_mode(MODE_MANUAL)
    end
end
-- Transition to HOVERING state
function transition_to_hovering()
    target_depth = depth - hover_offset:get()
    hover_start_time = millis()
    state = ASCEND_TOHOVER
    RC3:set_override(1500)
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
iteration_counter = 0

function loop()
    get_data() 
    updateswitch()
    control_dive_mission()
    -- Increment the iteration counter
    iteration_counter = iteration_counter + 1
    if iteration_counter % 50 == 0 then
        gcs:send_text(6,string.format("state:%d %.1f %.1f", state, depth, descent_rate))
        gcs:send_named_float("State", state)
        gcs:send_named_float("Depth", depth)
    end
    return loop, 100
end

return loop()
