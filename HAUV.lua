-- This script controls the dive mission of a hovering AUV. The vehicle must have the video recorder extension installed,
-- and the USB video device providing h264 on video 2, and removed from the Video Streams BlueOS page.
-- The script is executed each time the autopilot starts.
-- Follow the code to understand the structure, many comments are included...
PARAM_TABLE_KEY = 91
PARAM_TABLE_PREFIX = 'HAUV_'

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
hover_offset = bind_add_param('H_OFF',8,3) --hover this far above of target depth or impact (actual) max depth
target_depth = bind_add_param('T_DEPTH',9,410) --max depth, hover above this if reached (updated from 40 to 410)
hover_depth = target_depth:get()-hover_offset:get() -- this may be set shallower if bottom changes target depth (shallower than expected)
descent_throttle = bind_add_param('D_THRTL',10,1740 ) --descend at this throttle
ascent_throttle = bind_add_param('A_THRTL',11,1460) --ascend at this throttle, only if climb rate not sufficient?

-- Add simulation mode parameter
sim_mode = bind_add_param('SIM_MODE', 12, 0)  -- 0=normal, 1=simulation - change here and restart autopilot with SITL active
-- New parameters for dynamic throttle control
max_descent_rate = bind_add_param('MAX_D_RATE',13,1.0) -- Maximum descent rate (m/s)
min_ascent_rate = bind_add_param('MIN_A_RATE',14,0.5) -- Minimum ascent rate (m/s)
target_ascent_rate = bind_add_param('TGT_A_RATE',15,1.25) -- Target ascent rate (m/s)
throttle_step = bind_add_param('THRTL_STEP',16,10) -- Throttle adjustment step
timeout_buffer = bind_add_param('T_BUFFER',17,1.3) -- Buffer multiplier for timeout calculation

-- Simulation variables
sim_start_time = 0
sim_cycle_state = 0  -- 0=not started, 1=waiting to close, 2=closed, 3=waiting to open after surface, 4=open
sim_switch_timer = 0

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
last_depth = 0 --used to track depth to detect collision with bottom
descent_rate = 0 --m/s
has_disarmed = false  -- Add this new variable to track disarm status
abort_timer = 0  -- Add timer for abort sequence
start_ah = 0 -- track power consumption
hover_start_time = 0  --  variable to track hover start time, determine duration
switch_state = 1
is_recording = 0
impact_threshold = 0.2-- in m/s, speed of descent is positive
dive_timeout = 10 --minutes - need to set based on descent rate measured in deployment 2
gpio:pinMode(27,0) -- set pwm0 to input, used to connect external "arming" switch
switch_opened_after_complete = false -- Flag to track if switch was opened after mission completion

function updateswitch()
    if sim_mode:get() == 1 then
        -- Simulation mode
        if sim_cycle_state == 0 then
            -- Initialize simulation timers
            sim_start_time = millis()
            sim_cycle_state = 1
            switch_state = 0  -- Start with switch open
            gcs:send_text(6, "Simulation mode active, waiting 5s to close switch")
        elseif sim_cycle_state == 1 and (millis() > (sim_start_time + 5000)) then
            -- Close switch after 5 seconds
            switch_state = 1
            sim_cycle_state = 2
            gcs:send_text(6, "Simulation: switch closed")
        elseif sim_cycle_state == 2 and state == COMPLETE then
            -- If we've just completed a mission, start the open timer
            sim_switch_timer = millis()
            sim_cycle_state = 3
            gcs:send_text(6, "Simulation: mission complete, switch opening for 30s")
            switch_state = 0
        elseif sim_cycle_state == 3 and (millis() > (sim_switch_timer + 30000)) then
            -- After 30 seconds in the open state, close switch again
            switch_state = 1
            sim_cycle_state = 2
            gcs:send_text(6, "Simulation: switch closed for next mission")
        end
    else
        -- Normal physical switch mode
        switch_state = gpio:read(27)
    end
    
    -- Common switch handling logic
    if not switch_state then
        if state == COMPLETE then
            -- Mark that the switch has been opened after completion
            switch_opened_after_complete = true
            gcs:send_text(6, "Switch opened after mission completion - ready for reset")
        elseif state ~= STANDBY then
            state = ABORT
            gcs:send_text(6, "Switch opened - aborting mission")
        end
    end
    
    -- Allow restart from COMPLETE only if switch was opened and then closed
    if switch_state and state == COMPLETE and switch_opened_after_complete and depth < 3.0 then
        state = STANDBY
        has_disarmed = false  -- Reset disarm flag
        abort_timer = 0       -- Reset abort timer
        switch_opened_after_complete = false  -- Reset the switch toggle flag
        gcs:send_text(6, "Switch closed after reset - ready for new mission")
    end
    
    -- Allow restart from ABORT if switch is closed and we're shallow
    if switch_state and state == ABORT and depth < 3.0 then
        state = STANDBY
        has_disarmed = false  -- Reset disarm flag
        abort_timer = 0       -- Reset abort timer
        gcs:send_text(6, "Switch closed at shallow depth - ready for new mission")
    end
end

-- Configuration for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1600  
local RC9 = rc:get_channel(9)  -- Using Navigator input channel 9 for lights
local RC3 = rc:get_channel(3)  -- Using Navigator inpout channel 3 for vertical control

-- Function to control lights
function set_lights(on)
    if on then
        RC9:set_override(PWM_Lightmed)
        -- gcs:send_text(6, "Lights turned ON")
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

-- Variables for dynamic throttle control
current_descent_throttle = descent_throttle:get()
current_ascent_throttle = ascent_throttle:get()

--function to control motors - will this conflict with alt_hode mode? Does not require vehicle to be armed...

function motor_output()
    if state == ABORT then-- turn motors off!!
        SRV_Channels:set_output_pwm_chan_timeout(5-1, 1500, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, 1500, 100)
    elseif state == DESCENDING then
        -- Apply dynamic throttle control during descent
        if descent_rate > max_descent_rate:get() then
            -- Descent too fast, reduce throttle
            current_descent_throttle = math.max(1500, current_descent_throttle - throttle_step:get())
            gcs:send_text(6, string.format("Reducing descent throttle to %d, rate: %.2f", current_descent_throttle, descent_rate))
        else
            -- Gradually restore to parameter value
            current_descent_throttle = math.min(descent_throttle:get(), current_descent_throttle + throttle_step:get()/2)
        end
        
        SRV_Channels:set_output_pwm_chan_timeout(5-1, current_descent_throttle, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, current_descent_throttle, 100)
    elseif state == ASCEND_TOHOVER or state == SURFACING then
        -- Check if ascent rate is too slow (descent_rate > -min_ascent_rate means not climbing fast enough)
        if descent_rate > -min_ascent_rate:get() then --add negative because climb rate is positive downwards when fetched
            -- Not ascending fast enough, increase throttle (decrease PWM value)
            current_ascent_throttle = math.max(1200, current_ascent_throttle - throttle_step:get())
            gcs:send_text(6, string.format("Increasing ascent throttle to %d, rate: %.2f", current_ascent_throttle, descent_rate))
        elseif descent_rate < -target_ascent_rate:get() then
            -- Ascending too fast, decrease throttle (increase PWM value)
            current_ascent_throttle = math.min(1500, current_ascent_throttle + throttle_step:get()/2)
        end
        
        SRV_Channels:set_output_pwm_chan_timeout(5-1, current_ascent_throttle, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, current_ascent_throttle, 100)
    else
        -- Default behavior for other states
        SRV_Channels:set_output_pwm_chan_timeout(5-1, 1500, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, 1500, 100)
    end
end

-- Calculate dive timeout based on target depth and rates
-- Formula: timeout = (descent_time + hover_time + ascent_time) * buffer
function calculate_dive_timeout()
    local descent_time_min = target_depth:get() / (max_descent_rate:get() * 60) -- Convert to minutes
    local ascent_time_min = target_depth:get() / (min_ascent_rate:get() * 60)   -- Convert to minutes
    local hover_time_min = hover_time:get()
    
    local estimated_total_min = descent_time_min + hover_time_min + ascent_time_min
    local timeout_with_buffer = estimated_total_min * timeout_buffer:get()
    
    -- Set a minimum timeout of 10 minutes
    return math.max(10, timeout_with_buffer)
end

-- Calculate the initial dive timeout
dive_timeout = calculate_dive_timeout()

-- State machine to control dive mission! When diving, we arm and go to alt_hold, and command a constant descent throttle determined experimentally. Then after detecting low descent rate from hitting bottom, we go to stabilize mode. When we cross hoverdepth, we revertback to alt_hold for hover time then manual mode to ascend to surface passively! 
function control_dive_mission()
    if state == STANDBY then
        set_lights(false)
    end
    
    if state == STANDBY and switch_state then
        -- Only recalculate timeout when starting a new mission
        dive_timeout = calculate_dive_timeout()
        state = COUNTDOWN
        gcs:send_text(6, string.format("Switch closed - starting countdown. Timeout: %.1f min", dive_timeout))
        timer = millis() -- start overall dive clock
    elseif state == COUNTDOWN then
        arming:arm()
        if millis() > (timer + dive_delay_s:get() * 1000) then
            state = DESCENDING
            gcs:send_text(6, "Starting descent")
        end
    elseif state == DESCENDING then
        -- Check battery voltage during descent - using fixed 13.5V threshold
        if batV < 13.5 then
            state = ABORT
            gcs:send_text(6, string.format("Low battery voltage during descent: %.1fV - aborting mission", batV))
        end
        
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
        if millis() > (timer + dive_timeout*60000) then 
            state = ABORT
            gcs:send_text(6, "Dive duration timeout")
        end
        if depth > 5 and descent_rate < impact_threshold then --handles impact detection
            gcs:send_text(6, "Impact detected, transitioning to Ascend to hover")
            transition_to_hovering()
        end
        if depth > target_depth:get() then --handles target depth reached
            gcs:send_text(6, "Target depth reached")
            
            transition_to_hovering()
        end


    elseif state == ASCEND_TOHOVER then
        if depth < hover_depth then 
            vehicle:set_mode(MODE_ALT_HOLD)
            hover_start_time = millis()
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
                arming:disarm()
                is_recording = 0
                state = COMPLETE
                -- Reset switch_opened_after_complete when entering COMPLETE state
                switch_opened_after_complete = false
                gcs:send_text(6, "Mission complete - open switch to reset")
            end
        end
    elseif state == ABORT then
        if abort_timer == 0 then
            -- Initialize abort timer when first entering ABORT state
            abort_timer = millis()
            vehicle:set_mode(MODE_MANUAL)
            if not has_disarmed then
                arming:disarm()
                has_disarmed = true
            end
        end
        
        motor_output()  -- Keep motors in safe state
        
        -- Wait 60 seconds before stopping recording and lights
        if millis() > (abort_timer + 60000) then
            if is_recording == 1 then
                stop_video_recording()
                is_recording = 0
            end
            set_lights(false)
            abort_timer = 0  -- Reset timer for next abort if it happens
        end
    end

    -- Reset throttle values when transitioning states
    if state == COUNTDOWN then
        current_descent_throttle = descent_throttle:get()
        current_ascent_throttle = ascent_throttle:get()
    end
end
-- Transition to HOVERING state
function transition_to_hovering()
    hover_depth = depth - hover_offset:get()
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

    local request = "GET /start HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    gcs:send_text(6, string.format("Sending request to http://%s:%d/start", HTTP_HOST, HTTP_PORT))
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
    -- Log data to bin log at higher frequency (every 10 iterations)
    if iteration_counter % 10 == 0 then
        -- Log state to bin log (5x more frequently)
        logger:write('STA', 'State', 'i', state)
        logger:write('DCR', 'DescentRate', 'f', 'm', '-', descent_rate)
        logger:write('THR', 'Throttle', 'i', current_descent_throttle)
    end
  
    -- GCS messages and other status checks (original frequency - every 50 iterations)
    if iteration_counter % 50 == 0 then
        gcs:send_text(6,string.format("state:%d %.1f %.1f", state, depth, descent_rate))
        if state == ABORT then
            gcs:send_text(6, "Abort active")
        end
        if sim_mode:get() == 1 then
            gcs:send_text(6, string.format("Sim mode: cycle=%d switch=%d", sim_cycle_state, switch_state))
        end
        gcs:send_named_float("State", state)
        gcs:send_named_float("Depth", depth)
        
        -- Only reset counter after completing a full cycle
        iteration_counter = 0
    end
    
    return loop, 50
end

return loop()