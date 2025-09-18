-- This script controls the dive mission of a hovering AUV. The vehicle must have the video recorder extension installed,
-- and the USB video device providing h264 on video 2, and removed from the Video Streams BlueOS page.
-- The script is executed each time the autopilot starts.
-- Follow the code to understand the structure, many comments are included...
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
light_depth = bind_add_param('LIGHT_D', 2, 200.0)     -- Depth to turn on lights (m)
hover_time = bind_add_param('HOVER_M', 3, 2.0)       -- Minutes to hover
surf_depth = bind_add_param('SURF_D', 4, 2.0)        -- Surface threshold
max_ah = bind_add_param('MAX_AH', 5, 12.0)           -- Max amp-hours
min_voltage = bind_add_param('MIN_V', 6, 13.0)       -- Min battery voltage
recording_depth = bind_add_param('REC_DEPTH', 7, 50.0)    -- Depth to start recording
hover_offset = bind_add_param('H_OFF',8,3) --hover this far above of target depth or impact (actual) max depth
target_depth = bind_add_param('T_DEPTH',9,410) --max depth, hover above this if reached (updated from 40 to 410)
hover_depth = target_depth:get()-hover_offset:get() -- this may be set shallower if bottom changes target depth (shallower than expected)
descent_throttle = bind_add_param('D_THRTL',10,1750 ) --descend at this throttle
ascent_throttle = bind_add_param('A_THRTL',11,1460) --ascend at this throttle, only if climb rate not sufficient?

-- Add simulation mode parameter
sim_mode = bind_add_param('SIM_MODE', 12, 0)  -- 0=normal, 1=simulation - change here and restart autopilot with SITL active
-- New parameters for dynamic throttle control
max_descent_rate = bind_add_param('MAX_D_RATE',13,1.0) -- Maximum descent rate (m/s)
min_ascent_rate = bind_add_param('MIN_A_RATE',14,0.7) -- Minimum ascent rate (m/s)
throttle_step = bind_add_param('THRTL_STEP',16,2) -- Throttle adjustment step
timeout_buffer = bind_add_param('T_BUFFER',17,1.3) -- Buffer multiplier for timeout calculation
-- New parameter for surface maintaining throttle
surface_depth_threshold = bind_add_param('SURF_DEPTH',19,0.65) -- Depth threshold to increase throttle (m)
-- Water sampling parameters
ws_interval = bind_add_param('WS_INTERVAL',20,5.0) -- Water sampling interval depth (m)
ws_htime = bind_add_param('WS_HTIME',21,0.5) -- Water sampling hover time (minutes)

-- Simulation variables
sim_start_time = 0
sim_cycle_state = 0  -- 0=not started, 1=waiting to close, 2=closed, 3=waiting to open after surface, 4=open
sim_switch_timer = 0

-- States
STANDBY = 0
COUNTDOWN = 1
DESCENDING = 2
WATER_SAMPLING = 3
ASCEND_TOHOVER = 4
HOVERING = 5
SURFACING = 6
ABORT = 7
COMPLETE = 8

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
impact_detection_count = 0  -- Count consecutive slow readings
slow_zone_entered = false   -- Flag to track if we've entered slow zone
slow_zone_time = 0          -- Time when we entered slow zone
slow_zone_grace_period = 5000  -- 5 second grace period after throttle change
dive_timeout = 10 --minutes - need to set based on descent rate measured in deployment 2
gpio:pinMode(27,0) -- set pwm0 to input, used to connect external "arming" switch
switch_opened_after_complete = false -- Flag to track if switch was opened after mission completion
lights_off_reported = false  -- Flag to track if we've reported turning off lights
last_log_time = 0  -- For tracking light intensity logging

-- Add these initializations near the other global variables
hover_iteration_count = 0
hover_total_iterations = 0

-- ALT_HOLD mode tracking
last_vehicle_mode = -1
alt_hold_exit_detected = false

-- Water sampling variables
ws_next_depth = 0  -- Next water sampling depth
ws_hover_start_time = 0  -- Start time for water sampling hover
ws_relay_toggled = false  -- Track if relay is currently toggled
ws_relay_toggle_start = 0  -- Start time of relay toggle
ws_relay_duration = 1000  -- Relay toggle duration in milliseconds
ws_relay_triggered = false  -- Track if relay has been triggered for this sampling session

-- Light control simplified - no incremental changes needed

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
            -- If we've just completed a mission, start the delay timer
            sim_switch_timer = millis()
            sim_cycle_state = 3
            gcs:send_text(6, "Simulation: mission complete, waiting 60s before opening switch")
            switch_state = 1  -- Keep switch closed during delay
        elseif sim_cycle_state == 3 and (millis() > (sim_switch_timer + 60000)) then
            -- After 60 seconds delay, open the switch for 30 seconds
            sim_switch_timer = millis()  -- Reset timer for the open period
            sim_cycle_state = 4
            gcs:send_text(6, "Simulation: opening switch for 30s")
            switch_state = 0  -- Open the switch
        elseif sim_cycle_state == 4 and (millis() > (sim_switch_timer + 30000)) then
            -- After 30 seconds in the open state, close switch again
            switch_state = 1
            sim_cycle_state = 2
            gcs:send_text(6, "Simulation: switch closed for next mission")
            
            -- Force reset to STANDBY state
            if state == COMPLETE then
                state = STANDBY
                has_disarmed = false
                abort_timer = 0
                switch_opened_after_complete = false
                -- Reset water sampling variables
                ws_next_depth = 0
                ws_hover_start_time = 0
                ws_relay_toggled = false
                gcs:send_text(6, "Simulation: state reset for new mission")
            end
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
            -- Now it's safe to disarm the vehicle
            arming:disarm()
            gcs:send_text(6, "Vehicle disarmed - motors off")
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
        -- Reset water sampling variables
        ws_next_depth = 0
        ws_hover_start_time = 0
        ws_relay_toggled = false
        ws_relay_triggered = false
        gcs:send_text(6, "Switch closed after reset - ready for new mission")
    end
    
    -- Allow restart from ABORT if switch is closed and we're shallow
    if switch_state and state == ABORT and depth < 3.0 then
        state = STANDBY
        has_disarmed = false  -- Reset disarm flag
        abort_timer = 0       -- Reset abort timer
        -- Reset water sampling variables
        ws_next_depth = 0
        ws_hover_start_time = 0
        ws_relay_toggled = false
        ws_relay_triggered = false
        gcs:send_text(6, "Switch closed at shallow depth - ready for new mission")
    end
end

-- Configuration for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1300  -- Changed from 1850 to 1300 as requested
PWM_Lightmax = 1900  -- Maximum brightness
local RC9 = rc:get_channel(9)  -- Using Navigator input channel 9 for lights
local RC3 = rc:get_channel(3)  -- Using Navigator input channel 3 for vertical control

-- Variables for light control (simplified - no incremental changes)

-- Function to control lights
function set_lights(on, brightness_override)
    local pwm_value = PWM_Lightoff
    
    if on then
        if brightness_override then
            -- Use the provided brightness value, converted to a plain number
            pwm_value = brightness_override + 0  -- Adding zero forces conversion to regular number
        else
            -- Default behavior when no override provided
            pwm_value = PWM_Lightmed
        end
        RC9:set_override(pwm_value)
    else
        RC9:set_override(PWM_Lightoff)
    end
end

-- Function to control water sampling relay
function control_water_sampling_relay()
    if ws_relay_toggled then
        -- Check if relay toggle duration has elapsed
        if millis() > (ws_relay_toggle_start + ws_relay_duration) then
            -- Toggle relay back to original state
            relay:toggle(0)
            ws_relay_toggled = false
            gcs:send_text(6, "Water sampling relay toggled back to original state")
        end
    end
end

-- Function to trigger water sampling relay
function trigger_water_sampling()
    if not ws_relay_toggled then
        -- Toggle relay
        relay:toggle(0)
        ws_relay_toggled = true
        ws_relay_toggle_start = millis()
        gcs:send_text(6, "Water sampling relay triggered")
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
    
    -- Override battery voltage in simulation mode
    if sim_mode:get() == 1 then
        batV = 14.0  -- Simulate full battery in sim mode
    end
    
    -- Track vehicle mode changes to detect ALT_HOLD exits
    local current_mode = vehicle:get_mode()
    if last_vehicle_mode ~= current_mode then
        if last_vehicle_mode == MODE_ALT_HOLD and current_mode ~= MODE_ALT_HOLD then
            -- ALT_HOLD mode was exited
            alt_hold_exit_detected = true
            gcs:send_text(6, string.format("ALT_HOLD mode exited - current mode: %d", current_mode))
        end
        last_vehicle_mode = current_mode
    end
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
        -- Calculate distance to target depth
        local remaining_distance = target_depth:get() - depth
        local slow_descent_zone = 40 -- slow down when within 40m of target depth
        
        -- Apply dynamic throttle control during descent
        if remaining_distance < slow_descent_zone then
            -- Track when we first enter the slow zone
            if not slow_zone_entered then
                slow_zone_entered = true
                slow_zone_time = millis()
                gcs:send_text(6, "Entering slow descent zone")
                
                -- Set lights to 80% brightness when entering slow zone
                local light_80_percent = PWM_Lightoff + (PWM_Lightmax - PWM_Lightoff) * 0.8
                set_lights(true, light_80_percent)
                gcs:send_text(6, "Lights set to 80% brightness in slow descent zone")
            end
            
            -- Ensure lights remain at 80% brightness throughout slow zone
            local light_80_percent = PWM_Lightoff + (PWM_Lightmax - PWM_Lightoff) * 0.8
            set_lights(true, light_80_percent)
            
            -- Within 40m of target depth - reduce to 50% descent speed
            -- Calculate 50% between neutral (1500) and full descent throttle
            local reduced_throttle = 1500 + (descent_throttle:get() - 1500) * 0.5
            current_descent_throttle = reduced_throttle
            
            if descent_rate > max_descent_rate:get() * 0.5 then
                -- Still descending too fast, reduce throttle further
                current_descent_throttle = math.max(1500, current_descent_throttle - throttle_step:get())
                gcs:send_text(6, string.format("Reducing throttle in slow zone: %d, rate: %.2f", current_descent_throttle + 0, descent_rate))
            end
        else
            slow_zone_entered = false  -- Reset flag when outside slow zone
            
            -- Normal descent rate control
            if descent_rate > max_descent_rate:get() then
                -- Descent too fast, reduce throttle
                current_descent_throttle = math.max(1500, current_descent_throttle - throttle_step:get())
                gcs:send_text(6, string.format("Reducing descent throttle to %d, rate: %.2f", current_descent_throttle + 0, descent_rate))
            else
                -- Gradually restore to parameter value
                current_descent_throttle = math.min(descent_throttle:get(), current_descent_throttle + throttle_step:get()/2)
            end
        end
        
        SRV_Channels:set_output_pwm_chan_timeout(5-1, current_descent_throttle, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, current_descent_throttle, 100)
    elseif state == WATER_SAMPLING then
        -- In water sampling state, motors are controlled by ALT_HOLD mode
        -- No direct motor control needed here
    elseif state == ASCEND_TOHOVER or state == SURFACING then
        -- Set different target ascent rates for each state
        local target_ascent_rate
        if state == ASCEND_TOHOVER then
            -- Use half the ascent rate for the more controlled ascent to hover
            target_ascent_rate = min_ascent_rate:get() * 0.5
        else -- SURFACING
            -- Use full ascent rate parameter for surfacing
            target_ascent_rate = min_ascent_rate:get()
        end
        
        -- Initialize current throttle value if needed
        if not current_ascent_throttle or current_ascent_throttle == 1500 then
            current_ascent_throttle = ascent_throttle:get()
            gcs:send_text(6, string.format("Initializing ascent throttle: %d", current_ascent_throttle + 0))
        end
        
        -- In ArduSub, descent_rate is positive when going down, negative when going up
        -- So current_ascent_rate should be the negative of descent_rate
        local current_ascent_rate = -descent_rate
        
        -- Log ascent status periodically rather than every iteration
        if iteration_counter % 50 == 0 then
            gcs:send_text(6, string.format("A stat: state=%d, rate=%.2f, target=%.2f, throttle=%d", 
                state, current_ascent_rate, target_ascent_rate, current_ascent_throttle + 0))
        end
        
        -- Adjust throttle if needed
        if current_ascent_rate < target_ascent_rate then
            -- Not ascending fast enough, increase throttle (reduce PWM)
            local old_throttle = current_ascent_throttle
            current_ascent_throttle = math.max(1200, current_ascent_throttle - throttle_step:get())
            
            -- Only log when the throttle actually changes
            if old_throttle ~= current_ascent_throttle then
                gcs:send_text(6, string.format("Increasing a pow: throttle=%d->%d", 
                    old_throttle, current_ascent_throttle))
            end
        elseif current_ascent_rate > target_ascent_rate * 1.5 then
            -- Ascending too fast, gradually reduce throttle
            local old_throttle = current_ascent_throttle
            current_ascent_throttle = math.min(1500, current_ascent_throttle + throttle_step:get()/2)
            
            -- Only log when the throttle actually changes
            if old_throttle ~= current_ascent_throttle then
                gcs:send_text(6, string.format("Reducing ascent power: throttle=%d->%d", 
                    old_throttle, current_ascent_throttle))
            end
        end
        
        -- Apply current throttle value to both motors
        SRV_Channels:set_output_pwm_chan_timeout(5-1, current_ascent_throttle, 100)
        SRV_Channels:set_output_pwm_chan_timeout(6-1, current_ascent_throttle, 100)
    else
        -- Default behavior for all other states
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
    local final_timeout = math.max(10, timeout_with_buffer)
    
    -- Print just the final calculated timeout
    gcs:send_text(6, string.format("Calculated dive timeout: %.1f minutes", final_timeout))
    
    return final_timeout
end

-- Calculate the initial dive timeout
dive_timeout = calculate_dive_timeout()

-- State machine to control dive mission! When diving, we arm and go to alt_hold, and command a constant descent throttle determined experimentally. Then after detecting low descent rate from hitting bottom, we go to stabilize mode. When we cross hoverdepth, we revertback to alt_hold for hover time then manual mode to ascend to surface passively! 
function control_dive_mission()
    if state == STANDBY then
        set_lights(false)
        lights_off_reported = false  -- Reset the reporting flag
    end
    
    if state == STANDBY and switch_state then
        -- Only recalculate timeout when starting a new mission
        dive_timeout = calculate_dive_timeout()
        state = COUNTDOWN
        gcs:send_text(6, string.format("Switch closed - starting countdown. Timeout: %.1f min", dive_timeout))
        timer = millis() -- start overall dive clock
        
        -- Initialize water sampling depth
        ws_next_depth = ws_interval:get()  -- First sampling at WS_INTERVAL depth
        gcs:send_text(6, string.format("Water sampling initialized - first sample at %.1fm, interval %.1fm", 
            ws_next_depth, ws_interval:get()))
        
        -- Reset ALT_HOLD tracking for new mission
        alt_hold_exit_detected = false
        last_vehicle_mode = -1
        gcs:send_text(6, "ALT_HOLD tracking reset for new mission")
    elseif state == COUNTDOWN then
        arming:arm()
        -- Ensure lights are off during countdown (vehicle likely at surface)
        set_lights(false)
        
        if millis() > (timer + dive_delay_s:get() * 1000) then
            state = DESCENDING
            gcs:send_text(6, "Starting descent")
        end
    elseif state == DESCENDING then
        -- Check battery voltage during descent - using parameter value, not hardcoded
        if batV < min_voltage:get() then
            state = ABORT
            gcs:send_text(6, string.format("Low battery voltage during descent: %.1fV - aborting mission", batV))
        end
        
        vehicle:set_mode(MODE_MANUAL)
        motor_output()

        -- Explicit light control
        if depth > light_depth:get() then
            -- If in slow descent zone, always use 80% brightness
            local remaining_distance = target_depth:get() - depth
            if remaining_distance < 40 then  -- 40m is the slow_descent_zone
                local light_80_percent = PWM_Lightoff + (PWM_Lightmax - PWM_Lightoff) * 0.8
                set_lights(true, light_80_percent)
            else
                set_lights(true)  -- Default medium brightness
            end
        else
            set_lights(false)  -- Lights off when shallow
        end
        
        if depth > recording_depth:get() and is_recording == 0 then
            if start_video_recording() then
                is_recording = 1
                gcs:send_text(6, "Video recording started at depth")
            end
        end
        
        -- Check for water sampling depth
        if ws_next_depth > 0 and depth >= ws_next_depth then
            -- Initialize water sampling
            ws_next_depth = ws_next_depth + ws_interval:get()  -- Set next sampling depth
            ws_hover_start_time = millis()
            ws_relay_triggered = false  -- Reset trigger flag for new sampling session
            state = WATER_SAMPLING
            gcs:send_text(6, string.format("Starting water sampling at %.1fm", depth))
        end
        
        if millis() > (timer + dive_timeout*60000) then 
            state = ABORT
            gcs:send_text(6, "Dive duration timeout")
        end
        if depth > 5 and descent_rate < impact_threshold then
            -- Don't detect impact during the grace period after entering slow zone
            if not (slow_zone_entered and (millis() - slow_zone_time < slow_zone_grace_period)) then
                impact_detection_count = impact_detection_count + 1
                if impact_detection_count >= 3 then  -- Require 3 consecutive detections
                    gcs:send_text(6, string.format("Impact detected at %.1fm, rate: %.2f", depth, descent_rate))
                    -- Turn lights to 100% brightness immediately when bottom strike detected
                    set_lights(true, PWM_Lightmax)
                    gcs:send_text(6, "Bottom strike detected - lights set to 100% brightness")
                    transition_to_hovering()
                end
            end
        else
            impact_detection_count = 0  -- Reset counter if descent rate is normal
        end
        if depth > target_depth:get() then --handles target depth reached
            gcs:send_text(6, "Target depth reached")
            -- Turn lights to 100% brightness immediately when target depth reached
            set_lights(true, PWM_Lightmax)
            gcs:send_text(6, "Target depth reached - lights set to 100% brightness")
            
            transition_to_hovering()
        end

    elseif state == WATER_SAMPLING then
        -- Set vehicle to ALT_HOLD mode for stable hovering during water sampling
        vehicle:set_mode(MODE_ALT_HOLD)
        
        -- Control water sampling relay
        control_water_sampling_relay()
        
        -- Calculate water sampling hover time in milliseconds
        local ws_hover_duration_ms = ws_htime:get() * 60 * 1000
        
        -- Check if halfway through hover time to trigger relay
        local current_time = millis()
        local hover_elapsed = current_time - ws_hover_start_time
        local halfway_time = ws_hover_duration_ms / 2
        
        if hover_elapsed >= halfway_time and not ws_relay_triggered then
            trigger_water_sampling()
            ws_relay_triggered = true  -- Mark that we've triggered for this session
            gcs:send_text(6, string.format("Water sampling relay triggered at %.1fm (halfway through hover)", depth))
        end
        
        -- Check if water sampling hover time is complete
        if hover_elapsed >= ws_hover_duration_ms then
            gcs:send_text(6, string.format("Water sampling complete at %.1fm - resuming descent", depth))
            state = DESCENDING
            -- Reset water sampling variables
            ws_relay_toggled = false
            ws_relay_triggered = false
            ws_hover_start_time = 0
        end
        
        -- Log water sampling status periodically
        if iteration_counter % 100 == 0 then  -- Every 5 seconds
            gcs:send_text(6, string.format("Water sampling at %.1fm", depth))
        end

    elseif state == ASCEND_TOHOVER then
        vehicle:set_mode(MODE_MANUAL)  -- Set to MANUAL to allow direct motor control
        motor_output()  -- Add call to motor_output for ASCEND_TOHOVER state
        -- Keep lights at 100% brightness during ascent to hover (maintain from previous state)
        set_lights(true, PWM_Lightmax)
        
        if depth < hover_depth then 
            vehicle:set_mode(MODE_ALT_HOLD)
            hover_start_time = millis()
            gcs:send_text(6, "Transitioning to HOVERING state")
            state = HOVERING
        end

    elseif state == HOVERING then
        -- First time entering this state, initialize
        if hover_start_time == 0 then
            hover_start_time = millis()
            -- Keep lights at 100% brightness throughout hover (no incremental changes)
            set_lights(true, PWM_Lightmax)
            gcs:send_text(6, "Hover started - lights at 100% brightness")
        end
        
        -- Always keep lights at 100% brightness during hover
        set_lights(true, PWM_Lightmax)
        
        -- Check if hover time is complete - ONLY based on elapsed time, not depth
        local current_time = millis()
        local hover_duration_ms = hover_time:get() * 60 * 1000
        
        if current_time > (hover_start_time + hover_duration_ms) then
            gcs:send_text(6, "Hover complete - lights at 100% brightness")
            
            -- Clear ALT_HOLD exit detection before transitioning to prevent re-entry
            alt_hold_exit_detected = false
            state = SURFACING
            vehicle:set_mode(MODE_MANUAL)
            
            hover_start_time = 0  -- Reset for next time
            
            -- Ensure ascent throttle is properly initialized
            current_ascent_throttle = ascent_throttle:get()
            gcs:send_text(6, string.format("Starting surfacing with throttle: %d", current_ascent_throttle + 0))
        elseif alt_hold_exit_detected then
            -- ALT_HOLD mode was exited during active hover - re-enter immediately
            vehicle:set_mode(MODE_ALT_HOLD)
            alt_hold_exit_detected = false
            gcs:send_text(6, "Re-entering ALT_HOLD mode after exit")
        end
        
        -- Log hover status periodically (every 10 seconds)
        if iteration_counter % 200 == 0 then  -- 200 iterations * 50ms = 10 seconds
            gcs:send_text(6, string.format("Hover: depth %.1fm, lights at 100%%", depth))
        end
    elseif state == SURFACING then
        vehicle:set_mode(MODE_MANUAL)  --Set to MANUAL to allow direct motor control
        motor_output()  -- Add call to motor_output for SURFACING state
        
        -- Calculate distance above hover depth to turn off lights (40m)
        local light_off_depth = hover_depth - 40
        
        -- Turn off lights when 40m above hover depth, but keep them on otherwise
        if depth < light_off_depth then
            set_lights(false)
            if not lights_off_reported then
                gcs:send_text(6, "Lights turned off during ascent (40m above hover depth)")
                lights_off_reported = true
            end
        else
            -- Keep lights at maximum brightness until 40m above hover depth
            set_lights(true, PWM_Lightmax)
        end
        
        if depth < surf_depth:get() then
            if stop_video_recording() then
                gcs:send_text(6, "Video recording stopped during surfacing")
                set_lights(false)  -- Ensure lights are off when surfacing
                -- Reintroduce the disarm command
                arming:disarm()
                is_recording = 0
                state = COMPLETE
                -- Reset switch_opened_after_complete when entering COMPLETE state
                switch_opened_after_complete = false
                gcs:send_text(6, "Mission complete - disarmed")
                gcs:send_text(6, "Open switch to reset")
            end
        end
    elseif state == COMPLETE then
        -- If depth increases above threshold, set to ALT_HOLD mode to maintain position
        -- This prevents sinking if the vehicle is at the surface and starts to drift deeper
        if depth > surface_depth_threshold:get() then
            -- Only arm and set mode if we're not already armed
            if not arming:is_armed() then
                arming:arm()
                vehicle:set_mode(MODE_ALT_HOLD)
                gcs:send_text(6, string.format("Depth increased to %.2fm - setting ALT_HOLD mode", depth))
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
        impact_detection_count = 0
        slow_zone_entered = false
        -- Reset water sampling variables for new mission
        ws_hover_start_time = 0
        ws_relay_toggled = false
        ws_relay_triggered = false
    end
end
-- Transition to HOVERING state
function transition_to_hovering()
    hover_depth = depth - hover_offset:get()
    state = ASCEND_TOHOVER
    current_ascent_throttle = ascent_throttle:get()  -- Initialize ascent throttle when transitioning
    gcs:send_text(6, string.format("Transitioning to ascend, initial throttle: %d", current_ascent_throttle + 0))
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
        
        gcs:send_named_float("State", state + 0.0)
        gcs:send_named_float("Depth", depth)
        
        -- Only reset counter after completing a full cycle
        iteration_counter = 0
    end
    
    return loop, 50
end

return loop()