-- Relay Test Script
-- Toggles relay 1 for 1 second when vehicle is armed
-- Performs 5 triggers with 5 second intervals between them

local RELAY_NUM = 0
local TRIGGER_COUNT = 5
local TRIGGER_INTERVAL = 5000  -- 5 seconds in milliseconds
local RELAY_DURATION = 1000    -- 1 second in milliseconds

-- State variables
local is_armed = false
local last_armed_state = false
local trigger_count = 0
local last_trigger_time = 0
local relay_toggle_start = 0
local is_relay_toggled = false
local test_complete = false

-- Function to control relay
function control_relay()
    if is_relay_toggled then
        -- Check if relay toggle duration has elapsed
        if millis() > (relay_toggle_start + RELAY_DURATION) then
            -- Toggle relay back to original state
            relay:toggle(RELAY_NUM)
            is_relay_toggled = false
            gcs:send_text(6, string.format("Relay %d toggled back to original state", RELAY_NUM))
        end
    end
end

-- Function to trigger relay toggle
function trigger_relay_toggle()
    if not is_relay_toggled then
        -- Toggle relay
        relay:toggle(RELAY_NUM)
        is_relay_toggled = true
        relay_toggle_start = millis()
        gcs:send_text(6, string.format("Relay %d toggled - trigger %d/%d", RELAY_NUM, trigger_count + 1, TRIGGER_COUNT))
    end
end

-- Main update function
function update()
    -- Check if vehicle is armed
    is_armed = arming:is_armed()
    
    -- Detect arming state change
    if is_armed and not last_armed_state then
        gcs:send_text(6, "Vehicle armed - starting relay test")
        trigger_count = 0
        last_trigger_time = millis()
        test_complete = false
    elseif not is_armed and last_armed_state then
        gcs:send_text(6, "Vehicle disarmed - stopping relay test")
        test_complete = true
    end
    
    last_armed_state = is_armed
    
    -- Only run test if armed and not complete
    if is_armed and not test_complete then
        local current_time = millis()
        
        -- Check if it's time for next trigger
        if trigger_count < TRIGGER_COUNT and (current_time - last_trigger_time) >= TRIGGER_INTERVAL then
            trigger_relay_toggle()
            trigger_count = trigger_count + 1
            last_trigger_time = current_time
            
            if trigger_count >= TRIGGER_COUNT then
                gcs:send_text(6, "Relay test complete - all 5 triggers performed")
                test_complete = true
            end
        end
    end
    
    -- Control relay state
    control_relay()
    
    -- Send status message every 2 seconds
    if millis() % 2000 < 50 then  -- Every 2 seconds (with 50ms tolerance)
        if is_armed and not test_complete then
            gcs:send_text(6, string.format("Relay test: %d/%d triggers completed", trigger_count, TRIGGER_COUNT))
        elseif test_complete then
            gcs:send_text(6, "Relay test complete - disarm to reset")
        else
            gcs:send_text(6, "Relay test ready - arm vehicle to start")
        end
    end
    
    return update, 50  -- Run at 20Hz (50ms interval)
end

-- Initialize
gcs:send_text(6, "Relay test script loaded - arm vehicle to start test")
gcs:send_text(6, string.format("Will toggle relay %d for %dms, %d times with %dms intervals", 
    RELAY_NUM, RELAY_DURATION, TRIGGER_COUNT, TRIGGER_INTERVAL))

return update()

