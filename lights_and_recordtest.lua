-- Configuration
LIGHTS1_SERVO = 13  -- Servo function for lights
PWM_Lightoff = 1000  -- PWM value for lights off
PWM_Lightmed = 1500  -- PWM value for medium brightness

-- Function to control lights
function set_lights(on)
    if on then
        SRV_Channels:set_output_pwm(LIGHTS1_SERVO, PWM_Lightmed)
        gcs:send_text(0, "Lights turned ON")
    else
        SRV_Channels:set_output_pwm(LIGHTS1_SERVO, PWM_Lightoff)
        gcs:send_text(0, "Lights turned OFF")
    end
end

-- Function to start video recording
function start_video_recording()
    local sock = Socket(0)
    if not sock:bind("0.0.0.0", 9988) then
        gcs:send_text(0, "Failed to bind socket")
        sock:close()
        return
    end

    if not sock:connect("localhost", 5423) then
        gcs:send_text(0, "Failed to connect to video recorder")
        sock:close()
        return
    end

    local request = "GET /start?split_duration=90 HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    sock:send(request)
    sock:close()
    gcs:send_text(0, "Video recording started")
end

-- Function to stop video recording
function stop_video_recording()
    local sock = Socket(0)
    if not sock:bind("0.0.0.0", 9988) then
        gcs:send_text(0, "Failed to bind socket")
        sock:close()
        return
    end

    if not sock:connect("localhost", 5423) then
        gcs:send_text(0, "Failed to connect to video recorder")
        sock:close()
        return
    end

    local request = "GET /stop HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
    sock:send(request)
    sock:close()
    gcs:send_text(0, "Video recording stopped")
end

-- Main test sequence
local start_time = millis()
local state = 0

function update()
    local now = millis()
    local elapsed = (now - start_time) / 1000  -- Convert to seconds

    if state == 0 and elapsed >= 30 then
        -- After 30 seconds, turn on lights
        set_lights(true)
        gcs:send_text(0, string.format("Test sequence: Lights ON at %.1f seconds", elapsed))
        state = 1
        
    elseif state == 1 and elapsed >= 60 then
        -- After 60 seconds (30s after lights), start recording
        start_video_recording()
        gcs:send_text(0, string.format("Test sequence: Recording started at %.1f seconds", elapsed))
        state = 2
        
    elseif state == 2 and elapsed >= 90 then
        -- After 90 seconds (30s after recording start), turn off lights
        set_lights(false)
        gcs:send_text(0, string.format("Test sequence: Lights OFF at %.1f seconds", elapsed))
        state = 3
        
    elseif state == 3 and elapsed >= 120 then
        -- After 120 seconds (30s after lights off), stop recording
        stop_video_recording()
        gcs:send_text(0, string.format("Test sequence: Recording stopped at %.1f seconds", elapsed))
        state = 4
        gcs:send_text(0, "Test sequence complete!")
        return
    end

    return update, 1000  -- Check again in 1 second
end

-- Start the test sequence
gcs:send_text(0, "Starting lights and recording test sequence...")
return update() 