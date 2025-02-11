gpio:pinMode(18,0) -- set pwm0 to input, used to connect external "arming" switch
function updateswitch()
    switch_state = gpio:read(18)
    gcs:send_text(6, string.format("switch state %d", switch_state))
    return updateswitch, 2000 
end
return updateswitch(), 2000