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
light_depth = bind_add_param('LIGHT_D', 2, 15.0)      -- Depth to turn on lights
hover_time = bind_add_param('HOVER_M', 3, 1.0)        -- Minutes to hover
surf_depth = bind_add_param('SURF_D', 4, 2.0)        -- Surface threshold
max_ah = bind_add_param('MAX_AH', 5, 12.0)           -- Max amp-hours
min_voltage = bind_add_param('MIN_V', 6, 13.0)       -- Min battery voltage

-- States
STANDBY=0
COUNTDOWN=1
DESCENDING=2
HOVERING=3
SURFACING=4
ABORT=5

-- Initialize variables
state = STANDBY
timer = 0
hover_depth = 0
last_depth = 0
descent_rate = 0
descent_throttle = 0
depth_stable_count = 0
start_ah = 0
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

-- Function to control lights
function set_lights(on)
    -- Assuming lights are controlled via relay
    if on then
        SRV_Channels:set_output_pwm_chan(lights1_channel, PWM_Lightmed) --is light channel valid use of servo function?

    else
        SRV_Channels:set_output_pwm_chan(lights1_channel, PWM_Lightoff) --is light channel valid use of servo function?
    end
end

-- Function to read mavlink depth, orientation, and battery voltage / consumed mah
function get_data()
    local depth = mavlink:get_depth() --?
    local roll = math.deg(ahrs:get_roll())
    local mah = battery:consumed_mah()
    return depth, roll, mah
end

-- Function to set depth target / descent rate / descent throttle


-- Function to control dive mission!

function control_dive_mission()
    if not switch_closed() and state ~= STANDBY then
        gcs:send_text(6, "Mission switch opened - aborting")
        state = ABORT
    end
    if state == STANDBY then
        SRV_Channels:set_output_pwm_chan(LIGHTS1_SERVO, PWM_Lightoff)
        if updateswitch() then
            timer = millis()
            state = COUNTDOWN
            start_mah = battery:consumed_mah()
        end
    end
    elseif state == COUNTDOWN then
        timer = millis()
        if millis() > (timer + dive_delay_s * 1000) then
            state = DESCENDING
            last_depth = depth
            gcs:send_text(6, "Starting descent")
        end
    end
    elseif state == DESCENDING then
        -- set depth target
        -- set descent rate
        -- set descent throttle
        -- if depth > vrecord_depth, start recording via http request to video recorder extension
        -- monitor descent rate / depth
        -- if decrease in descent rate > descent_rate_threshold, set max_depth, hover_depth = max_depth - hover_offset
        if hover_depth != 0 then
            state = HOVERING
        end
    elseif state == HOVERING then
        -- monitor depth, error depth - hover_depth
        timer = millis()
        if millis() > (timer + hover_time * 1000) then
            state = SURFACING
        end
    elseif state == SURFACING then
        -- disarm (?)
        -- verify ascent rate +
        -- if depth > surf_depth, set state = STANDBY
    end
elseif state == ABORT then
        vehicle:disarm()
    end
    gcs:send_named_float("State", state)
    gcs:send_named_float("Depth", depth)
    return update 1000
end

function mavlink_decode_header(message)
    -- build up a map of the result
    local result = {}
  
    local read_marker = 3
  
    -- id the MAVLink version
    result.protocol_version, read_marker = string.unpack("<B", message, read_marker)
    if (result.protocol_version == 0xFE) then       -- mavlink 1
      result.protocol_version = 1
    elseif (result.protocol_version == 0XFD) then   --mavlink 2
      result.protocol_version = 2
    else
      error("Invalid magic byte")
    end
  
    _, read_marker = string.unpack("<B", message, read_marker)   -- payload is always the second byte
  
    -- strip the incompat/compat flags
    result.incompat_flags, result.compat_flags, read_marker = string.unpack("<BB", message, read_marker)
  
    -- fetch seq/sysid/compid
    result.seq, result.sysid, result.compid, read_marker = string.unpack("<BBB", message, read_marker)
  
    -- fetch the message id
    result.msgid, read_marker = string.unpack("<I3", message, read_marker)
  
    return result, read_marker
  end
  
  function mavlink_decode(message)
    local result, offset = mavlink_decode_header(message)
    local message_map = MANUAL_CONTROL
    if not message_map then
      -- we don't know how to decode this message, bail on it
      return nil
    end
  
    -- map all the fields out
    for _, v in ipairs(message_map.fields) do
      if v[3] then
        result[v[1]] = {}
        for j = 1, v[3] do
          result[v[1]][j], offset = string.unpack(v[2], message, offset)
        end
      else
        result[v[1]], offset = string.unpack(v[2], message, offset)
      end
    end
    -- ignore the idea of a checksum
    return result
  end
  
-- Function to control video recording
function start_video_recording(max_duration, split_duration)
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

    local max_dur = max_duration or 60  -- default 60 seconds
    local split_dur = split_duration or 30  -- default 30 seconds
    local request = string.format("GET /start?max_duration=%d&split_duration=%d HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n", 
        max_dur, split_dur)
    
    sock:send(request)
    sock:close()
    gcs:send_text(0, "Video recording started")
end

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
  

