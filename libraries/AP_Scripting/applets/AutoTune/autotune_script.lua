-- Plane Autotune Script

local RC1 = rc:get_channel(1)
local RC2 = rc:get_channel(2)

local scripting_rc_0 = rc:find_channel_for_option(107) -- Auto tuning 開始用スイッチ ch
local scripting_rc_1 = rc:find_channel_for_option(300) -- roll,pitch チューニング用スイッチ ch

local roll_running = false
local pitch_running = false

local roll_tuning_start_time = 0.0
local pitch_tuning_start_time = 0.0

-- tuninn_stage: 0:pitch up, 1:pitch down
local roll_tuning_stage = 0
local pitch_tuning_state = 0

local prev_mode = 0
function tune_roll(sw, tune_period)
--    local lim_roll_deg = param:get("LIM_ROLL_CD")*0.01*0.8
    local lim_roll_deg = 30 
    local now = millis():tofloat() * 0.001
    local roll = math.deg(ahrs:get_roll())
    if sw and not roll_running and roll < 1.5 and roll > -1.5 then
        roll_running = true
        --roll_tuning_start_time = now - tune_period/2.0
        lim_roll_deg = lim_roll_deg/2.0
        roll_tuning_stage = 0
        
        
        prev_mode = vehicle:get_mode()
        -- Set mode to PLANE_MODE_AUTOTUNE
--        vehicle:set_mode(8)
        gcs:send_text(0, string.format("Starting roll tuning script"))
    end
    if not sw then
        if roll_running then
--            vehicle:set_mode(prev_mode)
            RC1:set_override(1500)
            gcs:send_text(0, string.format("Finished roll tuning script"))
        end
        roll_running = false
        roll_tuning_stage = 0
    end

    --改造20240614
    if roll_tuning_stage == 0 then
        if roll > lim_roll_deg then
            roll_tuning_stage =1
        end
    elseif roll_tuning_stage == 1 then
        if roll < -lim_roll_deg then
            roll_tuning_stage = 0
        end
    end 

    if roll_running then
        if roll_tuning_stage == 0 then
            RC1:set_override(1900)
        elseif roll_tuning_stage == 1 then
            RC1:set_override(1100)
        end
    end
end

function tune_pitch(sw, tune_period)
    local now = millis():tofloat() * 0.001
    if sw and not pitch_running then
        pitch_running = true
        pitch_tuning_start_time = now - tune_period/2.0
        pitch_tuning_stage = 0
        prev_mode = vehicle:get_mode()
        -- Set mode to PLANE_MODE_AUTOTUNE
--        vehicle:set_mode(8)
        gcs:send_text(0, string.format("Starting pitch tuning script"))
    end
    if not sw then
        if pitch_running then
--            vehicle:set_mode(prev_mode)
            RC2:set_override(1500)
            gcs:send_text(0, string.format("Finished pitch tuning script"))
        end
        pitch_running = false
        pitch_tuning_stage = 0
    end

    if pitch_running then
        if pitch_tuning_stage == 0 then
            RC2:set_override(1900)
        elseif pitch_tuning_stage == 1 then
            RC2:set_override(1100)
        end
        if (now - pitch_tuning_start_time) > tune_period then
            pitch_tuning_start_time = now
            if pitch_tuning_stage == 1 then
                pitch_tuning_stage = 0
            elseif pitch_tuning_stage == 0 then
                pitch_tuning_stage = 1
            end
        end
    end
end

function update()
    local tune_sw = scripting_rc_0:get_aux_switch_pos()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc_0 and scripting_rc_1 then
        local sw_pos = scripting_rc_1:get_aux_switch_pos()
        if tune_sw == 2 and sw_pos == 1 then 
            tune_roll(true, 0.6)
        else
            tune_roll(false, 0.6)
        end
    end

    if mode == 10 and scripting_rc_0 and scripting_rc_1 then
        local sw_pos = scripting_rc_1:get_aux_switch_pos()
        if tune_sw == 2 and sw_pos == 2 then 
            tune_pitch(true, 0.8)
        else
            tune_pitch(false, 0.8)
        end
    end

    return update, 10
end

return update()
