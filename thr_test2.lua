local RC2 = rc:get_channel(2) -- RC の 2 チャンネルを取得 (エレベータ)
local pitch_sw = false
local pitch_up_time = 0.0

-- メインループ関数
function update()
    local id, cmd = vehicle:nav_script_time()

    if id ~= nil then
        local now = millis():tofloat() * 0.001
        local dt = now - pitch_up_time

        if cmd == 1 and not pitch_sw then
            RC2:set_override(1900) -- PWM 値を上書き
            pitch_up_time = now
            vehicle:nav_script_time_done(id)
            pitch_sw = true
        
        elseif pitch_sw and dt >= 1.5 and cmd == 1 then
            vehicle:set_target_throttle_rate_rpy(80, 0, 0, 0)
        end
    end
    return update, 100
end

return update()


--[[
attitude_control:set_offset_roll_pitch(0, 28)
RC2:set_override(1900) -- PWM 値を上書き
vehicle:nav_script_time_done(id)
vehicle:set_target_throttle_rate_rpy(80, 0, 0, 0)
vehicle:nav_script_time_done(id)
]]--

SRV_Channels:set_angle(2, 28)