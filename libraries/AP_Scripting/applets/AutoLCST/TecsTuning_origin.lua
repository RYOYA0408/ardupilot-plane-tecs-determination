-- Plane TECS AutoTune script
local RC2 = rc:get_channel(2)

local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch

local FREQUENCY = 10
local thrmax_running = false

-- チューニングステージ
-- 0: 未実行
-- 1: 上昇中
-- 2: 最大高度到達。中断中
local thrmax_stage = 0

function tune_thrmax(sw)
    local pitch_deg = math.deg(ahrs:get_pitch())
    --local height = ahrs:get_position():alt()*0.01
    local height =  - ahrs:get_relative_position_NED_home():z()
    if sw and not thrmax_running then
        thrmax_running = true
        thrmax_stage = 1
        gcs:send_text(0, string.format("Starting THR_MAX tuning"))
    end
    if not sw then
        if thrmax_running then
            RC2:set_override(1500)
            gcs:send_text(0, string.format("Finished THR_MAX tuning"))
        end
        thrmax_running = false
        thrmax_stage = 0
    end

    -- 上昇中
    if thrmax_stage == 1 then
        -- 最大ピッチ角設定
        RC2:set_override(1900)
        if height >= 150.0 then
            RC2:set_override(1500)
            thrmax_stage = 2
        end
    end

    -- 最大高度到達。中断中
    if thrmax_stage == 2 then
        RC2:set_override(1500)
    end
end

function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc then
        local sw_pos = scripting_rc:get_aux_switch_pos()
        if sw_pos == 1 then 
            tune_thrmax(true)
        else
            tune_thrmax(false)
        end
    end

    return update, FREQUENCY
end

return update()
