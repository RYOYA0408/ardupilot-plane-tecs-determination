-- Plane TECS AutoTune script
local RC2 = rc:get_channel(2)
local RC3 = rc:get_channel(3)
--local tas_target = param:get("TRIM_ARSPD_CM") * 0.01
local pitch_max = param:get("LIM_PITCH_MAX") * 0.01
--local k_throttle = 70
--local RC3_buff = 100.0
local height_limit = 200

local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch

local FREQUENCY = 10
local thrmax_running = false
local pitchup_sw = false
local thrplus_sw = false

-- チューニングステージ
-- 0: 未実行
-- 1: 上昇中 (最大ピッチ角未到達時)
-- 2: 上昇中 (最大ピッチ角到達後)
-- 3: 最大高度到達。中断中
local thrmax_stage = 0

function tune_thrmax(sw)
    -- 状態変数取得 -------------------------------------------------------------
    local pitch_deg = math.deg(ahrs:get_pitch())
    local pitch_diff = math.abs(pitch_max - pitch_deg)
    local tas_now = ahrs:airspeed_estimate() * ahrs:get_EAS2TAS()
    --local tas_diff = tas_target - tas_now
    --local RC3_now = rc:get_pwm(3)
    local height = ahrs:get_position():alt() * 0.01
    ----------------------------------------------------------------------------

    -- フラグ管理　--------------------------------------------------------------
    if sw and not thrmax_running then
        thrmax_running = true
        thrmax_stage = 1
        gcs:send_text(0, string.format("Starting THR_MAX tuning"))
    end

    if sw and not pitchup_sw and thrmax_running and pitch_diff <= 0.5 then
        pitchup_sw = true
        thrmax_stage = 2
        gcs:send_text(0, string.format("Pitch max reached and stage==2"))
    end

    if not sw then
        if thrmax_running then
            RC2:set_override(1500)
            gcs:send_text(0, string.format("Finished THR_MAX tuning"))
        end
        thrmax_running = false
        thrmax_stage = 0
    end
    ---------------------------------------------------------------------------

    -- 飛行ステージ別アクション -------------------------------------------------

    -- 上昇中 (最大ピッチ角未到達時)
    if thrmax_stage == 1 then
        -- 最大ピッチ角設定
        RC2:set_override(1900)
    end

    -- 上昇中 (最大ピッチ角到達後)
    if thrmax_stage == 2 then
        RC2:set_override(1900)
        SRV_Channels:set_output_pwm(3, 1900)
        --RC3:set_override(1900)
        if height >= 200.0 then
            RC2:set_override(1500)
            thrmax_stage = 3
        end
    end

    -- 最大高度到達。中断中
    if thrmax_stage == 3 then
        RC2:set_override(1500)
    end
    ---------------------------------------------------------------------------
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