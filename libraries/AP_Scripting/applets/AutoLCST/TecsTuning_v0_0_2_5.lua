-- Plane TECS AutoTune script
local RC2 = rc:get_channel(2)
local k_throttle = 70
local arspd_max = param:get("ARSPD_FBW_MAX")

local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch

local FREQUENCY = 10
local tecstune_running = false
local spd_sw = false
--local lua_cnt = param:get("SCR_USER1") or 0 -- 当該luaスクリプトの動作回数の初期値取得, 基本は0

-- チューニングステージ
-- 0: 未実行
-- 1: 加速フェーズ
-- 2: 上昇中
-- 3: 最大高度到達。中断
local tecstune_stage = 0

function tune_thrmax(sw)
    local pitch_deg = math.deg(ahrs:get_pitch())
    local arspd_now = ahrs:airspeed_estimate()
    local height =  - ahrs:get_relative_position_NED_home():z() -- 地面からの相対高度
    local trim_arspd_now = param:get("TRIM_ARSPD_CM")

    if sw and not tecstune_running then
        tecstune_running = true
        tecstune_stage = 1
        gcs:send_text(0, string.format("Starting THR_MAX tuning"))
    end

    if not sw then
        if tecstune_running then
            RC2:set_override(1500)
            gcs:send_text(0, string.format("Finished THR_MAX tuning"))
        end
        tecstune_running = false
        tecstune_stage = 0
    end

    -- 加速フェーズ, トリム速度を一時的に 30 m/s (ARSPD_FBW_MAXと同値) に変更することで対応
    if tecstune_stage == 1 then
        if not spd_sw then
            --トリム速度値を変更
            param:set_and_save("TRIM_ARSPD_CM", 3000)
            --gcs:send_text(0, string.format("Change TRIM_ARSPD_CM to: %d", trim_arspd_now*0.01))
            --SRV_Channels:set_output_scaled(k_throttle, 100)
            --vehicle:set_target_throttle_rate_rpy(100, 0, 0, 0)    -- スロットル100%貼付不可確認済
            
            if arspd_now > arspd_max then
                thr_sw = true
                param:set_and_save("TRIM_ARSPD_CM", 2500)
                gcs:send_text(0, string.format("Return TRIM_ARSPD_CM to: %d", trim_arspd_now*0.01))
                tecstune_stage = 2
            end
        end
    end

    -- 上昇中
    if tecstune_stage == 2 then
        -- 最大ピッチ角指令
        RC2:set_override(1900)
        if height >= 150.0 then
            RC2:set_override(1500)
            tecstune_stage = 3
        end
    end

    -- 最大高度到達。中断
    if tecstune_stage == 3 then
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
