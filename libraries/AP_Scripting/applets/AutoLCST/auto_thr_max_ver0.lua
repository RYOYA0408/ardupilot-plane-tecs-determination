-- Plane thr_max of AutoLCST(Longitudinal Control System Tuning)

--　各種設定
local airspeed_error = 1.5  --速度判定の許容誤差
local elev_servo_ch = 2
local RC2 = rc:get_channel(2)
local RC3 = rc:get_channel(3)

local rc_thr_max = param:get("SERVO3_MAX")  -- RC3 の PWM 最大値取得
local rc_thr_min = param:get("SERVO3_MIN")  -- RC3 の PWM 最小値取得
local thr_max = param:get("THR_MAX") -- 100 %
local thr_min = param:get("THR_MIN") -- 10 %

-- フラグ管理
local pitch_sw = false
local airspeed_sw = false
local mission_sw = false

-- スロットルサーボの PWM値 からスロットル率 % に変換
function get_throttle(rc_thr)
    local throttle = ((rc_thr - rc_thr_min)/(rc_thr_max - rc_thr_min)) * (thr_max - thr_min) + thr_min  -- % に変換
    return throttle
end

-- 変更されたスロットル率を PWM値 に変換
function set_throttle(throttle_percent)
    local rc3_pwm = ((throttle_percent - thr_min)/(thr_max - thr_min)) * (rc_thr_max - rc_thr_min) + rc_thr_min  -- 目標とする pwm 値算出
    RC3:set_override(math.floor(rc3_pwm))
    gcs:send_text(6, string.format("Set throttle to %.2f%% (PWM: %d)", throttle_percent, rc3_pwm))
end

-- エレベータサーボの最大 PWM 値を取得する関数
function rc2_max()
    local rc2_max = param:get("SERVO2_MAX")  -- エレベータの最大 PWM 値を取得
    if rc2_max then
        gcs:send_text(6, string.format("MAX PWM for servo ch %d: %d", elev_servo_ch, rc2_max))    -- MPに送信
        return rc2_max
    else
        gcs:send_text(6, string.format("Failed to retrieve MAX PWM for servo ch %d", elev_servo_ch)) -- 取得失敗したらエラーメッセージ送信
        return nil
    end
end

-- エレベータサーボを最大ピッチ角に設定(サーボをMAXで出力すればFBWモードによって自動的に最大ピッチ角になる)
function set_servo2pitch_max()
    local pitch_max = rc2_max()
    if pitch_max then
        RC2:set_override(pitch_max)
        gcs:send_text(6, string.format("Servo ch %d set to max PWM: %d", elev_servo_ch, pitch_max))
    else
        gcs:send_text(6, "Unable to set servo to max pitch angle")
    end
end

-- ピッチ角と巡航速度に達するまで監視 → スロットル率取得
function set_pitch_and_get_throttle_at_cruise()
    set_servo2pitch_max()  -- 最大ピッチ角の指令
    --local airspeed_cruise = param:get("ARSPD_FBW_MAX") -- 30 m/s
    local airspeed_cruise = 28
    local current_pitch = math.deg(ahrs:get_pitch())    -- 現在のピッチ角
    gcs:send_text(6, string.format("Current Pitch: %.2f", current_pitch))   -- MP に現在のピッチ角送信

    if current_pitch > 20 and not pitch_sw then
        pitch_sw = true
        gcs:send_text(6, "Pitch exceeds 20 degrees, monitoring airspeed...")

    elseif pitch_sw and not airspeed_sw then
        
        local current_eas = ahrs:airspeed_estimate()
        local current_tas = current_eas * ahrs:get_EAS2TAS()
        --local rc_thr =  rc:get_pwm(3) -- RC3(スロットル)の PWM 値取得
        local throttle = vehicle:get_control_output(3) * 100
        gcs:send_text(6, string.format("Current TAS: %.2f", current_tas))
        gcs:send_text(6, string.format("Current throttle: %.2f%%", throttle))

        if math.abs(current_tas - airspeed_cruise) <= airspeed_error then
            airspeed_sw = true
            --local throttle = get_throttle(rc_thr)
            if airspeed_sw and throttle then
                gcs:send_text(6, string.format("Throttle at AIRSPEED_CRUISE: %.2f%%", throttle))
                return throttle
            else
                gcs:send_text(6, "Failed to retrieve throttle")
                return nil
            end
        else
            if current_tas < airspeed_cruise then
                --local throttle = get_throttle(rc_thr)
                local thr_plus = math.min(throttle + 1, thr_max)
                --set_throttle(throttle)
                vehicle:set_target_throttle_rate_rpy(thr_plus, 0, 0, 0)
            elseif current_tas > airspeed_cruise then
                --local throttle = get_throttle(rc_thr)
                local thr_minus = math.max(throttle - 1, thr_min)
                vehicle:set_target_throttle_rate_rpy(thr_minus, 0, 0, 0)
                --set_throttle(throttle)
            end
        end
    end
end

-- 取得したスロットル率を THR_MAX として Mission Planner に設定
function set_thr_max(thr_max)
    
    if param:set("THR_MAX", thr_max) then
        gcs:send_text(6, string.format("THR_MAX successfully set to %.2f%%", thr_max))
        return true
    else
        gcs:send_text(6, "Faild to set THR_MAX")
        return false
    end
end

-- コマンドの処理
function switch_command(cmd)
    
    if cmd == 1 and not mission_sw then
        
        -- 実行：最大ピッチ角に設定後、AIRSPEED_CRUISEに達するスロットル率を取得
        local thr_max = set_pitch_and_get_throttle_at_cruise()

        --if thr_max == nil then
            --gcs:send_text(6, "Error: Failed to retrieve throttle, thr = nil")
            --return  -- thr_max が nil の場合は処理を終了

        -- thr_max
        if set_thr_max(thr_max) and thr_max ~= nil then
            gcs:send_text(6, string.format("Final THR_MAX: %.2f%%", thr_max))
            mission_sw = true
        else
            gcs:send_text(6, "Failed to retrieve throttle or not ready yet")
        end
    
    elseif mission_sw then
        gcs:send_text(6, "Mission already completed")
    else
        gcs:send_text(6, "Invalid command")
    end
end

-- メインループ
function update_thr_max()
    --[[
    if mission_sw then
        vehicle:set_mode(10)
        gcs:send_text(6, "Set to Auto mode")
        vehicle:nav_script_time_done(1)
        return
    end
    ]]--

    -- コマンド監視
    local id, cmd = vehicle:nav_script_time()

    if id then
        gcs:send_text(6, string.format("Received cmd: %d", cmd))
        --vehicle:set_mode(5)
        --gcs:send_text(6, "Set to FBWA mode")
        switch_command(cmd)

    elseif mission_sw then
        --vehicle:set_mode(10)
        --gcs:send_text(6, "Set to Auto mode")
        vehicle:nav_script_time_done(id)
        gcs:send_text(6, "Script done")
        return
    end

    return update_thr_max, 10   --0.01秒ごとに確認
end

return update_thr_max()