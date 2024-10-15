-- Plane thr_max of AutoLCST(Longitudinal Control System Tuning)

--　各種設定
local airspeed_error = 1.5  --速度判定の許容誤差
local elev_servo_ch = 2
local RC2 = rc:get_channel(2)

-- スロットルサーボからスロットル率に変換
function get_throttle()
    
    local rc_throttle =  rc:get_pwm(3) -- RC3 のスロットルに関するサーボチャンネルのPWM値を取得
    local rc_throttle_max = param:get("SERVO3_MAX")  -- RC3のPWM値を取得
    
    if rc_throttle then
        local throttle = (rc_throttle * 100) / rc_throttle_max  -- % に変換
        return throttle
    else
        return nil
    end
end

-- エレベータサーボの最大 PWM 値を取得する関数
function get_elev_rc_max()
    
    local rc_max = param:get("SERVO2_MAX")  -- エレベータの最大 PWM 値を取得
    
    if rc_max then
        gcs:send_text(6, string.format("MAX PWM for servo ch %d: %d", elev_servo_ch, rc_max))    -- MPに送信
        return rc_max
    else
        gcs:send_text(6, string.format("Failed to retrieve MAX PWM for servo ch %d", elev_servo_ch)) -- 取得失敗したらエラーメッセージ送信
        return nil
    end
end

-- エレベータサーボを最大ピッチ角に設定(サーボをMAXで出力すればFBWモードによって自動的に最大ピッチ角になる)
function set_servo_to_pitch_max()
    
    local pitch_max = get_elev_rc_max()

    if pitch_max then
        --SRV_chs:set_output_pwm(ch, pitch_max)
        RC2:set_override(pitch_max)
        gcs:send_text(6, string.format("Servo ch %d set to max PWM: %d", elev_servo_ch, pitch_max))
    else
        gcs:send_text(6, "Unable to set servo to max pitch angle due to missing PWM value")
    end
end

-- ピッチ角が20度を超えた後に巡航速度に達するまで監視 → スロットル率取得
function set_pitch_and_get_throttle_at_cruise()

    set_servo_to_pitch_max()  -- 最大ピッチ角でのPWM値を取得

    -- ピッチ角と速度を監視
    current_pitch = ahrs:get_pitch() * 180 / math.pi
    gcs:send_text(6, string.format("Current Pitch: %.2f", current_pitch))

    -- ピッチ角が20度を超えたか確認
    if current_pitch > 20 then
        gcs:send_text(6, "Pitch exceeds 20 degrees, monitoring airspeed...")
        local airspeed_cruise = param:get("ARSPD_FBW_MAX")
        local current_eas = ahrs:airspeed_estimate()
        local current_tas = current_eas * ahrs:get_EAS2TAS() -- 現在の真対気速度を取得
        gcs:send_text(6, string.format("Current TAS: %.2f", current_tas))

        -- 速度がAIRSPEED_CRUISEに達したか確認
        if math.abs(current_tas - airspeed_cruise) <= airspeed_error then
            local throttle = get_throttle()
            if throttle then
                gcs:send_text(6, string.format("Throttle at AIRSPEED_CRUISE: %.2f%%", throttle))
                return throttle
            else
                gcs:send_text(6, "Failed to retrieve throttle")
        end
    end

    -- 次のチェックまで 0.5 秒待機
    return set_pitch_and_get_throttle_at_cruise, 500
end

-- THR_MAX を Mission Planner に設定
function set_thr_max(thr_max)
    
    if param:set("THR_MAX", thr_max) then
        gcs:send_text(6, string.format("THR_MAX successfully set to %.2f%%", thr_max))
        vehicle:set_mode(10) -- 10: AUTOモードに切り替え
        gcs:send_text(6, "Switched to AUTO mode")
        vehicle:nav_script_time_done(1)
        return true
    else
        gcs:send_text(6, "Faild to set THR_MAX")
        return false
    end
end

-- コマンドの処理
function switch_command(cmd)
    if cmd == 1 then
        -- 実行：最大ピッチ角に設定後, AIRSPEED_CRUISEを達成時のスロットル率を取得
        local thr_max = set_pitch_and_get_throttle_at_cruise()
        if thr_max then
            gcs:send_text(6, string.format("Final THR_MAX: %.2f%%", thr_max))
            set_thr_max(thr_max)
            --[[if set_thr_max(thr_max) then
                return nil  -- THR_MAXが設定されたらスクリプトを終了
            end
            ]]--
        else
            gcs:send_text(6, "THR_MAX retrieve failed")
        end
    else
        gcs:send_text(6, "Invalid command")
    end
end

-- メインループ
function update_thr_max()

    -- コマンド監視
    local id, cmd = vehicle:nav_script_time()

    if id then
        vehicle:set_mode(6) -- 5:FBWA, 6:FBWB
        gcs:send_text(6, "Switched to FBWB mode")
        gcs:send_text(6, string.format("Received cmd: %d", cmd))
        switch_command(cmd)
    end

    return update_thr_max, 1000   --1秒ごとに確認
end

return update_thr_max()