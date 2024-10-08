-- Plane thr_max of AutoLCST(Longitudinal Control System Tuning)

--　各種設定
local airspeed_cruise = param:get("ARSPD_FBW_MIN")  -- AIRSPEED_CRUISEに相当？
local airspeed_error = 1.5  --速度判定の許容誤差
local tas = ahrs:get_EAS2TAS()  -- 現在の真対気速度
local elev_servo_channel = 2

-- スロットルサーボからスロットル率に変換
function get_throttle()
    
    local rc_throttle = - rc:get_channel(3) -- RC3 のスロットルに関するサーボ情報 ※ - 符号は反転
    
    if rc_throttle then
        local throttle = (rc_throttle - 1000) / 10  -- % に変換
        return throttle
    else
        return nil
    end

end

-- エレベータサーボの最大 PWM 値を取得する関数
function get_elev_rc_max(channel)
    
    local rc_max = param:get(string.format("SERV0%d_MAX", channel))   -- パラメータの値を MP から取得
    
    if rc_max then
        gcs:send_text(6, string.format("Max PWM for servo channel %d: %d", channel, rc_max))    -- MPに送信
        return rc_max
    else
        gcs:send_text(6, string.format("Failed to retrieve max PWM for servo channel %d", channel)) -- 取得失敗したらエラーメッセージ送信
        return nil
    end

end

-- エレベータサーボを最大ピッチ角に設定(サーボをMAXで出力すればFBWモードによって自動的に最大ピッチ角になる)
function set_servo_to_pitch_max(channel)
    
    local pitch_max = get_elev_rc_max(channel)

    if pitch_max then
        SRV_Channels:set_output_pwm(channel, pitch_max)
        gcs:send_text(6, string.format("Servo channel %d set to max PWM: %d", channel, pitch_max))
    else
        gcs:send_text(6, "Unable to set servo to max pitch angle due to missing PWM value")
    end

end

-- 指定ピッチ角の指令 & 巡航速度に達するまで監視 → スロットル率取得
function set_pitch_and_get_throttle_at_cruise()

    set_servo_to_pitch_max(elev_servo_channel)  -- 最大ピッチ角でのPWM値を取得

    -- 速度がAIRSPEED_CRUISEに達するまで監視
    while true do
        local current_tas = tas -- 現在の真対気速度を取得
        gcs:send_text(6, string.format("Current TAS: %.2f", current_tas))

        -- 速度がAIRSPEED_CRUISEを達成したか監視
        if math.abs(current_tas - airspeed_cruise) <= airspeed_error then
            local throttle = get_throttle()
            if throttle then
                gcs:send_text(6, string.format("Throttle at AIRSPEED_CRUISE: %.2f%%", throttle))    -- 現在のスロットル率を送信
            else
                gcs:send_text(6, "Failed to retrieve throttle")
            end
            return throttle
        end
        
        -- 0.5秒待機して再度確認
        coroutine.yield(500)
    end

end

-- THR_MAX を Mission Planner に設定
function set_thr_max(thr_max)
    
    if param:set("THR_MAX", thr_max) then
        gcs:send_text(6, string.format("THR_MAX successfully set to %.2f%%", thr_max))
    else
        gcs:send_text(6, "Faild to set THR_MAX")
    end

end

-- cmd値を取得
local cmd = param:get("SCR_CMD")

if cmd == 1 then
    -- 実行：最大ピッチ角に設定後, AIRSPEED_CRUISEを達成時のスロットル率を取得
    local thr_max = set_pitch_and_get_throttle_at_cruise()
    if thr_max then
        gcs:send_text(6, string.format("Final THR_MAX: %.2f%%", thr_max))
        set_thr_max(thr_max)
    else
        gcs:send_text(6, "THR_MAX retrieve failed")
    end
else
    gcs:send_text(6, "No valid command received")
end