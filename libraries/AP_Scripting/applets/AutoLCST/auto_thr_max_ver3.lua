-- Plane thr_max of AutoLCST(Longitudinal Control System Tuning).Ver1

--　パラメータ取得
local RC2 = rc:get_channel(2) -- RC の 2 チャンネルを取得 (エレベータ)
local k_throttle = 70 -- スロットルの機能番号
local thr_max = param:get("THR_MAX") -- 100 %
local thr_min = param:get("THR_MIN") -- 10 %

--　要求性能設定
local tas_error = 1.5  --速度判定の許容誤差

-- フラグ管理
local pitch_up_sw = false
local pitch_up_time = nil
local thr_max_tune_complete = false

-- ピッチ角と巡航速度に達するまで監視 → スロットル率取得
function tune_thr_max()
    
    if not pitch_up_sw then
        local pitch_max = param:get("SERVO2_MAX") -- エレベータの最大 PWM 値を取得
        if pitch_max ~= nil then
            RC2:set_override(pitch_max) -- PWM 値を上書き
            gcs:send_text(6, string.format("Servo ch %d set to max PWM: %d", 2, pitch_max))
            pitch_up_sw = true
            pitch_up_time = millis():tofloat() -- 現在の時刻を記録
        else
            gcs:send_text(6, "Unable to set servo to max pitch angle")
        end
    else    -- pitch_up_sw が true の場合(エレベータのPWM値を最大に上書きした後)の処理
        -- 2 秒経過したか確認
        local now = millis():tofloat()  -- 現在の時刻を記録
        if now - pitch_up_time >= 2000 then

            local tas_target = 30   -- 目標巡航速度
            local tas_now = ahrs:airspeed_estimate() * ahrs:get_EAS2TAS()   -- 現在の対気速度
            local pitch_target = 28 -- 目標ピッチ角
            local pitch_now = math.deg(ahrs:get_pitch())    -- 現在のピッチ角

            -- 現在の状態をログ出力
            gcs:send_text(6, string.format("Current Pitch: %.2f deg., TAS: %.2f m/s", pitch_now, tas_now))

            -- delay 後 最大ピッチ角かつ巡航速度に達していることを確認 → 満たした場合は, その時のスロットル率を取得, 設定 → THR_MAX 決定
            if math.abs(pitch_target - pitch_now) <= 1  and math.abs(tas_target - tas_now) <=  tas_error then --ピッチアップ完了後の処理
                -- 条件を満たした時, スロットル率を取得
                local throttle_now = SRV_Channels:get_output_scaled(k_throttle)
                if throttle_now then
                    param:set("THR_MAX", throttle_now)
                    gcs:send_text(6, string.format("THR_MAX successfully set to %.2f%%", throttle_now))
                else
                    gcs:send_text(6, "Faild to set THR_MAX")
                end
            elseif tas_now < (tas_target - tas_error) then
                -- 速度が不足している場合はスロットルを増加
                local thr_now = SRV_Channels:get_output_scaled(k_throttle)
                local thr_plus = math.min(thr_now + 5, 100) -- スロットル率を 5 % 増
                SRV_Channels:set_output_scaled(k_throttle, thr_plus)
                gcs:send_text(6, "Increased throttle")
            elseif tas_now > (tas_target + tas_error) then
                -- 速度が過剰な場合はスロットルを減少
                local thr_now = SRV_Channels:get_output_scaled(k_throttle)
                local thr_minus = math.max(thr_now - 5, 10)
                SRV_Channels:set_output_scaled(k_throttle, thr_minus)
                gcs:send_text(6, "Decreased throttle")
            else
                gcs:send_text(6, "Pitch or TAS conditions not met, continuing adjustment")
            end
            
                --[[gcs:send_text(6, "Target pitch angle is reached")
                if math.abs(tas_target - tas_now) <=  tas_error then
                    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)
                    if throttle_now then
                        param:set("THR_MAX", throttle_now)
                        gcs:send_text(6, string.format("THR_MAX successfully set to %.2f%%", throttle_now))
                    else
                        gcs:send_text(6, "Faild to set THR_MAX")
                    end
                elseif tas_now < (tas_target - tas_error) then
                    local thr_now = SRV_Channels:get_output_scaled(k_throttle)
                    local thr_plus = math.min(thr_now + 5, 100) -- スロットル率を 5 % 増
                    SRV_Channels:set_output_scaled(k_throttle, thr_plus)
                    gcs:send_text(6, "Increased throttle")
                elseif tas_now > (tas_target + tas_error) then
                    local thr_now = SRV_Channels:get_output_scaled(k_throttle)
                    local thr_minus = math.max(thr_now - 5, 10)
                    SRV_Channels:set_output_scaled(k_throttle, thr_minus)
                    gcs:send_text(6, "Decreased throttle")
                else
                    gcs:send_text(4, "Throttle calculation fails")
                end
            else
                gcs:send_text(6, "Target pitch angle not reached")
            end
        ]]--
        else
            -- 2秒経過していない場合は待機
            gcs:send_text(6, "Waiting for 2 seconds after setting pitch up")
        end
    end
end

-- メインループ関数
function update()
    local id, cmd = vehicle:nav_script_time()

    if id ~= nil and cmd == 1 then
        if not thr_max_tune_complete then
            tune_thr_max()
        else
            vehicle:nav_script_time_done(id)
        end
    end
--[[
    if id ~= nil then
        if cmd == 1 then
            tune_thr_max()
            vehicle:nav_script_time_done(id)
        else
            vehicle:nav_script_time_done(id)
        end
    end
]]--
    -- 0.1秒ごとに更新
    return update, 100
end

return update()