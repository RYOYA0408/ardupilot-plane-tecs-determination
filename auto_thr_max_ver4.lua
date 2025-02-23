-- Plane thr_max of AutoLCST(Longitudinal Control System Tuning).Ver1

--　パラメータ取得
local RC2 = rc:get_channel(2) -- RC の 2 チャンネルを取得 (エレベータ)
local pitch_max = param:get("SERVO2_MAX") -- エレベータの最大 PWM 値を取得
local k_throttle = 70 -- スロットルの機能番号
local thr_max = param:get("THR_MAX") -- 100 %
local thr_min = param:get("THR_MIN") -- 10 %
local thr_buff = 5.0  -- スロットルの増減幅 %

--　要求性能設定
local tas_target = 28   -- 目標巡航速度
local tas_error = 1.5  -- 速度判定の許容誤差
local pitch_target = 28 -- 目標ピッチ角
local pitch_error = 1.0 -- ピッチ上昇判定の許容誤差

-- フラグ管理
local pitch_up_sw = false
--local pitch_up_time = nil
local thr_max_tune_complete = false

-- ピッチ角と巡航速度に達するまで監視 → スロットル率取得
function tune_thr_max()
    
    if not pitch_up_sw then
        --local pitch_max = param:get("SERVO2_MAX") -- エレベータの最大 PWM 値を取得
        if pitch_max ~= nil then
            RC2:set_override(pitch_max) -- PWM 値を上書き
            gcs:send_text(6, string.format("Servo ch %d set to max PWM: %d", 2, pitch_max))
            pitch_up_sw = true
        else
            gcs:send_text(6, "Unable to set servo to max pitch angle")
        end
    else
        -- pitch_up_sw が true の場合(エレベータのPWM値を最大に上書きした後)の処理
        local tas_now = ahrs:airspeed_estimate() * ahrs:get_EAS2TAS()   -- 現在の対気速度
        local pitch_now = math.deg(ahrs:get_pitch())    -- 現在のピッチ角
        -- 現在の状態をログ出力
        gcs:send_text(6, string.format("Current Pitch: %.2f deg., TAS: %.2f m/s", pitch_now, tas_now))

        -- 最大ピッチ角かつ巡航速度に達していることを確認 → 満たした場合は, その時のスロットル率を取得, 設定 → THR_MAX 決定
        if math.abs(pitch_target - pitch_now) <= pitch_error and math.abs(tas_target - tas_now) <=  tas_error then
            -- 条件を満たした時, スロットル率を取得
            local throttle_now = SRV_Channels:get_output_scaled(k_throttle)
            if not thr_max_tune_complete and throttle_now then
                param:set("THR_MAX", throttle_now)
                gcs:send_text(6, string.format("THR_MAX successfully set to %.2f%%", throttle_now))
                thr_max_tune_complete = true
            else
                gcs:send_text(6, "Faild to set THR_MAX")
            end
        else

            -- スロットル調整
            local thr_now = SRV_Channels:get_output_scaled(k_throttle)
            if tas_now < (tas_target - tas_error) then
                -- 速度が不足している場合
                local thr_plus = constrain(thr_now + thr_buff, 10, 100)
                vehicle:set_target_throttle_rate_rpy(thr_plus, 0, 0, 0)
                gcs:send_text(6, string.format("Plus throttle to %.2f%%", thr_plus))
            elseif tas_now > (tas_target + tas_error) then
                -- 速度が過剰な場合
                local thr_minus = constrain(thr_now - thr_buff, 10, 100)
                vehicle:set_target_throttle_rate_rpy(thr_minus, 0, 0, 0)
                gcs:send_text(6, string.format("Minus throttle to %.2f%%", thr_minus))
            else
                gcs:send_text(6, "Continuing adjustments")
            end
        end
    end
end

-- メインループ関数
-- フラグを追加して状態管理
local cmd_active = false

function update()
    local id, cmd = vehicle:nav_script_time()

    if id ~= nil and cmd == 1 then
        -- 初回受信時にフラグを有効化
        if not cmd_active then
            cmd_active = true
            gcs:send_text(6, string.format("NAV_SCRIPT_TIME activated with ID: %d", id))
        end

        -- フラグが有効な間はスクリプト処理を継続
        if not thr_max_tune_complete then
            tune_thr_max()
        else
            -- 全ての条件が満たされたら完了通知を送信
            --vehicle:nav_script_time_done(id)
            gcs:send_text(6, "NAV_SCRIPT_TIME completed.")
            cmd_active = false -- フラグをリセット
        end
    elseif id ~= nil then
        -- `cmd_active`がfalseの場合は完了通知を送らない
        gcs:send_text(6, string.format("Ignoring NAV_SCRIPT_TIME ID: %d because cmd_active is false", id))
    end

    -- 0.1秒ごとに更新
    return update, 100
end

return update()