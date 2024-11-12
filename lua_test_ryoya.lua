--[[ スロットル率を取得/更新する関数
function get_throttle()
    -- スロットルの機能番号
    local k_throttle = 70
    -- 現在のスロットル率を取得（0 ～ 100%）
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)

    if throttle_now ~= nil then
        -- スロットルに 10% を加算
        local throttle_new = throttle_now + 10
        -- スロットル率が 100% を超えないように制限
        if throttle_new > 100 then
            throttle_new = 100
        end

        -- 新しいスロットル値を設定
        SRV_Channels:set_output_scaled(k_throttle, throttle_new)

        -- 設定後のスロットル率を再取得
        local throttle_updated = SRV_Channels:get_output_scaled(k_throttle)

         -- GCS に送信
        if throttle_updated ~= nil then
            gcs:send_text(6, string.format("Throttle updated to: %.2f%%", throttle_updated))
            return throttle_updated
        else
            gcs:send_text(4, "Failed to update throttle")
            return false
        end
    else
        -- スロットル率を取得できない場合、エラーメッセージを送信
        gcs:send_text(4, "Throttle percent is nil")
        return false
    end
end


function get_pitch_angle()
    -- ピッチ角を取得
    local pitch_now = math.deg(ahrs:get_pitch())

    if pitch_now ~= nil then
        -- GCS に送信
        gcs:send_text(6, string.format("Pitch angle: %.2f deg.", pitch_now))
        return pitch_now
    else
        -- ピッチ角を取得できない場合、エラーメッセージを送信
        gcs:send_text(4, "Pitch angle is nil")
        return false
    end
end
]]--

local scripting_rc_1 = rc:find_channel_for_option(300) -- 任意スクリプト開始スイッチ
local pitch_up_sw = false
local RC2 = rc:get_channel(2)
local pitch_max = param:get("SERVO2_MAX")

-- エレベータサーボを最大ピッチ角に設定(サーボをMAXで出力すればFBWモードによって自動的に最大ピッチ角になる)
function set_servo2pitch_max(sw)
    if sw and not pitch_up_sw and pitch_max then
        RC2:set_override(pitch_max) -- PWM 値を上書き
        gcs:send_text(0, "Set to Pitch max")
        local pitch_now = math.deg(ahrs:get_pitch())
        local pitch_target = 28
        if pitch_now > pitch_target then
            pitch_up_sw = true
            gcs:send_text(0, "Finished pitch up")
        else
            return nil
        end
    end

    if not sw then
        if pitch_up_sw then
            RC2:set_override(1500)
            gcs:send_text(0, string.format("Cancel pitch up"))
        end
        pitch_up_sw = false
    end
end

-- メインループ関数
--[[
function update()
    local id, cmd = vehicle:nav_script_time()

    if id ~= nil then
        if cmd == 1 then
            --get_throttle()
            --get_pitch_angle()
            set_servo2pitch_max()
            vehicle:nav_script_time_done(id)
        else
            vehicle:nav_script_time_done(id)
        end
    end
    -- 0.1秒ごとに更新
    return update, 100
end
]]

function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc_1 then
        local sw_pos = scripting_rc_1:get_aux_switch_pos()
        if sw_pos == 2 then
            set_servo2pitch_max(true)
        else
            set_servo2pitch_max(false)
        end
    end
    -- 0.1秒ごとに更新
    return update, 100
end

return update()