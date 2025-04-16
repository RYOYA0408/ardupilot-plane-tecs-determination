-- スロットル率更新のテスト
local k_throttle = 70   -- スロットルの機能番号
local throttle_start = nil
local throttle_stop = 5000 -- スロットルを更新後維持する時間

--スロットル率を取得/更新する関数
function update_throttle()

    -- スロットル更新の維持開始時刻の記録
    if throttle_start == nil then
        throttle_start = millis():tofloat()
    end

    -- 5秒間は更新したスロットル率に維持
    local now = millis():tofloat()
    if (now - throttle_start) <= throttle_stop then
        -- 現在のスロットル率を取得 (0 ~ 100 %)
        local throttle_prev = SRV_Channels:get_output_scaled(k_throttle)
        if throttle_prev ~= nil then
            -- スロットルに 10% を加算
            local throttle_new = throttle_prev + 10
            -- スロットル率が 100% を超えないように制限
            if throttle_new > 100 then
                throttle_new = 100
            end
            -- 新しいスロットル率に更新
            SRV_Channels:set_output_scaled(k_throttle, throttle_new)
            -- 実機体の現在のスロットル率を取得
            local throttle = SRV_Channels:get_output_scaled(k_throttle)
            
            --　更新した(指令値)スロットル率を送信
            gcs:send_text(6, string.format("Throttle updated to: %.2f%%, Throttle now: %.2f%%", throttle_new, throttle))
        else
            -- スロットル率を取得できない場合はエラーを送信
            gcs:send_text(4, "Throttle percent is nil")
        end
    else
        -- スロットル率維持を終了
        gcs:send_text(6, "Throttle update end")
        throttle_start = nil
    end
end

-- メインループ関数
function update()
    local id, cmd = vehicle:nav_script_time()

    if id ~= nil and cmd == 1 then
        
        update_throttle()
    end

    -- 0.1秒ごとに更新
    return update, 100
end

return update()