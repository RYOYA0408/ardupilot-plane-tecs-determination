-- lua API コマンドの有効性確認, デバック用モニター

-- デバック用パラメータ監視リスト
function debug_param_monitor()
    
    -- スロットルの機能番号
    local k_throttle = 70

    -- パラメータ取得を適宜追加 (Luaコマンド)
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)
    local pitch_now = math.deg(ahrs:get_pitch())

    -- 取得したパラメータをリスト化(適宜同じフォーマットで追加)
    local params = {
        {name = "Throttle percent", value = throttle_now, format="%.2f%%"},
        {name = "Pitch angle", value = pitch_now, format = "%.2f deg."},
    }

    local all_params_valid = true

    for _, param in ipairs(params) do
        if param.value == nil then
            gcs:send_text(4, string.format("%s is nil", param.name))
            all_params_valid = false
        end
    end

    -- 全パラメータが有効な場合, GCS 送信
    if all_params_valid then
        for _, param in ipairs(params) do
            gcs:send_text(6, string.format("%s: " .. param.format, param.name, param.value))
        end
    end
    return debug_param_monitor, 1000
end

return debug_param_monitor()