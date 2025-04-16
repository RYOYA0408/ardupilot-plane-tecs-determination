-- Plane TECS AutoTune script
local RC2 = rc:get_channel(2)

local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch

local FREQUENCY = 10
local thrmax_running = false
local lua_cnt = param:get("SCR_USER1") or 0 -- 当該luaスクリプトの動作回数の初期値取得, 基本は0

-- チューニングステージ
-- 0: 未実行
-- 1: 上昇中
-- 2: 最大高度到達。中断
local thrmax_stage = 0

function tune_thrmax(sw)
    local pitch_deg = math.deg(ahrs:get_pitch())
    local height =  - ahrs:get_relative_position_NED_home():z() -- 地面からの相対高度

    if sw and not thrmax_running then
        thrmax_running = true
        thrmax_stage = 1
        gcs:send_text(0, string.format("Starting THR_MAX tuning"))
    end

    if not sw then
        if thrmax_running then
            RC2:set_override(1500)
            gcs:send_text(0, string.format("Finished THR_MAX tuning"))
            --　luaスクリプトの動作回数をカウントアップして保存
            lua_cnt = lua_cnt + 1
            param:set_and_save("SCR_USER1", lua_cnt)
            local lua_cnt_prev = param:get("SCR_USER1")
            gcs:send_text(0, string.format("Count: %d", lua_cnt_prev))
        end
        thrmax_running = false
        thrmax_stage = 0
    end

    -- 上昇中
    if thrmax_stage == 1 then
        -- 最大ピッチ角指令
        RC2:set_override(1900)
        if height >= 150.0 then
            RC2:set_override(1500)
            thrmax_stage = 2
        end
    end

    -- 最大高度到達。中断
    if thrmax_stage == 2 then
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
