-- Plane TECS AutoTune script
-- 使用した WayPoints : 

-- 各パラメータ取得
local thr_max = param:get("THR_MAX")
local thr_min = param:get("THR_MIN")
local k_throttle = 70 -- スロットルの機能番号
local arspd_max = param:get("ARSPD_FBW_MAX")
local arspd_min = param:get("ARSPD_FBW_MIN")
local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01 -- 目標巡航速度 25 [m/s]
local pitch_max = param:get("LIM_PITCH_MAX") * 0.01 -- 目標最大ピッチ角 21 [deg.]
local pitch_min = - (pitch_max - 5.0)   -- pitch_max を考慮した LIM_PITCH:_MIN [deg.]
local sink_max_rate = - arspd_max * math.sin(math.rad(pitch_min))

-- その他パラメータ目標・許容誤差
local height_min = 40 -- 最大上昇率計測を開始する最低高度
local cruise_height = 100 -- 巡航(水平定常飛行)時の高度
local cruise_dh = 0.02 -- 巡航(水平定常飛行)時の上昇率の許容誤差

local pitch_margin = 0.1 -- 目標最大ピッチ角の許容誤差 [deg.]
local arspd_margin = 0.4 -- 目標巡航速度の許容誤差 [m/s]
local height_margin = 0.5 -- 目標巡航高度の許容誤差 [m]
local dh_margin = 0.1 -- 巡航時の目標上昇率の許容誤差 [m/s]

-- フラグ管理
local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch
local FREQUENCY = 10    -- 計算刻み 10 ms
local tecstune_running = false
local get_climb_rate_sw = false
local get_trim_thr_sw = false
local get_sink_min_sw = false
local monitor_sw = false
local monitor_start_sw = false

-- チューニングステージ
-- 0: 未実行
-- 1: 離陸時の最大上昇率取得
-- 2: TRIM_THROTTLE 率の取得
-- 3: TECS_SINK_MIN の取得
local tecstune_stage = 0

function tune_tecs(sw)
    -- 状態変数取得 -------------------------------------------------------------
    local pitch_deg = math.deg(ahrs:get_pitch())
    local pitch_error = math.abs(pitch_max - pitch_deg)
    local arspd_now = ahrs:airspeed_estimate()
    local arspd_error = math.abs(arspd_target - arspd_now)
    local height_now = - ahrs:get_relative_position_NED_home():z()
    local height_error = math.abs(cruise_height - height_now)
    local dh_now =  - ahrs:get_velocity_NED():z() -- 右手系 z 軸下向きの速度ベクトル
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)
    ----------------------------------------------------------------------------

    -- フラグ管理　-----------------------------------------------------------------------------------------------------
    if sw and not tecstune_running then
        tecstune_running = true
        tecstune_stage = 1
        -- 既定値 or 既定値からの計算で求められる変数 -------------------------------------------
        gcs:send_text(0, string.format("Starting main TECS tuning"))
        gcs:send_text(6, string.format("Specification Parameters:"))
        gcs:send_text(0, string.format("THR_MAX %.2f%%", thr_max))
        gcs:send_text(6, string.format("AIRSPEED_MAX, MIN %.2f m/s", arspd_max, arspd_min))
        gcs:send_text(6, string.format("LIM_PITCH_MAX %.2f deg.", pitch_max))
        gcs:send_text(6, string.format("LIM_PITCH_MIN %.2f deg.", pitch_min))
        gcs:send_text(0, string.format("TECS_SINK_MAX get to %.2f m/s", sink_max_rate))
        --------------------------------------------------------------------------------------
    end

    if not sw then
        if tecstune_running then
            gcs:send_text(0, string.format("Finished main TECS tuning"))
        end
        tecstune_running = false
        tecstune_stage = 0
    end
    ------------------------------------------------------------------------------------------------------------------

    -- 飛行ステージ別アクション ----------------------------------------------------------------------------------------

    -- 離陸時の最大上昇率取得
    if tecstune_stage == 1 then
        local item = mission:get_item(1)    -- ミッション番号 1 取得
        -- TAKEOFF コマンドの特定 (22 : TAKEOFF)
        if not get_climb_rate_sw and item and item:command() == 22 then
            local takeoff_alt = item:z() -- TAKEOFFコマンドの目標高度を取得
            -- TAKEOFF コマンドの目標高度未満で最大ピッチ角を満たす時最大上昇率を取得
            if height_now >= height_min and height_now < takeoff_alt and pitch_error <= pitch_margin then
                local climb_max_rate = dh_now -- 上昇率の取得
                gcs:send_text(0, string.format("TECS_CLIMB_MAX get to %.2f m/s", climb_max_rate))
                get_climb_rate_sw = true
                tecstune_stage = 2
            end
        else
            gcs:send_text(6, string.format("Get climb rate failed"))
        end
    end
    
    -- TRIM_THROTTLE 率の取得
    if tecstune_stage == 2 then
        if not get_trim_thr_sw then
            if arspd_error <= arspd_margin and height_error <= height_margin and math.abs(dh_now) <= cruise_dh then
                local trim_throttle = throttle_now
                gcs:send_text(0, string.format("TRIM_THROTTLE get to %.2f%%", trim_throttle))
                get_trim_thr_sw = true
                tecstune_stage = 3
            end
        end
    end

    -- TECS_SINK_MIN の取得
    if tecstune_stage == 3 and not get_sink_min_sw then
        local throttle_list = {} -- throttle_nowのリスト化
        local monitor_start_time = nil
        local monitor_time = 8000 -- モニタリング時間 (ミリ秒)
        local monitor_in_progress = false -- モニタリング中フラグ

        if throttle_now == thr_min and not monitor_in_progress then
            -- モニタリング開始
            monitor_start_time = millis() -- 現在の時刻を取得
            if monitor_start_time then -- nil ではないか確認
                gcs:send_text(6, "Monitoring throttle_now at thr_min for 8 seconds...")
                monitor_in_progress = true
            else
                gcs:send_text(0, "Error: Failed to start monitoring due to nil monitor_start_time")
            end
        end

        if monitor_in_progress and monitor_start_time then
            -- 経過時間を計算
            local elapsed_time = millis() - monitor_start_time
            if elapsed_time <= monitor_time then
                -- throttle_now をリストに追加
                table.insert(throttle_list, throttle_now)
            else
                -- モニタリング終了
                monitor_in_progress = false

                -- throttle_now の平均値を計算
                local sum = 0
                for _, value in ipairs(throttle_list) do
                    sum = sum + value
                end
                local avg_throttle = sum / #throttle_list

                -- 平均が thr_min の ±3% に収束しているか判定
                if math.abs(avg_throttle - thr_min) <= (thr_min * 0.03) then
                    if arspd_error <= arspd_margin then
                        local sink_min_rate = dh_now -- 降下率の取得
                        gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                        get_sink_min_sw = true
                    end
                else
                    gcs:send_text(0, "Throttle average out of range. Retrying...")
                end

                -- モニタリングデータのリセット
                throttle_list = {}
                monitor_start_time = nil
            end
        end
    end
    --[[if tecstune_stage == 3 then
        if not get_sink_min_sw then
            if throttle_now == thr_min and arspd_error <= arspd_margin then
                local sink_min_rate = dh_now -- 降下率の取得
                gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                get_sink_min_sw = true
            end
        end
    end
    ]]--
    ---------------------------------------------------------------------------
end

function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc then
        local sw_pos = scripting_rc:get_aux_switch_pos()
        if sw_pos == 1 then 
            tune_tecs(true)
        else
            tune_tecs(false)
        end
    end

    return update, FREQUENCY
end

return update()
