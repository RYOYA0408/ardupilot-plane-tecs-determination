-- Plane TECS AutoTune script

local k_throttle = 70
local thr_max = param:get("THR_MAX")
local thr_min = param:get("THR_MIN")
local arspd_max = param:get("ARSPD_FBW_MAX")


local FREQUENCY = 10

-- バッファー用変数定義
local arspd_diff_margin = 0.05 -- 許容速度変化率
local arspd_val_buff = {}   -- 実測速度値バッファ 
local arspd_margin = 0.3    --許容速度
local arspd_buff = {}   -- 速度判定バッファ
local dh_margin = 0.25
--local dh_buff = {}
local buff_size = 40  -- バッファーサイズ N (N/FREQUENCY [秒])
local dt = 2.0

-- 高度に関する目標パラメータ・許容誤差
local target_alt = 140  -- 目標上昇高度
local cruise_height = 40.0 -- 目標巡航高度 [m]
local height_margin = 1.5  -- 目標巡航高度の許容誤差 [m]

local pitch_table = {23, 22, 21, 20, 19, 18, 17, 16, 15}

local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch

local tecstune_running = false
local trimspd_sw = false
local climb_sw = false
local get_climb_rate_sw = false
local ft_sw = false   -- 上昇強制終了フラグ
local get_sink_min_sw = false
local set_spdw_sw = false
local get_trim_thr_sw = false

-- チューニングステージ
-- 0: 未実行
-- 1: 加速フェーズ
-- 2: 最大上昇率&(最大・最小ピッチ角, 理論最大降下率)の取得
-- 3: 最小降下率の取得
-- 4: トリムスロットル率取得
local tecstune_stage = 0

function tecstuning(sw)
    -- 基本状態変数取得
    local pitch_deg = math.deg(ahrs:get_pitch())
    local arspd_now = ahrs:airspeed_estimate()  -- 速度
    local dh_now =  - ahrs:get_velocity_NED():z() -- 右手系 z 軸下向きの速度ベクトル (上昇率)
    local height_now =  - ahrs:get_relative_position_NED_home():z() -- 地面からの相対高度
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)

    if sw and not tecstune_running then
        tecstune_running = true
        tecstune_stage = 1
        gcs:send_text(0, string.format("TECS Tune Switch ON"))
    end

    if not sw then
        if tecstune_running then
            mission:set_current_cmd(2)
            param:set_and_save("TRIM_ARSPD_CM", 2500)
            param:set_and_save("LIM_PITCH_MAX", 2500)
            param:set_and_save("TECS_SPDWEIGHT", 1)
            gcs:send_text(0, string.format("TECS Tune Switch OFF"))
        end
        tecstune_running = false
        trimspd_sw = false
        climb_sw = false
        get_climb_rate_sw = false
        ft_sw = false
        get_sink_min_sw = false
        set_spdw_sw = false
        get_trim_thr_sw = false
        tecstune_stage = 0
    end

    -- 加速フェーズ
    if tecstune_stage == 1 then
        if not trimspd_sw then
            param:set_and_save("TRIM_ARSPD_CM", 3000)   --トリム速度値を変更
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(0, string.format("Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            gcs:send_text(6, "Start Acceleration (stage 1)")
            trimspd_sw = true
        end
        if trimspd_sw and arspd_now > arspd_max then
            tecstune_stage = 2
        end
    end

    -- 最大上昇率&(最大・最小ピッチ角, 理論最大降下率)の取得フェーズ
    if tecstune_stage == 2 then
        local item = mission:get_item(5) -- 5番目のミッションを取得
        -- 上昇させるためのミッションの書き換え処理 & 上昇回数の記録
        if item and not climb_sw then
            local lua_cnt = param:get("SCR_USER1")  -- 上昇回数を取得 (0スタート)
            gcs:send_text(6, string.format("Climb Count: %d", param:get("SCR_USER1")))
            local pitch_cmd = pitch_table[math.min(lua_cnt + 1, #pitch_table)]   -- 上昇回数に応じてピッチ角テーブルから次の上昇ピッチ角を取得
            param:set_and_save("LIM_PITCH_MAX", pitch_cmd*100) -- 機体の最大ピッチ角を再設定して上昇時のピッチ角オーバーシュートを防止
            gcs:send_text(6, string.format("Pitch up angle: %.2f deg.", pitch_cmd)) -- 上昇する時の最大ピッチ角を可視化
            
            item:command(22)    -- TKOFFコマンド番号
            item:param1(pitch_cmd)     -- 任意の上昇ピッチ角
            item:z(target_alt)   -- 上昇高度
            mission:set_item(5, item)   -- TKOFFコマンドの指示内容を上書き
            mission:set_current_cmd(5)  -- TKOFFコマンドへジャンプ
            gcs:send_text(6, "Start climb motion (stage 2)")
            
            param:set_and_save("TRIM_ARSPD_CM", 2500)
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(0, string.format("Return Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            
            lua_cnt = lua_cnt + 1
            param:set_and_save("SCR_USER1", lua_cnt)
            climb_sw = true
        end
        -- 上昇中の処理
        -- 速度の監視 (70 ~ 140 m)
        local height_climb_min = 70     -- 速度監視開始高度
        if climb_sw and height_now >= height_climb_min and throttle_now == thr_max then
            local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
            local arspd_err = math.abs(arspd_now - arspd_target)    -- 誤差監視

            -- 速度値を随時バッファに記録&サイズを超えたら古いデータから削除
            table.insert(arspd_val_buff, arspd_now)
            if #arspd_val_buff > buff_size then
                table.remove(arspd_val_buff, 1)
            end
            -- 傾き(速度変化率)を計算
            local slope_ok = false
            local N = math.floor(FREQUENCY * dt)
            if #arspd_val_buff >= N + 1 then   -- 最低1秒間の幅
                local dV = arspd_val_buff[#arspd_val_buff] - arspd_val_buff[#arspd_val_buff - N]
                local slope = math.abs(dV / dt)
                slope_ok = slope <= arspd_diff_margin
            end
            
            -- 誤差条件を満足すれば true, 否は false でバッファに記録
            if arspd_err <= arspd_margin and slope_ok then
                table.insert(arspd_buff, true)
            else
                table.insert(arspd_buff, false)
            end
            -- 古いデータはバッファから削除
            if #arspd_buff > buff_size then
                table.remove(arspd_buff, 1)
            end

            -- 一定期間速度が保持できていたら、上昇率を取得し, 最大上昇率と決定
            if not get_climb_rate_sw and #arspd_buff == buff_size then
                local all_ok = true
                for _, flag in ipairs(arspd_buff) do
                    if not flag then
                        all_ok = false
                        break
                    end
                end

                -- すべての条件を満たしたとき, 最大上昇率として取得&付随するパラメータの計算及び記録
                if all_ok then
                    local climb_max_rate = dh_now -- 最大上昇率の取得
                    local pitch_max = pitch_deg   -- この時の最大ピッチ角
                    local pitch_min = - (pitch_max - 5.0) -- 最大ピッチ角に合わせた最小ピッチ角算出
                    local sink_max_rate = - arspd_max * math.sin(math.rad(pitch_min))
                    gcs:send_text(0, string.format("TECS_CLIMB_MAX get to %.2f m/s", climb_max_rate))
                    gcs:send_text(0, string.format("LIM_PITCH_MAX_DEG get to %.2f deg.", pitch_max))
                    gcs:send_text(0, string.format("LIM_PITCH_MIN_DEG get to %.2f deg.", pitch_min))
                    gcs:send_text(0, string.format("TECS_SINK_MAX get to %.2f m/s", sink_max_rate))
                    -- 記録テスト
                    param:set_and_save("SCR_USER2", climb_max_rate)
                    param:set_and_save("SCR_USER3", pitch_max*100)
                    param:set_and_save("SCR_USER4", pitch_min*100)
                    param:set_and_save("SCR_USER5", sink_max_rate)
                    
                    get_climb_rate_sw = true
                    mission:set_current_cmd(2)
                    gcs:send_text(6, "Start get TECS_SINK_MIN (stage 3)")
                    tecstune_stage = 3
                end
            end
        end

        -- 最大上昇高度に達し強制終了
        if climb_sw and height_now >= target_alt and not ft_sw then
            mission:set_current_cmd(2)
            vehicle:set_mode(10)    -- 勝手にRTLモードになる場合があるため Auto モードを噛ませる
            gcs:send_text(0, "Return to MP 2")
            ft_sw = true
        end
    end

    --TECS_SINK_MIN の取得
    if tecstune_stage == 3 then
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01  -- 再度記述しているのは, このパラメータをよく変更しているので処理周期により意図しない値を読み込んでほしくないため
        local arspd_err = math.abs(arspd_now - arspd_target)    -- 誤差監視

        if not set_spdw_sw then
            param:set_and_save("TECS_SPDWEIGHT", 2.0)
            set_spdw_sw = true
        end

        if set_spdw_sw and not get_sink_min_sw then
            -- 速度値を随時バッファに記録&サイズを超えたら古いデータから削除 (再利用)
            table.insert(arspd_val_buff, arspd_now)
            if #arspd_val_buff > buff_size then
                table.remove(arspd_val_buff, 1)
            end

            -- 傾き(速度変化率)を計算
            local slope_ok = false
            local N = math.floor(FREQUENCY * dt)
            if #arspd_val_buff >= N + 1 then   -- 最低1秒間の幅
                local dV = arspd_val_buff[#arspd_val_buff] - arspd_val_buff[#arspd_val_buff - N]
                local slope = math.abs(dV / dt)
                slope_ok = slope <= arspd_diff_margin
            end

            -- 誤差条件を満足すれば true, 否は false でバッファに記録
            if arspd_err <= arspd_margin and slope_ok then
                table.insert(arspd_buff, true)
            else
                table.insert(arspd_buff, false)
            end
            -- 古いデータはバッファから削除
            if #arspd_buff > buff_size then
                table.remove(arspd_buff, 1)
            end

            -- 一定期間速度が保持できていたら、上昇率を取得し, 最小降下率と決定
            if #arspd_buff == buff_size then
                local all_ok = true
                for _, flag in ipairs(arspd_buff) do
                    if not flag then
                        all_ok = false
                        break
                    end
                end

                if all_ok and throttle_now == thr_min and dh_now < 0 then
                    local sink_min_rate = dh_now
                    gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                    -- 記録テスト
                    param:set_and_save("SCR_USER6", math.abs(sink_min_rate))
                    param:set_and_save("TECS_SPDWEIGHT", 1)
                    get_sink_min_sw = true
                    tecstune_stage = 4
                    gcs:send_text(6, "Start get TRIM_THROTTLE (stage 4)")
                    mission:set_current_cmd(2)
                end
            end
        end
    end
    
    -- TRIM_THROTTLE 率の取得
    if tecstune_stage == 4 then
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
        local arspd_error = math.abs(arspd_target - arspd_now)
        local height_error = math.abs(cruise_height - height_now)
        if not get_trim_thr_sw and arspd_error <= arspd_margin and
           height_error <= height_margin and math.abs(dh_now) <= dh_margin then
            local trim_throttle = throttle_now
            gcs:send_text(0, string.format("TRIM_THROTTLE get to %.2f%%", trim_throttle))
            gcs:send_text(0, string.format("Finished TecsTune"))
            get_trim_thr_sw = true
            mission:set_current_cmd(2)
            vehicle:set_mode(10)
        end
    end
end

function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc then
        local sw_pos = scripting_rc:get_aux_switch_pos()
        if sw_pos == 1 then 
            tecstuning(true)
        else
            tecstuning(false)
        end
    end

    return update, FREQUENCY
end

return update()