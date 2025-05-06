-- Plane TECS AutoTune script

-- 必要な固定変数を取得
local FREQUENCY = 10                            -- コードのサンプリング周波数
local k_throttle = 70                           -- スロットルチャンネル番号
local thr_max = param:get("THR_MAX")            -- 最大スロットル率
local thr_min = param:get("THR_MIN")            -- 最小スロットル率
local arspd_max = param:get("ARSPD_FBW_MAX")    -- FBW の最大速度 (これを全体の最大速度と定義)

-- 値安定性チェック用のバッファー&変数定義
local arspd_val_buff = {}       -- 実測速度値バッファ 
local arspd_buff = {}           -- 速度判定バッファ
--local dh_buff = {}            -- 上昇率バッファ
local arspd_margin = 0.3        -- 許容速度誤差
local arspd_diff_margin = 0.05  -- 許容速度変化率
local dh_margin = 0.25          -- 許容上昇率誤差
local buff_size = 40            -- バッファーサイズ N (N/FREQUENCY [秒])
local dt = 2.0                  -- 変化率計算時の刻み幅

-- 高度に関する目標パラメータ・許容誤差
local height_climb_min = 70     -- 速度監視開始高度
local target_alt = 140      -- 目標上昇高度
local cruise_height = 40.0  -- 目標巡航高度 [m]
local height_margin = 1.5   -- 目標巡航高度の許容誤差 [m]

-- 上昇回数に応じた上昇ピッチ角のテーブルリスト
local pitch_table_max = 23  -- 初回上昇時のピッチ角
local pitch_table_min = 15  -- 最終上昇時のピッチ角
local pitch_table = {}      -- ピッチ角テーブルの生成
-- テーブルへインサート
for p = pitch_table_max, pitch_table_min, -1 do
    table.insert(pitch_table, p)
end

local scripting_rc = rc:find_channel_for_option(300) -- TECS チューニング用スイッチ ch

-- フラグ管理&初期化
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


-- 現在の基本状態変数取得関数
-- @ return pitch_deg, arspd_now, dh_now, height_now, throttle_now
function update_sensors()
    local pitch_deg = math.deg(ahrs:get_pitch())
    local arspd_now = ahrs:airspeed_estimate()  -- 速度
    local dh_now =  - ahrs:get_velocity_NED():z() -- 右手系 z 軸下向きの速度ベクトル (上昇率)
    local height_now =  - ahrs:get_relative_position_NED_home():z() -- 地面からの相対高度
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)
    return pitch_deg, arspd_now, dh_now, height_now, throttle_now
end


-- 値と値の変化率の両面から安定性を評価し, 判定バッファに記録, 全体が安定していれば all_ok を返す
-- @param now_val           現在の値 (例：arspd_now)
-- @param target_val        目標値 (例：TRIM_ARSPD_CM * 0.01)
-- @param val_buff          値バッファ (例：arspd_val_buff)
-- @param flag_buff         安定性判定用バッファ (例：arspd_buff ← これは値を持っているんじゃなくて, 要素が boolean 的)
-- @param err_margin        値の許容誤差 (例：arspd_margin)
-- @param slope_margin      許容値変化率 (例：arspd_diff_margin)
-- @return boolean all_ok
function val_stab_check(now_val, target_val, val_buff, flag_buff, stab_sw, err_margin, slope_margin)
    -- バッファ値を更新
    -- 値を随時バッファに記録&サイズを超えたら古いデータから削除
    table.insert(val_buff, now_val)
    if #val_buff > buff_size then
        table.remove(val_buff, 1)
    end

    -- 値の変化率計算 (※ N < buff_size)
    local slope_ok = false
    local N = math.floor(FREQUENCY * dt)
    if #val_buff >= N + 1 then   -- 最低1秒間の幅確保
        local dV = val_buff[#val_buff] - val_buff[#val_buff - N]
        local slope = math.abs(dV / dt)
        slope_ok = slope <= slope_margin
    end

    -- 誤差監視 + 変化率チェック → true/false (true = stable) フラグに変換
    local err = math.abs(now_val - target_val)
    local stable = err <= err_margin and slope_ok
    
    table.insert(flag_buff, stable)
    if #flag_buff > buff_size then
        table.remove(flag_buff, 1)
    end

    -- 全体の安定性チェック (stab_sw が false のときのみ判定)
    -- stab_sw が nil ならチェックをスキップ
    local all_ok = false
    local stable_sw_check = (stab_sw == nil) or (stab_sw == false)
    if stable_sw_check and #flag_buff == buff_size then
        all_ok = true
        for _, flag in ipairs(flag_buff) do
            if not flag then
                all_ok = false
                break
            end
        end
    end
    return all_ok
end


function tecstuning(sw)

    if sw and not tecstune_running then
        tecstune_running = true
        tecstune_stage = 1
        gcs:send_text(0, string.format("TECS Tune Switch ON"))
    end

    if not sw then
        if tecstune_running then
            param:set_and_save("TRIM_ARSPD_CM", 2500)
            param:set_and_save("LIM_PITCH_MAX", 2500)
            param:set_and_save("TECS_SPDWEIGHT", 1)
            gcs:send_text(0, string.format("TECS Tune Switch OFF"))
            mission:set_current_cmd(2)
            vehicle:set_mode(10)
        end
        -- sw 管理 (TecsTune Switch OFF で, すべて false)
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

    -- 加速フェーズ：上昇飛行直前に TRIM_ARSPD 一時的に 25 → 30 m/s とすることで間接的にスロットル率を上昇させる
    if tecstune_stage == 1 then
        local _, arspd_now, _, _, _ = update_sensors()
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

    -- 最大上昇率 & 最大・最小ピッチ角 & 理論最大降下率 取得フェーズ
    if tecstune_stage == 2 then
        local pitch_deg, arspd_now, dh_now, height_now, throttle_now = update_sensors()
        local item = mission:get_item(5) -- 5番目のミッションを取得

        -- 上昇させるために TKOFF ミッションを書き換え & 上昇回数の記録
        if item and not climb_sw then
            local lua_cnt = param:get("SCR_USER1")                                  -- 上昇回数を取得 (0スタート)
            local pitch_cmd = pitch_table[math.min(lua_cnt + 1, #pitch_table)]      -- 上昇回数に応じてピッチ角テーブルから次の上昇ピッチ角を取得
            param:set_and_save("LIM_PITCH_MAX", pitch_cmd*100)                      -- 機体の最大ピッチ角を再設定して上昇時のピッチ角オーバーシュートを防止
            gcs:send_text(6, string.format("Pitch up angle set to: %.2f deg.", pitch_cmd))
            
            item:command(22)            -- TKOFFコマンド番号
            item:param1(pitch_cmd)      -- 任意の上昇ピッチ角
            item:z(target_alt)          -- 上昇高度
            mission:set_item(5, item)   -- TKOFFコマンドの指示内容を上書き
            mission:set_current_cmd(5)  -- TKOFFコマンドへジャンプ
            gcs:send_text(6, "Start climb motion (stage 2)")
            
            -- 上昇飛行に移行後 TRIM_ARSPD = 25 m/s に復帰させる
            param:set_and_save("TRIM_ARSPD_CM", 2500)
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(0, string.format("Return Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            
            -- 上昇回数を加算
            lua_cnt = lua_cnt + 1
            param:set_and_save("SCR_USER1", lua_cnt)
            gcs:send_text(6, string.format("Climb Count: %d", param:get("SCR_USER1")))
            climb_sw = true
        end
        
        -- 上昇中処理 & 速度の監視 (70 ~ 140 m)
        if climb_sw and height_now >= height_climb_min and throttle_now == thr_max then
            local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
            
            -- 安定性チェック (速度バッファ・誤差・変化率)
            local arspd_stable = val_stab_check(arspd_now, arspd_target,
                                          arspd_val_buff, arspd_buff,
                                          get_climb_rate_sw,
                                          arspd_margin, arspd_diff_margin)

            -- 安定性チェック後, 最大上昇率として取得&付随するパラメータの計算及び記録
            if arspd_stable then
                -- GCS 送信用の生データ
                local climb_max_rate = dh_now                                       -- 最大上昇率の取得
                local pitch_max = pitch_deg                                         -- この時の最大ピッチ角
                local pitch_min = - (pitch_max - 5.0)                               -- 最大ピッチ角に合わせた最小ピッチ角算出
                local sink_max_rate = arspd_max * math.sin(math.rad(pitch_min))     -- 理論最大降下率算出
                -- GCS へ結果を送信
                gcs:send_text(0, string.format("TECS_CLIMB_MAX get to %.2f m/s", climb_max_rate))
                gcs:send_text(0, string.format("LIM_PITCH_MAX_DEG get to %.2f deg.", pitch_max))
                gcs:send_text(0, string.format("LIM_PITCH_MIN_DEG get to %.2f deg.", pitch_min))
                gcs:send_text(0, string.format("TECS_SINK_MAX get to %.2f m/s", sink_max_rate))
                -- パラメータセット用データ (Increment & Range 調整後)
                local climb_max_rate_IR = math.max(0.1, math.min(math.floor(climb_max_rate*10 + 0.5) / 10))
                local pitch_max_IR = math.max(0.0, math.min(9000, math.floor(pitch_max*100 + 0.5)))
                local pitch_min_IR = math.max(-9000, math.min(0, math.floor(pitch_min*100 + 0.5)))
                local sink_max_rate_IR = math.max(0.1, math.min(20.0, math.floor(math.abs(sink_max_rate)*10 - 0.5) / 10))
                -- 記録テスト
                param:set_and_save("SCR_USER2", climb_max_rate_IR)
                param:set_and_save("SCR_USER3", pitch_max_IR)
                param:set_and_save("SCR_USER4", pitch_min_IR)
                param:set_and_save("SCR_USER5", sink_max_rate_IR)
                --[[
                param:set_and_save("TECS_CLMB_MAX", climb_max_rate_IR)
                param:set_and_save("LIM_PITCH_MAX", pitch_max_IR)
                param:set_and_save("LIM_PITCH_MIN", pitch_min_IR)
                param:set_and_save("TECS_SINK_MAX", sink_max_rate_IR)
                ]]--

                get_climb_rate_sw = true
                mission:set_current_cmd(2)
                gcs:send_text(6, "Start get TECS_SINK_MIN (stage 3)")
                tecstune_stage = 3
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
        local _, arspd_now, dh_now, _, throttle_now = update_sensors()
        
        -- 再度記述しているのは, このパラメータをよく変更しているので処理周期により意図しない値を読み込んでほしくないため
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01

        if not set_spdw_sw then
            param:set_and_save("TECS_SPDWEIGHT", 2.0)
            gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
            set_spdw_sw = true
        end

        if set_spdw_sw and not get_sink_min_sw then
            -- 安定性チェック (速度バッファ・誤差・変化率)
            local all_ok = val_stab_check(arspd_now, arspd_target,
                                          arspd_val_buff, arspd_buff,
                                          nil,
                                          arspd_margin, arspd_diff_margin)

            if all_ok and throttle_now == thr_min and dh_now < 0 then
                -- GCS 送信用の生データ
                local sink_min_rate = dh_now
                -- GCS へ結果を送信
                gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                -- パラメータセット用データ (Increment & Range 調整後)
                local sink_min_rate_IR = math.max(0.1, math.min(10.0, math.floor(math.abs(sink_min_rate)*10 - 0.5) / 10))
                -- 記録テスト
                param:set_and_save("SCR_USER6", sink_min_rate_IR)
                --param:set_and_save("TECS_SINK_MIN", sink_min_rate_IR)
                
                param:set_and_save("TECS_SPDWEIGHT", 1)
                gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
                gcs:send_text(6, "Start get TRIM_THROTTLE (stage 4)")
                get_sink_min_sw = true
                mission:set_current_cmd(2)
                tecstune_stage = 4
            end
        end
    end
    
    -- TRIM_THROTTLE 率の取得：速度が 25 m/s で安定 かつ 高度と上昇率が所定で安定
    if tecstune_stage == 4 then
        local _, arspd_now, dh_now, height_now, throttle_now = update_sensors()
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
        local height_error = math.abs(cruise_height - height_now)

        -- 速度安定性のチェック
        local arspd_stable = val_stab_check(arspd_now, arspd_target,
                                            arspd_val_buff, arspd_buff,
                                            get_trim_thr_sw,
                                            arspd_margin, arspd_diff_margin)

        if not get_trim_thr_sw
            and arspd_stable
            and height_error <= height_margin
            and math.abs(dh_now) <= dh_margin then
            
            -- GCS 送信用の生データ
            local trim_throttle = throttle_now
            -- GCS へ結果を送信
            gcs:send_text(0, string.format("TRIM_THROTTLE get to %.2f%%", trim_throttle))
            -- パラメータセット用データ (Increment & Range 調整後)
            --local trim_throttle_IR = math.max(0, math.min(100, math.floor(trim_throttle + 0.5)))
            -- 記録テスト
            --param:set_and_save("TRIM_THROTTLE", trim_throttle_IR)
            
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