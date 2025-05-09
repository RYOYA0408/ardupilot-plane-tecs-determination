-- Plane TECS Tune Script

-- 必要な固定変数を取得/設定
local k_throttle = 70                           -- スロットルチャンネル番号
local thr_max = param:get("THR_MAX")            -- 最大スロットル率
local thr_min = param:get("THR_MIN")            -- 最小スロットル率
local arspd_max = param:get("ARSPD_FBW_MAX")    -- FBW の最大速度 (これを全体の最大速度と定義)
local set2up_arspd = 3000

-- MissionPlanner 関連の定義
local last_nav_index = -1   -- 前回のミッションインデックス記録用
local tkoff_cmd_num = 6     -- 巡航時用ピッチアップコマンドの代替となる TAKEOFF コマンドの番号
local dojump_cmd_num = 5    -- DO_JUMP コマンドの番号

-- 安定性評価用のバッファー&変数定義
local arspd_val_buff = {}       -- 実測速度値バッファ 
local arspd_buff = {}           -- 速度安定性判定バッファ
local height_val_buff = {}      -- 実測高度値バッファ
local height_buff = {}          -- 高度安定性判定バッファ
local FREQUENCY = 10            -- コードのサンプリング周波数(ms)
local dt = FREQUENCY / 1000     -- 安定性評価時の刻み幅
local dt_log = 0.1              -- UAV Log Viewer のログ周期
local obs_time = 2              -- 観測時間
local buff_size = math.floor(obs_time / dt) -- バッファーサイズ (整数)

-- 目標パラメータ・許容誤差
local scale = math.sqrt(dt / dt_log)    -- UAV Log Viewer から取得した諸々の閾値を実験用にスケーリングする係数
-- 20241218 の実験データから算出した閾値 (0.1秒周期で計算したデータ値)
local arspd_sigma_log = 0.6869                  -- 速度不偏標準偏差
local arspd_diff_sigma_log = 0.5626             -- 機首方向加速度不偏標準偏差
local dh_sigma_log = 0.5510                     -- 上昇率不偏標準偏差
local height_sigma_log = 0.5360                 -- 高度不偏標準偏差
-- ノイズ有り状態変数生成用
local arspd_std = arspd_sigma_log * scale       -- 速度不偏標準偏差
local dh_std = dh_sigma_log * scale             -- 上昇率不偏標準偏差
local height_std = height_sigma_log * scale     -- 高度不偏標準偏差
-- 安定性評価用マージン
local margin_scale = 1.5                                        -- マージンスケール
local ci = 1.96                                                 -- 95 % 信頼区間
local arspd_margin = ci * arspd_std                             -- 許容速度誤差
local arspd_std_margin = arspd_std * margin_scale               -- 許容速度不偏標準偏差
local arspd_diff_margin = ci * arspd_diff_sigma_log * scale     -- 許容加速度誤差
local dh_margin = ci * dh_std                                   -- 許容上昇率誤差
--local dh_std_margin = dh_std                                  -- 許容上昇率不偏標準偏差
local height_margin = ci * height_std                           -- 許容目標巡航高度誤差
local height_std_margin = height_std * margin_scale             -- 許容高度不偏標準偏差
--[[
-- SITL シミュレーション用のタイトな閾値 (ノイズ無)
local arspd_margin = 0.3        -- 許容速度誤差
local arspd_diff_margin = 0.05  -- 許容速度変化率(加速度)
local dh_margin = 0.25          -- 許容上昇率誤差
local height_margin = 1.5       -- 目標巡航高度の許容誤差
]]--

local height_climb_min = 60     -- 速度監視開始高度 70
local target_alt = 150          -- 目標上昇高度
local target_alt_limit = target_alt - 10
local cruise_height = 40.0      -- 目標巡航高度 (ミッションに依存)

-- 上昇回数に応じた上昇ピッチ角のテーブルリスト
local climb_cnt = 0         -- 上昇回数の初期化
local pitch_table_max = 22  -- 初回上昇時のピッチ角
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


-- フラグ管理&初期化
function reset_all_flags()
    trimspd_sw = false
    climb_sw = false
    get_climb_rate_sw = false
    get_sink_min_sw = false
    set_spdw_sw = false
    get_trim_thr_sw = false
end

-- DO_JUMP 検出関数
function check_jump_trigger()
    local current_index = mission:get_current_nav_index()
    if current_index == 2 and last_nav_index ~= 2 then
        last_nav_index = 2
        return true
    end
    last_nav_index = current_index
    return false
end

-- Box-Muller 法によるガウシアンノイズ生成関数
function noise(sigma)
    local a, b
    repeat
        a = math.random()
    until a > 0

    b = math.random()

    local z = math.sqrt(-2*math.log(a)) * math.cos(2*math.pi*b)
    return z * sigma
end

-- 現在の基本状態変数取得関数
-- @return pitch_deg, arspd_now, dh_now, height_now, throttle_now
function update_sensors()
    local pitch_deg = math.deg(ahrs:get_pitch())                        -- ピッチ角
    local arspd_now = ahrs:airspeed_estimate() + noise(arspd_std)                          -- 速度
    local dh_now =  - ahrs:get_velocity_NED():z() + noise(dh_std)                       -- 右手系 z 軸下向きの速度ベクトル (上昇率)
    local height_now =  - ahrs:get_relative_position_NED_home():z() + noise(height_std)     -- 地面からの相対高度 (下向き正)
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)     -- スロットル率
    return pitch_deg, arspd_now, dh_now, height_now, throttle_now
end

-- 安定性評価し, 判定バッファに記録, 全体が安定していれば all_ok を返す
-- @param  now_val           現在の値 (例：arspd_now)
-- @param  target_val        目標値 (例：TRIM_ARSPD_CM * 0.01)
-- @param  val_buff          値バッファ (例：arspd_val_buff)
-- @param  flag_buff         安定性判定用バッファ (例：arspd_buff ← これは値を持っているんじゃなくて, 要素が boolean 的)
-- @param  err_margin        値の許容誤差 (例：arspd_margin)
-- @param  slope_margin      許容値変化率 (例：arspd_diff_margin)
-- @return boolean all_ok
function val_stab_check(now_val, target_val, val_buff, flag_buff, stab_sw, err_margin, slope_margin, stddev_margin)
    -- バッファ値を更新
    -- 値を随時バッファに記録&サイズを超えたら古いデータから削除
    table.insert(val_buff, now_val)
    if #val_buff > buff_size then
        table.remove(val_buff, 1)
    end

    -- 目標値との誤差チェック
    local err = math.abs(now_val - target_val)
    local err_ok = err <= err_margin
    --[[
    -- 区間変化率の最大値チェック
    --local slope_ok = true
    local total_slope, count = 0, 0
    for i = 2, #val_buff do
        local dv = val_buff[i] - val_buff[i-1]
        local slope = math.abs(dv / dt)
        total_slope = total_slope + slope
        count = count + 1
        --[[
        if slope > slope_margin then
            slope_ok = false
            break
        end
    end
    local avg_slope = total_slope / count
    local slope_ok = avg_slope <= slope_margin
    ]]--

    -- 不偏標準偏差チェック (統計的安定性)
    local stddev_ok = false
    local stddev = 0
    --local stddev_ok = true
    
    if #val_buff >= 2 then
        local sum, sumsq = 0, 0
        for _, v in ipairs(val_buff) do
            sum = sum + v
            sumsq = sumsq + v*v
        end
        local mean = sum / #val_buff
        local variance = math.max(0, (sumsq - #val_buff*mean*mean) / (#val_buff - 1))
        stddev = math.sqrt(variance)
        stddev_ok = stddev <= stddev_margin
    end

    -- 総合的な安定性フラグ
    -- local stable = err_ok and slope_ok and stddev_ok
    local stable = err_ok and stddev_ok

    -- 安定性の判定失敗時には判定バッファをリセット
    if not stable then
        flag_buff = {}
    end

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

    -- プロポスイッチ ON でスクリプト有効化 (初回限定)
    if sw and not tecstune_running then
        gcs:send_text(0, string.format("TECS Tune Switch ON (Script Standby)"))
        tecstune_running = true
        tecstune_stage = 0
    end

    -- スイッチ ON 中かつ DO_JUMP によって WP2 になった瞬間に Stage=1 に移行
    if tecstune_running and tecstune_stage == 0 and check_jump_trigger() then
        tecstune_stage = 1
        gcs:send_text(6, "Start Acceleration (stage 1)")
    end

    if not sw then
        if tecstune_running then
            param:set_and_save("TRIM_ARSPD_CM", 2500)
            param:set_and_save("LIM_PITCH_MAX", 2500)
            param:set_and_save("TECS_SPDWEIGHT", 1)
            gcs:send_text(0, string.format("TECS Tune Switch OFF"))
            --mission:set_current_cmd(2)
        end
        -- sw 管理 (TecsTune Switch OFF で, すべて false)
        tecstune_stage = 0
        tecstune_running = false
        climb_cnt = 0
        reset_all_flags()
    end

    -- 加速フェーズ：上昇飛行直前に TRIM_ARSPD 一時的に 25 → 30 m/s とすることで間接的にスロットル率を上昇させる
    if tecstune_stage == 1 then
        local _, arspd_now, _, _, _ = update_sensors()
        local arspd_limit = set2up_arspd*0.01
        if not trimspd_sw then
            param:set_and_save("TRIM_ARSPD_CM", set2up_arspd)   --トリム速度値を変更
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(6, string.format("Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            trimspd_sw = true
        end

        if trimspd_sw and arspd_now > arspd_limit then
            tecstune_stage = 2
        end
    end

    -- 最大上昇率 & 最大・最小ピッチ角 & 理論最大降下率 取得フェーズ
    if tecstune_stage == 2 then
        local pitch_deg, arspd_now, dh_now, height_now, throttle_now = update_sensors()
        local item = mission:get_item(tkoff_cmd_num) -- 6番目のミッションを取得

        -- 上昇させるために TKOFF ミッションを書き換え & 上昇回数の記録
        if item and not climb_sw then
            --local lua_cnt = param:get("SCR_USER1")                                  -- 上昇回数を取得 (0スタート)
            local pitch_cmd = pitch_table[math.min(climb_cnt + 1, #pitch_table)]      -- 上昇回数に応じてピッチ角テーブルから次の上昇ピッチ角を取得
            param:set_and_save("LIM_PITCH_MAX", pitch_cmd*100)                        -- 機体の最大ピッチ角を再設定して上昇時のピッチ角オーバーシュートを防止
            gcs:send_text(6, string.format("Pitch up angle set to: %.2f deg.", pitch_cmd))
            
            item:command(22)            -- TKOFFコマンド番号
            item:param1(pitch_cmd)      -- 任意の上昇ピッチ角
            item:z(target_alt)          -- 上昇高度
            mission:set_item(tkoff_cmd_num, item)   -- TKOFFコマンドの指示内容を上書き
            mission:set_current_cmd(tkoff_cmd_num)  -- TKOFFコマンドへジャンプ
            gcs:send_text(0, "Start climb motion (stage 2)")
            
            -- 上昇飛行に移行後 TRIM_ARSPD = 25 m/s に復帰させる
            param:set_and_save("TRIM_ARSPD_CM", 2500)
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(6, string.format("Return Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            
            -- 上昇回数を加算
            --lua_cnt = lua_cnt + 1
            --param:set_and_save("SCR_USER1", lua_cnt)
            climb_cnt = climb_cnt + 1
            gcs:send_text(6, string.format("Climb Count: %d", climb_cnt))
            climb_sw = true
        end
        
        -- 上昇中処理 & 速度の監視 (70 ~ 140 m)
        --if climb_sw and height_now >= height_climb_min and throttle_now >= 99 then
        if climb_sw and height_now >= height_climb_min then
            local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
            -- 安定性チェック (速度バッファ・誤差・変化率)
            local arspd_stable = val_stab_check(arspd_now, arspd_target,
                                                arspd_val_buff, arspd_buff,
                                                get_climb_rate_sw, arspd_margin,
                                                arspd_diff_margin, arspd_std_margin)

            -- 安定性チェック後, 最大上昇率として取得&付随するパラメータの計算及び記録
            if arspd_stable then
                -- GCS 送信用の生データ
                local throttle_max = throttle_now
                local climb_max_rate = dh_now                                       -- 最大上昇率の取得
                local pitch_max = pitch_deg                                         -- この時の最大ピッチ角
                local pitch_min = - (pitch_max - 5.0)                               -- 最大ピッチ角に合わせた最小ピッチ角算出
                local sink_max_rate = arspd_max * math.sin(math.rad(pitch_min))     -- 理論最大降下率算出
                -- GCS へ結果を送信
                gcs:send_text(6, string.format("THR_MAX get to %.2f%%", throttle_max))
                gcs:send_text(6, string.format("TECS_CLIMB_MAX get to %.2f m/s", climb_max_rate))
                gcs:send_text(6, string.format("LIM_PITCH_MAX_DEG get to %.2f deg.", pitch_max))
                gcs:send_text(6, string.format("LIM_PITCH_MIN_DEG get to %.2f deg.", pitch_min))
                gcs:send_text(6, string.format("TECS_SINK_MAX get to %.2f m/s", sink_max_rate))
                --[[
                -- パラメータセット用データ (Increment & Range 調整後)
                local climb_max_rate_IR = math.max(0.1, math.min(math.floor(climb_max_rate*10 + 0.5) / 10))
                local pitch_max_IR = math.max(0.0, math.min(9000, math.floor(pitch_max / 1.0 + 0.5)*100))
                local pitch_min_IR = math.max(-9000, math.min(0, math.floor(pitch_min / 1.0 + 0.5)*100))
                local sink_max_rate_IR = math.max(0.1, math.min(20.0, math.floor(math.abs(sink_max_rate)*10 + 0.5) / 10))
                ]]--
                -- 記録テスト
                param:set_and_save("SCR_USER1", climb_max_rate)
                param:set_and_save("SCR_USER2", pitch_max*100)
                param:set_and_save("SCR_USER3", pitch_min*100)
                param:set_and_save("SCR_USER4", math.abs(sink_max_rate))
                --[[
                param:set_and_save("TECS_CLMB_MAX", climb_max_rate)
                param:set_and_save("LIM_PITCH_MAX", pitch_max)
                param:set_and_save("LIM_PITCH_MIN", pitch_min)
                param:set_and_save("TECS_SINK_MAX", sink_max_rate)
                ]]--

                get_climb_rate_sw = true
                mission:set_current_cmd(2)
                gcs:send_text(0, "Start get TECS_SINK_MIN (stage 3)")
                tecstune_stage = 3
            end
        end

        -- 最大上昇高度に達し強制終了
        if climb_sw and height_now >= target_alt_limit then
            tecstune_stage = 0
            reset_all_flags()
            mission:set_current_cmd(dojump_cmd_num)      -- DO_JUMP コマンドへ移行
            --vehicle:set_mode(10)                       -- 勝手にRTLモードになる場合があるため Auto モードを噛ませる
            gcs:send_text(0, "Return to Stage 1")
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

        if set_spdw_sw and not get_sink_min_sw and throttle_now == thr_min then
            -- 安定性チェック (速度バッファ・誤差・変化率)
            local arspd_stable = val_stab_check(arspd_now, arspd_target,
                                                arspd_val_buff, arspd_buff,
                                                nil, arspd_margin,
                                                arspd_diff_margin, arspd_std_margin)

            if arspd_stable and dh_now < 0 then
                -- GCS 送信用の生データ
                local sink_min_rate = dh_now
                -- GCS へ結果を送信
                gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                -- パラメータセット用データ (Increment & Range 調整後)
                --local sink_min_rate_IR = math.max(0.1, math.min(10.0, math.floor(math.abs(sink_min_rate)*10 + 0.5) / 10))
                -- 記録テスト
                param:set_and_save("SCR_USER5", math.abs(sink_min_rate))
                --param:set_and_save("TECS_SINK_MIN", sink_min_rate)
                
                param:set_and_save("TECS_SPDWEIGHT", 1)
                gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
                gcs:send_text(0, "Start get TRIM_THROTTLE (stage 4)")
                get_sink_min_sw = true
                --mission:set_current_cmd(2)
                tecstune_stage = 4
            end
        end
    end
    
    -- TRIM_THROTTLE 率の取得：速度が 25 m/s で安定 かつ 高度と上昇率が所定で安定
    if tecstune_stage == 4 then
        local _, arspd_now, dh_now, height_now, throttle_now = update_sensors()
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01

        -- 速度安定性チェック
        local arspd_stable = val_stab_check(arspd_now, arspd_target,
                                            arspd_val_buff, arspd_buff,
                                            get_trim_thr_sw, arspd_margin,
                                            arspd_diff_margin, arspd_std_margin)

        -- 高度&上昇率安定性チェック (解析的)
        local alt_dh_stable = val_stab_check(height_now, cruise_height,
                                             height_val_buff, height_buff,
                                             get_trim_thr_sw, height_margin,
                                             dh_margin, height_std_margin)

        if not get_trim_thr_sw
            and arspd_stable
            and alt_dh_stable
            and math.abs(dh_now) <= dh_margin then
            
            -- GCS 送信用の生データ
            local trim_throttle = throttle_now
            -- GCS へ結果を送信
            gcs:send_text(0, string.format("TRIM_THROTTLE get to %.2f%%", trim_throttle))
            -- パラメータセット用データ (Increment & Range 調整後)
            --local trim_throttle_IR = math.max(0, math.min(100, math.floor(trim_throttle + 0.5)))
            -- 記録テスト
            param:set_and_save("SCR_USER6", trim_throttle)
            --param:set_and_save("TRIM_THROTTLE", trim_throttle)
            
            gcs:send_text(0, string.format("Finished TecsTune"))
            get_trim_thr_sw = true
            --mission:set_current_cmd(2)
            --vehicle:set_mode(10)
        end
    end
end


function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc then
        local sw_pos = scripting_rc:get_aux_switch_pos()
        if sw_pos == 1 then
            vehicle:set_mode(10)
            tecstuning(true)
        else
            tecstuning(false)
        end
    end

    return update, FREQUENCY
end

return update()